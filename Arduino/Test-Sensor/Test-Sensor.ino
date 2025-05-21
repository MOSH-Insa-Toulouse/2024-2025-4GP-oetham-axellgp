#include <Servo.h>                          // Librairie Servo Moteur
#include <SoftwareSerial.h>                 // Librairie Bluetooth
#include <Adafruit_SSD1306.h>               // Librairie OLED
#include <Wire.h>                           // Utilisation de l'OLED
#include <SPI.h>                            // Librairie Potentiometre 
#include <RunningAverage.h>
#include <stdlib.h>

/* 
*   CONSTANTES ET DECLARATIONS : OLED
*/

#define nombreDePixelsEnLargeur 128         // Taille de l'écran OLED, en pixel, au niveau de sa largeur
#define nombreDePixelsEnHauteur 64          // Taille de l'écran OLED, en pixel, au niveau de sa hauteur
#define brocheResetOLED         -1          // Reset de l'OLED partagé avec l'Arduino (d'où la valeur à -1, et non un numéro de pin)
#define adresseI2CecranOLED     0x3C 

Adafruit_SSD1306 ecranOLED(nombreDePixelsEnLargeur, nombreDePixelsEnHauteur, &Wire, brocheResetOLED);

/*
*   CONSTANTES ET DECLARATIONS : BLUETOOTH
*/

#define RX_PIN                  7
#define TX_PIN                  8

SoftwareSerial mySerial(RX_PIN, TX_PIN);

/*
*   CONSTANTES ET DECLARATIONS : POTENTIOMETRE DIGITAL
*/

#define MCP_NOP                 0b00000000
#define MCP_WRITE               0b00010001
#define MCP_SHTDWN              0b00100001
#define ssMCPin                 10
int buffer1[512];
const int csPin                = 10;
const int maxPositions         = 256;
const long rAB                 = 33800;
const byte rWiper              = 125;
const byte pot0                = 0x11;
const byte pot0shutdown        = 0x21;
float R3;
float intposServo =0 ;
/*
*   CONSTANTES ET DECLARATIONS : ENCODEUR ROTATOIRE
*/

#define encoder0PinA            2  // CLK Output A Do not use other pin for clock as we are using interrupt
#define encoder0PinB            4  // DT Output B
#define Switch                  6  // Switch connection if available

volatile int encoder0Pos = 0;
int oldPOSencodeur = 0, buttonState = 0;

/*
*   CONSTANTES ET DECLARATIONS : SERVO MOTEUR
*/

#define servoPin                9

Servo myServo;

int posmoteur = 0;

/*
*   CONSTANTES ET DECLARATIONS : FLEX SENSOR
*/

#define flexPin                 A1

/*
*   CONSTANTES ET DECLARATIONS : GRAPHITE SENSOR
*/

#define graphitePin             A0
volatile int encoderFullPos = 0;
unsigned long previousMillis = 0;         // Important to check the time that has passed since the last millis()
float moy = 0;                            // Initialize mean to 0 in order to computate the average
char chaine[10],                          // Placeholder char variable of size 10 to display on the OLED
  cringe[16],  
  gain[10],
  posServo[10];                           // Placeholder char variable of size 16 to display on the OLED (to accommodate the larger values of the graphite resistance) i.e. I was annoyed when I found the solution. The name of the variable is fitting...
int inMenu = 0,                           // Value to see if the user has selected the menu
  Calibrated = 0;                         // Value to check if Calibration() has been completed

// #define WAIT_DELAY              5000

#define etat                    5
#define bufferSize              16
#define baudRate                9600

char bufferInput[bufferSize], constant = '0';

byte c = 0, Cgain = 0;                    // Quand on appelle un Serial.println(c), il faut que c ait une valeur. Sinon, ca va faire buguer l'OLED 


RunningAverage myRA(20);

void setup() {
  myRA.clear();

  pinMode (ssMCPin, OUTPUT);              // Select Pin Output
  digitalWrite(ssMCPin, HIGH);            // SPI Chip Disabled
  SPI.begin();
  
  if(!ecranOLED.begin(SSD1306_SWITCHCAPVCC, adresseI2CecranOLED))
    while(1);     

  ecranOLED.clearDisplay();

  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);       // Turn on Pullup Resistor

  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);       // Turn on Pullup Resistor

  pinMode(Switch, INPUT);
  digitalWrite(Switch, HIGH);             // Turn on Pullup Resistor

  attachInterrupt(0, doEncoder, RISING);
  pinMode(flexPin, INPUT); 

  Serial.begin(baudRate);
  mySerial.begin(baudRate);

  myServo.attach(servoPin);
  
  delay(1000);

  Serial.println(F("[Arduino Sensor - HAHN & LONGEPIERRE]"));

  Calibration();  
  clearRead();                     // Oublie pas a enlever le commentaire
}

/*
*   Dsiplays the menu on the OLED screen
*/

void loop() {
  menuChoice();
  if (mySerial.available() > 0) {
    c = mySerial.read();
    c = c + 0;
    
    if (c == 'f')
      if (mySerial.read() == 'f')
        if (mySerial.read() == 'f')
          if (mySerial.read() == 'f') {
            encoder0Pos = 1;
            inMenu = 1;
          }
    if (c == 'g')
      if (mySerial.read() == 'g')
        if (mySerial.read() == 'g')
          if (mySerial.read() == 'g') {
            encoder0Pos = 0; 
            inMenu = 1;
          }
    if (c == 's')
      if (mySerial.read() == 's')
        if (mySerial.read() == 's')
          if (mySerial.read() == 's') {
            encoder0Pos=2;
            inMenu=1;
          }
    if (c == 'c')
      if (mySerial.read() == 'c')
        if (mySerial.read() == 'c')
          if (mySerial.read() == 'c') {
            Calibrated = 0;
            Calibration();
          }
    clearRead();
  }
}

/*
*   Reads if switch button on the rotary encoder has been pushed
*/

void getSW() {

  if (!(digitalRead(Switch))) {
    if (inMenu == 0)
      inMenu++;
    else
      inMenu--;
  }
}

/*
*   Sets the position of the Digital Potentiometer
*/


void setPotWiper(int addr, int pos) {
  pos = constrain(pos, 0, 255);
  digitalWrite(csPin, LOW);
  SPI.transfer(addr);
  SPI.transfer(pos);
  digitalWrite(csPin, HIGH);

  R3 = ((rAB * pos) / maxPositions) + rWiper;
}

/*
*   Calibrates the received signal at 3V with a tolerance of 0.15V
*/

void Calibration() {
  float target = 2.5, tol = 0.4;
  int pos = 0;

  do {
    setPotWiper(pot0, pos);
    pos += 2;
    delay(100);
  } while ((graphiteSensor() < (target - tol) || graphiteSensor() > (target + tol)) && pos <= 265);
  
  dtostrf(pos, 5, 2, chaine);
  
  if (pos < 265) {
    Serial.println(F("Potentiometer calibrated at position : "));
    Serial.print(chaine);

    float val = graphiteSensor();
    dtostrf(val, 10, 2, chaine);
    dtostrf(pos, 5, 0, gain);
    Cgain=pos;
    setPotWiper(pot0, Cgain);
    Serial.println(F("Value : "));
    Serial.print(chaine);
  }
  else {
    Serial.println(F("Potentiometer not calibrated at target 3V"));
    setPotWiper(pot0, Cgain);
  }
  Calibrated = 1;
} 

/*
*   Reads the rotary encoder's rotation (clockwise or counter-clockwise)
*/

void doEncoder() {
  if (inMenu == 0) {
    if (digitalRead(encoder0PinA)==HIGH && digitalRead(encoder0PinB)==HIGH) {
      encoder0Pos++;
    } 
    else if (digitalRead(encoder0PinA)==HIGH && digitalRead(encoder0PinB)==LOW) {
      encoder0Pos--;   
    }
    encoder0Pos %= 3;
  }
  if (digitalRead(encoder0PinA)==HIGH && digitalRead(encoder0PinB)==HIGH) {
     encoderFullPos+=18;
   } 
   else if (digitalRead(encoder0PinA)==HIGH && digitalRead(encoder0PinB)==LOW) {
     encoderFullPos-=18;   
   }
   encoderFullPos = (encoderFullPos) %180;
  //Serial.println (encoder0PinA, DEC);  //Angle = (360 / Encoder_Resolution) * encoder0Pos
}

/*
*   Clears the bluetooth's buffer
*/

void clearRead() {
  while (mySerial.available() > 0) {
    byte a = mySerial.read();
  }
}

float flexSensor() {
  float ADCflex = analogRead(flexPin);
  int VCC = 5; // 5V
  int R_DIV = 10000;
  float Vflex = ADCflex * VCC / 1024.0;
  float Rflex = R_DIV * (VCC / Vflex - 1.0);
  byte toBlue = ADCflex / 4.0;

  mySerial.write(toBlue);

  return Rflex;
}

/*
*   Retrieves the Graphite Sensor's value 
*/

float graphiteSensor() {
  float R2 = 100000, R1 = 10000, R4 = 100000, Res;
  float ADCgraph = analogRead(graphitePin);
  int VCC = 5;
  float Vgraph = ADCgraph * VCC / 1024.0;
  
  Res = R2 * (1 + R4/R3) * (VCC / Vgraph) - R2 - R1;
  mySerial.write(ADCgraph / 4);

  if (Calibrated == 0) {
    return Vgraph;
  }
  else {
    return Res;
  }
}

/*
*   Sends the value to the Servo Motor
*/

int servoMotor() {
  int val = 0;
  static int xd = 0;
  char str[256];
  int i = 0, bytes;

  return val;
}

/*    
*     Menu Choice :
*     1 : Graphite Sensor
*     2 : Flex Sensor
*     3 : Servo Motor
*/

void menuChoice() { 

  unsigned long currentMillis = millis();
  float value;

  if (currentMillis - previousMillis >= 200) {
    previousMillis = currentMillis;

    getSW();
    ecranOLED.clearDisplay();                                    // Effaçage de l'intégralité du buffer
    ecranOLED.setTextSize(2);                                    // Taille des caractères (1:1, puis 2:1, puis 3:1)
    ecranOLED.setCursor(0, 0);
    
    if (inMenu == 0) {   
      encoderFullPos=0;          
      ecranOLED.setTextColor(SSD1306_WHITE,SSD1306_BLACK);       // Affichage du texte en "blanc" (avec la couleur principale, en fait, car l'écran monochrome peut être coloré)
      ecranOLED.println(F("---MENU---"));
      ecranOLED.setTextSize(1);

      switch (abs(encoder0Pos)) {
        case 0 :
          ecranOLED.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
          ecranOLED.println(F("1. Graphite Sensor"));
          ecranOLED.setTextColor(SSD1306_WHITE,SSD1306_BLACK);   // Couleur du texte, et couleur du fond
          ecranOLED.println(F("2. Flex Sensor"));
          ecranOLED.println(F("3. Servo Motor"));
          break;

        case 1 :
          ecranOLED.println(F("1. Graphite Sensor"));         
          ecranOLED.setTextColor(SSD1306_BLACK, SSD1306_WHITE);         
          ecranOLED.println(F("2. Flex Sensor"));
          ecranOLED.setTextColor(SSD1306_WHITE,SSD1306_BLACK);
          ecranOLED.println(F("3. Servo Motor"));
          break;

        case 2 :
          ecranOLED.println(F("1. Graphite Sensor"));         
          ecranOLED.println(F("2. Flex Sensor"));
          ecranOLED.setTextColor(SSD1306_BLACK, SSD1306_WHITE); 
          ecranOLED.println(F("3. Servo Motor"));
          break;
      }
    }  

    else if (inMenu == 1) {

      switch (abs(encoder0Pos)) {
        case 0 :
          value = graphiteSensor();
          dtostrf(value, 16, 2, cringe);
          
          if (c != 'c')
            if (c != 'g')
              if (c != 'f')
                if (c != 's')
                  if (c != 0)
                    if (c != Cgain) {
                      setPotWiper(pot0, c);
                      Cgain=c;
                      clearRead(); 
                    }

          clearRead();   
          dtostrf(Cgain, 6, 0, gain);

          ecranOLED.setTextColor(SSD1306_WHITE,SSD1306_BLACK);
          ecranOLED.println(F("--Menu 1--"));
          ecranOLED.setTextSize(1);
          ecranOLED.println(F("Graphite Sensor"));
          ecranOLED.setTextSize(1);
          ecranOLED.println(F(""));
          ecranOLED.print(cringe);
          ecranOLED.println(F(" Ohms"));
          ecranOLED.print(F(" Gain :"));
          ecranOLED.print(gain);
          break;

        case 1 :
        
          value = flexSensor();
          dtostrf(value, 5, 2, chaine);
          clearRead(); 

          ecranOLED.setTextColor(SSD1306_WHITE,SSD1306_BLACK);
          ecranOLED.println(F("--Menu 2--"));
          ecranOLED.setTextSize(1);
          ecranOLED.println(F("Flex Sensor"));
          ecranOLED.println(F(""));
          ecranOLED.print(chaine);
          ecranOLED.println(F(" Ohms"));
          break;

        case 2 :
        char stringencoderFullPos[20];
        intposServo = abs(c / 256.0 * 360.0);
          if (c != 'c')
            if (c != 'g')
              if (c != 'f')
                if (c != 's')
                  dtostrf(int(intposServo), 3, 0, posServo); 
                  
          int val = servoMotor();
          sprintf(chaine, "%d", val);
          Serial.flush();
          clearRead(); 
          dtostrf(abs(encoderFullPos), 3, 0, stringencoderFullPos);
          ecranOLED.setTextColor(SSD1306_WHITE,SSD1306_BLACK);
          ecranOLED.println(F("--Menu 3--"));
          ecranOLED.setTextSize(1);
          ecranOLED.println(F("Servo Motor"));
          ecranOLED.println(F(""));
          ecranOLED.print(F("Position : "));
          ecranOLED.print(posServo);
          ecranOLED.print(F("+ "));
          ecranOLED.print(stringencoderFullPos);
          //myServo.write((valservo+abs(encoderFullPos)) % 360);
          myServo.write((int(intposServo)+abs(encoderFullPos)) % 180);
          break;
      }
    }
    ecranOLED.display();
  }
}

/*
*   Writes the command and data through SPI to the MCP IC connected to the ssPin
*/

void SPIWrite(uint8_t cmd, uint8_t data, uint8_t ssPin)
{
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0)); //https://www.arduino.cc/en/Reference/SPISettings
  
  digitalWrite(ssPin, LOW); // SS pin low to select chip
  
  SPI.transfer(cmd);        // Send command code
  SPI.transfer(data);       // Send associated value
  
  digitalWrite(ssPin, HIGH);// SS pin high to de-select chip
  SPI.endTransaction();
}
