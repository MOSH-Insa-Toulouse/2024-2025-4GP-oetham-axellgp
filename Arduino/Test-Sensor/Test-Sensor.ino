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

const int csPin                = 10;
const int maxPositions         = 256;
const long rAB                 = 33800;
const byte rWiper              = 125;
const byte pot0                = 0x11;
const byte pot0shutdown        = 0x21;
float R3;

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

unsigned long previousMillis = 0;
float moy = 0;
char chaine[10], valueChar, cringe[16];
int inMenu = 0;

// #define WAIT_DELAY              5000

#define etat                    5
#define bufferSize              16
#define baudRate                9600

char bufferInput[bufferSize];

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
}

void loop() {
  menuChoice();
}

void getSW() {

  if (!(digitalRead(Switch))) {
    if (inMenu == 0)
      inMenu++;
    else
      inMenu--;
  }
}

void setPotWiper(int addr, int pos) {
  pos = constrain(pos, 0, 255);
  digitalWrite(csPin, LOW);
  SPI.transfer(addr);
  SPI.transfer(pos);
  digitalWrite(csPin, HIGH);

  R3 = ((rAB * pos) / maxPositions) + rWiper;
}

void Calibration() {
  float target = 3.0, tol = 0.15;
  int pos = 0;

  do {
    setPotWiper(pot0, pos);
    pos += 5;
    delay(200);
  } while ((graphiteSensor() < (target - tol) || graphiteSensor() > (target + tol)) && pos <= 265);
  
  dtostrf(pos, 5, 2, chaine);
  
  if (pos < 265) {
    Serial.println(F("Potentiometer calibrated at position : "));
    Serial.print(chaine);

    float val = graphiteSensor();
    dtostrf(val, 10, 2, chaine);
    Serial.println(F("Value : "));
    Serial.print(chaine);
  }
  else {
    Serial.println(F("Potentiometer not calibrated at target 3V"));
  }
} 

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

  //Serial.println (encoder0PinA, DEC);  //Angle = (360 / Encoder_Resolution) * encoder0Pos
}

float flexSensor() {
  float ADCflex = analogRead(flexPin);
  int VCC = 5; // 5V
  int R_DIV = 10000;
  float Vflex = ADCflex * VCC / 1024.0;
  float Rflex = R_DIV * (VCC / Vflex - 1.0);

  return Rflex;
}

float graphiteSensor() {
  float R2 = 100000, R1 = 10000, R4 = 100000, Res;
  float ADCgraph = analogRead(graphitePin);
  int VCC = 5;
  float Vgraph = ADCgraph * VCC / 1024.0;
  
  Res = R2 * (1 + R4/R3) * (VCC / Vgraph) - R2 - R1; 

  return Res;
}

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

  if (currentMillis - previousMillis >= 500) {
    previousMillis = currentMillis;

    getSW();
    ecranOLED.clearDisplay();                                    // Effaçage de l'intégralité du buffer
    ecranOLED.setTextSize(2);                                    // Taille des caractères (1:1, puis 2:1, puis 3:1)
    ecranOLED.setCursor(0, 0);
    
    if (inMenu == 0) {             
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
          ecranOLED.println("1. Graphite Sensor");         
          ecranOLED.println("2. Flex Sensor");
          ecranOLED.setTextColor(SSD1306_BLACK, SSD1306_WHITE); 
          ecranOLED.println("3. Servo Motor");
          break;
      }
    }  

    else if (inMenu == 1) {

      switch (abs(encoder0Pos)) {
        case 0 :
          value = graphiteSensor();
          dtostrf(value, 16, 2, cringe);
          ecranOLED.setTextColor(SSD1306_WHITE,SSD1306_BLACK);
          ecranOLED.println(F("--Menu 1--"));
          ecranOLED.setTextSize(1);
          ecranOLED.println(F("Graphite Sensor"));
          ecranOLED.setTextSize(1);
          ecranOLED.println(F(""));
          ecranOLED.print(cringe);
          ecranOLED.print(F(" Ohms"));
          break;

        case 1 :
          value = flexSensor();
          dtostrf(value, 5, 2, chaine);
          ecranOLED.setTextColor(SSD1306_WHITE,SSD1306_BLACK);
          ecranOLED.println(F("--Menu 2--"));
          ecranOLED.setTextSize(1);
          ecranOLED.println(F("Flex Sensor"));
          ecranOLED.println(F(""));
          ecranOLED.print(chaine);
          ecranOLED.print(F(" Ohms"));
          break;

        case 2 :
          int val = servoMotor();
          sprintf(chaine, "%d", val);
          Serial.flush();
          ecranOLED.setTextColor(SSD1306_WHITE,SSD1306_BLACK);
          ecranOLED.println(F("--Menu 3--"));
          ecranOLED.setTextSize(1);
          ecranOLED.println(F("Servo Motor"));
          ecranOLED.println(F(""));
          ecranOLED.print(F("Position : "));
          ecranOLED.print(chaine);
          break;
      }
    }
    ecranOLED.display();
  }
}

void SPIWrite(uint8_t cmd, uint8_t data, uint8_t ssPin) // SPI write the command and data to the MCP IC connected to the ssPin
{
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0)); //https://www.arduino.cc/en/Reference/SPISettings
  
  digitalWrite(ssPin, LOW); // SS pin low to select chip
  
  SPI.transfer(cmd);        // Send command code
  SPI.transfer(data);       // Send associated value
  
  digitalWrite(ssPin, HIGH);// SS pin high to de-select chip
  SPI.endTransaction();
}

/* Communicate
  OPTION 1 :

  if(Serial.available()) {
    do{
      bufferInput[i] = (char) Serial.read();
      i++;
      delay(5);
    } while(Serial.available () > 0);

    Serial.println(bufferInput);
    Serial.println("DONE");

    bytes = atoi(bufferInput);

    if (bytes == 1234) {
      Serial.println("GOOD JOB");
    }
  }

  OPTION 2 : 

  if (btSerial.available()) {
    byte c = btSerial.read();
    Serial.write(c);
    Serial.println("");
  }

  // Überprüfe, ob Daten vom seriellen Port eingehen
  if (Serial.available() > 0) {
    byte b = Serial.parseInt(); // lit un entier (ex: "200\n" => 200)
    Serial.print("Envoyé vers Bluetooth : ");
    Serial.println(b);
    btSerial.write(b); // envoie le byte brut
  }

  switch(etat){
    case 1:
      //Serial.println(analogRead(flexPin));
      break;

    case 2 :
      if (mySerial.available()) {
      // Lire les données de mySerial
        char data_Servo = mySerial.read();
        // Envoyer les données à Serial
        Serial.print(data_Servo);
        myServo.write(data_Servo);

        // Increment the position of the servo motor
        data_Servo++;
      }

      myServo.write(200);

      delay(100);
      break;

    case 3 ://encodeur
      Serial.println (encoder0Pos, DEC);
      break;

    case 4 : // bluetooth
      if (mySerial.available()) {
      // Lire les données de mySerial
        byte data = mySerial.read();
      // Envoyer les données à Serial
        Serial.println(data);
        Serial.println("");
      }

      // Vérifier si des données sont disponibles sur Serial
      if (Serial.available()) {
        // Lire les données de Serial
        byte b = Serial.parseInt(); // lit un entier (ex: "200\n" => 200)
        Serial.print("Envoyé vers Bluetooth : ");
        Serial.println(b);
        mySerial.write(b);
      }
      break;

    case 5 : //oled
      
      menuChoice();

      break;

    case 6:
    
      if (oldPOSencodeur != encoder0Pos) {
        SPIWrite(MCP_WRITE, abs(encoder0Pos)%256, ssMCPin);
        delay(100);
        oldPOSencodeur= encoder0Pos;
      }
      myRA.addValue(analogRead(flexPin));
      
      Serial.println(myRA.getAverage());

      break;

    case 7 :
      moy = 0;
      for (int i = 0; i < 10; i++) {
        float val = graphiteSensor();
        moy += val;
      }
      moy /= 10;
      
      dtostrf(moy, 5, 2, bufferInput);
      Serial.println(bufferInput);
      delay(500);

      break;

    case 8 :
      ecranOLED.clearDisplay();                                   // Effaçage de l'intégralité du buffer
      ecranOLED.setTextSize(2);                   // Taille des caractères (1:1, puis 2:1, puis 3:1)
      ecranOLED.setCursor(0, 0);
      ecranOLED.setTextColor(SSD1306_WHITE,SSD1306_BLACK);
      ecranOLED.println(F("---MENU---"));
      ecranOLED.display();
    break;
  }
*/



