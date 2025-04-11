#include <Servo.h>
#include <SoftwareSerial.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <SPI.h>
#include <RunningAverage.h>
#include <stdlib.h>

#define WAIT_DELAY               5000
#define nombreDePixelsEnLargeur 128         // Taille de l'écran OLED, en pixel, au niveau de sa largeur
#define nombreDePixelsEnHauteur 64          // Taille de l'écran OLED, en pixel, au niveau de sa hauteur
#define brocheResetOLED         -1          // Reset de l'OLED partagé avec l'Arduino (d'où la valeur à -1, et non un numéro de pin)
#define adresseI2CecranOLED     0x3C   
#define MCP_NOP                 0b00000000
#define MCP_WRITE               0b00010001
#define MCP_SHTDWN              0b00100001

#define encoder0PinA            2  //CLK Output A Do not use other pin for clock as we are using interrupt
#define encoder0PinB            4  //DT Output B
#define RX_PIN                  7
#define TX_PIN                  8
#define Switch                  6 // Switch connection if available
#define flexPin                 A1
#define amplificateur           A0
#define servomoteur             9
#define ssMCPin                 10

#define etat                    7
#define bufferSize              16

const byte csPin               = 10;
const int maxPositions         = 256;
const long rAB                 = 33800;
const byte rWiper              = 125;
const byte pot0                = 0x11;
const byte pot0shutdown        = 0x21;

float R3;
char bufferInput[bufferSize];

// Définir les broches pour SoftwareSerial
Adafruit_SSD1306 ecranOLED(nombreDePixelsEnLargeur, nombreDePixelsEnHauteur, &Wire, brocheResetOLED);
Servo myServo;
RunningAverage myRA(20);
SoftwareSerial mySerial(RX_PIN, TX_PIN);

// byte position = 0;
volatile int encoder0Pos = 0;
int posmoteur = 0, oldPOSencodeur = 0;
unsigned long previousMillis = 0, previousMillis2 = 0;
float moy = 0;
char OK_Blue = '0', chaine[10], valueChar;


void setup() {
  myRA.clear();
  pinMode (ssMCPin, OUTPUT); //select pin output
  digitalWrite(ssMCPin, HIGH); //SPI chip disabled
  SPI.begin(); 
 
  
  
  if(!ecranOLED.begin(SSD1306_SWITCHCAPVCC, adresseI2CecranOLED))
    while(1);     

  ecranOLED.clearDisplay();
  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor

  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor

  attachInterrupt(0, doEncoder, RISING);
  pinMode(flexPin,INPUT); 
  
  myServo.attach(9);
  setPotWiper(pot0, 100);
  Serial.begin(9600);
  mySerial.begin(9600);
  Serial.println(F("[Arduino Sensor - HAHN & LONGEPIERRE]"));
}

void loop() {
  switch(etat){
    case 1:
      Serial.println(analogRead(flexPin));
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

void doEncoder() {
  if (OK_Blue != '1') {
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
  float ADCgraph = analogRead(amplificateur);
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
  unsigned long currentMillis2 = millis();
  float value;

  if (currentMillis - previousMillis >= 500) {
    previousMillis = currentMillis;

    /*
    if (Serial.available() > 0) {
     OK_Blue = Serial.read();
     Serial.println(OK_Blue);
     Serial.flush();
    } 
    */

    if (OK_Blue == '0') {
      ecranOLED.clearDisplay();                                   // Effaçage de l'intégralité du buffer
      ecranOLED.setTextSize(2);                   // Taille des caractères (1:1, puis 2:1, puis 3:1)
      ecranOLED.setCursor(0, 0);
      ecranOLED.setTextColor(SSD1306_WHITE,SSD1306_BLACK);
      ecranOLED.println(F("---MENU---"));
      ecranOLED.setTextSize(1);

      switch (abs(encoder0Pos)) {
        case 0 :
          ecranOLED.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
          ecranOLED.println(F("1. Graphite Sensor"));
          ecranOLED.setTextColor(SSD1306_WHITE,SSD1306_BLACK);   // Couleur du texte, et couleur du fond
                              // Affichage du texte en "blanc" (avec la couleur principale, en fait, car l'écran monochrome peut être coloré)
          ecranOLED.println(F("2. Flex Sensor"));
          ecranOLED.println(F("3. Servo Motor"));
          ecranOLED.display();
          break;

        case 1 :
          ecranOLED.println(F("1. Graphite Sensor"));         
          ecranOLED.setTextColor(SSD1306_BLACK, SSD1306_WHITE);         
          ecranOLED.println(F("2. Flex Sensor"));
          ecranOLED.setTextColor(SSD1306_WHITE,SSD1306_BLACK);
          ecranOLED.println(F("3. Servo Motor"));
          ecranOLED.display(); 
          break;

        case 2 :
          ecranOLED.println("1. Graphite Sensor");         
          ecranOLED.println("2. Flex Sensor");
          ecranOLED.setTextColor(SSD1306_BLACK, SSD1306_WHITE); 
          ecranOLED.println("3. Servo Motor");
          ecranOLED.display(); 
          break;
      }
    }  

    if (OK_Blue == '1') {

      switch (abs(encoder0Pos)) {
        case 0 :
          value = graphiteSensor();
          dtostrf(value, 5, 2, chaine);
          Serial.println(chaine);
          ecranOLED.clearDisplay();
          ecranOLED.setCursor(0, 0);
          ecranOLED.setTextSize(2);
          ecranOLED.setTextColor(SSD1306_WHITE,SSD1306_BLACK);
          ecranOLED.println(F("--Menu 1--"));
          ecranOLED.setTextSize(1);
          ecranOLED.println(F("Graphite Sensor"));
          ecranOLED.setTextSize(1);
          ecranOLED.println(F(""));
          ecranOLED.print(chaine);
          ecranOLED.print(F(" Ohms"));
          ecranOLED.display();
          break;

        case 1 :
          value = flexSensor();
          dtostrf(value, 5, 2, chaine);
          ecranOLED.clearDisplay();
          ecranOLED.setCursor(0, 0);
          ecranOLED.setTextSize(2);
          ecranOLED.setTextColor(SSD1306_WHITE,SSD1306_BLACK);
          ecranOLED.println(F("--Menu 2--"));
          ecranOLED.setTextSize(1);
          ecranOLED.println(F("Flex Sensor"));
          ecranOLED.println(F(""));
          ecranOLED.print(chaine);
          ecranOLED.print(F(" Ohms"));
          ecranOLED.display(); 
          break;

        case 2 :
          int val = servoMotor();
          sprintf(chaine, "%d", val);
          Serial.flush();
          ecranOLED.clearDisplay();
          ecranOLED.setCursor(0, 0);
          ecranOLED.setTextSize(2);
          ecranOLED.setTextColor(SSD1306_WHITE,SSD1306_BLACK);
          ecranOLED.println(F("--Menu 3--"));
          ecranOLED.setTextSize(1);
          ecranOLED.println(F("Servo Motor"));
          ecranOLED.println(F(""));
          ecranOLED.print(F("Position : "));
          ecranOLED.print(chaine);
          ecranOLED.display();
          break;
      }
    }

    if (currentMillis2 - previousMillis2 >= 5000) {
      previousMillis2 = currentMillis2;
      if (OK_Blue == '1')
        OK_Blue = '0';
      else
        OK_Blue = '1';
    }
  }
  // delay(500);
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



