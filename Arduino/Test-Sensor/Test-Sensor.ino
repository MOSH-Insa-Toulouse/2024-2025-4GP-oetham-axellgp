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

#define etat                    5

// Définir les broches pour SoftwareSerial
Adafruit_SSD1306 ecranOLED(nombreDePixelsEnLargeur, nombreDePixelsEnHauteur, &Wire, brocheResetOLED);
Servo myServo;
RunningAverage myRA(20);
SoftwareSerial mySerial(RX_PIN, TX_PIN);

byte position = 0;
volatile int encoder0Pos = 0;
int posmoteur = 0, oldPOSencodeur = 0;
unsigned long previousMillis = 0;

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
  Serial.begin(9600);
  mySerial.begin(9600);
  Serial.println(F("[MCP Digital Pot Test]"));
}

void loop() {
  switch(etat){
    case 1:
      Serial.println(analogRead(flexPin));
      break;
    case 2 :
      myServo.write(posmoteur);
      delay(100);
      posmoteur++;
      break;
    case 3 ://encodeur
      Serial.println (encoder0Pos, DEC);
      break;
    case 4 : // bleutooth
      if (mySerial.available()) {
      // Lire les données de mySerial
      char data = mySerial.read();
      // Envoyer les données à Serial
      Serial.print(data);
      }

  // Vérifier si des données sont disponibles sur Serial
      if (Serial.available()) {
        // Lire les données de Serial
        char data = Serial.read();
        // Envoyer les données à mySerial
        mySerial.print(data);
      }
      break;
    case 5 : //oled
      
      menuChoice();

    case 6:
    
      if (oldPOSencodeur != encoder0Pos) {
        SPIWrite(MCP_WRITE, abs(encoder0Pos)%256, ssMCPin);
        delay(100);
        oldPOSencodeur= encoder0Pos;
      }
      myRA.addValue(analogRead(flexPin));
      
      Serial.println(myRA.getAverage());
    }
  }
}

/*    
*     Menu Choice :
*     1 : Graphite Sensor
*     2 : Flex Sensor
*     3 : Servo Motor
*/

void menuChoice() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 500) {
    previousMillis = currentMillis;
    ecranOLED.clearDisplay();                                   // Effaçage de l'intégralité du buffer
    ecranOLED.setTextSize(2);                   // Taille des caractères (1:1, puis 2:1, puis 3:1)
    ecranOLED.setCursor(0, 0);
    ecranOLED.setTextColor(SSD1306_WHITE,SSD1306_BLACK);
    ecranOLED.println("---MENU---");
    ecranOLED.setTextSize(1);

    switch (abs(encoder0Pos)) {
      case 0 :                                 // Déplacement du curseur en position (0,0), c'est à dire dans l'angle supérieur gauche
        ecranOLED.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        ecranOLED.println("1. Graphite Sensor");
        ecranOLED.setTextColor(SSD1306_WHITE,SSD1306_BLACK);   // Couleur du texte, et couleur du fond
                            // Affichage du texte en "blanc" (avec la couleur principale, en fait, car l'écran monochrome peut être coloré)
        ecranOLED.println("2. Flex Sensor");
        ecranOLED.println("3. Servo Motor");
        ecranOLED.display(); 
        break;
      
      case 1 :
          
      // ecranOLED.setTextColor(SSD1306_BLACK, SSD1306_WHITE); 
        ecranOLED.println("1. Graphite Sensor");         
        ecranOLED.setTextColor(SSD1306_BLACK, SSD1306_WHITE);         
        ecranOLED.println("2. Flex Sensor");
        ecranOLED.setTextColor(SSD1306_WHITE,SSD1306_BLACK);
        ecranOLED.println("3. Servo Motor");
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
  delay(50);
}


void doEncoder() {
  if (digitalRead(encoder0PinA)==HIGH && digitalRead(encoder0PinB)==HIGH) {
    encoder0Pos++;
  } 
  else if (digitalRead(encoder0PinA)==HIGH && digitalRead(encoder0PinB)==LOW) {
    encoder0Pos--;   
  }
  encoder0Pos %= 3;

  //Serial.println (encoder0PinA, DEC);  //Angle = (360 / Encoder_Resolution) * encoder0Pos
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

