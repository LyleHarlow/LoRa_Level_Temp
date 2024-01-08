//
// --------------  "01234567890123456789"
char this_file[] = "Tilt_Temp_Tx";
char descp[] = "ESP32 TTGO LoRa";
char ver[] = "ver 1.1  12/31/22 ";
char ver2[] = "Tilt + 2 Temps";
char ver3[] = "8 samples/50 ms loop";

#include <Adafruit_MPU6050.h>

//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>

//Libraries for DS18B20 sensor
#include <OneWire.h>
byte temp1 = 25;
byte temp2 = 13;

byte button1 = 17;        // pushbutton I/O pin
byte button2 = 39;        // pushbutton I/O pin
byte button3 = 38;        // pushbutton I/O pin
byte button4 = 12;        // pushbutton I/O pin
byte IO_1 = 23;           //  I/O pin 1
byte IO_2 = 32;           //  I/O pin 2
byte IO_3 = 21;           //  I/O pin 3
byte LED_1 = 2;           //  LED_1
byte LED_2 = 36;          //  LED_2
byte TEMP3 = 33;          //  TEMP3
byte AMBLIGHTSENSE = 37;  //  AMBIENTLIGHTSENSE

//Libraries for OLED Display
#include <Wire.h>  // uncomment if not included elsewhere
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

#define BAND 915E6  //915E6 for North America

//OLED pins
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

Adafruit_MPU6050 mpu;

Adafruit_SSD1306 display(128, 64, &Wire, OLED_RST);
// Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);

const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

int minVal = 265;
int maxVal = 402;

// float x;
// float y;
float z;

int xsamp[26];
int ysamp[26];
int xavI;  // integer angle in hundredths
int yavI;  // integer angle in hundredths
float xav;
float yav;
int xsum;
int ysum;
int xmax;
int xmin;
int ymax;
int ymin;

int Temp_1;
int Temp_2;
int Temp_3;

byte sync_byte;
byte cmd_byte;
byte counter;  // loop counter

byte i;
String LoRaData;
double LoRa_Tx_time;
double Tilt_clk;  // max 10 minutes
bool Tilt_flag;   // true=tilt mode, false=temps mode
double temp_clk;  // sets the trigger for sending temps
double ds_clk;    // allow 1 sec conversion time between starts

//
// =======================================================================
void setup() {

  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(button4, INPUT_PULLUP);
  pinMode(IO_1, OUTPUT);
  pinMode(IO_2, OUTPUT);
  pinMode(IO_3, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);

  Serial.begin(9600);
  // while (!Serial);
  delay(100);

  Serial.println();
  Serial.println(this_file);  //file name
  Serial.println(descp);      //description
  Serial.println(ver);        //version and date
  Serial.println(ver2);       //version and date
  Serial.println(ver3);       //version and date
  Serial.println();

  Wire.begin(OLED_SDA, OLED_SCL);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }

  display.display();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(0);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(this_file);
  display.setCursor(0, 15);
  display.print(descp);
  display.setCursor(0, 30);
  display.print(ver);
  display.setCursor(0, 40);
  display.print(ver2);
  display.display();

  delay(3000);  // reading time

  // Serial.println("MPU6050 OLED demo");

  // if (!mpu.begin()) {
  // 	Serial.println("Sensor init failed");
  // 	while (1)yield() ; // halt here
  // }
  // Serial.println("Found a MPU-6050 sensor");

  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }

  display.clearDisplay();
  Serial.println("LoRa Initializing OK!");
  display.setCursor(0, 10);
  display.print("LoRa Initializing OK!");
  display.display();
  delay(2000);

  Tilt_flag = false;
  LoRa.setSpreadingFactor(12);  // temp mode

  display.setTextSize(3);
  LoRa_Tx_time = millis();  // start the clock
  temp_clk = millis();      // start the clock (~5 sec)

  ds_clk = millis();  // start the 1 sec conversion time
}

// ============================ LOOP() ====================================
void loop() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.print("Runing Loop");
  display.display();
  if (!digitalRead(button1)) {  // button1 down?
    delay(20);                  // start a clock
    if (!digitalRead(button1)) {
      delay(20);  // start another clock
      if (!digitalRead(button1)) {
        Serial.println("Button1 pushed");
        display.clearDisplay();
        display.setCursor(0, 10);
        display.println("Button1 pushed");
        display.print("IO_1 HIGH");        
        display.display();
        digitalWrite(LED_1, HIGH);
        delay(1000);
        digitalWrite(LED_1, LOW);
        display.display();
        delay(1000);
        display.clearDisplay();
      }
    }
  }
  display.clearDisplay();
  if (!digitalRead(button2)) {  // button1 down?
    delay(20);                  // start a clock
    if (!digitalRead(button2)) {
      delay(20);  // start another clock
      if (!digitalRead(button2)) {
        Serial.println("Button2 pushed");
        display.clearDisplay();
        display.setCursor(0, 10);
        display.println("Button2 pushed");
        display.print("IO_2 HIGH");        
        display.display();
        digitalWrite(IO_2, HIGH);
        delay(1000);
        digitalWrite(IO_2, LOW);
        
        display.display();
        delay(1000);
        display.clearDisplay();
      }
    }
  }
  display.clearDisplay();
  if (!digitalRead(button3)) {  // button1 down?
    delay(20);                  // start a clock
    if (!digitalRead(button3)) {
      delay(20);  // start another clock
      if (!digitalRead(button3)) {
        Serial.println("Button3 pushed");
        display.clearDisplay();
        display.setCursor(0, 10);
        display.println("Button3 pushed");
        display.print("IO_3 HIGH");        
        display.display();
        digitalWrite(IO_3, HIGH);
        delay(1000);
        digitalWrite(IO_3, LOW);
        display.display();
        delay(1000);
        display.clearDisplay();
      }
    }
  }
// Test Button 4 and TEMP3
  if (!digitalRead(button4)) {  // button1 down?
    delay(20);                  // start a clock
    if (!digitalRead(button4)) {
      delay(20);  // start another clock
      if (!digitalRead(button4)) {
        Serial.println("Button4 pushed");
        display.clearDisplay();
        display.setCursor(0, 10);
        display.println("Button4 pushed");
        display.print("TEMP3  ");
        display.print(analogRead(TEMP3));
        display.display();
        delay(1000);
        display.clearDisplay();
      }
    }
  }
// Test Button 4 and AMBIENTLIGHTSENSE
  // if (!digitalRead(button4)) {  // button1 down?
  //   delay(20);                  // start a clock
  //   if (!digitalRead(button4)) {
  //     delay(20);  // start another clock
  //     if (!digitalRead(button4)) {
  //       Serial.println("Button4 pushed");
  //       display.clearDisplay();
  //       display.setCursor(0, 10);
  //       display.println("Button4 pushed");
  //       display.print("AMBLIGHTSENSE  ");  
  //       display.print(analogRead(AMBLIGHTSENSE));
  //       display.display();
  //       delay(1000);
  //       display.clearDisplay();
  //     }
  //   }
  // }
 // Test to see if IO_1 works as an input
  // if (!digitalRead(IO_1)) {  // IO_1 LOW?
  //   delay(20);               // start a clock
  //   if (!digitalRead(IO_1)) {
  //     delay(20);  // start another clock
  //     if (!digitalRead(IO_1)) {
  //       Serial.println("IO_1 Low");
  //       display.clearDisplay();
  //       display.setCursor(0, 10);
  //       display.print("IO_1 Low");
  //       display.display();
  //       delay(1000);
  //       display.clearDisplay();
  //     }
  //   }
  // }

 // Test to see if IO_2 works as an input
  // if (!digitalRead(IO_2)) {  // IO_2 LOW?
  //   delay(20);               // start a clock
  //   if (!digitalRead(IO_2)) {
  //     delay(20);  // start another clock
  //     if (!digitalRead(IO_2)) {
  //       Serial.println("IO_2 Low");
  //       display.clearDisplay();
  //       display.setCursor(0, 10);
  //       display.print("IO_2 Low");
  //       display.print("LED_1 HIGH");
  //       display.display();
  //       digitalWrite(LED_1, HIGH);
  //       delay(1000);
  //       digitalWrite(LED_1, LOW);
  //       display.clearDisplay();
  //     }
  //   }
  // }

 // Test to see if IO_3 works as an input
  // if (!digitalRead(IO_3)) {  // IO_3 LOW?
  //   delay(20);               // start a clock
  //   if (!digitalRead(IO_3)) {
  //     delay(20);  // start another clock
  //     if (!digitalRead(IO_3)) {
  //       Serial.println("IO_3 Low");
  //       display.clearDisplay();
  //       display.setCursor(0, 10);
  //       display.print("IO_3 Low");
  //       display.print("LED_2 HIGH");        
  //       display.display();
  //       digitalWrite(LED_2, HIGH);
  //       delay(1000);
  //       digitalWrite(LED_2, LOW);
  //       display.clearDisplay();
  //     }
  //   }
  // }

  delay(200);
}
