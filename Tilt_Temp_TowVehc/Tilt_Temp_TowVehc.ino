//
// --------------  "01234567890123456789"
char this_file[] = "Tilt_Temp_TowVehc" ;
char descp[] =     "ESP32 TTGO LoRa OLED" ;
char ver[] =       "ver 1.1  12/31/22 " ;
char ver2[] =      "LORA/angles/temps  " ;
char ver3[] =      "discrete/door status" ;

//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

byte button = 17 ; // pushbutton I/O pin

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

#define BAND 915E6 //915E6 for North America

//OLED pins
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

bool Tilt_flag ;
double Tilt_clk ;
byte sync_byte ;
byte counter ;
int packetSize ;
unsigned int loop_Time ;

 /* &&&&&&&&&&&&&&&&&&&&&&& LoRa receive functions &&&&&&&&&&&&&&&&&&&&&&&&
  // go to https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md
  // for the full API commands list
  
  int packetSize = LoRa.parsePacket();
  int packetSize = LoRa.parsePacket(size);
  //  Check if a packet has been received.
  // size - (optional) if > 0 implicit header mode is enabled with 
  // the expected packet of size bytes, default mode is explicit header mode
  // Returns the packet size in bytes or 0 if no packet was received. 
  
  int rssi = LoRa.packetRssi();
  // Returns the averaged RSSI of the last received packet (dBm).

  float snr = LoRa.packetSnr();
  // Returns the estimated SNR of the received packet in dB.

  int rssi = LoRa.rssi();
  // Returns the current RSSI of the radio (dBm). RSSI can be read 
  // at any time (during packet reception or not)

  int availableBytes = LoRa.available()
  // Returns number of bytes available for reading.

  byte b = LoRa.peek();
  // Returns the next byte in the packet or -1 if no bytes are available.

  byte b = LoRa.read();
  // Returns the next byte in the packet or -1 if no bytes are available.
  // Note: Other Arduino Stream API's can also be used to read data from 
  // the packet

*/ // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void setup() { 

  pinMode(button, INPUT_PULLUP) ;

  Serial.begin(9600);
  // while (!Serial);
  delay(100) ;
  
  // show the file name and version
  Serial.println() ;
  Serial.println(this_file) ; //file name
  Serial.println(descp) ;     //description
  Serial.println(ver) ;       //version and date
  Serial.println(ver2) ;      //version and date
  Serial.println(ver3) ;      //version and date
  Serial.println() ;
  delay(3000) ;
  
  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  
  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("LORA RECEIVER ");
  display.display();
  
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  display.setCursor(0,15);
  display.println("LoRa Initializing OK!");
  display.setCursor(0,30);
  display.println("Ready for temps");
  display.display(); 

  LoRa.setSpreadingFactor(12);  // long range, 10 sec loop
  
  display.setTextSize(3);
}
// ========================== loop() ==========================
void loop() {

	if(get_key()) change_mode() ; // change mode?
	if(Tilt_flag == false) {      // temp mode
		Temps() ; // read and display temperatures/door status
	} else { // tilt mode, display the angles
	// you can only leave the tilt mode via pushbutton or
	// when the clock runs out.
		// auto revert to temp mode, 10 minutes
		if(((millis() - Tilt_clk) < 600000)) { 
			Angles() ;
		} else {
			change_mode() ; // go to temp mode
		}
	}
	
} // ========================= end of loop() ======================

// -------------------------- get_key() -------------------------------
// most keypad functions only activate when the key is released.
// this routine activates once the debounce time has been met.
// the reason is that there are LoRa transmit times that can
// be longer than 1 second, so the pushbutton needs to be held
// down until the activation is apparent. The debounce hopefully
// will prevent a re-dection upon release of the button.

bool get_key() { // button = I/O pin 17

bool key_flag = false ;

	// debounce 40 ms
	if(!digitalRead(button)) { // button down?
		delay(20) ;  // start a clock
		if(!digitalRead(button)) {
			delay(20) ;  // start another clock
			if(!digitalRead(button)) {
				key_flag = true ;
				Serial.println("Button push");
			}
		}
	}
	return key_flag ;
	
} // ---------------- end of get_key() ----------------------------

// --------------------- change_mode() ----------------------------
void change_mode() {
	
	Serial.println("Change Mode");
	
// switch the spreading factor and mode flag
// if the remote is in tilt/angle mode, it will remain in that
// mode until the tilt mode time expires, the change back
// to temperature mode cannot be commanded remotely
	if(Tilt_flag) { // tilt mode
		//restart LoRa transceiver module
		LoRa.setPins(SS, RST, DIO0);
		if(!LoRa.begin(BAND)) {
			Serial.println("Starting LoRa failed!");
			while (1); // halt here
		}
		LoRa.setSpreadingFactor(12); // temp mode	
		Serial.println("Starting LoRa SF12 OK");
		Tilt_flag = false ; // this starts temp mode
	} else {
	// command the remote to change to tilt mode
	// *********************
	// Send LoRa packet to receiver, still in SF12
	// repeat 3 times to skirt the remote Tx on-period
		LoRa.beginPacket();
		LoRa.write(0xAB) ; // sync_byte, embedded by user
		LoRa.write(0x05) ; // cmd_byte for the remote
		LoRa.endPacket();  // send the change mode command
		delay(400) ; 
		LoRa.beginPacket();
		LoRa.write(0xAB) ; // sync_byte, embedded by user
		LoRa.write(0x05) ; // cmd_byte for the remote
		LoRa.endPacket();  // send the change mode command
		delay(400) ;
		LoRa.beginPacket();
		LoRa.write(0xAB) ; // sync_byte, embedded by user
		LoRa.write(0x05) ; // cmd_byte for the remote
		LoRa.endPacket();  // send the change mode command
	// *******************
		//restart LoRa transceiver module
		LoRa.setPins(SS, RST, DIO0);
		if(!LoRa.begin(BAND)) {
			Serial.println("Starting LoRa failed!");
			while (1); // halt here
		}	
		LoRa.setSpreadingFactor(9); // go to tilt mode
		Serial.println("Starting LoRa SF-9 OK");
		Tilt_flag = true ; // this starts the tilt mode
		Tilt_clk = millis() ; // auto revert to temp mode clk
	}
/*	
// dump any packet
	while (LoRa.available()) {
		sync_byte = LoRa.read(); //
	}
*/	
// display the mode
	display.clearDisplay();
	display.setTextSize(3);
	display.setCursor(0,15);
	if(Tilt_flag) {
		display.print("TILT") ;
	} else {
		display.print("Temp") ;
	}
	display.display(); 
	delay(3000) ; // reading time
	
} // ---------------- end of change_mode() -------------------

// -------------------------- Angles() ------------------------------
void Angles() { // display trailer angles

float xav ;
float yav ;
int xavI ;
int yavI ;
int bL ;
int bH ;

  //try to parse packet
  packetSize = LoRa.parsePacket();
  Serial.print("Tilt packet size = ") ;
  Serial.println(packetSize) ;
  if (packetSize > 3) {
    //received a packet
    Serial.print("Received packet ");
    //read packet
      bL = LoRa.read();   // low byte, angle in hundredths
      bH = LoRa.read() ;  // high byte
	  xavI = bH<<8|bL ;
	  bL = LoRa.read();   // low byte, angle in hundredths
      bH = LoRa.read() ;  // high byte
	  yavI = bH<<8|bL ;

	if(xavI>32000) xavI = xavI - 65536 ;
	if(yavI>32000) yavI = yavI - 65536 ;
	xav = xavI / 100.0 ;
	yav = yavI / 100.0 ;
	
    //print RSSI of packet
    int rssi = LoRa.packetRssi();
    Serial.print(" with RSSI ");    
    Serial.println(rssi);

  // Display packet information
  display.clearDisplay();
  display.setCursor(0,4);
  if(!(yav < 0)) {
		display.print("Up ");
  } else {
		display.print("Dn ") ;
		yav = -yav ;
  } 
  if((abs(yav)<10)) display.print(" ");
  display.print(yav, 1);
  display.setCursor(0,40);
  if(!(xav < 0)) {
		display.print("R  ");
  } else {
		display.print("L  ") ;
		xav = -xav ;
  }
  if((abs(xav)<10)) display.print(" ");
  display.print(xav, 1);
  display.display();
  }
} // -------------------------- end of Angles() ----------------------

// ----------------------- Temps() ------------------------------------
void Temps() { // display the temperatures or door status

byte t1L ;
byte t1H ;
byte t2L ;
byte t2H ;
int T_1 ;
int T_2 ;

// flash the onboard LED every 5 seconds
	if(((millis() - loop_Time) > 5000)) {
		digitalWrite(2, 1) ;
		delay(200) ;
		digitalWrite(2, 0) ;
		loop_Time = millis() ;
	}

// parse packet
	packetSize = LoRa.parsePacket();
	if (packetSize) {
		Serial.print("Temp packet size = ");
		Serial.println(packetSize);
	//read packet
		sync_byte = LoRa.read(); // 
		if(sync_byte == 0x69) {  // user sync byte
			counter = LoRa.read();
			t1L = LoRa.read();
			t1H = LoRa.read();
			t2L = LoRa.read();
			t2H = LoRa.read();

			T_1 = t1H<<8 | t1L ;
			T_2 = t2H<<8 | t2L ;
			// fix the 4-byte integer sign bit problem
			if(T_1 > 20000) T_1 = T_1 - 65536 ;
			if(T_2 > 20000) T_2 = T_2 - 65536 ;

			//print RSSI of packet
			int rssi = LoRa.packetRssi();
			Serial.print(" with RSSI ");    
			Serial.println(rssi);

			if((T_1<12000) && (T_2<12000)) { // Less than +120
				if((T_1>-3000) && (T_2>-3000)) { // > -30
			// Display the packet message
					display.clearDisplay();
					display.setTextSize(1);
					display.setCursor(0,0);
					display.print("LORA Receiver: ");
					display.println(counter) ;
					display.setTextSize(3);
					display.setCursor(0,15);
					display.print(" ") ;
					if(!(T_1<0)) {
						if(T_1<10000) display.print(" ") ;
						if(T_1<1000) display.print(" ") ;
					} else {
						if(abs(T_1)<1000) display.print(" ") ;
					}
					display.print(T_1/100.0, 1);   
					display.setCursor(0,40);
					display.print(" ") ;
					if(!(T_2<0)) {
						if(T_2<10000) display.print(" ") ;
						if(T_2<1000) display.print(" ") ;
					} else {
						if(abs(T_2)<1000) display.print(" ") ;
					}
					display.print(T_2/100.0, 1); 
					display.display(); 
				}
			}
		} else {
		// garbled packet: dump any remaing bytes
			Serial.println("dumping");
			while (LoRa.available()) {
				sync_byte = LoRa.read();
				Serial.println("dumping loop");
			}
		}
	}
}
