//
// --------------  "01234567890123456789"
char this_file[] = "Tilt_Temp_Tx" ;
char descp[] =     "ESP32 TTGO LoRa" ;
char ver[] =       "ver 1.1  12/31/22 " ;
char ver2[] =      "Tilt + 2 Temps" ;
char ver3[] =      "8 samples/50 ms loop" ;

#include <Adafruit_MPU6050.h>

//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>

//Libraries for DS18B20 sensor
#include <OneWire.h>
byte temp1 = 13 ;
byte temp2 = 25 ;

byte button = 17 ; // pushbutton I/O pin

//Libraries for OLED Display
#include <Wire.h> // uncomment if not included elsewhere
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
 
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

Adafruit_MPU6050 mpu;

Adafruit_SSD1306 display(128, 64, &Wire, OLED_RST);
// Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);

// const int MPU_addr=0x68; // Typical Address
const int MPU_addr=0x69; // Moved to allow other device at 0x68
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
// float x;
// float y;
float z;

int xsamp[26] ;
int ysamp[26] ;
int xavI ;      // integer angle in hundredths
int yavI ;      // integer angle in hundredths
float xav ;
float yav ;
int xsum ;
int ysum ;
int xmax ;
int xmin ;
int ymax ;
int ymin ;

int Temp_1 ;
int Temp_2 ;
int Temp_3 ;

byte sync_byte ;
byte cmd_byte ;
byte counter ; // loop counter

byte i ;
String LoRaData ;
double LoRa_Tx_time ;
double Tilt_clk ; // max 10 minutes
bool Tilt_flag ;  // true=tilt mode, false=temps mode
double temp_clk ; // sets the trigger for sending temps
double ds_clk ; // allow 1 sec conversion time between starts

 
 /* &&&&&&&&&&&&&&&&& LoRa receive and transmit functions &&&&&&&&&&&&&&&&&
  // go to https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md
  // for the full API commands list
  
  LoRa.setPins(ss, reset, dio0);
  // Must be called before LoRa.begin().
  // ss - new slave select pin to use, defaults to 10
  // reset - new reset pin to use, defaults to 9
  // dio0 - new DIO0 pin to use, defaults to 2. Must be interrupt capable via 
  // attachInterrupt(...).

  LoRa.beginPacket();
  LoRa.beginPacket(implicitHeader);
  // implicitHeader - (optional) true enables implicit header mode, false 
  // enables explicit header mode (default)
  // Returns 1 if radio is ready to transmit, 0 if busy or on failure.
  
  LoRa.write(byte);
  LoRa.write(buffer, length);
  // byte - single byte to write to packet
  // or
  // buffer - data to write to packet
  // length - size of data to write
  // can contain up to 255 bytes.
  // Returns the number of bytes written.
  // Note: Other Arduino Print API's can also be used to write data into the packet

  LoRa.endPacket();
  LoRa.endPacket(async);
  // async - (optional) true enables non-blocking mode, false waits for transmission
  // to be completed (default)
  // Returns 1 on success, 0 on failure.
  
  LoRa.setSpreadingFactor(spreadingFactor);
  // defaults to 7. Supported values are between 6 and 12. If a spreading 
  // factor of 6 is set, implicit header mode must be used to transmit 
  // and receive packets.
  
  // the TOA below is highly dependent on message length, as wel as SF
  // do a loop timer test to determine your specific TOA
  // these times are approximate upper limits for 50-byte messages
  //		Data Rate	Range	Time on Air
  // SF7	5470 bps	2 km	 113 ms
  // SF8	3125 bps	4 km	 205 ms
  // SF9	1760 bps	6 km	 369 ms
  // SF10	980 bps		8 km	 698 ms
  // SF11	440 bps		11 km	1478 ms
  // SF12	290 bps		14 km	2629 ms
  
  LoRa.setCodingRate4(codingRateDenominator);
  // denominator of the coding rate, defaults to 5. Supported values are 
  // between 5 and 8, these correspond to coding rates 
  // of 4/5 and 4/8. The coding rate numerator is fixed at 4.
  
  LoRa.setSignalBandwidth(signalBandwidth);
  // Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 
  // 41.7E3, 62.5E3, 125E3, 250E3, and 500E3.
  
  LoRa.setSyncWord(syncWord);
  // byte value to use as the sync word, defaults to 0x12
  
// Pick one ******************************
  // LoRa.enableCrc();
  // LoRa.disableCrc();
  //   Enable or disable CRC usage, by default a CRC is not used.

  int packetSize = LoRa.parsePacket();
  int packetSize = LoRa.parsePacket(size);
  // Check if a packet has been received.
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

/* ########################## OLED Commands ##################################

  display.setRotation(x)
  // x = 0,1,2,3 for 0, 90, 180, 270 degrees of rotation
  
*/ // ########################################################################

//
// =======================================================================
void setup() {
	
	pinMode(button, INPUT_PULLUP);
	
	Serial.begin(9600);
	// while (!Serial);
	delay(100) ;

	Serial.println() ;
	Serial.println(this_file) ; //file name
	Serial.println(descp) ;     //description
	Serial.println(ver) ;       //version and date
	Serial.println(ver2) ;      //version and date
	Serial.println(ver3) ;      //version and date
	Serial.println() ;
 
	Wire.begin(OLED_SDA, OLED_SCL);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;) ; // Don't proceed, loop forever
  }
  
	display.display();
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setRotation(0);

	display.clearDisplay();
	display.setCursor(0,0);
	display.print(this_file);
	display.setCursor(0,15);
	display.print(descp);
	display.setCursor(0,30);
	display.print(ver);
	display.setCursor(0,40); 
	display.print(ver2);  
	display.display();

	delay(3000) ; // reading time  

	Serial.println("MPU6050 OLED demo");

	if (!mpu.begin()) {
		Serial.println("Sensor init failed");
		while (1)yield() ; // halt here
	}
	Serial.println("Found a MPU-6050 sensor");

	//SPI LoRa pins
	SPI.begin(SCK, MISO, MOSI, SS);
	//setup LoRa transceiver module
	LoRa.setPins(SS, RST, DIO0);

	if (!LoRa.begin(BAND)) {
		Serial.println("Starting LoRa failed!");
		while (1);
	}

	display.clearDisplay();
	Serial.println("LoRa Initializing OK!");
	display.setCursor(0,10);
	display.print("LoRa Initializing OK!");
	display.display();
	delay(2000);
  
	Tilt_flag = false ;
	LoRa.setSpreadingFactor(12); // temp mode

	display.setTextSize(3);
	LoRa_Tx_time = millis() ; // start the clock
	temp_clk = millis() ; // start the clock (~5 sec)
	
	readtemps() ; // start the first conversion
	ds_clk = millis() ; // start the 1 sec conversion time
  
}

// ============================ LOOP() ====================================
void loop() {
int packetSize ;

	sync_byte = 0 ;
	cmd_byte = 0 ;
	if(get_key()) change_mode() ; // change mode?
	if(Tilt_flag == false) {      // temp mode
		Temp() ; // read and send temperatures/door status
		//try to parse packet, look for mode switch command
		packetSize = LoRa.parsePacket();
		if (packetSize) { // packet received
		Serial.print("Command packetSize = ") ;
		Serial.println(packetSize) ;
			if (packetSize == 2) { //
				//read packet
				sync_byte = LoRa.read();  // sync byte
				cmd_byte = LoRa.read() ;  // command byte
				Serial.print("Sync byte = ") ;
				Serial.println(sync_byte, HEX) ;
				Serial.print("Cmd byte = ") ;
				Serial.println(cmd_byte, HEX) ;
				if((sync_byte == 0xAB) && (cmd_byte == 0x05)) {
					change_mode() ; // go to tilt mode
				}		
			} 
		}
	} else { // tilt mode
	// you can only leave the tilt mode via pushbutton or
	// when the clock runs out.
		// auto revert to temp mode, 10 minutes
		if(((millis() - Tilt_clk) < 600000)) { 
			Tilt() ;
		} else {
			LoRa.parsePacket(); // dump leftover packets
			change_mode() ; // go to temp mode
		}
	}
	
} // ========================= end of loop =================================

// --------------------------------------------------------------------------

bool get_key() { // button = I/O pin 17

bool key_flag = false ;
// most keypad functions only activate when the key is released.
// this routine activates once the debounce time has been met.
// the reason is that there are LoRa transmit times that can
// be longer than 1 second, so the pushbutton needs to be held
// down until the activation is apparent. The debounce hopefully
// will prevent a re-dection upon release of the button.
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

void change_mode() {
	
	Serial.println("Change Mode");
	
//setup LoRa transceiver module
	LoRa.setPins(SS, RST, DIO0);
	if(!LoRa.begin(BAND)) {
		Serial.println("Starting LoRa failed!");
		while (1); // halt here
	}
	if(Tilt_flag) { // tilt mode
	  LoRa.setSpreadingFactor(12); // temp mode	
	  Tilt_flag = false ;
	  Serial.println("Starting LoRa SF12 OK");
	} else {
	  LoRa.setSpreadingFactor(9); // tilt mode
	  Tilt_flag = true ;
	  Tilt_clk = millis() ; // auto revert to temp mode
	  Serial.println("Starting LoRa SF-9 OK");
	}
	
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
	delay(5000) ; // reading time
	
} // ---------------- end of change_mode() -------------------

// --------------------------- Tilt() ---------------------------
void Tilt() { // get the tilt x,y (left/right, up/down) angle data 
  
    Wire.beginTransmission(MPU_addr);
	Wire.write(0x3B);
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_addr,14,true);
	AcX=Wire.read()<<8|Wire.read();
	AcY=Wire.read()<<8|Wire.read();
	AcZ=Wire.read()<<8|Wire.read();
	int xAng = map(AcX,minVal,maxVal,-90,90);
	int yAng = map(AcY,minVal,maxVal,-90,90);
	int zAng = map(AcZ,minVal,maxVal,-90,90);

int	x = 5729.6 * (atan2(-yAng, -zAng)+PI);
int	y = 5730 * (atan2(-xAng, -zAng)+PI);	 
//	x = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
//	y = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
	// z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
	 
//	Serial.print("X= ");
	if(x > 27000) x = x - 36000 ; // hundredths
	rollx(x) ;
		
//	Serial.print("Y= ");
	if(y > 27000) y = y - 36000 ; // hundredths
	rolly(y) ;
	
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

  if(LoRa.beginPacket() == 1) {
//  Loop time w/o Tx is 27 ms
//	Serial.print("MS Margin = ") ;
//	Serial.println(200-(millis()-LoRa_Tx_time)) ;
	  if(((millis() - LoRa_Tx_time) > 200)) {
		// Tx ~5 per sec, ~85 ms quiet time
		// Tx time is 31 ms at SF7 (default)
	    // Tx time is 104 ms at SF9
		LoRa_Tx_time = millis() ;   // restart Tx clock
		LoRa.write(lowByte(xavI)) ; // byte
		LoRa.write(highByte(xavI)); // byte
		LoRa.write(lowByte(yavI)) ; // byte
		LoRa.write(highByte(yavI)); // byte
		LoRa.endPacket() ; // /Tx the data
//		Serial.print("On air time = ") ;
//		Serial.println(millis()-LoRa_Tx_time) ;
//		Serial.println(" ----------------------- ") ;
	  }
  }
} // --------------------------- end of Tilt() ---------------------------

void rollx(int a) {
	for(i=16;i>0; i --) { // i max is 26
		xsamp[i] = xsamp[i - 1] ;
	}
	xsamp[0] = a ;
	xsum = 0 ;
//	xmax = 0 ;
//	xmin = 45 ;
	for(i=0;i<8; i ++) {
//		if(abs(xsamp[i]) > xmax) xmax = abs(xsamp[i]) ;
//		if(abs(xsamp[i]) < xmin) xmin = abs(xsamp[i]) ;
		xsum = xsum + xsamp[i] ;
	}
	xavI = xsum / 8 ; // divide by 8
	xav = xavI / 100.0 ;
}

void rolly(int a) {
	for(i=16;i>0; i --) {  // i max is 26
		ysamp[i] = ysamp[i - 1] ;
	}
	ysamp[0] = a ;
	ysum = 0 ;
//	ymax = 0 ;
//	ymin = 45 ;
	for(i=0;i<8;i ++) {
//		if(abs(ysamp[i]) > ymax) ymax = abs(ysamp[i]) ;
//		if(abs(ysamp[i]) < ymin) ymin = abs(ysamp[i]) ;
		ysum = ysum + ysamp[i] ;
	}
	yavI = ysum / 8 ; // divide by 8
	yav = yavI / 100.0 ;
}

// -------------------------- Temp() -------------------------------
void Temp() { // read and send two temperatures
	
	// time for DS18B20 conversion
	if((millis() - ds_clk) > 1000) { // allow 1 second
		readtemps() ;
		ds_clk = millis() ;
	}
	if((millis() - temp_clk) > 10000) { // send temps ~10 secs
		temp_clk = millis() ; // reset the clock
		Serial.print("Sending packet: ");
		Serial.print(counter);
		Serial.print(":  ");
		Serial.print(Temp_1 / 100.0) ;
		Serial.print("  ");
		Serial.println(Temp_2 / 100.0) ;
	//	Serial.println(Temp_3 / 100.0) ;
		
	//Send LoRa packet to receiver
		LoRa.beginPacket();
		LoRa.write(0x69) ; // user sync byte
		LoRa.write(lowByte(counter)) ;
		LoRa.write(lowByte(Temp_1));
		LoRa.write(highByte(Temp_1));
		LoRa.write(lowByte(Temp_2));
		LoRa.write(highByte(Temp_2));
		LoRa.endPacket();

		display.clearDisplay();
		display.setTextSize(1);
		display.setCursor(0,0);
		display.print("LORA SENDER: ");
		display.println(counter) ;
		display.setTextSize(3);
		display.setCursor(0,15);
		display.print(" ") ;
		if(!(Temp_1<0)) {
			if(Temp_1<10000) display.print(" ") ;
			if(Temp_1<1000) display.print(" ") ;
		} else {
			if(abs(Temp_1)<1000) display.print(" ") ;
		}
		display.print(Temp_1/100.0, 1);   
		display.setCursor(0,40);
		display.print(" ") ;
		if(!(Temp_2<0)) {
			if(Temp_2<10000) display.print(" ") ;
			if(Temp_2<1000) display.print(" ") ;
		} else {
			if(abs(Temp_2)<1000) display.print(" ") ;
		}
		display.print(Temp_2/100.0, 1); 
		display.display();

		counter++;
		if(counter > 99) counter = 0 ;
	}
	
} // -------------------- end of temp() ------------------------------

// ------------------ readtemps() -------------------------------------
void readtemps() {
	
// one sensor per I/O line, simple OneWire call
	Temp_1 = dallas(temp1) ; 
	Temp_2 = dallas(temp2) ; 
//	Temp_3 = dallas(21) ; 

} // ----------------- end of readtemps() -------------------

// --------------------- quick dallas() -------------------------------
// DS18B20 sensor chip driver code
// the standard library takes several seconds, this takes usecs.
// the logic is (1) read the chip (2) start the conversion.
// a one-second delay between calls is required to allow the
// conversion process to complete so that the next read is good.
// limited to only one chip per I/O pin

int dallas(byte x) {
	
OneWire ds(x) ;
int data0 ;
int data1 ;
int result ;

	ds.reset() ;
	ds.write(0xCC) ;  // skip command
	ds.write(0xBE) ;  // Read 1st 2 bytes of Scratchpad
	data0 = ds.read() ;
	data1 = ds.read() ;
	result = (data1 << 8) | data0 ;
//	Serial.print("===== ") ;
//	Serial.print(result, HEX) ;
//	Serial.println(" ======") ;
	if(result > 4095) result = (result - 0x0ffff) -1 ;
	result = (((result * 90)+0x04) >> 3 ) + 3200 ;  // F x 100
	ds.reset() ;
	ds.write(0xCC) ;   // skip command
	ds.write(0x44,1) ; // start conversion
	return result ;
	
} // ----------------- end of dallas() -------------------
