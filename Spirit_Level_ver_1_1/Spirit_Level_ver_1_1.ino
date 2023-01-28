/*
 * Arduino based digital spirit level Version 1:1 by Bob Syers - 2020.
 * Based on original code by DroneBot workshop (dronebotworkshop.com) and Paul McWhorter (toptechboy.com)
 * With addition improvements!
 * Please feel free to use this code, but if you do, give me a mention, thanks!
 */


// Include libraries for I2C, liquid crystal display, math and EEPROM.
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <EEPROM.h>

//Define LCD address and configuration. If 0x27 doesn't work, 0x3F might.  If using a 20 x 4 display, change 16 to 20 and 2 to 4!
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define switchPin 2     //The Calibration/Buzzer pin!
#define LED_2 3         //LED showing -2 degrees
#define LED_1 4         //LED showing -1 degrees
#define LED_0 5         //LED showing level
#define LED_P1 6        //LED showing +1 degree
#define LED_P2 7        //LED showing +2 degrees
#define BUZZER 8        //Buzzer
#define HORIZONTAL 0    // Constant for horizontal
#define VERTICAL 1      //Constant for vertical


//////////////////////////////// 
//Variables Defenition Section//
////////////////////////////////

float gyro_x, gyro_y, gyro_z;                 //Raw gyro values from MPU6050
float gyro_x_cal, gyro_y_cal, gyro_z_cal = 0; //Stores the offsets for the gyro
float acc_x, acc_y, acc_z;                    //Raw acceleration values from MPU6050
float acc_x_cal, acc_y_cal, acc_z_cal = 0;    //Stores the offsets for the accelerometer
float thetaM;                                 //theta and thetaM relate to the roll angle
float theta;
float phiM;                                   //phi and phiM relate to the pitch angle
float phi; 
int LEDCount = 0;                             //When calibrating, allows easy iteration of LED's in setup loop
long loop_timer;                              //Timer for main loop to ensure it doesn't run too fast or too slow
int temp;                                     //Temperature gained from the MPU6050
int displaycount = 0;                         //Each time main loop executed, displaycount increments.  Only on every 100 counts does the display update
float dt;                                     //Time constant - difference between old and new time (for angle calculations)
unsigned long millisOld;                      //Variable to store old time
boolean orientation;                          //Either HORIZONTAL or VERTICAL dependant on orientation of MPU6050 module
int eeprom_address = 0;                       //To keep track of current eeprom address
int horizonalCalibration = 0;                 //0 if not calibrated, 255 if calibrated
int verticalCalibration = 0;                  //0 if not calibrated, 255 if calibrated
boolean horizonalCalibrationNotice = 0;       //On first iteration of loop allows display of 'Not calibrated' notice if not calibrated!
boolean verticalCalibrationNotice = 0;        //On first iteration of loop allows display of 'Not calibrated' notice if not calibrated!
boolean playBuzzer = 0;

//////////////////////
//Main setup routine//
/////////////////////

void setup() 
{

  //Assigning all the pins as required.
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_0, OUTPUT);
  pinMode(LED_P1, OUTPUT);
  pinMode(LED_P2, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  
  // Start Serial Monitor                                                 
  Serial.begin(115200);

  //Start I2C
  Wire.begin();
  
  //Start LCD
  lcd.begin();

  //Just so they know, we will tell them!
  lcd.print("Setting up"); 
    
   //Setup the registers of the MPU-6050                                                    
  setup_mpu_6050_registers(); 

  delay(500);   //Without this, would never see the 'Setting up' notice!

  // Init Timer 
  loop_timer = micros();

  //If the switchPin is low (Pressed), the calibration mode starts.  This allows 5 seconds to settle, then calibrates current orientation to 0.
  if(digitalRead(switchPin)==0)
  {
    delay(500);
    lcd.clear();
    lcd.print("Calibrating in ");
    for(int i = 5; i > 0; i--)
    {
      lcd.setCursor(15,0);
      lcd.print(i);
      LEDCount = i + 2;
      digitalWrite(LEDCount, HIGH);
      delay(1000);
      digitalWrite(LEDCount, LOW);
    }
    lcd.setCursor(11,0);
    lcd.print("-----");
    delay(500);
    
    read_mpu_6050_data();         //To determine orientation, we first need data from the MPU6050
    
    if(acc_x < acc_z)             //If Horizontal, acc_x will be less than acc_z, if vertical, it will be reversed (due to the effect of gravity
    {
      lcd.setCursor(0,1);
      lcd.print("Horizontal!");
      orientation = HORIZONTAL;
      //Read the raw acc and gyro data from the MPU-6050 1000 times                                          
      for (int cal_int = 0; cal_int < 1000 ; cal_int ++)
      {                  
        read_mpu_6050_data(); 
        //Add the gyro x offset to the gyro_x_cal variable                                            
        gyro_x_cal += gyro_x;
        //Add the gyro y offset to the gyro_y_cal variable                                              
        gyro_y_cal += gyro_y; 
        //Add the gyro z offset to the gyro_z_cal variable                                             
        gyro_z_cal += gyro_z; 
        //Add the acc x offset to the acc_x_cal variable                                            
        acc_x_cal += acc_x;
        //Add the acc y offset to the acc_y_cal variable                                            
        acc_y_cal += acc_y;                                                          
      }
    
      // Divide all results by 1000 to get average offset
      gyro_x_cal /= 1000.0;                                                 
      gyro_y_cal /= 1000.0;                                                 
      gyro_z_cal /= 1000.0;
      acc_x_cal /= 1000.0;
      acc_y_cal /= 1000.0;

      horizonalCalibration = 255;
      eeprom_address = 0;
      EEPROM.put(eeprom_address, horizonalCalibration);
      eeprom_address += sizeof(int);
      EEPROM.put(eeprom_address, gyro_x_cal);
      eeprom_address += sizeof(float);
      EEPROM.put(eeprom_address, gyro_y_cal);
      eeprom_address += sizeof(float);
      EEPROM.put(eeprom_address, gyro_z_cal);
      eeprom_address += sizeof(float);
      EEPROM.put(eeprom_address, acc_x_cal);
      eeprom_address += sizeof(float);
      EEPROM.put(eeprom_address, acc_y_cal);
      eeprom_address += sizeof(float);
      //Note we are not storing an offset for acc_z, due to gravity!

      delay(500);
    }
    else
    {
      lcd.setCursor(0,1);
      lcd.print("Vertical!");
      orientation = VERTICAL;
      //Read the raw acc and gyro data from the MPU-6050 1000 times                                          
      for (int cal_int = 0; cal_int < 1000 ; cal_int ++)
      {                  
        read_mpu_6050_data(); 
        //Add the gyro x offset to the gyro_x_cal variable                                            
        gyro_x_cal += gyro_x;
        //Add the gyro y offset to the gyro_y_cal variable                                              
        gyro_y_cal += gyro_y; 
        //Add the gyro z offset to the gyro_z_cal variable                                             
        gyro_z_cal += gyro_z; 
        //Add the acc x offset to the acc_x_cal variable                                            
        acc_y_cal += acc_y;
        //Add the acc y offset to the acc_y_cal variable                                            
        acc_z_cal += acc_z;                                                         
      }
    
      // Divide all results by 1000 to get average offset
      gyro_x_cal /= 1000.0;                                                 
      gyro_y_cal /= 1000.0;                                                 
      gyro_z_cal /= 1000.0;
      acc_y_cal /= 1000.0;
      acc_z_cal /= 1000.0;

      verticalCalibration = 255;
      eeprom_address = 24;
      EEPROM.put(eeprom_address, verticalCalibration);
      eeprom_address += sizeof(int);
      EEPROM.put(eeprom_address, gyro_x_cal);
      eeprom_address += sizeof(float);
      EEPROM.put(eeprom_address, gyro_y_cal);
      eeprom_address += sizeof(float);
      EEPROM.put(eeprom_address, gyro_z_cal);
      eeprom_address += sizeof(float);
      EEPROM.put(eeprom_address, acc_y_cal);
      eeprom_address += sizeof(float);
      EEPROM.put(eeprom_address, acc_z_cal);
      //Note we are not storing an offset for acc_x, due to gravity!
      
      delay(500);
    }
  }
  //We are just about to go into main loop, so we setup display ready to show data
  setLcdBaseline();                                                
}

/////////////////////
//Main program loop//
/////////////////////


void loop()
{

  if(digitalRead(switchPin) == 0)     //Check to see if switch is activated (low).  If so, toggles beeper on/off
  {
    playBuzzer = !playBuzzer;
    delay(200);                       //Delay added to give enough time for user to respond
  }
  // Get data from MPU-6050
  read_mpu_6050_data();
  //acc_x and acc_z data used to determine correct orientation of module
  if(acc_x < acc_z)
  {
    orientation = HORIZONTAL;
  }
  else
  {
    orientation = VERTICAL;
  }

  if(orientation == HORIZONTAL)   //If horizontal, eeprom calibration data is loaded (if not present, will display 'Not calibrated')
  {
    if(horizonalCalibrationNotice == 0)   //This section is only run once through then ignored
    {
      eeprom_address = 0;                 //Set EEPROM address to 0 (start of horizontal calibration offsets
      if(EEPROM.get(eeprom_address, horizonalCalibration) == 255)   //Data only accessed if it actually exists!
      {
         eeprom_address += sizeof(int);
         EEPROM.get(eeprom_address, gyro_x_cal);
         eeprom_address += sizeof(float);
         EEPROM.get(eeprom_address, gyro_y_cal);
         eeprom_address += sizeof(float);
         EEPROM.get(eeprom_address, gyro_z_cal);
         eeprom_address += sizeof(float);
         EEPROM.get(eeprom_address, acc_x_cal);
         eeprom_address += sizeof(float);
         EEPROM.get(eeprom_address, acc_y_cal);
         eeprom_address += sizeof(float);
      }
      else   //If no calibration data, it tells you so!
      {
        lcd.clear();
        lcd.print("Not calibrated!");
        delay(1000);
        setLcdBaseline();
      }
      horizonalCalibrationNotice = 1;
    }
    
    //Subtract the offset values from the raw gyro values
    gyro_x -= gyro_x_cal;                                                
    gyro_y -= gyro_y_cal;                                                
    gyro_z -= gyro_z_cal; 
    acc_x -= acc_x_cal;
    acc_y -= acc_y_cal;
  
    /*
     * The next few lines process the raw data to change it into angles that can be output to the LCD and LED's.
     * The value of 4096, which the acceleration data is divided by is taken from the MPU6050 datasheet and is based on sample rate.
     * The value of 9.8 is gravity
     * The atan2 function is from the math module and is used to calculate the angles from the given data
     */
    thetaM =-atan2((acc_x/4096.0)/9.8 , (acc_z/4096.0)/9.8)/2/3.141592656 * 360;  //Raw data
    phiM =-atan2((acc_y/4096.0)/9.8 , (acc_z/4096.0)/9.8)/2/3.141592656 * 360;  //Raw data
  
    dt=(millis()-millisOld)/1000.;
    millisOld=millis();
    /*
     * This section uses the gyro data to make the system more responsive
     * the value of 65.5, which the gyro data is divided by is taken from the MPU6050 datasheet and is based on the sample rate
     */
    theta=(theta+(gyro_y/65.5)*dt)*.96 + thetaM*.04;  //Low pass filter
    phi=(phi+(gyro_x/65.5)*dt)*.96 + phiM*.04;  //Low pass filter
  
    
  }
  else      //If vertical, eeprom calibration data is loaded (if not present, will display 'Not calibrated')
  {
    if(verticalCalibrationNotice == 0)    //This section is only run once through then ignored
    {
      eeprom_address = 24;                //Set EEPROM address to 24 (start of vertical calibration offsets
      if(EEPROM.get(eeprom_address, verticalCalibration) == 255)      //Data only accessed if it actually exists!
      {
         eeprom_address += sizeof(int);
         EEPROM.get(eeprom_address, gyro_x_cal);
         eeprom_address += sizeof(float);
         EEPROM.get(eeprom_address, gyro_y_cal);
         eeprom_address += sizeof(float);
         EEPROM.get(eeprom_address, gyro_z_cal);
         eeprom_address += sizeof(float);
         EEPROM.get(eeprom_address, acc_y_cal);
         eeprom_address += sizeof(float);
         EEPROM.get(eeprom_address, acc_z_cal);
         eeprom_address += sizeof(float);
      }
      else    //If no calibration data, it tells you so!
      {
        lcd.clear();
        lcd.print("Not calibrated!");
        delay(1000);
        setLcdBaseline();
      }
      verticalCalibrationNotice = 1;
    }
    
      //Subtract the offset values from the raw gyro values
    gyro_x -= gyro_x_cal;                                                
    gyro_y -= gyro_y_cal;                                                
    gyro_z -= gyro_z_cal; 
    acc_y -= acc_y_cal;
    acc_z -= acc_z_cal;


    /*
     * The next few lines process the raw data to change it into angles that can be output to the LCD and LED's.
     * The value of 4096, which the acceleration data is divided by is taken from the MPU6050 datasheet and is based on sample rate.
     * The value of 9.8 is gravity
     * The atan2 function is from the math module and is used to calculate the angles from the given data
     */
     
    thetaM =-atan2((acc_z/4096.0)/9.8 , (acc_x/4096.0)/9.8)/2/3.141592656 * 360;  //Raw data
    phiM =-atan2((acc_y/4096.0)/9.8 , (acc_x/4096.0)/9.8)/2/3.141592656 * 360;  //Raw data
  
    dt=(millis()-millisOld)/1000.;
    millisOld=millis();
    /*
     * This section uses the gyro data to make the system more responsive
     * the value of 65.5, which the gyro data is divided by is taken from the MPU6050 datasheet and is based on the sample rate
     */
    theta=(theta+(gyro_y/65.5)*dt)*.96 + thetaM*.04;  //Low pass filter
    phi=(phi+(gyro_z/65.5)*dt)*.96 + phiM*.04;  //Low pass filter
  }

  /*
   * Regardless of orientation, this section updates the LCD, LED's and buzzer
   */
  // Increment the display counter
  displaycount = displaycount +1;
  
  if (displaycount > 100) 
  {
    // Check Angle for Level LEDs
    if (phi < -2.01) 
    {
      // Turn on Level LED
      digitalWrite(LED_2, HIGH);
      digitalWrite(LED_1, LOW);
      digitalWrite(LED_0, LOW);
      digitalWrite(LED_P1, LOW);
      digitalWrite(LED_P2, LOW);
      if(playBuzzer)
      {
        tone(8,200,200);
      }
      
    } 
    else if ((phi > -2.00) && (phi < -1.01)) 
    {
      // Turn on Level LED
      digitalWrite(LED_2, LOW);
      digitalWrite(LED_1, HIGH);
      digitalWrite(LED_0, LOW);
      digitalWrite(LED_P1, LOW);
      digitalWrite(LED_P2, LOW);
      if(playBuzzer)
      {
        tone(8,1000,400);
      }
      
    } 
    else if ((phi < 1.0) && (phi > -1.0)) 
    {
      // Turn on Level LED
      digitalWrite(LED_2, LOW);
      digitalWrite(LED_1, LOW);
      digitalWrite(LED_0, HIGH);
      digitalWrite(LED_P1, LOW);
      digitalWrite(LED_P2, LOW);
      if(playBuzzer)
      {
        tone(8,2000,600);
      }
      
    } 
    else if ((phi > 1.01) && (phi < 2.00)) 
    {
      // Turn on Level LED
      digitalWrite(LED_2, LOW);
      digitalWrite(LED_1, LOW);
      digitalWrite(LED_0, LOW);
      digitalWrite(LED_P1, HIGH);
      digitalWrite(LED_P2, LOW);
      if(playBuzzer)
      {
        tone(8,1000,400);
      }
      
    } 
    else if (phi > 2.01) 
    {
      // Turn on Level LED
      digitalWrite(LED_2, LOW);
      digitalWrite(LED_1, LOW);
      digitalWrite(LED_0, LOW);
      digitalWrite(LED_P1, LOW);
      digitalWrite(LED_P2, HIGH);
      if(playBuzzer)
      {
        tone(8,200,200);
      }
    }

  //Before printing the angle to the display, it makes any negative numbers positive, so it always shows 0 degrees +.
  //If you want to show negative angles, omit this section
  if(phi<0)
  {
    phi = phi - (2*phi);
  }
  if(theta<0)
  {
    theta = theta - (2*theta);
  }

  //End of section

  lcd.setCursor(9,0);
  lcd.print(phi,1);
  lcd.print("   ");
  lcd.setCursor(9,1);
  lcd.print(theta,1);
  lcd.print("   ");

  Serial.print("Pitch = ");
  Serial.print(phi);
  Serial.print(", Roll = ");
  Serial.println(theta);
    
    
  displaycount = 0;
  
  }
  

 while(micros() - loop_timer < 4000);  //4000
 //Reset the loop timer                                
 loop_timer = micros();
 //Back to the top of the loop to start again!
}

//////////////////////
//Subroutine section//
//////////////////////

// This routine just clears the lcd and prints Angle and Roll ready to show values.
void setLcdBaseline()
{
  lcd.clear();
  lcd.print("Angle = ");
  lcd.setCursor(0,1);
  lcd.print("Roll = ");
}


//Required for setting up the MPU6050 for first use.
void setup_mpu_6050_registers(){

  //Activate the MPU-6050
  //Serial.println("Setting up MPU6050 - Wake up");
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
                                              
  //Configure the accelerometer (+/-8g)
  //Serial.println("Setting up MPU6050 - Wake up - Done");
  //Serial.println("Setting up MPU6050 - setup acc");
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68); 
  //Send the requested starting register                                       
  Wire.write(0x1C);   
  //Set the requested starting register                                                 
  Wire.write(0x10); 
  //End the transmission                                                   
  Wire.endTransmission(); 
    //Serial.println("Setting up MPU6050 - setup acc - done");                                          
  //Configure the gyro (500dps full scale)
  //Serial.println("Setting up MPU6050 - setup gyro");
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register                                        
  Wire.write(0x1B);
  //Set the requested starting register                                                    
  Wire.write(0x08); 
  //End the transmission                                                  
  Wire.endTransmission(); 
 //Serial.println("Setting up MPU6050 - setup gyro - done");                                             
}


//This routine is required to get the gyro and acceleration values from the MPU6050
void read_mpu_6050_data(){ 

  //Read the raw gyro and accelerometer data
  //Serial.println("Reading data - 1");
  //Start communicating with the MPU-6050                                          
  Wire.beginTransmission(0x68);  
  //Serial.println("Reading data - 2");
  //Send the requested starting register                                      
  Wire.write(0x3B);
  //Serial.println("Reading data - 3");
  //End the transmission                                                    
  Wire.endTransmission(); 
  //Serial.println("Reading data - 4");
  //Request 14 bytes from the MPU-6050                                  
  Wire.requestFrom(0x68,14); 
  //Serial.println("Reading data - 5");   
  //Wait until all the bytes are received                                       
  while(Wire.available() < 14);
  //Serial.println("Reading data - 6");
  //Following statements left shift 8 bits, then bitwise OR.  
  //Turns two 8-bit values into one 16-bit value                                       
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                  
  temp = Wire.read()<<8|Wire.read();                                   
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                 
}
