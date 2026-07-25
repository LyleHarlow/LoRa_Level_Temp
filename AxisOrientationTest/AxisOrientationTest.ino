//      ******************************************************************
//      *                                                                *
//      *   MPU6050 axis-orientation test for the Airstream v2.1 board  *
//      *                                                                *
//      ******************************************************************

//
// The Airstream board's MPU6050 will be mounted VERTICALLY in the final
// install, so it's not obvious which raw accelerometer axis (and which
// sign) corresponds to real-world "nose up/down" (pitch, front-to-back
// leveling) vs. "left up/down" (roll, side-to-side leveling).
//
// Flash this to the Airstream board, mount it (or hold it) the way it will
// actually sit in the trailer, open the Serial Monitor at 115200 baud, and
// physically tilt it:
//   - raise/lower the nose  -> watch which reading changes, and which way
//   - raise/lower the left side -> same
//
// Once you know which axis+sign maps to which motion, plug those into
// PITCH_AXIS_SIGN / ROLL_AXIS_SIGN (and swap which raw angle feeds pitch vs.
// roll, if needed) in Tilt_Temp_AirStream.ino.
//
// I2C address 0x69 matches the board's ADO jumper (tied to 3.3V), which
// keeps the MPU6050 off the PCF8523 RTC's fixed address of 0x68.
//

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const int SDA_PIN = 4;
const int SCL_PIN = 15;
const int MPU6050_ADDRESS = 0x69;

Adafruit_MPU6050 mpu;

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("AxisOrientationTest -- tilt the board and watch which reading moves");
  Serial.println();

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!mpu.begin(MPU6050_ADDRESS, &Wire))
  {
    Serial.println("MPU6050 not found at 0x69 -- check the ADO jumper and wiring");
    while (1)
      delay(1000);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 ready");
  Serial.println();
}

void loop()
{
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  // Every plausible "angle off level" computed from each axis pair, in
  // degrees, so you can watch all of them at once and see which two
  // actually correspond to nose up/down and left up/down when you tilt
  // the physically-mounted board. Once identified, only two of these
  // (with the correct sign) are needed in the real sketch.
  float angleFromXZ = atan2(ax, az) * 180.0 / PI;
  float angleFromYZ = atan2(ay, az) * 180.0 / PI;
  float angleFromXY = atan2(ax, ay) * 180.0 / PI;

  Serial.print("raw accel (m/s^2)  x=");
  Serial.print(ax, 2);
  Serial.print("  y=");
  Serial.print(ay, 2);
  Serial.print("  z=");
  Serial.print(az, 2);

  Serial.print("   |   angles (deg)  XZ=");
  Serial.print(angleFromXZ, 1);
  Serial.print("  YZ=");
  Serial.print(angleFromYZ, 1);
  Serial.print("  XY=");
  Serial.print(angleFromXY, 1);
  Serial.println();

  delay(200);
}
