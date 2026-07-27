// Shared LoRa packet format between Tilt_Temp_AirStream and Tilt_Temp_TowVehc.
// Keep this file identical in both sketch folders -- Arduino sketches can't
// #include a file outside their own directory, so it's duplicated rather than
// shared via a library.

#ifndef LoRaPacket_h
#define LoRaPacket_h

struct LoRaPacket
{
  float pitch;  // nose up(+)/down(-); degrees, or inches once LEVEL_POINT_DISTANCE_INCHES is set; zero-offset already applied
  float roll;   // left up(+)/down(-); same units/offset handling as pitch
  float temp1;  // Fridge, deg F
  float temp2;  // Freezer, deg F
  float temp3;  // Inside Airstream, deg F
  float temp4;  // DC Electrical Cabinet (battery bank + inverter), deg F

  // RTC date/time from the Airstream board -- all 0 if its RTC isn't
  // available. hour is 24-hour; TowVehicle converts to 12-hour+AM/PM for
  // display, matching the Airstream's own Set Time screen.
  int year;
  int month;
  int day;
  int hour;
  int minute;

  // Fan control status, mirrors the Airstream's Fan Control screens.
  int fanOn;          // 0/1
  int fanProbeIndex;  // 0=Fridge, 1=Freezer, 2=Inside AS, 3=DC Cabinet
  float fanOffTemp;   // deg F -- fan turns off at/below this
  float fanOnTemp;    // deg F -- fan turns on at/above this
  int fanStartHour;   // 24-hour
  int fanEndHour;     // 24-hour
};

#endif
