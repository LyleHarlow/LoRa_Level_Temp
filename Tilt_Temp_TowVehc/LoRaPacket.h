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
};

#endif
