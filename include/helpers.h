#include <Arduino.h>
#include "config.h"

// Log the debug message only if the DEBUG flag is set
inline void logMsg(String msg) {
  if (DEBUG) Serial.println("DEBUG: " + msg);
}
inline void logErr(String msg) { Serial.println("Error: " + msg); }

// Helper to map floating point values, map() only maps integers
inline float mapFloat(float x, float in_min, float in_max, float out_min,
                      float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
