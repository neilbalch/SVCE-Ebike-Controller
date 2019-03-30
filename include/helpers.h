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

// Structure to hold sensor values and goals
struct State {
  // Controller Powered On-time
  unsigned long time;  // Units: ms, overflows in ~9hrs

  // Motion Data (from MPU6050)
  float accX, accY, accZ;     // Units: m/s^2
  float gyroX, gyroY, gyroZ;  // Units: rad/s

  // Power Consumption (from VESC)
  long rpm;
  float motorPower, inputPower;      // Units: W
  float motorVoltage, inputVoltage;  // Units: V
  float motorCurrent, inputCurrent;  // Units: A

  // Goals (Generated)
  bool enabled;
  float targetRPM;
  float targetW;          // Units: rad/s
  float throttleVoltage;  // Units: V

  // Returns a formatted CSV line of the struct contents
  inline String generateLogLine() {
    return (String)time + ", " + (String)accX + ", " + (String)accY + ", " +
           (String)accZ + ", " + (String)gyroX + ", " + (String)gyroY + ", " +
           (String)gyroZ + ", " + (String)rpm + ", " + (String)motorPower +
           ", " + (String)inputPower + ", " + (String)motorVoltage + ", " +
           (String)inputVoltage + ", " + (String)motorCurrent + ", " +
           (String)inputCurrent + ", " + (enabled ? "true" : "false") + ", " +
           (String)targetRPM + ", " + (String)targetW + ", " +
           (String)throttleVoltage;
  }
};
