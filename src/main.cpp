#include <Arduino.h>
#include <HardwareSerial.h>  // For Serial
#include <MPU6050_tockn.h>   // For MPU6050 (I2C)
#include <SD.h>              // For SD Card
#include <SPI.h>             // For SD Card
#include <VescUart.h>        // For VESC
#include <Wire.h>            // For MPU6050 (I2C)
#include "config.h"

// 6DOF Accelerometer/Gyro
MPU6050 mpu6050(Wire);

// Log the debug message only if the DEBUG flag is set
inline void logMsg(String msg) {
  if (DEBUG) Serial.println("DEBUG: " + msg);
}
inline void logErr(String msg) { Serial.println("Error: " + msg); }

struct State {
  // Powered Time
  unsigned long time;  // Units: ms

  // Motion Data (from MPU6050)
  float accX, accY, accZ;     // Units: m/s^2
  float gyroX, gyroY, gyroZ;  // Units: rad/s

  // Power Consumption (from VESC)
  float motorPower, inputPower;      // Units: W
  float motorVoltage, inputVoltage;  // Units: V
  float motorCurrent, inputCurrent;  // Units: A

  // Goals (Generated)
  bool enabled;
  float targetRPM;
  float targetW;          // Units: rad/s
  float throttleVoltage;  // Units: V
};
State state;

void setup() {
  Serial.begin(250000);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(DEBUG);

  pinMode(THROTTLE_PIN, INPUT);

  // TODO(Neil): Open file io
}

void loop() {
  state.time = millis();

  {  // Populate MPU6050's state fields
    state.accX = mpu6050.getAccX() * 9.80;
    state.accY = mpu6050.getAccY() * 9.80;
    state.accZ = mpu6050.getAccZ() * 9.80;

    state.gyroX = mpu6050.getGyroX() * (PI / 180);
    state.gyroY = mpu6050.getGyroY() * (PI / 180);
    state.gyroZ = mpu6050.getGyroZ() * (PI / 180);
  }

  // TODO(Neil): Implement VESC Telemetry Recording Here

  // Read throttle voltage
  float throttle =
      analogRead(THROTTLE_PIN) * (5.0 /* V*/ / 4095.0 /* steps */);  // Units: V
  logMsg("Throttle Voltage: " + (String)throttle);
  state.throttleVoltage = throttle;

  // Are we enabled?
  if (throttle < THROTTLE_CUTOFF) {  // Disabled
    logMsg("Disabled");
    state.enabled = false;
    state.targetRPM = 0.0;
    state.targetW = 0.0;
  } else {  // Enabled
    logMsg("Enabled");
    state.enabled = true;

    // Compute the desired throttle setting
    float rpm = map(throttle, 0, 5, 0, MAX_RPM);
    constrain(rpm, 0, MAX_RPM);  // Should be extranous, but just make sure
    state.targetRPM = rpm;
    state.targetW = rpm * 2 * PI / 60;

    // TODO(Neil): Send target RPM to ESC

    // TODO(Neil): Write info to CSV
  }

  // Generate the loop time delay
  delay(1000 / LOOP_SPEED);
}