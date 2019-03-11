#include <Arduino.h>
#include <HardwareSerial.h>  // For Serial
#include <MPU6050_tockn.h>   // For MPU6050 (I2C)
#include <SD.h>              // For SD Card
#include <VescUart.h>        // For VESC
#include <Wire.h>            // For MPU6050 (I2C)
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
  // Powered Time
  unsigned long time;  // Units: ms

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

// Global objects
MPU6050 mpu6050(Wire);  // 6DOF Accelerometer/Gyro
VescUart vesc;          // VESC
String logFile = "";    // Path to currently in-use log file
State state;

void setup() {
  // Init USB Serial
  Serial.begin(250000);

  // Init I2C and MPU6050
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(DEBUG);

  // Init throttle sense pin
  pinMode(THROTTLE_PIN, INPUT);

  // Init SD file IO
  logMsg("\nSD card init...");
  if (!SD.begin(SD_CS_PIN)) {
    logErr("SD Card failed, or not present");
    while (1) {
    }
  }
  logMsg("SD card initialized");

  // Find out what the latest log file is, and use the next one.
  int num = 0;
  while (logFile == "") {
    if (SD.exists("log" + (String)num + ".csv"))
      num++;
    else
      logFile = "log" + (String)num + ".csv";
  }
  logMsg("Using log file: " + (String)logFile);

  // Send header line
  File log = SD.open(logFile, FILE_WRITE);
  if (log) {
    log.println(
        "time, accX, accY, accZ, gyroX, gyroY, gyroZ, rpm, motorPower, "
        "inputPower, motorVoltage, inputVoltage, motorCurrent, inputCurrent, "
        "enabled, targetRPM, targetW, throttleVoltage");
    log.close();
    logMsg("Header Line Written");
  } else
    logErr("Error opening " + (String)logFile);

  // Init VESC
  Serial1.begin(19200);
  while (!Serial1) {
  }
  vesc.setSerialPort(&Serial1);
}

void loop() {
  state.time = millis();

  {  // Populate MPU6050 state fields
    mpu6050.update();
    state.accX = mpu6050.getAccX() * 9.80;
    state.accY = mpu6050.getAccY() * 9.80;
    state.accZ = mpu6050.getAccZ() * 9.80;

    state.gyroX = mpu6050.getGyroX() * (PI / 180);
    state.gyroY = mpu6050.getGyroY() * (PI / 180);
    state.gyroZ = mpu6050.getGyroZ() * (PI / 180);
  }

  {  // Populate VESC state fields
    if (vesc.getVescValues()) {
      state.rpm = vesc.data.rpm;
      state.motorVoltage =
          vesc.data.inpVoltage;  // It's the same. BLDC ESCs drive the same
                                 // voltage, just changing pulse times
      state.inputVoltage = vesc.data.inpVoltage;
      state.motorCurrent = vesc.data.avgMotorCurrent;
      state.inputCurrent = vesc.data.avgInputCurrent;
      state.motorPower = state.motorVoltage * state.motorCurrent;
      state.inputPower = state.inputVoltage * state.inputCurrent;
    } else {
      state.rpm = 0;
      state.motorPower = 0;
      state.inputPower = 0;
      state.motorVoltage = 0;
      state.inputVoltage = 0;
      state.motorCurrent = 0;
      state.inputCurrent = 0;
    }
  }

  // Read throttle voltage
  float throttle =
      analogRead(THROTTLE_PIN) * (5.0 /* V*/ / 4095.0 /* steps */);  // Units: V
  state.throttleVoltage = throttle;

  // Are we enabled?
  if (throttle < THROTTLE_CUTOFF) {  // Disabled
    state.enabled = false;
    state.targetRPM = 0.0;
    state.targetW = 0.0;
  } else {  // Enabled
    state.enabled = true;

    // Compute the desired throttle setting
    float rpm = mapFloat(throttle, THROTTLE_LOW, THROTTLE_HIGH, 0, MAX_RPM);
    constrain(rpm, 0, MAX_RPM);  // Should be extranous, but just make sure
    state.targetRPM = rpm;
    state.targetW = rpm * 2 * PI / 60;

    // TODO(Neil): Do I need to send current? brakeCurrent?
    vesc.setRPM(rpm);
  }

  // Write state info to SD file, log to serial
  File log = SD.open(logFile, FILE_WRITE);
  if (log) {
    String msg = state.generateLogLine();
    logMsg(msg);
    log.println(msg);
    log.close();
  } else
    logErr("Error opening " + (String)logFile);

  // Generate the loop time delay
  delay(1000 / LOOP_SPEED);
}