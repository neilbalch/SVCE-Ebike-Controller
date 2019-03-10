#define LOOP_SPEED 200    // x cycles/sec
#define DEBUG true        // Print serial messages?
#define SD_CS_PIN PA4     // Pin associated with the SD Card CS line
#define THROTTLE_PIN PA0  // Pin associated with the throttle
#define THROTTLE_LOW 1.3  // LOW Voltage from the throttle (Units: V)
#define THROTTLE_HIGH 5   // HIGH Voltage from the throttle (Units: V)
// TODO(Neil): Tune these parameters
#define THROTTLE_CUTOFF \
  THROTTLE_LOW + 0.15     // Cutoff voltage for enabling the motor
#define WHEEL_RADIUS 0.5  // Units: meters
#define MAX_SPEED 8.94    // Units: m/s
// rot/min = m/s * 1rot/(2*PI*r)m * 60s/1min
#define MAX_RPM MAX_SPEED * 60 / (2 * PI * WHEEL_RADIUS)
