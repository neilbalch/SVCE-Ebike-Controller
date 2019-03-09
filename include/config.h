#undef LED_BUILTIN
#define LED_BUILTIN PC13

#define LOOP_SPEED 200  // x cycles/sec
#define DEBUG true      // Print serial messages?
// TODO(Neil): Which are we going to use? Must be analog!!
#define THROTTLE_PIN PB5  // Pin associated with the throttle
// TODO(Neil): Tune these parameters
#define THROTTLE_CUTOFF 1.0  // Cutoff voltage for enabling the motor
#define WHEEL_RADIUS 0.5     // Units: meters
#define MAX_SPEED 8.94       // Units: m/s
// rot/min = m/s * 1rot/(2*PI*r)m * 60s/1min
#define MAX_RPM MAX_SPEED * 60 / (2 * PI * WHEEL_RADIUS)