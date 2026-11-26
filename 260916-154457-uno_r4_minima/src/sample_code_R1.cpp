#include "api/Common.h"
#include "geeWhiz.h"
#include <Arduino.h>

// ================== Pins ==================
int MOT_PIN = A0; // motor angle sensor
int BAL_PIN = A1; // ball position sensor

// ================== Setup ==================
void setup() {

  analogReadResolution(14);
  pinMode(A5,
          OUTPUT); // A5 can be used to measure cycle time using an oscilloscope
                   // by connecting the scope to the Arduino Box Motor Leads
  Serial.begin(230400);

  geeWhizBegin();
  set_control_interval_ms(2); // 2 ms loop
  setMotorVoltage(0.0f);

  Serial.println("geeWhiz Started");
}

// ================== Loop ==================
void loop() {}

float controller(float targetAngle, float currentAngle) {
  static float last_error = 0.0f;
  static float last_controlSignal = 0.0f;

  // saturate target angle
  if (targetAngle > 0.7)
    targetAngle = 0.7;
  if (targetAngle < -0.7)
    targetAngle = -0.7;

  float error = targetAngle - currentAngle; // radians
  float b1_d = -2.702602561764911;
  float b0_d = 2.270448497190859;
  float a1_d = 1.000000000000000;
  float a0_d = -0.914329121458805;

  float controlSignal = (b1_d * error) + (b0_d * last_error) - (a0_d * last_controlSignal);

  if (currentAngle > 0.8 || currentAngle < -0.8) {
    controlSignal = 0.0f;
  }

  // Saturate control signal to ±6 V
  // if (controlSignal > 6.0f)
  //   controlSignal = 6.0f;
  // if (controlSignal < -6.0f)
  //   controlSignal = -6.0f;

  last_error = error;
  last_controlSignal = controlSignal;

  return controlSignal;
}

float outerController(float targetPosition, float currentPosition) {
  // Use double (64-bit) so 15-digit Tustin coefficients do not suffer
  static double last_error = 0.0;
  static double last_theta_ref = 0.0;

  const double b1_outer = -6.984987531172070;
  const double b0_outer =  6.980099750623441;
  const double a0_outer = -0.995012468827930;

  double error = (double)targetPosition - (double)currentPosition; // meters
  double theta_ref = (b1_outer * error) + (b0_outer * last_error) - (a0_outer * last_theta_ref);

  // Store unsaturated output (lab manual cautions: "do not feed back saturated output")
  last_error = error;
  last_theta_ref = theta_ref;

  // Gear angle saturator in a separate variable before passing to inner loop
  double theta_ref_sat = theta_ref;
  if (theta_ref_sat > 0.7) {
    theta_ref_sat = 0.7;
  } else if (theta_ref_sat < -0.7) {
    theta_ref_sat = -0.7;
  }

  return (float)theta_ref_sat;
}

// ================== Stiction / Deadband Compensation ==================
const float STICTION_CW = -0.08;
const float STICTION_CCW = -0.46;
const float DEADBAND_EPS = 0.0f;

float setMotorVoltageWithStiction(float volts) {
  float compensatedVoltage = 0.0f;

  if (volts > DEADBAND_EPS) {
    compensatedVoltage = volts + STICTION_CW;
  } else if (volts < -DEADBAND_EPS) {
    compensatedVoltage = volts + STICTION_CCW;
  } else {
    compensatedVoltage = 0.0f;
  }

  // setMotorVoltage() already saturates the pins at ±6 V. Clamp the value
  // returned here too, so the number printed on serial is the pin voltage
  // and not the extra stiction offset past the rail.
  // if (compensatedVoltage > 6.0f)
  //   compensatedVoltage = 6.0f;
  // if (compensatedVoltage < -6.0f)
  //   compensatedVoltage = -6.0f;

  setMotorVoltage(compensatedVoltage);
  return compensatedVoltage;
}
// ================== Calibration & Conversions ==================
const int ZERO_OFFSET_TICKS = 5191;               // ticks at 0 deg/rad
const float SLOPE_DEG_PER_TICK = -0.02088253981f; // deg / tick

inline int radians_to_ticks(float rad) {
  return static_cast<int>(
      roundf(ZERO_OFFSET_TICKS + ((rad * RAD_TO_DEG) / SLOPE_DEG_PER_TICK)));
}

inline float ticks_to_radians(int ticks) {
  return (ticks - ZERO_OFFSET_TICKS) * SLOPE_DEG_PER_TICK * DEG_TO_RAD;
}

inline float ball_pos_to_m(int pos) {
  return 0.01372830381 + 0.00002447997789*((float)pos);
}

#define NUM_TARGETS 2
float targetPositions[NUM_TARGETS] = {0.10f, 0.25f};         // meters
unsigned long targetDurations[NUM_TARGETS] = {10000, 10000}; // milliseconds (20 s period, 50% duty)

// ================== Control ISR ==================
void interval_control_code(void) {
  digitalWrite(
      A5, HIGH); // A5 can be used to measure cycle time using an oscilloscope
                 // by connecting the scope to the Arduino Box Motor Leads

  static unsigned long lastSwitchTime = 0;
  static int currentTargetIndex = 0;

  if (millis() - lastSwitchTime >= targetDurations[currentTargetIndex]) {
    currentTargetIndex = (currentTargetIndex + 1) % NUM_TARGETS;
    lastSwitchTime = millis();
  }

  // ---- Read sensors ----
  int motor = analogRead(MOT_PIN);
  int ball = analogRead(BAL_PIN);

  float currentAngle = ticks_to_radians(motor);
  float ballPosition = ball_pos_to_m(ball);
  float targetPosition = targetPositions[currentTargetIndex];
  float targetAngle = outerController(targetPosition, ballPosition);
  float voltage = setMotorVoltageWithStiction(controller(targetAngle, currentAngle));

  // This callback is the control timer ISR. SerialUSB::write() waits forever
  // once the host stops reading, so skip the sample when the USB buffer cannot
  // hold a full line.

  if (Serial.availableForWrite() >= 64) {
    Serial.print(currentAngle, 5);
    Serial.print(',');
    Serial.print(targetAngle, 5);
    Serial.print(',');
    Serial.print(voltage, 5);
    Serial.print(',');
    Serial.print(ballPosition, 5);
    Serial.print(',');
    Serial.print(targetPosition, 5);
    Serial.println();
  }



  digitalWrite(
      A5, LOW); // A5 can be used to measure cycle time using an oscilloscope by
                // connecting the scope to the Arduino Box Motor Leads
}