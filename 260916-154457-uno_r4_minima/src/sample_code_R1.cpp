#include <Arduino.h>
#include "geeWhiz.h"

// ================== Pins ==================
int MOT_PIN = A0;   // motor angle sensor
int BAL_PIN = A1;   // ball position sensor


// ================== Setup ==================
void setup() {

  analogReadResolution(14);
  pinMode(A5, OUTPUT);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
  Serial.begin(115200);

  geeWhizBegin();
  set_control_interval_ms(10); // 10 ms loop
  setMotorVoltage(0.0f);

  Serial.println("geeWhiz Started");
}

// ================== Loop ==================
void loop() {

}

float controller(float targetAngle, float currentAngle){
  float error = targetAngle - currentAngle; // radians
  float Kp = 15.0f; // Proportional gain (Volts / radian - tune as needed)
  float controlSignal = Kp * error;

  // Saturate control signal to ±6 V
  if (controlSignal > 6.0f) controlSignal = 6.0f;
  if (controlSignal < -6.0f) controlSignal = -6.0f;

  return controlSignal;
}

// ================== Stiction / Deadband Compensation ==================
const float STICTION_CW  = 0.17f; // Volts needed to overcome clockwise (+ve) friction
const float STICTION_CCW = 0.20f; // Volts needed to overcome counterclockwise (-ve) friction
const float DEADBAND_EPS = 0.005f; // Small deadband to prevent motor chattering around 0V

void setMotorVoltageWithStiction(float volts) {
  float compensatedVoltage = 0.0f;

  if (volts > DEADBAND_EPS) {
    // Clockwise (+ve): boost voltage to overcome CW stiction
    compensatedVoltage = volts + STICTION_CW;
  } else if (volts < -DEADBAND_EPS) {
    // Counterclockwise (-ve): boost voltage in negative direction to overcome CCW stiction
    compensatedVoltage = volts - STICTION_CCW;
  } else {
    // At zero: keep voltage at 0 to prevent jitter/chatter
    compensatedVoltage = 0.0f;
  }

  // Pass to geeWhiz driver (which handles direction, PWM duty & ±6V clamping)
  setMotorVoltage(compensatedVoltage);
}
// ================== Calibration & Conversions ==================
const int   ZERO_OFFSET_TICKS   = 5191;               // ticks at 0 deg/rad
const float SLOPE_DEG_PER_TICK  = -0.02088253981f;    // deg / tick

inline int radians_to_ticks(float rad) {
  return static_cast<int>(roundf(ZERO_OFFSET_TICKS + ((rad * RAD_TO_DEG) / SLOPE_DEG_PER_TICK)));
}

inline float ticks_to_radians(int ticks) {
  return (ticks - ZERO_OFFSET_TICKS) * SLOPE_DEG_PER_TICK * DEG_TO_RAD;
}

#define NUM_TARGETS 2
float targetAngles[NUM_TARGETS] = {-0.1, 0.1}; //radians
int targetDurations[NUM_TARGETS] = {1000, 1000}; // milliseconds

// ================== Control ISR ==================
void interval_control_code(void) {
  digitalWrite(A5,HIGH);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
  
  static unsigned long lastSwitchTime = 0;
  static int currentTargetIndex = 0;

  if (millis() - lastSwitchTime > targetDurations[currentTargetIndex]) {
    currentTargetIndex = (currentTargetIndex + 1) % NUM_TARGETS;
    lastSwitchTime = millis();
  }

  // ---- Read sensors ----
  int motor = analogRead(MOT_PIN);
  int ball  = analogRead(BAL_PIN);

  float currentAngle = ticks_to_radians(motor);
  float targetAngle  = targetAngles[currentTargetIndex];
  setMotorVoltageWithStiction(controller(targetAngle, currentAngle));
  Serial.println(currentAngle);

  digitalWrite(A5,LOW);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads

}