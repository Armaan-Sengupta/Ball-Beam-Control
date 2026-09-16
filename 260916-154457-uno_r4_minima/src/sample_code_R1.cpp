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
  delay(5000);

  geeWhizBegin();
  set_control_interval_ms(10); // 10 ms loop
  setMotorVoltage(0.0f);

  Serial.println("geeWhiz Started");
}

// ================== Loop ==================
void loop() {

}

float controller(int targetAngle, int currentAngle){
  int error = targetAngle - currentAngle;
  float Kp = 0.5; // Proportional gain
  float controlSignal = Kp * error;

  // Saturate control signal to ±6 V
  if (controlSignal > 6.0f) controlSignal = 6.0f;
  if (controlSignal < -6.0f) controlSignal = -6.0f;

  return controlSignal;
}

float radians_to_degrees(float radians) {
  return radians * (180.0f / 3.14159265358979323846f);
}

int radians_to_ticks(float radians) {
  return static_cast<int>((radians_to_degrees(radians) * (1/-0.0208)) + 5191);
}

#define NUM_TARGETS 2
float targetAngles[NUM_TARGETS] = {-0.1, 0.1}; //radians
int targetDurations[NUM_TARGETS] = {1000, 1000}; // milliseconds

// ================== Control ISR ==================
void interval_control_code(void) {
  static unsigned long lastSwitchTime = 0;
  static int currentTargetIndex = 0;

  if (millis() - lastSwitchTime > targetDurations[currentTargetIndex]) {
    currentTargetIndex = (currentTargetIndex + 1) % NUM_TARGETS;
    lastSwitchTime = millis();
  }

  // ---- Read sensors ----
  int motor = analogRead(MOT_PIN);
  int ball  = analogRead(BAL_PIN);

  digitalWrite(A5,HIGH);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
  targetAngle
  setMotorVoltage(controller(targetAngles[currentTargetIndex], motor));  // target angle = 8000, current angle = motor
  Serial.println(motor);
  digitalWrite(A5,LOW);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads

}