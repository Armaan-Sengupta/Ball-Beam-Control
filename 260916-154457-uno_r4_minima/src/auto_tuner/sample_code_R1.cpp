// Auto-tuner firmware. Select the PlatformIO environment "auto_tuner".
// "uno_r4_minima" still builds src/sample_code_R1.cpp, the fixed sweep.
//
// The port stays open across trials. These commands are accepted any time:
//   C <a1> <b1> <b0>     replace the controller immediately
//   M <radians>          move the angle reference (clamped to +/-0.7)
//
// Difference equation. a1 is the coefficient of u[k-1], which the sweep
// sketch stores as a0_d (its a1_d is 1 and is not used):
//   u[k] = b1*e[k] + b0*e[k-1] - a1*u[k-1]
//
// Status lines start with '#'. Samples stay "millis,theta,ref,voltage"
// and begin once the first move command has been stored.

#include "api/Common.h"
#include "geeWhiz.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

// ================== Pins ==================
int MOT_PIN = A0; // motor angle sensor
int BAL_PIN = A1; // ball position sensor

// ================== Runtime controller ==================
// Defaults match the sweep sketch so a C command can restore that design.
volatile float g_a1 = -0.914329121458805f;
volatile float g_b1 = -2.702602561764911f;
volatile float g_b0 = 2.270448497190859f;
volatile bool g_gains_ready = false;
volatile bool g_reset_ctrl = false;

volatile float g_target = 0.0f;
volatile bool g_target_ready = false;

volatile bool g_ack_ready = false;
volatile char g_ack[96];

static void queueAck(const char *msg) {
  noInterrupts();
  size_t i = 0;
  for (; msg[i] != '\0' && i + 1 < sizeof(g_ack); ++i) {
    g_ack[i] = msg[i];
  }
  g_ack[i] = '\0';
  g_ack_ready = true;
  interrupts();
}

static bool writeIfRoom(const char *text, size_t len) {
  if (len == 0 || Serial.availableForWrite() < (int)len) {
    return false;
  }
  Serial.write(reinterpret_cast<const uint8_t *>(text), len);
  return true;
}

static void flushAck() {
  if (!g_ack_ready) {
    return;
  }
  char local[sizeof(g_ack)];
  size_t i = 0;
  for (; i + 1 < sizeof(local) && g_ack[i] != '\0'; ++i) {
    local[i] = g_ack[i];
  }
  if (i + 1 >= sizeof(local)) {
    return;
  }
  local[i++] = '\n';
  local[i] = '\0';
  // Called from the control ISR. Leave the ack queued if USB cannot take it;
  // SerialUSB::write() otherwise spins forever when the host stops reading.
  if (writeIfRoom(local, i)) {
    g_ack_ready = false;
  }
}

static float clampTarget(float target) {
  if (target > 0.7f) {
    return 0.7f;
  }
  if (target < -0.7f) {
    return -0.7f;
  }
  return target;
}

static void handleCommand(char *line) {
  while (*line == ' ' || *line == '\t') {
    ++line;
  }
  if (*line == '\0') {
    return;
  }

  char cmd = line[0];
  char *args = line + 1;
  for (char *p = args; *p != '\0'; ++p) {
    if (*p == ',') {
      *p = ' ';
    }
  }

  if (cmd == 'C' || cmd == 'c') {
    float a1 = 0.0f;
    float b1 = 0.0f;
    float b0 = 0.0f;
    if (sscanf(args, "%f %f %f", &a1, &b1, &b0) == 3) {
      noInterrupts();
      g_a1 = a1;
      g_b1 = b1;
      g_b0 = b0;
      g_gains_ready = true;
      g_reset_ctrl = true;
      interrupts();
      char msg[96];
      snprintf(msg, sizeof(msg), "# OK C %.6f %.6f %.6f", a1, b1, b0);
      queueAck(msg);
      return;
    }
  } else if (cmd == 'M' || cmd == 'm') {
    float target = 0.0f;
    if (sscanf(args, "%f", &target) == 1) {
      target = clampTarget(target);
      noInterrupts();
      g_target = target;
      g_target_ready = true;
      interrupts();
      char msg[48];
      snprintf(msg, sizeof(msg), "# OK M %.4f", target);
      queueAck(msg);
      return;
    }
  }

  queueAck("# ERR");
}

static void pollSerial() {
  static char buf[96];
  static size_t len = 0;

  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      buf[len] = '\0';
      if (len > 0) {
        handleCommand(buf);
      }
      len = 0;
      continue;
    }
    if (len + 1 < sizeof(buf)) {
      buf[len++] = c;
    } else {
      len = 0;
    }
  }
}

// ================== Setup ==================
void setup() {
  analogReadResolution(14);
  pinMode(A5, OUTPUT); // scope probe for the control-cycle timing
  Serial.begin(230400);

  geeWhizBegin();
  set_control_interval_ms(2); // 2 ms loop
  setMotorVoltage(0.0f);

  Serial.println("# READY");
}

// ================== Loop ==================
void loop() { pollSerial(); }

float controller(float targetAngle, float currentAngle) {
  static float last_error = 0.0f;
  static float last_controlSignal = 0.0f;

  // loop() publishes a new set with interrupts off, so one sample sees
  // either the old gains or the new ones. This runs inside the control ISR,
  // so it must not turn interrupts back on.
  float a1 = g_a1;
  float b1 = g_b1;
  float b0 = g_b0;
  bool reset = g_reset_ctrl;
  if (reset) {
    g_reset_ctrl = false;
  }

  if (reset) {
    last_error = 0.0f;
    last_controlSignal = 0.0f;
  }

  if (targetAngle > 0.7f) {
    targetAngle = 0.7f;
  }
  if (targetAngle < -0.7f) {
    targetAngle = -0.7f;
  }

  float error = targetAngle - currentAngle; // radians
  // u[k] = b1*e[k] + b0*e[k-1] - a1*u[k-1]
  float controlSignal = (b1 * error) + (b0 * last_error) - (a1 * last_controlSignal);

  // Saturate control signal to ±6 V
  // if (controlSignal > 6.0f)
  //   controlSignal = 6.0f;
  // if (controlSignal < -6.0f)
  //   controlSignal = -6.0f;

  last_error = error;
  last_controlSignal = controlSignal;
  return controlSignal;
}

// ================== Stiction / Deadband Compensation ==================
const float STICTION_CW = 0.17f;
const float STICTION_CCW = 0.20f;
const float DEADBAND_EPS = 0.0f;

float setMotorVoltageWithStiction(float volts) {
  float compensatedVoltage = 0.0f;

  if (volts > DEADBAND_EPS) {
    compensatedVoltage = volts + STICTION_CW;
  } else if (volts < -DEADBAND_EPS) {
    compensatedVoltage = volts - STICTION_CCW;
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

  // invert to make it such that a +ve voltage results in a CCW rotation (+ve by
  // convention)
  setMotorVoltage(-compensatedVoltage);
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

// ================== Control ISR ==================
void interval_control_code(void) {
  digitalWrite(A5, HIGH);
  flushAck();

  // Hold the motor off until the host has sent both a controller and a target.
  // Later C/M commands update those values without rebooting.
  if (!g_gains_ready || !g_target_ready) {
    setMotorVoltage(0.0f);
    digitalWrite(A5, LOW);
    return;
  }

  int motor = analogRead(MOT_PIN);
  int ball = analogRead(BAL_PIN);
  (void)ball;

  float currentAngle = ticks_to_radians(motor);
  float targetAngle = g_target;
  float voltage = setMotorVoltageWithStiction(controller(targetAngle, currentAngle));

  // Same guard as the sweep sketch. Serial.print formats floats itself;
  // snprintf %f does not work with this core's nano printf.
  if (Serial.availableForWrite() >= 64) {
    Serial.print(millis());
    Serial.print(',');
    Serial.print(currentAngle, 5);
    Serial.print(',');
    Serial.print(targetAngle, 4);
    Serial.print(',');
    Serial.println(voltage, 4);
  }

  digitalWrite(A5, LOW);
}
