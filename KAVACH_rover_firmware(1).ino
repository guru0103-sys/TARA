/*
  ============================================================================
  PS26039 — KAVACH Rover Firmware (Rover-side ESP32)
  ============================================================================
  Underground mine safety rover: drive control, gas/vibration/env sensing,
  servo-swept HC-SR04 echolocation mapping, dead-reckoning position tracking,
  and HC-12 telemetry/command link to the laptop-based Ground Control Station.

  PIN MAP (matches the combined wiring diagram)
  ----------------------------------------------------------------------------
  TB6612FNG #1 (left side, motors 1&2)
    AIN1 = GPIO18   AIN2 = GPIO19   PWMA = GPIO13
    BIN1 = GPIO14   BIN2 = GPIO27   PWMB = GPIO26
    STBY = GPIO25 (shared with driver #2)

  TB6612FNG #2 (right side, motors 3&4)
    AIN1 = GPIO23   AIN2 = GPIO22   PWMA = GPIO21
    BIN1 = GPIO5    BIN2 = GPIO4    PWMB = GPIO32
    STBY = GPIO25 (shared)

  Sensors
    MQ-2  AOUT = GPIO34 (ADC, input-only)
    MQ-7  AOUT = GPIO35 (ADC, input-only)
    DHT11 DATA = GPIO15
    SW-420 DO  = GPIO39 (input-only)
    HC-SR04 TRIG = GPIO12   ECHO = GPIO36 (input-only, needs 5V->3.3V divider)
    Servo (echolocation sweep) SIGNAL = GPIO33
    Buzzer + LED (combined)   = GPIO2

  Comms
    HC-12  RXD = GPIO17 (ESP32 UART2 TX)
           TXD = GPIO16 (ESP32 UART2 RX)
           SET -> tied to 3.3V (normal transparent mode)

  LIBRARIES REQUIRED (install via Arduino IDE Library Manager)
  ----------------------------------------------------------------------------
    1. "DHT sensor library" by Adafruit        (for DHT11)
    2. "Adafruit Unified Sensor" by Adafruit    (dependency of the above)
    3. "ESP32Servo" by Kevin Harrington         (standard Servo.h does not
                                                  reliably drive servos on ESP32)
  Board package required:
    "esp32 by Espressif Systems" (install via Boards Manager if not present)

  PROTOCOL (plain text lines over HC-12, newline-terminated)
  ----------------------------------------------------------------------------
  Rover -> GCS:
    T,<gas2>,<gas7>,<tempC>,<humidity>,<vibeEvents>,<frontDistCm>,
      <x_cm>,<y_cm>,<headingDeg>,<battPct>,<riskLevel>\n
    S,<sweepAngleDeg>,<sweepDistCm>\n     (one line per echolocation sample)

  GCS -> Rover:
    D,<leftPWM>,<rightPWM>\n             (range -255..255, sign = direction)

  NOTE ON POSITION TRACKING:
  There is no GPS or wheel encoder in this build, so position (x, y, heading)
  is estimated purely from commanded motor PWM and elapsed time (dead
  reckoning). This drifts over time and is only as accurate as the SPEED_CONST
  calibration below — treat the on-dashboard map as an approximate trail, not
  a precise survey. This is stated in the report as an honest scoping limit.
  ============================================================================
*/

#include <DHT.h>
#include <ESP32Servo.h>
#include <math.h>

// ---------------- Pin definitions ----------------
#define AIN1 18
#define AIN2 19
#define PWMA 13
#define BIN1 14
#define BIN2 27
#define PWMB 26
#define STBY 25

#define A2IN1 23
#define A2IN2 22
#define PWMA2 21
#define B2IN1 5
#define B2IN2 4
#define PWMB2 32

#define MQ2_PIN 34
#define MQ7_PIN 35
#define DHT_PIN 15
#define SW420_PIN 39
#define TRIG_PIN 12
#define ECHO_PIN 36
#define SERVO_PIN 33
#define ALERT_PIN 2       // buzzer + LED wired together

#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);
Servo sweepServo;

// HC-12 on hardware UART2
HardwareSerial HC12(2); // RX2=16, TX2=17

// ---------------- PWM channels (ESP32 ledc) ----------------
const int PWMA_CH = 0, PWMB_CH = 1, PWMA2_CH = 2, PWMB2_CH = 3;
const int PWM_FREQ = 5000, PWM_RES = 8; // 8-bit: 0-255

// ---------------- Thresholds (calibrate against real sensor readings) ----------------
const int GAS2_WARN = 280, GAS2_CRIT = 450;   // MQ-2 raw ADC-derived value
const int GAS7_WARN = 35,  GAS7_CRIT = 70;    // MQ-7
const int VIBE_WARN = 40,  VIBE_CRIT = 60;    // vibration events/min
const int OBSTACLE_STOP_CM = 15;              // forward auto-stop distance

// ---------------- State ----------------
float gas2Val = 0, gas7Val = 0, tempC = 0, humidity = 0;
int vibeEvents = 0;
float frontDistCm = 0;
int battPct = 100;
String riskLevel = "safe";

// Dead-reckoning position
float posX = 0, posY = 0;       // cm
float headingDeg = 0;           // degrees, 0 = forward at boot
unsigned long lastOdomMs = 0;
const float SPEED_CONST = 0.02;   // cm per (ms * avgPWM) -- CALIBRATE against real rover
const float TURN_CONST  = 0.05;   // deg per (ms * pwmDiff) -- CALIBRATE against real rover

// Motor command from GCS
int cmdLeft = 0, cmdRight = 0;
unsigned long lastCmdMs = 0;
const unsigned long CMD_TIMEOUT_MS = 1500; // stop motors if no command received

// Echolocation servo sweep
int sweepAngle = 30;
int sweepDir = 1;
unsigned long lastSweepMs = 0;
const unsigned long SWEEP_INTERVAL_MS = 60;

// Vibration event counting (debounced)
unsigned long lastVibeMs = 0;
int vibeCounter = 0;
unsigned long vibeWindowStart = 0;

// Timers
unsigned long lastTelemetryMs = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 500;

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  HC12.begin(9600, SERIAL_8N1, 16, 17); // RX2=16, TX2=17, matches HC-12 default baud

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(A2IN1, OUTPUT); pinMode(A2IN2, OUTPUT);
  pinMode(B2IN1, OUTPUT); pinMode(B2IN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // enable both drivers

  // ESP32 core 3.x API: ledcAttach(pin, freq, resolutionBits) — no separate
  // channel number needed, PWM is now addressed directly by pin.
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);
  ledcAttach(PWMA2, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB2, PWM_FREQ, PWM_RES);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(SW420_PIN, INPUT);
  pinMode(ALERT_PIN, OUTPUT);
  digitalWrite(ALERT_PIN, LOW);

  dht.begin();

  sweepServo.setPeriodHertz(50);
  sweepServo.attach(SERVO_PIN, 500, 2400);
  sweepServo.write(sweepAngle);

  lastOdomMs = millis();
  vibeWindowStart = millis();
  lastCmdMs = millis();

  Serial.println("KAVACH rover firmware booted.");
}

// ---------------- Main loop ----------------
void loop() {
  handleIncomingCommands();
  applyMotorCommand();
  updateOdometry();
  readSensors();
  handleVibration();
  runEcholocationSweep();
  evaluateRisk();
  handleSafetyInterlock();

  if (millis() - lastTelemetryMs > TELEMETRY_INTERVAL_MS) {
    sendTelemetry();
    lastTelemetryMs = millis();
  }
}

// ---------------- Motor control ----------------
void setMotor(int in1, int in2, int pwmPin, int speed) {
  // speed: -255..255
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    speed = -speed;
  }
  ledcWrite(pwmPin, speed); // core 3.x: ledcWrite takes the GPIO pin, not a channel
}

void applyMotorCommand() {
  // Left side = driver #1 (both channels), Right side = driver #2 (both channels)
  setMotor(AIN1, AIN2, PWMA, cmdLeft);
  setMotor(BIN1, BIN2, PWMB, cmdLeft);
  setMotor(A2IN1, A2IN2, PWMA2, cmdRight);
  setMotor(B2IN1, B2IN2, PWMB2, cmdRight);
}

void stopMotors() {
  cmdLeft = 0; cmdRight = 0;
  applyMotorCommand();
}

// ---------------- Command parsing (from GCS over HC-12) ----------------
String rxBuffer = "";

void handleIncomingCommands() {
  while (HC12.available()) {
    char c = HC12.read();
    if (c == '\n') {
      parseLine(rxBuffer);
      rxBuffer = "";
    } else if (c != '\r') {
      rxBuffer += c;
    }
  }
  // Safety: if no drive command received recently, stop (link-loss failsafe)
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    cmdLeft = 0;
    cmdRight = 0;
  }
}

void parseLine(String line) {
  line.trim();
  if (line.length() < 3 || line.charAt(0) != 'D') return;
  // Format: D,<left>,<right>
  int c1 = line.indexOf(',');
  int c2 = line.indexOf(',', c1 + 1);
  if (c1 < 0 || c2 < 0) return;
  cmdLeft = line.substring(c1 + 1, c2).toInt();
  cmdRight = line.substring(c2 + 1).toInt();
  lastCmdMs = millis();
}

// ---------------- Dead-reckoning position tracking ----------------
void updateOdometry() {
  unsigned long now = millis();
  unsigned long dt = now - lastOdomMs;
  if (dt < 20) return; // update at ~50Hz max
  lastOdomMs = now;

  float avgPWM = (cmdLeft + cmdRight) / 2.0;
  float diffPWM = (cmdRight - cmdLeft);

  float linearSpeed = avgPWM * SPEED_CONST;     // cm/ms-equivalent, scaled by dt below
  float angularSpeed = diffPWM * TURN_CONST;    // deg/ms-equivalent

  headingDeg += angularSpeed * (dt / 1000.0);
  if (headingDeg >= 360) headingDeg -= 360;
  if (headingDeg < 0) headingDeg += 360;

  float rad = headingDeg * PI / 180.0;
  posX += linearSpeed * (dt / 1000.0) * cos(rad);
  posY += linearSpeed * (dt / 1000.0) * sin(rad);
}

// ---------------- Sensors ----------------
float readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 25000); // 25ms timeout (~4m range)
  if (duration == 0) return 400; // no echo = treat as clear/far
  return duration * 0.0343 / 2.0;
}

void readSensors() {
  static unsigned long lastRead = 0;
  if (millis() - lastRead < 200) return;
  lastRead = millis();

  gas2Val = analogRead(MQ2_PIN);
  gas7Val = analogRead(MQ7_PIN);

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h)) humidity = h;
  if (!isnan(t)) tempC = t;

  // Forward-facing reading only used for the safety interlock;
  // full sweep (below) is separate and drives the mapping feature.
  if (abs(sweepAngle - 90) < 5) {
    frontDistCm = readUltrasonicCm();
  }

  battPct = max(0, battPct); // placeholder — wire to a voltage divider + map() if available
}

void handleVibration() {
  if (digitalRead(SW420_PIN) == HIGH) {
    if (millis() - lastVibeMs > 50) { // debounce
      vibeCounter++;
      lastVibeMs = millis();
    }
  }
  if (millis() - vibeWindowStart > 60000) {
    vibeEvents = vibeCounter;
    vibeCounter = 0;
    vibeWindowStart = millis();
  }
}

// ---------------- Echolocation sweep (servo + HC-SR04) ----------------
void runEcholocationSweep() {
  if (millis() - lastSweepMs < SWEEP_INTERVAL_MS) return;
  lastSweepMs = millis();

  sweepServo.write(sweepAngle);
  delay(15); // brief settle time for the servo to reach position

  float d = readUltrasonicCm();
  sendSweepSample(sweepAngle, d);

  sweepAngle += sweepDir * 3; // step size in degrees
  if (sweepAngle >= 150) { sweepAngle = 150; sweepDir = -1; }
  if (sweepAngle <= 30)  { sweepAngle = 30;  sweepDir = 1; }
}

// ---------------- Risk evaluation + safety interlock ----------------
void evaluateRisk() {
  if (gas2Val > GAS2_CRIT || gas7Val > GAS7_CRIT || frontDistCm < 12) {
    riskLevel = "critical";
  } else if (gas2Val > GAS2_WARN || gas7Val > GAS7_WARN || vibeEvents > VIBE_WARN) {
    riskLevel = "warning";
  } else {
    riskLevel = "safe";
  }
}

void handleSafetyInterlock() {
  // Obstacle auto-stop: block forward motion if something is too close ahead
  if (frontDistCm < OBSTACLE_STOP_CM && (cmdLeft > 0 || cmdRight > 0)) {
    cmdLeft = 0;
    cmdRight = 0;
    applyMotorCommand();
  }

  // Local alarm on critical gas/obstacle condition
  digitalWrite(ALERT_PIN, riskLevel == "critical" ? HIGH : LOW);
}

// ---------------- Telemetry out (to GCS over HC-12) ----------------
void sendTelemetry() {
  HC12.print("T,");
  HC12.print(gas2Val, 0); HC12.print(",");
  HC12.print(gas7Val, 0); HC12.print(",");
  HC12.print(tempC, 1); HC12.print(",");
  HC12.print(humidity, 0); HC12.print(",");
  HC12.print(vibeEvents); HC12.print(",");
  HC12.print(frontDistCm, 0); HC12.print(",");
  HC12.print(posX, 1); HC12.print(",");
  HC12.print(posY, 1); HC12.print(",");
  HC12.print(headingDeg, 1); HC12.print(",");
  HC12.print(battPct); HC12.print(",");
  HC12.println(riskLevel);
}

void sendSweepSample(int angle, float distCm) {
  HC12.print("S,");
  HC12.print(angle); HC12.print(",");
  HC12.println(distCm, 0);
}
