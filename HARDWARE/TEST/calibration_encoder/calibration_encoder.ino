/*
 * ═══════════════════════════════════════════════════════════
 *  ENCODER ONE-ROTATION CALIBRATION
 *  Usage:
 *    1. Type 's' → motor starts + pulse counting begins
 *    2. Watch wheel complete exactly ONE full rotation
 *    3. Type 's' again → motor stops, pulses counted = PPR
 * ═══════════════════════════════════════════════════════════
 */

#include <Arduino.h>

// ── Pins ──────────────────────────────────────────────────────
#define ENCODER_C1       15
#define ENCODER_C2       4

#define LEFT_IN1         14
#define LEFT_IN2         12
#define LEFT_PWM         13
#define RIGHT_IN1        26
#define RIGHT_IN2        25
#define RIGHT_PWM        33

// ── Wheel ─────────────────────────────────────────────────────
#define WHEEL_DIAMETER_MM  34.0f
#define MOTOR_SPEED        150     // slow speed for easy observation

// ── Encoder globals ───────────────────────────────────────────
volatile long encoderCount = 0;
volatile int  lastC1State  = 0;

void IRAM_ATTR encoderISR() {
  encoderCount++;   // count every edge — direction not needed here
}

// ── State machine ─────────────────────────────────────────────
enum State { IDLE, COUNTING };
State state = IDLE;
long  countAtStart = 0;

// ── Motor helpers ─────────────────────────────────────────────
void motorsForward(int speed) {
  digitalWrite(LEFT_IN1,  HIGH);
  digitalWrite(LEFT_IN2,  LOW);
  ledcWrite(LEFT_PWM, speed);

  digitalWrite(RIGHT_IN1, HIGH);  // adjust polarity if needed
  digitalWrite(RIGHT_IN2, LOW);
  ledcWrite(RIGHT_PWM, speed);
}

void motorsStop() {
  digitalWrite(LEFT_IN1,  LOW);
  digitalWrite(LEFT_IN2,  LOW);
  ledcWrite(LEFT_PWM, 0);

  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
  ledcWrite(RIGHT_PWM, 0);
}

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  // Encoder
  pinMode(ENCODER_C1, INPUT_PULLUP);
  pinMode(ENCODER_C2, INPUT_PULLUP);
  delay(10);
  lastC1State = digitalRead(ENCODER_C1);
  attachInterrupt(digitalPinToInterrupt(ENCODER_C1),
                  encoderISR, CHANGE);

  // Motors
  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  ledcAttach(LEFT_PWM,  5000, 8);
  ledcAttach(RIGHT_PWM, 5000, 8);
  motorsStop();

  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║   ENCODER CALIBRATION — ONE ROTATION ║");
  Serial.println("╠══════════════════════════════════════╣");
  Serial.println("║  Type 's' → START motor + counting   ║");
  Serial.println("║  Watch wheel do exactly ONE rotation  ║");
  Serial.println("║  Type 's' → STOP and show result     ║");
  Serial.println("╚══════════════════════════════════════╝\n");
}

// ─────────────────────────────────────────────────────────────
void loop() {

  // ── Live pulse display while counting ────────────────────
  if (state == COUNTING) {
    static uint32_t lastDisplay = 0;
    if (millis() - lastDisplay >= 100) {
      lastDisplay = millis();
      noInterrupts();
      long current = encoderCount;
      interrupts();
      long pulsesSoFar = current - countAtStart;
      Serial.printf("\r  Pulses so far: %ld   ", pulsesSoFar);
    }
  }

  // ── Serial input handler ──────────────────────────────────
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c != 's' && c != 'S') return;

    if (state == IDLE) {
      // ── START ─────────────────────────────────────────────
      noInterrupts();
      encoderCount = 0;
      interrupts();
      countAtStart = 0;

      motorsForward(MOTOR_SPEED);
      state = COUNTING;

      Serial.println("\n[START] Motor running — counting pulses...");
      Serial.println("        Watch wheel, type 's' after ONE full rotation.\n");

    } else {
      // ── STOP ──────────────────────────────────────────────
      motorsStop();
      state = IDLE;

      noInterrupts();
      long finalCount = encoderCount;
      interrupts();

      long pulsesPerRev = finalCount - countAtStart;

      float circumferenceMM = PI * WHEEL_DIAMETER_MM;
      float mmPerPulse      = circumferenceMM / (float)pulsesPerRev;
      float cmPerPulse      = mmPerPulse / 10.0f;

      Serial.println("\n\n╔══════════════════════════════════════╗");
      Serial.println("║         CALIBRATION RESULT           ║");
      Serial.println("╠══════════════════════════════════════╣");
      Serial.printf( "║  Pulses counted  : %-18ld║\n", pulsesPerRev);
      Serial.printf( "║  Wheel diameter  : %-14.1f mm  ║\n", WHEEL_DIAMETER_MM);
      Serial.printf( "║  Circumference   : %-14.2f mm  ║\n", circumferenceMM);
      Serial.printf( "║  mm per pulse    : %-18.5f║\n", mmPerPulse);
      Serial.printf( "║  cm per pulse    : %-18.5f║\n", cmPerPulse);
      Serial.println("╠══════════════════════════════════════╣");
      Serial.println("║  Copy this to config.h:              ║");
      Serial.printf( "║  #define PULSES_PER_REV  %-12ld║\n", pulsesPerRev);
      Serial.println("╚══════════════════════════════════════╝\n");
      Serial.println("Type 's' again to run another calibration.\n");
    }
  }
}
// 1944        1716                1982            1837   
// 1858                               