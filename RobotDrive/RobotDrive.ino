#include <Servo.h>

// ----------------- ENCODER & MOTOR PINS -----------------
const byte ENC_FL = A0;
const byte ENC_FR = A1;
const byte ENC_BL = A2;
const byte ENC_BR = A3;

const byte FL_IN1 = 4;
const byte FL_IN2 = 3;
const byte FR_IN1 = 7;
const byte FR_IN2 = 5;
const byte BL_IN1 = 8;
const byte BL_IN2 = 6;
const byte BR_IN1 = 12;
const byte BR_IN2 = 11;

// Servos on A4 and A5 (not Timer1 pins, safe with Servo library)
const byte SERVO1_PIN = A4;   // tilt
const byte SERVO2_PIN = A5;   // pan

// ----------------- SOFTWARE PWM VARS --------------------
volatile byte pwm_FL_IN1 = 0;
volatile byte pwm_FL_IN2 = 0;
volatile byte pwm_FR_IN1 = 0;
volatile byte pwm_FR_IN2 = 0;
volatile byte pwm_BL_IN1 = 0;
volatile byte pwm_BL_IN2 = 0;
volatile byte pwm_BR_IN1 = 0;
volatile byte pwm_BR_IN2 = 0;

volatile byte pwm_counter = 0;

// PORTD masks (D2–D7)
#define FL_IN1_MASK  (1 << 4)   // D4 -> PD4
#define FL_IN2_MASK  (1 << 3)   // D3 -> PD3
#define FR_IN1_MASK  (1 << 7)   // D7 -> PD7
#define FR_IN2_MASK  (1 << 5)   // D5 -> PD5
#define BL_IN2_MASK  (1 << 6)   // D6 -> PD6

// PORTB masks (D8–D13)
#define BL_IN1_MASK  (1 << 0)   // D8  -> PB0
#define BR_IN2_MASK  (1 << 3)   // D11 -> PB3
#define BR_IN1_MASK  (1 << 4)   // D12 -> PB4

// ----------------- ENCODER / PID VARS -------------------
volatile long count_FL = 0;
volatile long count_FR = 0;
volatile long count_BL = 0;
volatile long count_BR = 0;

long prev_count_FL = 0;
long prev_count_FR = 0;
long prev_count_BL = 0;
long prev_count_BR = 0;

int speed_FL = 0, speed_FR = 0, speed_BL = 0, speed_BR = 0;
int target_FL = 0, target_FR = 0, target_BL = 0, target_BR = 0;

byte motor_pwm_FL = 0, motor_pwm_FR = 0, motor_pwm_BL = 0, motor_pwm_BR = 0;
int motor_dir_FL = 0, motor_dir_FR = 0, motor_dir_BL = 0, motor_dir_BR = 0;

unsigned long last_speed_update = 0;
const int SPEED_UPDATE_INTERVAL = 50;

// PID tuning
const float Kp = 5.0;         
const float Ki = 0.3;         
const float Kd = 0.5;         
const float MAX_PWM_SCALE = 0.70f;
const int   MAX_TICKS_PER_INTERVAL = 30;   // "full" speed

float integral_FL = 0, integral_FR = 0, integral_BL = 0, integral_BR = 0;
int last_error_FL = 0, last_error_FR = 0, last_error_BL = 0, last_error_BR = 0;

// Servos
Servo servo1;
Servo servo2;
int servo1Pos = 60;   // tilt
int servo2Pos = 100;  // pan

// ------------- SERIAL LINE PARSER -----------------------
char serialBuf[32];
byte serialPos = 0;

// Watchdog timer for safety
unsigned long last_command_time = 0;
const unsigned long COMMAND_TIMEOUT = 500; // 500ms timeout

// ----------------- TIMER2 ISR: SOFTWARE PWM -------------
ISR(TIMER2_COMPA_vect)
{
  pwm_counter++;

  uint8_t outD = 0;
  uint8_t outB = 0;

  if (pwm_counter < pwm_FL_IN1) outD |= FL_IN1_MASK;
  if (pwm_counter < pwm_FL_IN2) outD |= FL_IN2_MASK;
  if (pwm_counter < pwm_FR_IN1) outD |= FR_IN1_MASK;
  if (pwm_counter < pwm_FR_IN2) outD |= FR_IN2_MASK;
  if (pwm_counter < pwm_BL_IN2) outD |= BL_IN2_MASK;

  if (pwm_counter < pwm_BL_IN1) outB |= BL_IN1_MASK;
  if (pwm_counter < pwm_BR_IN1) outB |= BR_IN1_MASK;
  if (pwm_counter < pwm_BR_IN2) outB |= BR_IN2_MASK;

  PORTD = (PORTD & ~(
            FL_IN1_MASK | FL_IN2_MASK |
            FR_IN1_MASK | FR_IN2_MASK |
            BL_IN2_MASK
          )) | outD;

  PORTB = (PORTB & ~(
            BL_IN1_MASK | BR_IN1_MASK | BR_IN2_MASK
          )) | outB;
}

// ----------------- ENCODER ISR (LIGHTWEIGHT) ------------
// Read PORTC and detect rising edges on A0..A3
volatile uint8_t last_enc_state = 0;

ISR(PCINT1_vect)
{
  uint8_t state = PINC & 0x0F;          // A0..A3 = PC0..PC3
  uint8_t rising = (~last_enc_state) & state;

  if (rising & (1 << 0)) count_FL++;    // A0 rising
  if (rising & (1 << 1)) count_FR++;    // A1 rising
  if (rising & (1 << 2)) count_BL++;    // A2 rising
  if (rising & (1 << 3)) count_BR++;    // A3 rising

  last_enc_state = state;
}

// ----------------- SETUP TIMER2 -------------------------
void setupSoftwarePWM()
{
  cli();
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  OCR2A = 63;              // ISR frequency
  TCCR2A |= (1 << WGM21);  // CTC
  TCCR2B |= (1 << CS21);   // prescaler 8
  TIMSK2 |= (1 << OCIE2A);
  sei();
}

// ----------------- MOTOR + PID --------------------------
void applyMotorPWM(volatile byte &in1, volatile byte &in2, int dir, byte pwm)
{
  if (dir > 0) {
    in1 = pwm;
    in2 = 0;
  } else if (dir < 0) {
    in1 = 0;
    in2 = pwm;
  } else {
    in1 = 0;
    in2 = 0;
  }
}

byte calculatePID(int current_speed, int target_speed,
                  float &integral, int &last_error, byte current_pwm)
{
  if (target_speed == 0) {
    integral = 0;
    last_error = 0;
    return 0;
  }

  int error = target_speed - current_speed;
  integral += error;
  integral = constrain(integral, -200, 200);  // wind-up protection

  int derivative = error - last_error;
  last_error = error;

  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  int new_pwm = current_pwm + (int)output;

  int max_pwm = (int)(255.0f * MAX_PWM_SCALE); // ~178
  new_pwm = constrain(new_pwm, 30, max_pwm);   // min 30 for low-speed control

  return (byte)new_pwm;
}

void updateSpeedControl()
{
  unsigned long now = millis();
  if (now - last_speed_update < SPEED_UPDATE_INTERVAL) return;

  noInterrupts();
  long temp_FL = count_FL;
  long temp_FR = count_FR;
  long temp_BL = count_BL;
  long temp_BR = count_BR;
  interrupts();

  speed_FL = temp_FL - prev_count_FL;
  speed_FR = temp_FR - prev_count_FR;
  speed_BL = temp_BL - prev_count_BL;
  speed_BR = temp_BR - prev_count_BR;

  motor_pwm_FL = calculatePID(speed_FL, target_FL, integral_FL, last_error_FL, motor_pwm_FL);
  motor_pwm_FR = calculatePID(speed_FR, target_FR, integral_FR, last_error_FR, motor_pwm_FR);
  motor_pwm_BL = calculatePID(speed_BL, target_BL, integral_BL, last_error_BL, motor_pwm_BL);
  motor_pwm_BR = calculatePID(speed_BR, target_BR, integral_BR, last_error_BR, motor_pwm_BR);

  applyMotorPWM(pwm_FL_IN1, pwm_FL_IN2, motor_dir_FL, motor_pwm_FL);
  applyMotorPWM(pwm_FR_IN1, pwm_FR_IN2, motor_dir_FR, motor_pwm_FR);
  applyMotorPWM(pwm_BL_IN1, pwm_BL_IN2, motor_dir_BL, motor_pwm_BL);
  applyMotorPWM(pwm_BR_IN1, pwm_BR_IN2, motor_dir_BR, motor_pwm_BR);

  prev_count_FL = temp_FL;
  prev_count_FR = temp_FR;
  prev_count_BL = temp_BL;
  prev_count_BR = temp_BR;

  last_speed_update = now;
}

// movement helpers
void setMotorTargets(int tgt_FL, int tgt_FR, int tgt_BL, int tgt_BR,
                     int dir_FL, int dir_FR, int dir_BL, int dir_BR)
{
  target_FL = abs(tgt_FL);
  target_FR = abs(tgt_FR);
  target_BL = abs(tgt_BL);
  target_BR = abs(tgt_BR);

  motor_dir_FL = dir_FL;
  motor_dir_FR = dir_FR;
  motor_dir_BL = dir_BL;
  motor_dir_BR = dir_BR;

  if (target_FL == 0) { 
    integral_FL = 0; 
    last_error_FL = 0; 
    motor_pwm_FL = 0; 
  }
  else if (motor_pwm_FL == 0) motor_pwm_FL = 60;

  if (target_FR == 0) { 
    integral_FR = 0; 
    last_error_FR = 0; 
    motor_pwm_FR = 0; 
  }
  else if (motor_pwm_FR == 0) motor_pwm_FR = 60;

  if (target_BL == 0) { 
    integral_BL = 0; 
    last_error_BL = 0; 
    motor_pwm_BL = 0; 
  }
  else if (motor_pwm_BL == 0) motor_pwm_BL = 60;

  if (target_BR == 0) { 
    integral_BR = 0; 
    last_error_BR = 0; 
    motor_pwm_BR = 0; 
  }
  else if (motor_pwm_BR == 0) motor_pwm_BR = 60;
}

// ============ JOYSTICK → DRIVE (XYZ) ===================
// x, y, z are -100..100 for x/y, 0..100 for z (magnitude)
void driveFromJoystick(int x, int y, int z)
{
  last_command_time = millis();
  
  if (z == 0 || (x == 0 && y == 0)) {
    setMotorTargets(0, 0, 0, 0, 0, 0, 0, 0);
    return;
  }

  float fx = x / 100.0f;   // -1..1
  float fy = y / 100.0f;   // -1..1
  float throttle = z / 100.0f; // 0..1

  if (throttle < 0) throttle = -throttle;
  if (throttle > 1) throttle = 1;

  float base = throttle * MAX_TICKS_PER_INTERVAL;

  // exponential curve
  float curve_factor = 1.5;
  if (base > 0) {
    base = MAX_TICKS_PER_INTERVAL * pow(throttle, curve_factor);
  }

  float left  = (fy + fx) * base;
  float right = (fy - fx) * base;

  float max_val = max(abs(left), abs(right));
  if (max_val > MAX_TICKS_PER_INTERVAL) {
    float scale = MAX_TICKS_PER_INTERVAL / max_val;
    left *= scale;
    right *= scale;
  }

  int leftTicks  = (int)left;
  int rightTicks = (int)right;

  int tgt_FL=0, tgt_FR=0, tgt_BL=0, tgt_BR=0;
  int dir_FL=0, dir_FR=0, dir_BL=0, dir_BR=0;

  if (leftTicks > 0) {
    dir_FL = dir_BL = +1;
    tgt_FL = tgt_BL = leftTicks;
  } else if (leftTicks < 0) {
    dir_FL = dir_BL = -1;
    tgt_FL = tgt_BL = -leftTicks;
  }

  if (rightTicks > 0) {
    dir_FR = dir_BR = +1;
    tgt_FR = tgt_BR = rightTicks;
  } else if (rightTicks < 0) {
    dir_FR = dir_BR = -1;
    tgt_FR = tgt_BR = -rightTicks;
  }

  setMotorTargets(tgt_FL, tgt_FR, tgt_BL, tgt_BR,
                  dir_FL, dir_FR, dir_BL, dir_BR);
}

// ============ JOYSTICK → CAMERA ========================
// void camFromJoystick(int x, int y)
// {
//   last_command_time = millis();
  
//   const int DEADZONE = 5;
//   if (abs(x) < DEADZONE) x = 0;
//   if (abs(y) < DEADZONE) y = 0;
  
//   // If you want inverted axes, swap 100/-100 like you had
//   int pan  = map(x, 100, -100, 10, 170);  // servo2 (pan)
//   int tilt = map(y, 100, -100, 10, 170);  // servo1 (tilt)

//   servo2Pos = constrain(pan, 10, 170);
//   servo1Pos = constrain(tilt, 10, 170);

//   servo2.write(servo2Pos);
//   servo1.write(servo1Pos);
// }


void camFromJoystick(int x, int y)
{
  last_command_time = millis();

  const int STEP = 5; // degrees per press

  // Up/Down control tilt (servo1)
  if (y > 0) {
    servo1Pos += STEP;     // up
  } else if (y < 0) {
    servo1Pos -= STEP;     // down
  }

  // Left/Right control pan (servo2)
  if (x > 0) {
    servo2Pos += STEP;     // right
  } else if (x < 0) {
    servo2Pos -= STEP;     // left
  }

  servo1Pos = constrain(servo1Pos, 10, 170);
  servo2Pos = constrain(servo2Pos, 10, 170);

  servo1.write(servo1Pos);
  servo2.write(servo2Pos);
}


// ============ SAFETY WATCHDOG ==========================
void checkCommandTimeout()
{
  if (millis() - last_command_time > COMMAND_TIMEOUT) {
    if (target_FL != 0 || target_FR != 0 || target_BL != 0 || target_BR != 0) {
      setMotorTargets(0, 0, 0, 0, 0, 0, 0, 0);
    }
  }
}

// ============ SERIAL COMMAND PARSER ====================
void processLine(char *line)
{
  if (line[0] == 'D') {
    int x, y, z;
    if (sscanf(line + 1, "%d %d %d", &x, &y, &z) == 3) {
      driveFromJoystick(x, y, z);
    }
  }
  else if (line[0] == 'C') {
    int x, y;
    if (sscanf(line + 1, "%d %d", &x, &y) == 2) {
      camFromJoystick(x, y);
    }
  }
  else if (line[0] == 'S') {
    setMotorTargets(0, 0, 0, 0, 0, 0, 0, 0);
    last_command_time = millis();
  }
}

void handleSerial()
{
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      serialBuf[serialPos] = '\0';
      if (serialPos > 0) {
        processLine(serialBuf);
      }
      serialPos = 0;
    } else {
      if (serialPos < sizeof(serialBuf) - 1) {
        serialBuf[serialPos++] = c;
      } else {
        serialPos = 0; // overflow protection
      }
    }
  }
}

// ----------------- SETUP -------------------------------
void setup()
{
  Serial.begin(115200);

  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  pinMode(BL_IN1, OUTPUT); pinMode(BL_IN2, OUTPUT);
  pinMode(BR_IN1, OUTPUT); pinMode(BR_IN2, OUTPUT);

  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW);
  digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, LOW);
  digitalWrite(BR_IN1, LOW); digitalWrite(BR_IN2, LOW);

  pinMode(ENC_FL, INPUT_PULLUP);
  pinMode(ENC_FR, INPUT_PULLUP);
  pinMode(ENC_BL, INPUT_PULLUP);
  pinMode(ENC_BR, INPUT_PULLUP);

  // Init encoder state before enabling PCINT
  last_enc_state = PINC & 0x0F;

  PCICR  |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11);

  setupSoftwarePWM();

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(servo1Pos);
  servo2.write(servo2Pos);
  
  last_command_time = millis();
  Serial.println("Robot controller ready");
}

// ----------------- LOOP: SERIAL + PID + WATCHDOG -------
void loop()
{
  handleSerial();
  updateSpeedControl();
  checkCommandTimeout();
}
