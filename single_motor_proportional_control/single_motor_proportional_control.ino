#include <SimplyAtomic.h>  // For the position read, to avoid missed counts

// ---------- Motor 1 pins ----------
#define ENCA 0     // interrupt pin encoder A
#define ENCB 1     // interrupt pin encoder B
#define PWM_pin 2  // pin to send pwm value to motor driver
#define IN1 3      // motor driver direction pins
#define IN2 4

// ---------- Motor 2 pins (change these to match your wiring) ----------
#define ENCA_2 5
#define ENCB_2 6
#define PWM_pin_2 7
#define IN3 8
#define IN4 9

// ---------- Motor 1 variables ----------
volatile int posi = 0;     // volatile, integer because counts are discrete
float ref = 0;
float kp = 0.9;            // proportional gain
int pos = 0;               // output position from the gearbox, in pulses
int e = 0;                 // error between ref and pos
int u_amplitude = 0;       // amplitude of the control signal
float u_f = 0;             // float version of u
int u_max_amplitude = 150; // control signal amplitude (must be < 256)
int u_sign = 0;            // required sign of control signal
long time_start = 0;
int rotations = 0;
float time_per_rotation = 5000;  // time allowed per rotation, in milliseconds

// ---------- Motor 2 variables (independent of motor 1) ----------
volatile int posi_2 = 0;
float ref_2 = 0;
float kp_2 = 0.9;
int pos_2 = 0;
int e_2 = 0;
int u_amplitude_2 = 0;
float u_f_2 = 0;
int u_max_amplitude_2 = 150;
int u_sign_2 = 0;
long time_start_2 = 0;
int rotations_2 = 0;
float time_per_rotation_2 = 5000;  // can differ from motor 1

// ---------- Shared ----------
int print_interval = 4000;  // how often values are sent to the serial monitor
int interval_count = 0;
float counts_per_rotation = 131.25 * 16;
float ref_amplitude = counts_per_rotation;      // motor 1 target amplitude
float ref_amplitude_2 = counts_per_rotation;    // motor 2 target amplitude

void setup() {
  Serial.begin(230400);  // baud rate for communication between USB & raspberry pi pico

  // Motor 1
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  pinMode(PWM_pin, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Motor 2
  pinMode(ENCA_2, INPUT);
  pinMode(ENCB_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_2), readEncoder2, RISING);
  pinMode(PWM_pin_2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  time_start = millis();
  time_start_2 = millis();
}

// This loop implements proportional control for both motors
void loop() {

  // ===================== MOTOR 1 =====================
  // Reference changes by +/-360 degrees every time_per_rotation milliseconds
  if ((millis() - time_start) > time_per_rotation) {
    rotations = 1 - rotations;
    time_start = millis();
  }
  ref = ref_amplitude * rotations;

  ATOMIC() {
    pos = posi;
  }

  e = ref - pos;

  u_sign = 0;
  if (e < 0) u_sign = -1;
  if (e > 0) u_sign = 1;

  u_f = abs(1.0 * kp * e);
  u_amplitude = u_f;
  if (u_amplitude > u_max_amplitude) {
    u_amplitude = u_max_amplitude;
  }

  setMotor(u_sign, u_amplitude, PWM_pin, IN1, IN2);

  // ===================== MOTOR 2 =====================
  if ((millis() - time_start_2) > time_per_rotation_2) {
    rotations_2 = 1 - rotations_2;
    time_start_2 = millis();
  }
  ref_2 = ref_amplitude_2 * rotations_2;

  ATOMIC() {
    pos_2 = posi_2;
  }

  e_2 = ref_2 - pos_2;

  u_sign_2 = 0;
  if (e_2 < 0) u_sign_2 = -1;
  if (e_2 > 0) u_sign_2 = 1;

  u_f_2 = abs(1.0 * kp_2 * e_2);
  u_amplitude_2 = u_f_2;
  if (u_amplitude_2 > u_max_amplitude_2) {
    u_amplitude_2 = u_max_amplitude_2;
  }

  setMotor(u_sign_2, u_amplitude_2, PWM_pin_2, IN3, IN4);

  // ===================== PRINTING =====================
  // Columns: time, ref1, pos1, e1, u1, ref2, pos2, e2, u2
  interval_count = interval_count + 1;
  if (interval_count >= print_interval) {
    interval_count = 0;
    Serial.print(millis());
    Serial.print(" ");
    Serial.print(ref);
    Serial.print(" ");
    Serial.print(pos);
    Serial.print(" ");
    Serial.print(e);
    Serial.print(" ");
    Serial.print(u_sign * u_amplitude);
    Serial.print(" ");
    Serial.print(ref_2);
    Serial.print(" ");
    Serial.print(pos_2);
    Serial.print(" ");
    Serial.print(e_2);
    Serial.print(" ");
    Serial.print(u_sign_2 * u_amplitude_2);
    Serial.println();
  }
}

// Send signal to motor driver (shared by both motors)
void setMotor(int dir, float pwmVal, int pwm_pin, int in1, int in2) {
  analogWrite(pwm_pin, pwmVal);  // 0 - 255
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

// Interrupt service routine, motor 1
void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  } else {
    posi--;
  }
}

// Interrupt service routine, motor 2
void readEncoder2() {
  int b = digitalRead(ENCB_2);
  if (b > 0) {
    posi_2++;
  } else {
    posi_2--;
  }
}