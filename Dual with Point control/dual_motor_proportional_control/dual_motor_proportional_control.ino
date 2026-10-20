#include <SimplyAtomic.h>  // For the position read, to avoid missed counts

// ---------- Motor 1 pins ----------
#define ENCA 0     // interrupt pin encoder A
#define ENCB 1     // interrupt pin encoder B
#define PWM_pin 2  // pin to send pwm value to motor driver
#define IN1 3      // motor driver direction pins
#define IN2 4

// ---------- Motor 2 pins (change these to match your wiring) ----------
#define ENCA_2 11
#define ENCB_2 12
#define PWM_pin_2 13
#define IN3 14
#define IN4 15

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
const int NUM_POINTS = 1000;
const int setpoints[NUM_POINTS] = {
    0, 21, 42, 63, 85, 106, 127, 148, 169, 190,
    211, 233, 254, 275, 296, 317, 338, 359, 381, 402,
    423, 444, 465, 486, 507, 529, 550, 571, 592, 613,
    634, 655, 677, 698, 719, 740, 761, 782, 803, 825,
    846, 867, 888, 909, 930, 951, 973, 994, 1015, 1036,
    1057, 1078, 1099, 1121, 1142, 1163, 1184, 1205, 1226, 1247,
    1269, 1290, 1311, 1332, 1353, 1374, 1395, 1417, 1438, 1459,
    1480, 1501, 1522, 1543, 1565, 1586, 1607, 1628, 1649, 1670,
    1691, 1713, 1734, 1755, 1776, 1797, 1818, 1839, 1861, 1882,
    1903, 1924, 1945, 1966, 1987, 2009, 2030, 2051, 2072, 2096,
    2096, 2072, 2051, 2030, 2009, 1987, 1966, 1945, 1924, 1903,
    1882, 1861, 1839, 1818, 1797, 1776, 1755, 1734, 1713, 1691,
    1670, 1649, 1628, 1607, 1586, 1565, 1543, 1522, 1501, 1480,
    1459, 1438, 1417, 1395, 1374, 1353, 1332, 1311, 1290, 1269,
    1247, 1226, 1205, 1184, 1163, 1142, 1121, 1099, 1078, 1057,
    1036, 1015, 994, 973, 951, 930, 909, 888, 867, 846,
    825, 803, 782, 761, 740, 719, 698, 677, 655, 634,
    613, 592, 571, 550, 529, 507, 486, 465, 444, 423,
    402, 381, 359, 338, 317, 296, 275, 254, 233, 211,
    190, 169, 148, 127, 106, 85, 63, 42, 21, 0
};
int point_index = 0;            // which setpoint we're currently targeting
float time_per_point = 150;    // ms spent on each point
int e_sum = 0; //Integral sum
int ki = 0.4; //Integral control gain

// ---------- Motor 2 variables (independent of motor 1) ----------
volatile int posi_2 = 0;
float ref_2 = 0;
float kp_2 = 1;
int pos_2 = 0;
int e_2 = 0;
int u_amplitude_2 = 0;
float u_f_2 = 0;
int u_max_amplitude_2 = 150;
int u_sign_2 = 0;
long time_start_2 = 0;
const int NUM_POINTS_2 = 200;
const int setpoints_2[NUM_POINTS] = {
    0, 21, 42, 63, 85, 106, 127, 148, 169, 190,
    211, 233, 254, 275, 296, 317, 338, 359, 381, 402,
    423, 444, 465, 486, 507, 529, 550, 571, 592, 613,
    634, 655, 677, 698, 719, 740, 761, 782, 803, 825,
    846, 867, 888, 909, 930, 951, 973, 994, 1015, 1036,
    1057, 1078, 1099, 1121, 1142, 1163, 1184, 1205, 1226, 1247,
    1269, 1290, 1311, 1332, 1353, 1374, 1395, 1417, 1438, 1459,
    1480, 1501, 1522, 1543, 1565, 1586, 1607, 1628, 1649, 1670,
    1691, 1713, 1734, 1755, 1776, 1797, 1818, 1839, 1861, 1882,
    1903, 1924, 1945, 1966, 1987, 2009, 2030, 2051, 2072, 2096,
    2096, 2072, 2051, 2030, 2009, 1987, 1966, 1945, 1924, 1903,
    1882, 1861, 1839, 1818, 1797, 1776, 1755, 1734, 1713, 1691,
    1670, 1649, 1628, 1607, 1586, 1565, 1543, 1522, 1501, 1480,
    1459, 1438, 1417, 1395, 1374, 1353, 1332, 1311, 1290, 1269,
    1247, 1226, 1205, 1184, 1163, 1142, 1121, 1099, 1078, 1057,
    1036, 1015, 994, 973, 951, 930, 909, 888, 867, 846,
    825, 803, 782, 761, 740, 719, 698, 677, 655, 634,
    613, 592, 571, 550, 529, 507, 486, 465, 444, 423,
    402, 381, 359, 338, 317, 296, 275, 254, 233, 211,
    190, 169, 148, 127, 106, 85, 63, 42, 21, 0
};
int point_index_2 = 0;            // which setpoint we're currently targeting
float time_per_point_2 = 150;    // ms spent on each point
int e_sum_2 = 0; //Integral sum
int ki_2 = 1; //Integral control gain

// ---------- Shared ----------
int print_interval = time_per_point;  // how often values are sent to the serial monitor
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
  // Reference changes by set amount every time_per_point milliseconds

  if ((millis() - time_start) > time_per_point) {
  point_index = (point_index + 1) % NUM_POINTS;   // wraps 9 -> 0
  time_start = millis();
  }
  ref = setpoints[point_index];

  ATOMIC() {
    pos = posi;
  }

  e = ref - pos;
  e_sum = e_sum + e; // Integral error sum

  u_sign = 0;
  if (e < 0) u_sign = -1;
  if (e > 0) u_sign = 1;

  u_f = abs(1.0 * kp * e + ki * e_sum);
  u_amplitude = u_f;
  if (u_amplitude > u_max_amplitude) {
    u_amplitude = u_max_amplitude;
  }

  setMotor(u_sign, u_amplitude, PWM_pin, IN1, IN2);

  // ===================== MOTOR 2 =====================
  if ((millis() - time_start_2) > time_per_point_2) {
  point_index_2 = (point_index_2 + 1) % NUM_POINTS_2;   // wraps 9 -> 0
  time_start_2 = millis();
  }
  ref_2 = setpoints_2[point_index_2];

  ATOMIC() {
    pos_2 = posi_2;
  }

  e_2 = ref_2 - pos_2;
  e_sum_2 = e_sum_2 + e_2; // Integral error sum

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
    Serial.print(e_sum);
    Serial.print(" ");
    Serial.print(u_sign * u_amplitude);
    Serial.print(" ");
    Serial.print(time_start);
    Serial.print(" ");
    Serial.print(ref_2);
    Serial.print(" ");
    Serial.print(pos_2);
    Serial.print(" ");
    Serial.print(e_2);
    Serial.print(" ");
    Serial.print(e_sum_2);
    Serial.print(" ");
    Serial.print(u_sign_2 * u_amplitude_2);
    Serial.print(" ");
    Serial.print(time_start_2);
    Serial.print(" ");
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