#include <SimplyAtomic.h>  // For the position read, to avoid missed counts

// Define Encoder pins, pwm pin, and motor driver pins
#define ENCA 0     // interrupt pin encoder A
#define ENCB 1     // interrupt pin encoder B
#define PWM_pin 2  // pin to send pwm value to motor driver
#define IN1 3      // motor driver direction pins
#define IN2 4

volatile int posi = 0;     // specify posi as volatile, integer because counts are discrete
int print_interval = 4000; // define how often values are sent to the serial monitor
int interval_count = 0;
float interval_start = 0;
float ref = 0;
float kp = 0.9;         // proportional gain 
int pos = 0;            // the angular position of the output from the gearbox, in pulses
int e = 0;              // the error between ref and pos
int u_amplitude = 0;    // the amplitude of the control signal
float u_f = 0;          // float version of u
int u_max_amplitude = 150;  // control signal amplitude (must be < 256)
int u_sign = 0;         // used to specify required sign of control signal
long time_start = 0;
float counts_per_rotation = 131.25 * 16;
float ref_amplitude = counts_per_rotation;
int rotations = 0;
float time_per_rotation = 5000;  // time allowed per rotation, in milliseconds

unsigned long last_print = 0;
const unsigned long print_interval_ms = 100;


void setup() {
  Serial.begin(230400);  // set baud rate for communication between USB & raspberry pi pico
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  // pwm and direction pins are outputs that go to the motor
  pinMode(PWM_pin, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  time_start = millis();
}

// This loop implements proportional code
void loop() {

  // Set a reference angle position for the motor. It will be in counts, so conversion is needed to degrees for output.
  // Will change by +/-360 degrees every time_per_rotation milliseconds
  if ((millis() - time_start) > time_per_rotation) {
    rotations = 1-rotations;
    time_start = millis();
  }

  ref = ref_amplitude * rotations;

  ATOMIC() {  // lines between these brackets are executed even if an interrupt occurs
    pos = posi;
  }

  // calculate position error
  e = ref - pos;

  // calculate sign of control signal
  u_sign = 0;
  if (e < 0) {
    u_sign = -1;
  }
  if (e > 0) {
    u_sign = 1;
  }

  u_f = abs(1.0 * kp * e);
  u_amplitude = u_f;
  if (u_amplitude > u_max_amplitude) {
    u_amplitude = u_max_amplitude;
  }  

  // call setMotor function to drive the motor at a set direction and torque
  setMotor(u_sign, u_amplitude, PWM_pin, IN1, IN2);

  // print target and position to see the response every print_interval times around the loop
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
    Serial.print(0);
    Serial.print(" ");
    Serial.println();
  }
}

// function to send signal to motor driver (could also be defined at top of code)
// set inputs and their classifications
void setMotor(int dir, float pwmVal, int pwm_pin, int in1, int in2) {
  //set the pwm pin to the appropriate level (0 - 255)
  analogWrite(pwm_pin, pwmVal);
  //set direction
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

// Interrupt service routine 
void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  } else {
    posi--;
  }
}
