#include <SimplyAtomic.h>  // For the position read, to avoid missed counts

// --- Util Pins ---
#define BUTTON_PIN 5 // on-off button

// ---------- Motor 1 pins ----------
#define ENCA 0     // interrupt pin encoder A
#define ENCB 1     // interrupt pin encoder B
#define PWM_pin 2  // pin to send pwm value to motor driver
#define IN1 3      // motor driver direction pins
#define IN2 4

// ---------- Motor 2 pins (change these to match your wiring) ---------- (flipped cause its upside down)
#define ENCA_2 11
#define ENCB_2 12
#define PWM_pin_2 13
#define IN3 14
#define IN4 15

// ---------- Motor 1 variables ----------
volatile int posi = 0;     // volatile, integer because counts are discrete
float ref = 0;
float kp = 3;            // proportional gain
int pos = 0;               // output position from the gearbox, in pulses
int e = 0;                 // error between ref and pos
int u_amplitude = 0;       // amplitude of the control signal
float u_f = 0;             // float version of u
int u_max_amplitude = 150; // control signal amplitude (must be < 256)
int u_sign = 0;            // required sign of control signal
long time_start = 0;
const int NUM_POINTS = 1000;
const int setpoints[NUM_POINTS] = {468,467,467,466,466,465,465,464,464,463,463,462,462,461,460,460,459,459,458,458,457,457,456,456,455,455,454,454,453,453,452,452,451,451,450,450,449,449,448,448,447,446,446,445,445,444,444,443,443,442,442,441,441,440,440,439,439,438,438,437,437,436,436,435,434,434,433,433,432,432,431,431,430,430,429,429,428,428,427,427,426,425,425,424,424,423,423,422,422,421,421,420,420,419,419,418,417,417,416,416,415,415,414,414,413,413,412,412,411,411,410,409,409,408,408,407,407,406,406,405,405,404,403,403,402,402,401,401,400,400,399,399,398,397,397,396,396,395,395,394,394,393,392,392,391,391,390,390,389,389,388,387,387,386,386,385,385,384,384,383,382,382,381,381,380,380,379,378,378,377,377,376,376,375,375,374,373,373,372,372,371,371,370,369,369,368,368,367,366,366,365,365,364,364,363,362,362,361,361,360,360,359,358,358,357,357,356,355,355,354,354,353,352,352,351,351,350,349,349,348,348,347,346,346,345,345,344,343,343,342,342,341,340,340,339,339,338,337,337,336,336,335,334,334,333,332,332,331,331,330,329,330,331,332,332,333,334,334,335,336,337,337,338,339,339,340,341,341,342,343,344,344,345,346,346,347,348,348,349,350,350,351,351,352,353,353,354,355,355,356,357,357,358,358,359,360,360,361,362,362,363,363,364,364,365,366,366,367,367,368,369,369,370,370,371,371,372,372,373,374,374,375,375,376,376,377,377,378,378,379,379,380,380,381,381,382,382,383,383,384,384,385,385,386,386,387,387,388,388,388,389,389,390,390,391,391,392,392,392,393,393,394,394,395,395,395,396,396,397,397,397,398,398,398,399,399,400,400,400,401,401,401,402,402,402,403,403,403,404,404,404,405,405,405,406,406,406,407,407,407,407,408,408,408,409,409,409,409,410,410,410,410,411,411,411,411,412,412,412,412,413,413,413,413,413,414,414,414,414,414,415,415,415,415,415,415,416,416,416,416,416,416,417,417,417,417,417,417,417,418,418,418,418,418,418,418,418,418,419,419,419,419,419,419,419,419,419,419,419,419,419,419,419,419,420,420,420,420,420,420,420,420,420,420,420,420,420,420,420,420,420,420,419,419,419,419,419,419,419,419,419,420,421,422,423,424,425,426,427,428,429,430,431,432,433,434,435,436,437,438,439,440,441,442,443,443,444,445,446,447,448,449,450,451,452,453,454,455,456,457,458,459,460,461,462,463,463,464,465,466,467,468,469,470,471,472,473,474,475,476,477,478,478,479,480,481,482,483,484,485,486,487,488,489,490,491,492,492,493,494,495,496,497,498,499,500,501,502,503,504,504,505,506,507,508,509,510,511,512,513,514,515,516,517,517,518,519,520,521,522,523,524,525,526,527,528,529,529,530,531,532,533,534,535,536,537,538,539,540,541,541,542,543,544,545,546,547,548,549,550,551,552,553,553,554,555,556,557,558,559,560,561,562,563,564,565,566,566,567,568,569,570,571,572,573,574,575,576,577,578,579,580,580,581,582,583,584,585,586,587,588,589,590,591,592,593,594,595,595,596,597,598,599,600,601,602,603,604,605,606,607,608,609,610,611,611,612,613,614,615,616,617,618,619,620,621,622,623,624,625,626,627,628,628,629,630,631,632,633,634,635,636,637,638,639,640,641,642,643,644,645,646,647,648,648,649,650,651,652,653,654,654,654,655,655,655,655,655,655,655,655,655,655,655,655,655,655,655,656,656,656,656,656,656,656,656,656,656,656,656,656,656,656,656,656,656,656,656,655,655,655,655,655,655,655,655,655,655,655,655,655,655,654,654,654,654,654,654,654,654,653,653,653,653,653,653,653,652,652,652,652,652,651,651,651,651,651,650,650,650,650,649,649,649,649,648,648,648,648,647,647,647,646,646,646,645,645,645,644,644,644,643,643,642,642,642,641,641,640,640,640,639,639,638,638,637,637,636,636,635,635,634,634,633,633,632,632,631,631,630,630,629,628,628,627,627,626,625,625,624,623,623,622,621,621,620,619,618,618,617,616,615,615,614,613,612,611,611,610,609,608,607,606,605,604,603,603,602,601,600,599,598,597,596,595,594,593,591,590,589,588,587,586,585,584,582,581,580,579,578,576,575,574,572,571,570,569,567,566,564,563,562,560,559,557,556,554,553,551,550,548,546,545,543,542,540,538,536,535,533,531,529,528,526,524,522,520,518,516,514,512,510,508,506,504,502,500,498,496,493,491,489,487,484,482,480,477,475,473,470
};
int point_index = 0;            // which setpoint we're currently targeting
float time_per_point = 5;    // ms spent on each point
float e_sum = 0; //Integral sum
float ki = 3; //Integral control gain
const float i_max = 30;
float kd = 0.12;     // start small, tune up
int e_prev = 0;
float d_term = 0;    // filtered de/dt (counts/s)

// ---------- Motor 2 variables (independent of motor 1) ----------
volatile int posi_2 = 0;
float ref_2 = 0;
float kp_2 = 3;
int pos_2 = 0;
int e_2 = 0;
int u_amplitude_2 = 0;
float u_f_2 = 0;
int u_max_amplitude_2 = 150;
int u_sign_2 = 0;
long time_start_2 = 0;
const int NUM_POINTS_2 = 1000;
const int setpoints_2[NUM_POINTS] = {924,923,922,921,919,918,917,916,915,914,913,911,910,909,908,907,906,905,904,902,901,900,899,898,897,896,894,893,892,891,890,889,888,886,885,884,883,882,881,880,878,877,876,875,874,873,872,870,869,868,867,866,865,863,862,861,860,859,858,857,855,854,853,852,851,850,848,847,846,845,844,843,841,840,839,838,837,836,834,833,832,831,830,829,827,826,825,824,823,822,820,819,818,817,816,814,813,812,811,810,809,807,806,805,804,803,801,800,799,798,797,795,794,793,792,791,789,788,787,786,785,783,782,781,780,779,777,776,775,774,773,771,770,769,768,766,765,764,763,762,760,759,758,757,755,754,753,752,751,749,748,747,746,744,743,742,741,739,738,737,736,734,733,732,731,729,728,727,726,724,723,722,721,719,718,717,716,714,713,712,711,709,708,707,706,704,703,702,700,699,698,697,695,694,693,691,690,689,688,686,685,684,682,681,680,678,677,676,675,673,672,671,669,668,667,665,664,663,661,660,659,657,656,655,653,652,651,649,648,647,645,644,643,641,640,638,637,636,634,633,632,630,629,628,626,625,623,622,621,619,618,617,617,617,616,616,615,615,614,614,613,613,613,612,612,611,611,610,610,609,609,608,608,607,607,606,606,605,604,604,603,603,602,602,601,601,600,600,599,598,598,597,597,596,596,595,594,594,593,593,592,591,591,590,590,589,588,588,587,586,586,585,584,584,583,582,582,581,580,580,579,578,578,577,576,576,575,574,574,573,572,571,571,570,569,569,568,567,566,566,565,564,563,563,562,561,560,560,559,558,557,556,556,555,554,553,552,552,551,550,549,548,548,547,546,545,544,543,543,542,541,540,539,538,537,536,536,535,534,533,532,531,530,529,528,528,527,526,525,524,523,522,521,520,519,518,517,516,515,514,514,513,512,511,510,509,508,507,506,505,504,503,502,501,500,499,498,497,496,494,493,492,491,490,489,488,487,486,485,484,483,482,481,480,478,477,476,475,474,473,472,471,469,468,467,466,465,464,463,461,460,459,458,457,456,454,453,452,451,450,448,447,446,445,443,442,441,440,438,437,436,435,433,432,431,429,428,427,426,424,423,422,420,419,418,416,415,414,412,411,409,408,407,405,404,403,401,400,398,397,395,397,398,400,401,403,404,405,407,408,409,411,412,414,415,416,418,419,420,422,423,424,426,427,428,429,431,432,433,435,436,437,438,440,441,442,443,445,446,447,448,450,451,452,453,454,456,457,458,459,460,461,463,464,465,466,467,468,469,471,472,473,474,475,476,477,478,480,481,482,483,484,485,486,487,488,489,490,491,492,493,494,496,497,498,499,500,501,502,503,504,505,506,507,508,509,510,511,512,513,514,514,515,516,517,518,519,520,521,522,523,524,525,526,527,528,528,529,530,531,532,533,534,535,536,536,537,538,539,540,541,542,543,543,544,545,546,547,548,548,549,550,551,552,552,553,554,555,556,556,557,558,559,560,560,561,562,563,563,564,565,566,566,567,568,569,569,570,571,571,572,573,574,574,575,576,576,577,578,578,579,580,580,581,582,582,583,584,584,585,586,586,587,588,588,589,590,590,591,591,592,593,593,594,594,595,596,596,597,597,598,598,599,600,600,601,601,602,602,603,603,604,604,605,606,606,607,607,608,608,609,609,610,610,611,611,612,612,613,613,613,614,614,615,615,616,616,617,617,617,618,619,621,622,623,625,626,628,629,630,632,633,634,636,637,638,640,641,643,644,645,647,648,649,651,652,653,655,656,657,659,660,661,663,664,665,667,668,669,671,672,673,675,676,677,678,680,681,682,684,685,686,688,689,690,691,693,694,695,697,698,699,700,702,703,704,706,707,708,709,711,712,713,714,716,717,718,719,721,722,723,724,726,727,728,729,731,732,733,734,736,737,738,739,741,742,743,744,746,747,748,749,751,752,753,754,755,757,758,759,760,762,763,764,765,766,768,769,770,771,773,774,775,776,777,779,780,781,782,783,785,786,787,788,789,791,792,793,794,795,797,798,799,800,801,803,804,805,806,807,809,810,811,812,813,814,816,817,818,819,820,822,823,824,825,826,827,829,830,831,832,833,834,836,837,838,839,840,841,843,844,845,846,847,848,850,851,852,853,854,855,857,858,859,860,861,862,863,865,866,867,868,869,870,872,873,874,875,876,877,878,880,881,882,883,884,885,886,888,889,890,891,892,893,894,896,897,898,899,900,901,902,904,905,906,907,908,909,910,911,913,914,915,916,917,918,919,921,922,923
};
int point_index_2 = 0;            // which setpoint we're currently targeting
float time_per_point_2 = 5;    // ms spent on each point
float e_sum_2 = 0; //Integral sum
float ki_2 = 3; //Integral control gain
const float i_max_2 = 30;
float kd_2 = 0.12;
int e_prev_2 = 0;
float d_term_2 = 0;

// ---------- Shared ----------
int print_interval = 1000;  // how often values are sent to the serial monitor
int interval_count = 0;
float counts_per_rotation = 131.25 * 16;
float ref_amplitude = counts_per_rotation;      // motor 1 target amplitude
float ref_amplitude_2 = counts_per_rotation;    // motor 2 target amplitude

const unsigned long Ts_us = 2000;  // derivative sample period (2 ms)
const float d_alpha = 0.2;         // filter: smaller = smoother, laggier
unsigned long t_prev_us = 0;

// ---------- Homing / state machine ----------
enum RigState { ST_IDLE, ST_HOMING, ST_HOMED, ST_RUNNING, ST_PAUSED };
RigState state = ST_IDLE;                   // starts idle (motors off)

const int U_MAX_RUN  = 150;                 // normal amplitude limit
const int U_MAX_HOME = 50;                  // slow amplitude limit while homing
const int HOME_TOLERANCE = 5;               // encoder counts
const unsigned long HOME_SETTLE_MS = 5000;  // both motors must stay in tolerance this long
unsigned long home_in_band_since = 0;
const unsigned long debounce_ms = 30;

void setup() {
  Serial.begin(230400);  // baud rate for communication between USB & raspberry pi pico

  // button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

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

void loop() {
  
  printState();

  // ===================== BUTTON / STATE MACHINE =====================
  if (buttonPressed()) {
    switch (state) {
      case ST_IDLE:                       // press 1: start homing
        point_index   = 0;
        point_index_2 = 0;
        setLimits(U_MAX_HOME);
        resetController();
        home_in_band_since = millis();
        state = ST_HOMING;
        Serial.println("HOMING");
        break;

      case ST_HOMING:                     // ignored until both motors have settled
        break;                            // (set state = ST_IDLE here if you want a press to abort)

      case ST_HOMED:                      // press 2: run the shape
        setLimits(U_MAX_RUN);             // controller is already running, so no reset needed
        time_start   = millis();
        time_start_2 = millis();
        state = ST_RUNNING;
        Serial.println("RUNNING");
        break;

      case ST_RUNNING:
        state = ST_PAUSED;
        Serial.println("PAUSED");
        break;

      case ST_PAUSED:                     // resume from the current point
        time_start   = millis();
        time_start_2 = millis();
        resetController();
        state = ST_RUNNING;
        Serial.println("RUNNING");
        break;
    }
  }

  if (state == ST_IDLE || state == ST_PAUSED) {
    setMotor(0, 0, PWM_pin,   IN1, IN2);   // motors off
    setMotor(0, 0, PWM_pin_2, IN3, IN4);
    return;                                // skip control and printing
  }

  // ===================== DERIVATIVE / INTEGRAL TIMING =====================
  bool d_update = false;
  float dt = 0;
  unsigned long now_us = micros();
  if (now_us - t_prev_us >= Ts_us) {
    dt = (now_us - t_prev_us) / 1e6;
    t_prev_us = now_us;
    d_update = true;
  }

  // ===================== MOTOR 1 =====================
  // Setpoint only advances while the shape is running; homing/homed hold point 0
  if (state == ST_RUNNING && (millis() - time_start) > time_per_point) {
    point_index++;
    time_start = millis();
    if (point_index >= NUM_POINTS) {
      state = ST_IDLE;     // stop after one pass; next press re-homes
      point_index   = 0;
      point_index_2 = 0;
      return;              // the idle block switches the motors off next loop
    }
  }
  ref = setpoints[point_index];

  ATOMIC() {
    pos = posi;
  }

  e = ref - pos;

  if (d_update) {
    float d_raw = (e - e_prev) / dt;
    d_term = d_alpha * d_raw + (1 - d_alpha) * d_term;
    e_prev = e;

    e_sum += e * dt;
    if (ki > 0) e_sum = constrain(e_sum, -i_max / ki, i_max / ki);
  }

  u_f = (kp * e + ki * e_sum + kd * d_term);

  u_sign = 0;
  if (u_f < 0) u_sign = -1;
  if (u_f > 0) u_sign = 1;

  u_amplitude = abs(u_f);
  if (u_amplitude > u_max_amplitude) {
    u_amplitude = u_max_amplitude;
  }

  setMotor(u_sign, u_amplitude, PWM_pin, IN1, IN2);

  // ===================== MOTOR 2 =====================
  if (state == ST_RUNNING && (millis() - time_start_2) > time_per_point_2) {
    point_index_2 = (point_index_2 + 1) % NUM_POINTS_2;   // wraps
    time_start_2 = millis();
  }
  ref_2 = setpoints_2[point_index_2];

  ATOMIC() {
    pos_2 = posi_2;
  }

  e_2 = ref_2 - pos_2;

  if (d_update) {
    float d_raw_2 = (e_2 - e_prev_2) / dt;
    d_term_2 = d_alpha * d_raw_2 + (1 - d_alpha) * d_term_2;
    e_prev_2 = e_2;

    e_sum_2 += e_2 * dt;
    if (ki_2 > 0) e_sum_2 = constrain(e_sum_2, -i_max_2 / ki_2, i_max_2 / ki_2);
  }

  u_f_2 = (kp_2 * e_2 + ki_2 * e_sum_2 + kd_2 * d_term_2);

  u_sign_2 = 0;
  if (u_f_2 < 0) u_sign_2 = -1;
  if (u_f_2 > 0) u_sign_2 = 1;

  u_amplitude_2 = abs(u_f_2);
  if (u_amplitude_2 > u_max_amplitude_2) {
    u_amplitude_2 = u_max_amplitude_2;
  }

  setMotor(u_sign_2, u_amplitude_2, PWM_pin_2, IN3, IN4);

  // ===================== HOMING SETTLE CHECK =====================
  if (state == ST_HOMING) {
    bool in_band = abs(e) <= HOME_TOLERANCE && abs(e_2) <= HOME_TOLERANCE;
    if (!in_band) {
      home_in_band_since = millis();                    // left the band: restart the 5 s
    } else if (millis() - home_in_band_since > HOME_SETTLE_MS) {
      state = ST_HOMED;
      Serial.println("HOMED - press button to run shape");
    }
  }

  // ===================== PRINTING =====================
  interval_count = interval_count + 1;
  if (interval_count >= print_interval) {
    interval_count = 0;

    // PWM contribution of each term (before the saturation clamp)
    float p_pwm   = kp * e;
    float i_pwm   = ki * e_sum;
    float d_pwm   = kd * d_term;
    float p_pwm_2 = kp_2 * e_2;
    float i_pwm_2 = ki_2 * e_sum_2;
    float d_pwm_2 = kd_2 * d_term_2;

    printField("t", (int)millis());

    Serial.print("| M1 ");
    printField("ref_1", ref);
    printField("pos_1", pos);
    printField("e_1", e);
    printField("P_1", p_pwm);
    printField("I_1", i_pwm);
    printField("D_1", d_pwm);
    printField("u_1", u_sign * u_amplitude);

    Serial.print("| M2 ");
    printField("ref_2", ref_2);
    printField("pos_2", pos_2);
    printField("e_2", e_2);
    printField("P_2", p_pwm_2);
    printField("I_2", i_pwm_2);
    printField("D_2", d_pwm_2);
    printField("u_2", u_sign_2 * u_amplitude_2);

    Serial.println();
  }
}

// Cap the control amplitude of both motors
void setLimits(int limit) {
  u_max_amplitude   = limit;
  u_max_amplitude_2 = limit;
}

// Clear controller memory (same as your old "fresh start" block)
void resetController() {
  t_prev_us = micros();
  e_sum = 0;    e_sum_2 = 0;
  d_term = 0;   d_term_2 = 0;

  ATOMIC() { pos = posi; pos_2 = posi_2; }
  e_prev   = setpoints[point_index]     - pos;
  e_prev_2 = setpoints_2[point_index_2] - pos_2;
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

bool buttonPressed() {
  static bool stable = HIGH;              // last accepted state
  static bool last_reading = HIGH;
  static unsigned long last_change = 0;

  bool reading = digitalRead(BUTTON_PIN);
  if (reading != last_reading) {          // contact moved: restart the timer
    last_change = millis();
    last_reading = reading;
  }
  if ((millis() - last_change) > debounce_ms && reading != stable) {
    stable = reading;
    if (stable == LOW) return true;       // accepted press (pull-up: LOW = pressed)
  }
  return false;
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

// Print "label=value " (int and float versions)
void printField(const char* label, int value) {
  Serial.print(label);
  Serial.print("=");
  Serial.print(value);
  Serial.print(" ");
}

void printField(const char* label, float value) {
  Serial.print(label);
  Serial.print("=");
  Serial.print(value);
  Serial.print(" ");
}

// Prints the current state once a second, independent of the control loop
void printState() {
  static unsigned long last_print = 0;
  if (millis() - last_print < 1000) return;
  last_print = millis();

  Serial.print("state=");
  switch (state) {
    case ST_IDLE:    Serial.println("IDLE");    break;
    case ST_HOMING:  Serial.println("HOMING");  break;
    case ST_HOMED:   Serial.println("HOMED");   break;
    case ST_RUNNING: Serial.println("RUNNING"); break;
    case ST_PAUSED:  Serial.println("PAUSED");  break;
  }
}