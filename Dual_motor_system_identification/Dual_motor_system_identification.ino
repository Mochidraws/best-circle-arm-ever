#include <SimplyAtomic.h>

// ========================================================
// MOTOR IDENTIFICATION TEST
//
// Select motor:
//      1 = Motor 1
//      2 = Motor 2
//
// Press button to begin each test.
// After each test the motor stops.
// Reposition arm manually.
// Press button for next PWM level.
//
// Output format:
// time_ms,pwm,encoder_count,velocity_counts_per_sec
// ========================================================

#define TEST_MOTOR 1

// ========================================================
// BUTTON
// ========================================================

#define BUTTON_PIN 5

// ========================================================
// MOTOR SELECTION
// ========================================================

#if TEST_MOTOR == 1

#define ENCA     0
#define ENCB     1

#define PWM_PIN  2
#define IN_A     3
#define IN_B     4

#elif TEST_MOTOR == 2

#define ENCA     11
#define ENCB     12

#define PWM_PIN  13
#define IN_A     14
#define IN_B     15

#endif

// ========================================================
// TEST SETTINGS
// ========================================================

const int pwmLevels[] = {50,100,150};

const int NUM_TESTS = 3;

const unsigned long TEST_DURATION_MS = 3000;

volatile long encoderCount = 0;

// ========================================================
// TEST STATE
// ========================================================

int testIndex = 0;

bool runningTest = false;

unsigned long testStartTime = 0;

unsigned long lastLogTime = 0;

long previousCount = 0;
unsigned long previousVelocityTime = 0;

// ========================================================
// BUTTON DEBOUNCE
// ========================================================

bool buttonPressed()
{
    static bool stable = HIGH;
    static bool lastReading = HIGH;
    static unsigned long lastChange = 0;

    bool reading = digitalRead(BUTTON_PIN);

    if(reading != lastReading)
    {
        lastChange = millis();
        lastReading = reading;
    }

    if((millis() - lastChange) > 30)
    {
        if(reading != stable)
        {
            stable = reading;

            if(stable == LOW)
            {
                return true;
            }
        }
    }

    return false;
}

// ========================================================
// SETUP
// ========================================================

void setup()
{
    Serial.begin(230400);

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);

    pinMode(PWM_PIN, OUTPUT);

    pinMode(IN_A, OUTPUT);
    pinMode(IN_B, OUTPUT);

    attachInterrupt(
        digitalPinToInterrupt(ENCA),
        readEncoder,
        RISING);

    Serial.println();
    Serial.println("=================================");
    Serial.println("Motor Identification Test");
    Serial.println("Press button to start PWM = 50");
    Serial.println("=================================");
    Serial.println();

    Serial.println("time_ms,pwm,encoder_count,velocity_counts_per_sec");
}

// ========================================================
// LOOP
// ========================================================

void loop()
{
    // ----------------------------------
    // START NEXT TEST
    // ----------------------------------

    if(!runningTest &&
       testIndex < NUM_TESTS &&
       buttonPressed())
    {
        runningTest = true;

        testStartTime = millis();

        previousVelocityTime = 0;

        Serial.println();
        Serial.print("# Starting PWM = ");
        Serial.println(pwmLevels[testIndex]);
    }

    // ----------------------------------
    // RUN TEST
    // ----------------------------------

    if(runningTest)
    {
        int currentPWM = pwmLevels[testIndex];

        setMotor(1,currentPWM);

        if(millis() - lastLogTime >= 10)
        {
            lastLogTime = millis();

            long count;

            ATOMIC()
            {
                count = encoderCount;
            }

            float velocity = 0.0f;

            if(previousVelocityTime != 0)
            {
                float dt =
                    (millis() - previousVelocityTime)
                    / 1000.0f;

                if(dt > 0)
                {
                    velocity =
                        (count - previousCount)
                        / dt;
                }
            }

            previousCount = count;
            previousVelocityTime = millis();

            Serial.print(millis());
            Serial.print(",");

            Serial.print(currentPWM);
            Serial.print(",");

            Serial.print(count);
            Serial.print(",");

            Serial.println(velocity);
        }

        // ------------------------------
        // END TEST
        // ------------------------------

        if(millis() - testStartTime
           >= TEST_DURATION_MS)
        {
            runningTest = false;

            setMotor(0,0);

            Serial.println();
            Serial.print("# Finished PWM = ");
            Serial.println(pwmLevels[testIndex]);

            testIndex++;

            if(testIndex < NUM_TESTS)
            {
                Serial.println();
                Serial.print(
                    "# Reposition arm then press button for PWM = ");

                Serial.println(
                    pwmLevels[testIndex]);
            }
            else
            {
                Serial.println();
                Serial.println(
                    "# ALL TESTS COMPLETE");
            }
        }
    }
}

// ========================================================
// MOTOR DRIVER
// ========================================================

void setMotor(int direction,
              int pwmValue)
{
    analogWrite(PWM_PIN,pwmValue);

    if(direction > 0)
    {
        digitalWrite(IN_A,HIGH);
        digitalWrite(IN_B,LOW);
    }
    else if(direction < 0)
    {
        digitalWrite(IN_A,LOW);
        digitalWrite(IN_B,HIGH);
    }
    else
    {
        digitalWrite(IN_A,LOW);
        digitalWrite(IN_B,LOW);
    }
}

// ========================================================
// ENCODER ISR
// ========================================================

void readEncoder()
{
    int b = digitalRead(ENCB);

    if(b > 0)
    {
        encoderCount++;
    }
    else
    {
        encoderCount--;
    }
}