#include <Arduino.h>
#include <util/atomic.h>

/** CHANGEABLE PARAMETERS */
int PWM_speed = 255;
const int ROT_ENC_SLOTS = 25; // Number of slots in the disk
const int slotsToEmergency = 220;
const unsigned long millisToStop = 300000; // 5 minutes
//const unsigned long millisToStop = 80000; // 80 seconds

/** PIN SETUP */
// Buttons
const int PIN_BUT_L = 19;  // Max Position switch Right
const int PIN_BUT_R = 2;   // Max Position switch Left
const int PIN_ROT_ENC = 3; // Rotary encoder tick signal

// Motor Controller
const int PIN_MOT_R_STATUS = A7; // Current output and status 1A = 27 -> 30
const int PIN_MOT_L_STATUS = A6; // Current output and status 1A = 33
const int PIN_MOT_FOR_EN = 12;   // Forward enable pin
const int PIN_MOT_REV_EN = 11;   // Reverse enable pin
const int PIN_MOT_FOR_PWM = 10;  // Forward PWN signal pin (980 Hz)
const int PIN_MOT_REV_PWM = 9;   // Reverse PWN signal pin (980 Hz)

// LEDs
const int PIN_LED_BATT0 = 4;   // OK battery (ORANGE)
const int PIN_LED_BATT1 = 5;   // Full battery (GREEN)
const int PIN_LED_RC = 6;      // RC inverse connection status (RED)
const int PIN_LED_STATUS = 13; // Status LED = (internal) LED

// PIR sensor
const int PIN_PIR_SIG = 7;

/** GLOBAL VARIABLES */
bool reverse;
volatile uint16_t slotCount;
int currentAverage = 0;
unsigned long lastMovement;

/** FUNCTIONS */
// Move the motor forward by adjusting the PWM signal
// The motor-enable pins are high all the time
void moveForward()
{
    // First set the other direction to low to stop the motor
    // only then we set a new PWM for the right direction
    analogWrite(PIN_MOT_REV_PWM, LOW);
    delay(100);
    analogWrite(PIN_MOT_FOR_PWM, PWM_speed);
}
void moveBackward()
{
    analogWrite(PIN_MOT_FOR_PWM, LOW);
    delay(100);
    analogWrite(PIN_MOT_REV_PWM, PWM_speed);
}

void pauseMotor()
{
    // Stop the motor
    digitalWrite(PIN_MOT_FOR_PWM, 0);
    digitalWrite(PIN_MOT_REV_PWM, 0);
}

void stopMotor()
{
    digitalWrite(PIN_MOT_REV_EN, 0);
    digitalWrite(PIN_MOT_FOR_EN, 0);
    pauseMotor();

}

void STOP(int blinkDelay, String reason)
{
    // Stop the motor
    stopMotor();

    Serial.println(reason);

    while (true)
    {
        digitalWrite(PIN_LED_STATUS, !digitalRead(PIN_LED_STATUS));
        delay(blinkDelay);
    }
}

void rotationCount()
{
    slotCount++;
}

/** LOOP */
void setup()
{
    Serial.begin(9600); // Set up serial communication to PC for debugging
    Serial.println("MillisToStop: " + (String)millisToStop);
    // Motor Controller
    pinMode(PIN_MOT_FOR_PWM, OUTPUT);
    pinMode(PIN_MOT_REV_PWM, OUTPUT);
    pinMode(PIN_MOT_FOR_EN, OUTPUT);
    pinMode(PIN_MOT_REV_EN, OUTPUT);
    digitalWrite(PIN_MOT_FOR_EN, HIGH);
    digitalWrite(PIN_MOT_REV_EN, HIGH);
    digitalWrite(PIN_MOT_FOR_PWM, LOW);
    digitalWrite(PIN_MOT_REV_PWM, LOW);

    // Buttons
    pinMode(PIN_BUT_L, INPUT_PULLUP);
    pinMode(PIN_BUT_R, INPUT_PULLUP);

    // LEDs
    pinMode(PIN_LED_BATT0, OUTPUT);
    pinMode(PIN_LED_BATT1, OUTPUT);
    pinMode(PIN_LED_RC, OUTPUT);
    pinMode(PIN_LED_STATUS, OUTPUT);

    // RotEnc
    pinMode(PIN_ROT_ENC, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_ROT_ENC), rotationCount, RISING);

    // PIR sensor
    pinMode(PIN_PIR_SIG, INPUT);
    for (int i = 0; i <= 40; i++) { // Wait 40 seconds to tune PIR sensor
        delay(1000);
        Serial.println("Tuning PIR sensor for 40 s: " + (String)i);
    }
    reverse = true;
    slotCount = 0;
    lastMovement = 0;
}

void loop()
{
    // Over-Current protection
    // Charger is rated at 10A
    // 1A corresponds to a analog read of '33'
    // We stop at 15A => 15*33
    int current = analogRead(PIN_MOT_L_STATUS);
    currentAverage = (15*currentAverage + current)/16;
    if (current > (15 * 33))
    {
        STOP(200, "over-current");
    }

    if (slotCount > slotsToEmergency)
    {
        STOP(500, "too many slots");
    }
    // Buttons
    // We're NOT working with interrupts
    // We just read the push button status during the loop
    // No debouncing. Triggered once is enough.
    // First we check if buttons are pressed and set our direction accordingly
    if (!digitalRead(PIN_BUT_L))
    {
        reverse = false;
        digitalWrite(PIN_LED_BATT0, HIGH);
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            // code with interrupts blocked (consecutive atomic operations will not get interrupted)
            slotCount = 0;
        }
    }
    else
    {
        digitalWrite(PIN_LED_BATT0, LOW);
    }
    if (!digitalRead(PIN_BUT_R))
    {
        reverse = true;
        digitalWrite(PIN_LED_BATT1, HIGH);
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            // code with interrupts blocked (consecutive atomic operations will not get interrupted)
            slotCount = 0;
        }
    }
    else
    {
        digitalWrite(PIN_LED_BATT1, LOW);
    }

    // PIR sensor
    if(digitalRead(PIN_PIR_SIG)){
        lastMovement = millis();
    }
    
    if((millis() - lastMovement) > millisToStop){
        pauseMotor();
        Serial.println("No motion detected");
        delay(25);
        return;
    }
    // implicit else: lastMovement was less than millisToStop ago.

    // Now we can move the motor
    if (reverse)
    {
        moveBackward();
        Serial.println("Forward\tCurrent: " + (String)current + "\tAvg: "+(String)currentAverage + "\tcount: " +(String)slotCount + "\tLM: " + (String)lastMovement + "\tmillis: " + (String)millis());
    }
    else
    {
        moveForward();
        Serial.println("Backward\tCurrent: " + (String)current + "\tAvg: "+(String)currentAverage + "\tcount: " +(String)slotCount + "\tLM: " + (String)lastMovement + "\tmillis: " + (String)millis());
    }

    delay(25);
}