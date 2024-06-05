#include <Arduino.h>
#include <ServoInput.h>
#include <util/atomic.h>

/************/
/* Approach */
/************
 *
 * 1) Don't listen to throttle, use a rampRate
 *    If throttle translate to a speed higher/lower than the current speed,
 *    in/decrease the speed by rampRate
 *
 * 2) If we exceed an Amperage above X for Y milliseconds, kill the engine
 *    Wait a fixed time to let things cool down and retry.
 *    Keep track of the retry count
 *
 * 3) Check for stalls based on the encoder.
 *    If things are going too slow for too long, kill the engine and retry
 *    Keep track of the retry count
 *    This requries some fine-tuning, which takes time that we don't have...
 *
 * 4) implement PID to achieve the calculated and ramped speed
 *
 */

/**************/
/** CONSTANTS */
/**************/

// Tweakable params
const int MAX_SPEED_RPM = 1000;          // target RPM for "pedal to the metal"
const float overcurrentThreshold = 4.0;  // Example threshold in volts; 5V is 43A
const int stallSpeedThreshold = 10;      // Speed below which motor is considered stalled
const int stallTimeThreshold = 2000;     // Time in milliseconds motor must be stalled to trigger protection
float rampRate = 1.0;                    // Rate of PWM change in speed per loop iteration (units per second)
float Kp = 1.0, Ki = 0.5, Kd = 0.1;      // PID tuning variables
const unsigned long cooldownTime = 5000; // Cooldown time in milliseconds
const int maxRetries = 3;                // Maximum number of recovery attempts
const int motorPin = 9;

/*********************/
/** Global variables */
/*********************/

// Speed variables
float targetSpeedPercentage, maxSpeedPercentage, targetSpeed;

// PID control variables
float setpoint, currentSpeed, output;
float integral = 0, previousError = 0;

// Stall control variables
unsigned long stallStartTime = 0;
bool isStalled = false;

// Overcurrent recovery parameters
int retryCount = 0;
bool isRecovering = false;
unsigned long recoveryStartTime;

/**************/
/** Shortcuts */
/**************/
// Placeholder functions
float readRCsignalPercent() {}
float readMaxSpeedPercentage() {}
float readCurrentSpeed() {}
float readCurrentCurrent() {}
void STOP() {} // Emergency stop. Press reset button to restart
void haltMotor() {} // halt motor before retry
void START() {} 

/*********/
/** Code */
/*********/
void startRecovery() {
  isRecovering = true;
  recoveryStartTime = millis();
}

void startMotor() {
  // Reset PID control
  integral = 0;
  previousError = 0;
}

void setup()
{
    // pin setup
    // Attach interrupts
    setpoint = 0.0; // Start slow
}

void loop()
{

    unsigned long currentTime = millis();

    // Check if in recovery mode
    if (isRecovering)
    {
        if (currentTime - recoveryStartTime >= cooldownTime)
        {
            isRecovering = false;
            retryCount++;
            if (retryCount > maxRetries)
            {
                STOP(); // infinite while
            }
            else
            {
                START();
            }
        }
        return;
    }

    // Read throttle input
    float throttleValue = readRCsignalPercent();
    float desiredSpeed = map(throttleValue, 0, 100, 0, readMaxSpeedPercentage()*MAX_SPEED_RPM);

    // Smoothly adjust the setpoint
    if (setpoint < desiredSpeed)
    {
        setpoint += rampRate; // Increase setpoint gradually
        if (setpoint > desiredSpeed)
            setpoint = desiredSpeed;
    }
    else if (setpoint > desiredSpeed)
    {
        setpoint -= rampRate; // Decrease setpoint gradually
        if (setpoint < desiredSpeed)
            setpoint = desiredSpeed;
    }

    currentSpeed = readCurrentSpeed(); // RPM

    // Check for overcurrent
    float currentVoltage = readCurrentCurrent();
    if (currentVoltage > overcurrentThreshold)
    {
        STOP();
        startRecovery();
        return;
    }

    // Check for stall condition
    if (currentSpeed < stallSpeedThreshold)
    {
        if (!isStalled)
        {
            stallStartTime = millis();
            isStalled = true;
        }
        else if (millis() - stallStartTime > stallTimeThreshold)
        {
            haltMotor();
            startRecovery();
            return;
        }
    }
    else
    {
        isStalled = false;
    }

    // PID Computation
    double error = setpoint - currentSpeed;
    integral += error;
    double derivative = error - previousError;
    output = Kp * error + Ki * integral + Kd * derivative;

    // Limit the output to the motor's acceptable range
    if (output > 255)
        output = 255;
    if (output < 0)
        output = 0;

    analogWrite(motorPin, output); // Assuming motor control via PWM

    previousError = error;

    delay(10); // Loop delay to control the update rate
}
