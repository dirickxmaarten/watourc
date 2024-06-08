/**
 * I'll be quite verbose in my comments and follow Jerry Coffin's advice:
 * "The code is only there to explain the comments to the machine"
 */

/**
 * CODE OUTLINE
 * ------------
 * The best overall approach to reading PWM signals is using Interrupts.
 * That is, stop the loop when receiving a signal and process it. Depending on 
 * the microcontrollere, a number of pins are specifically designed to trigger
 * interrupts on pin state change. In this setup, I am using an Arduino Nano.
 * The atmega328 used on that board has 2 External Interrupt pins. These are the
 * most reliable, but not the only way to interrupt the processor. In case more
 * than 2 channels are required, a pin state change listener must be added to
 * transform state changes into interrupts.
 * 
 * As both are quite common, robust libraries to do this already exists. 
 * * For hardware intterupts this is ServoInput
 *     https://github.com/dmadison/ServoInput
 * * For software interrupts this is PinChangeInterrupt
 *     https://github.com/NicoHood/PinChangeInterrupt
 * 
 * The RC transceiver we will be using has 6 Channels. We only utilize one 
 * channel at this point (forward/stop). If more channels are
 * needed, a different approach is required as well. We'd need to switch from
 * hardware interrupt to software interrupt, and detecting software interrupts
 * requires some finetuning. 
 * 
 * The DBH-1A driver board used to control the motor has it's own library:
 * *  https://github.com/thevolget/DBH1-Controller
 * It is however so rudimentary that we'll implement this ourselves. 
 * 
 * Using the ServoInput library for input and DBH1-Controller for output, all
 * we need to do is write up a converter between the servo PWM and driving 
 * (duty cyle) PWM.
*/


/** DOCS
 *  ----
 * https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm/
 * https://www.partsnotincluded.com/how-to-use-an-rc-controller-with-an-arduino/
*/
/* including the required libraries.*/
#include <Arduino.h>
#include <ServoInput.h> 
#include <util/atomic.h>
// Not needed if there are enoug interrupt pins available
//#include <PinChangeInterrupt.h>

/**************/
/** CONSTANTS */
/**************/

/** Configuration variables */
const int MAX_SPEED_RPM = 1000;			   // Never exceed this RPM speed
const int BATT_VOLTAGE = 18;			   // Voltage of 1 battery
const float MIN_BATT_PERCENTAGE = 3.0/3.6; // cell min voltage divided by normal
const float VOLT_DIV_FACTOR = 47/(4.7+47); // Voltage drop factor by R1 and R2
const int ROT_ENC_SLOTS = 25;			   // Number of slots in the disk
const float RC_CENTER_DEADZONE_PERCENTAGE = 0.05; // 5% around center mapped to 0
const float overcurrentThreshold = 4.0;	   // Example threshold in volts; 5V is 43A
const int stallSpeedThreshold = 10;		   // Speed below which motor is considered stalled
const int stallTimeThreshold = 2000;	   // Time in milliseconds motor must be stalled to trigger protection
float rampRate = 10.0;					   // Rate of PWM change in speed per loop iteration (units per second)
float Kp = 1.0, Ki = 0.5, Kd = 0.1;		   // PID tuning variables
const unsigned long cooldownTime = 5000;   // Cooldown time in milliseconds
const int maxRetries = 3;				   // Maximum number of recovery attempts
const int slotsPerRevolution = 25;		   // Number of slots on the gated disk
const boolean reverseMotor = false;		   // Reverse the motor pins if needed

/** Constants */
const boolean RECOVERABLE=true;
const boolean UNRECOVERABLE=false;


/** PIN layout */
// LEDs
const int PIN_LED_BATT0 = 4;		// OK battery (ORANGE)
const int PIN_LED_BATT1 = 5;		// Full battery (GREEN)
const int PIN_LED_RC = 6;			// RC inverse connection status (RED)
const int PIN_LED_STATUS = 13;		// Status LED = (internal) LED

//DIP switches
const int PIN_DIP_DOUBLE_VOLT = 15;	// DIP4: Switch between 18V and 36V
const int PIN_DIP_LED_OFF = 16;		// DIP3: Don't power status or battery LEDs
const int PIN_DIP_SLEEP = 17;		// DIP2: Sleep mode
const int PIN_DIP_DRY_RUN = 18;		// DIP1: Dry run (don't power the motor)

// Motor Controller
const int PIN_MOT_R_STATUS = A7;	// Current output and status
const int PIN_MOT_L_STATUS = A6;	// Current output and status
const int PIN_MOT_FOR_EN = 12;		// Forward enable pin
const int PIN_MOT_REV_EN = 11;		// Reverse enable pin
const int PIN_MOT_FOR_PWM = 10;		// Forward PWN signal pin (980 Hz)
const int PIN_MOT_REV_PWM = 9;		// Reverse PWN signal pin (980 Hz)

// Interrupts
const int PIN_RC_PWM = 2;			// RC PWM signal
const int PIN_ROT_ENC = 3;			// Rotary encoder tick signal

// OTHER
const int PIN_BATT_VOLT = A0;		// Analog read battery voltage
const int PIN_MAX_SPEED = A5;		// Analog max speed potentiometer




/*********************/
/** Global variables */
/*********************/
// Rotary encoder
volatile unsigned long slotCount = 0;
unsigned long prevRotTimestamp = millis();

// Battery
float minBattVoltage, percentVoltage33, percentVoltage66;

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

// DIP switches setup
bool doubleVoltage = false;
bool cameraMode = false;
bool sleepMode = false;
bool dryRun = false;


// Throttle Setup and init of RC listener
const int ThrottlePulseMin = 1000;  // microseconds (us)
const int ThrottlePulseMax = 2000;  // Ideal values for your servo can be found with the "Calibration" environment
ServoInputPin<PIN_RC_PWM> throttle(ThrottlePulseMin, ThrottlePulseMax);


void rotationCount(){
	slotCount++;
}

void STOP(boolean recoverable, String reason){
    // Stop the motor
    digitalWrite(PIN_MOT_FOR_EN, 0);
    digitalWrite(PIN_MOT_REV_EN, 0);
    digitalWrite(PIN_MOT_FOR_PWM, 0);
    digitalWrite(PIN_MOT_REV_PWM, 0);

	Serial.println(reason);

	if (! recoverable){
		// When maxRetries is exceeded, this is no longer a recoverable stop
		// Send robot to an infinite loop of fast blinking status LED
		// Press the Reset button to exit this state
		// TODO: add an argument for the blink speed depending on stop reason
		while (true) {
			digitalWrite(PIN_LED_STATUS,!digitalRead(PIN_LED_STATUS));
			Serial.println(reason);
			delay(200);
		}
	}
	

}

void awaitRCSignal(){
	// The RC receiver send out a PWM signal as soon as the RC is connected.
	// If there is no PWM signal, there is very little to do. Best we wait.
	// Start an inifinite while loop as long as there is no PWM signal.
	while (!ServoInput.available()) {  
		Serial.println("Waiting for RC signals...");
		// Blink the RC LED. To do this, we toggle it: set it to NOT what it 
		// currently is. The loop will do the rest of the blinking.
		digitalWrite(PIN_LED_RC, !digitalRead(PIN_LED_RC));
		delay(500);
	}
	// Got a signal! We can continue, but first turn off the LED
	digitalWrite(PIN_LED_RC, LOW);
}

float readMaxSpeedPercentage(){
	// Read the potentiometer with an analogRead. To turn it into a percentage,
	// we divide the 0-1023 read signal by its max value: 1023.
	// The value is stored in the global maxSpeedPercantage variable...
	maxSpeedPercentage = analogRead(PIN_MAX_SPEED)/1023.0;
	// ...as well as returned
	return 1.0;
    return maxSpeedPercentage;
}

float readRCsignalPercent(){
	float throttlePercent = throttle.mapDeadzone(-100,100,
	 											RC_CENTER_DEADZONE_PERCENTAGE)
	 											/100.0;
	if (throttlePercent < 0)
	{
		return 0.0;
	}
	return throttlePercent;
}

float readCurrentSpeed() {
	// Calculate speed based on slot count
	unsigned long currentTime = millis();
	unsigned long currentSlotCount = slotCount;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // code with interrupts blocked (consecutive atomic operations will not get interrupted)
        slotCount = 0;
    }
	unsigned long timeElapsed = currentTime - prevRotTimestamp;

	// Reset counters
	prevRotTimestamp = currentTime;



	// Calculate speed in slots per second
	float speed = (currentSlotCount / (float)timeElapsed) * 1000;  // slots per second

	// Convert speed to RPM
	float rpm = (speed / slotsPerRevolution) * 60;  // RPM

	return rpm;
}

float readCurrentCurrent(){
	// TODO: set correct voltage divider
	float currentCurrentL = analogRead(PIN_MOT_L_STATUS)/1023.0*5.0/VOLT_DIV_FACTOR;
	float currentCurrentR = analogRead(PIN_MOT_R_STATUS)/1023.0*5.0/VOLT_DIV_FACTOR;
	return currentCurrentL + currentCurrentR;
}

void startRecovery() {
  isRecovering = true;
  recoveryStartTime = millis();
}

void startMotor() {
  // Reset PID control
  integral = 0;
  previousError = 0;
  digitalWrite(PIN_MOT_FOR_EN, HIGH);
}

boolean BatteryCheck(){
    // Check if voltage is OK
	// Get the analogRead of the battery pin. This is an int on a scale from
	// 0-1023. Divide by 1023 to get a ratio of 0-1. Multiply by 5 for the max
	// voltage that corresponds with "1023", and by 10 for the ratio of our 
	// voltage divider. Finally divide this by the voltage drop coefficient 
	// introduced in the voltage divider: R2/(R1+R2)
	float battVoltage = analogRead(PIN_BATT_VOLT)/1023.0*50.0/VOLT_DIV_FACTOR;

	// debug
	// Serial.println(battVoltage);
	
	// Return false if the voltage is below the minimum voltage 
	// calculated in setup()
	if(battVoltage<minBattVoltage){
		return false;
	}

	if (cameraMode){
		digitalWrite(PIN_LED_BATT0,LOW);
		digitalWrite(PIN_LED_BATT1,LOW);
	} else {
		// Set LEDs: bat0 if above 33% of the difference between max and min voltage
		// batt1 LED if above 66% of difference
		// We can use the implicit boolean to int conversion here
		digitalWrite(PIN_LED_BATT0,battVoltage>percentVoltage33);
		digitalWrite(PIN_LED_BATT1,battVoltage>percentVoltage66);
	}

	// Battery is OK!
	return true;
}

void setup() {
	Serial.begin(9600); // Set up serial communication to PC for debugging

	/** PIN SETUP */
	// LEDs
	pinMode(PIN_LED_BATT0, OUTPUT);
	pinMode(PIN_LED_BATT1, OUTPUT);
	pinMode(PIN_LED_RC, OUTPUT);
	pinMode(PIN_LED_STATUS, OUTPUT);
	// DIP switches with Pull-up resistor
	// Switches are connected to ground -> open = HIGH; closed = LOW
	pinMode(PIN_DIP_DOUBLE_VOLT, INPUT_PULLUP);
	pinMode(PIN_DIP_LED_OFF, INPUT_PULLUP);
	pinMode(PIN_DIP_SLEEP, INPUT_PULLUP);
	pinMode(PIN_DIP_DRY_RUN, INPUT_PULLUP);
	// Motor Controller
	pinMode(PIN_MOT_FOR_EN, OUTPUT);
	pinMode(PIN_MOT_REV_EN, OUTPUT);
	pinMode(PIN_MOT_FOR_PWM, OUTPUT);
	pinMode(PIN_MOT_REV_PWM, OUTPUT);

	// Set DIP switch values
	// We need to NOT the DIP switches when reading. 
	// They are pulled high when open, and go low when closed.
	doubleVoltage = !(bool)digitalRead(PIN_DIP_DOUBLE_VOLT);
	cameraMode = !(bool)digitalRead(PIN_DIP_LED_OFF);
	sleepMode = !(bool)digitalRead(PIN_DIP_SLEEP);
	dryRun = !(bool)digitalRead(PIN_DIP_DRY_RUN);
	// debug
	Serial.println("Double Voltage: " + (String)doubleVoltage);
	Serial.println("Camera Mode: " + (String)cameraMode);
	Serial.println("Sleep Mode: " + (String)sleepMode);
	Serial.println("Dry run: " + (String)dryRun);


	// Do some calculation that won't change during execution
	// Set battery minimum voltage. Multiply by two if the we have 2 batteries 
	// in series, and multiply it by the minimum percentage for an OK battery
	minBattVoltage = (BATT_VOLTAGE << doubleVoltage) * MIN_BATT_PERCENTAGE;
	// Same, but calculate the 33% charge and 66% charge voltages
	percentVoltage33 = ((BATT_VOLTAGE << doubleVoltage) - minBattVoltage ) * 
						0.333333 + minBattVoltage;
	percentVoltage66 = ((BATT_VOLTAGE << doubleVoltage) - minBattVoltage ) * 
						0.666666 + minBattVoltage;

	// debug
	Serial.println("Minimum battery voltage: " + (String)minBattVoltage);
	Serial.println("33% charge battery voltage: " + (String)percentVoltage33);
	Serial.println("66% charge battery voltage: " + (String)percentVoltage66);
	// Give us some time to read these values
	delay(1500);

	// Attach interrupts
	// for the RC using the servoInput class
	throttle.attach();  // attaches the throttle servo input interrupt
	// and for the rotary encoder using a callback: rotationCount
    attachInterrupt(digitalPinToInterrupt (PIN_ROT_ENC), rotationCount, RISING);

	// Wait for RC signal
	awaitRCSignal();

	// Start slow
	setpoint = 0.0;

	// debug
	Serial.println("setup complete, entering loop");
	startMotor();
// void loop() {
// 	int output = (int)(readRCsignalPercent()*255);
// 	Serial.println(output);
// 	analogWrite(PIN_MOT_FOR_PWM, output);
// }

void loop() {
	/************/
	/* Approach */
	/************
	 *
	 * 1) Don't listen to throttle, use a rampRate
	 *    If throttle translates to a speed higher/lower than the current speed,
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

	unsigned long currentTime = millis();

	// Checks
	// Check the battery
	if(!BatteryCheck() && !dryRun){
		Serial.println("Battery Check");
        STOP(UNRECOVERABLE, "Battery check failed");
		
    }

    // Check if in recovery mode
    if (isRecovering)
    {
        if (currentTime - recoveryStartTime >= cooldownTime)
        {
            isRecovering = false;
            retryCount++;
            if (retryCount > maxRetries)
            {
                STOP(UNRECOVERABLE, "Recovery mode failed"); // infinite while
            }
            else
            {
                startMotor();
            }
        }
        return;
    }

	// Read throttle input
    float throttleValue = readRCsignalPercent();
    float desiredSpeed = throttleValue * readMaxSpeedPercentage() * MAX_SPEED_RPM;

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
    float currentAmp = readCurrentCurrent();
    if (currentAmp > overcurrentThreshold)
    {
        STOP(RECOVERABLE, "Over Current!");
        startRecovery();
        return;
    }

    // Check for stall condition
    if (currentSpeed < stallSpeedThreshold && setpoint > 0)
    {
        if (!isStalled)
        {
            stallStartTime = millis();
            isStalled = true;
        }
        else if (millis() - stallStartTime > stallTimeThreshold)
        {
			if(!dryRun){
            STOP(RECOVERABLE, "Stall detected!");
            startRecovery();
            return;
			}
        }
    }
    else
    {
        isStalled = false;
    }

    // PID Computation
	double error = setpoint - currentSpeed;
	if(dryRun){
		error = 0;
	}

    integral += error;
    double derivative = error - previousError;
    output = Kp * error + Ki * integral + Kd * derivative;

    // Limit the output to the motor's acceptable range
    if (output > 255)
        output = 255;
    if (output < 0)
        output = 0;

	if (dryRun){
		// Serial.println("PWM: " + (String)output);
	} else {
    	analogWrite(PIN_MOT_FOR_PWM, output); // Motor control via PWM
	}
	
    previousError = error;

    delay(10); // Loop delay to control the update rate
	
	//debug
	Serial.print("Recovering: ");
	Serial.print(isRecovering);
	Serial.print("\t");
	Serial.print("Stalled: ");
	Serial.print(isStalled);
	Serial.print("\t");
	Serial.print("forward");
	Serial.print(digitalRead(PIN_MOT_FOR_EN));
	Serial.print("\t");
	Serial.print("desired: ");
	Serial.print(desiredSpeed);
	Serial.print("\t");
	Serial.print("sp: ");
	Serial.print(setpoint);
	Serial.print("\t");
	Serial.print("Speed: ");
	Serial.print(currentSpeed);
	Serial.print("\t");
	Serial.print("Amp");
	Serial.print(currentAmp);
	Serial.print("\t");
	Serial.print("out");
	Serial.print(output);
	Serial.print("\t");
	Serial.println();

	// Serial.println("Throttle: " + (String)throttlePercent);
	// Serial.println("Max Speed %: " + (String)maxSpeedPercentage);
	// Serial.println(rotCount);
	// Serial.println("targetSpeed: " + (String)targetSpeed + " MaxSpeedPercentage: " + (String)maxSpeedPercentage + " throttlePercentage: " + (String)throttlePercent);

}
