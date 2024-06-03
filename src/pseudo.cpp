#include <Arduino.h>
#include <ServoInput.h> 
#include <util/atomic.h>
/**************/
/** CONSTANTS */
/**************/

/** Configuration variables */
const int BATT_VOLTAGE = 18; // Voltage of 1 battery
const float MIN_BATT_PERCENTAGE = 3.0/3.6; // cell min voltage devided by normal
const int MAX_SPEED_RPM = 1000;


/** PIN layout */
// LEDs
const int PIN_LED_BATT1 = 9;		// Low battery (RED)
const int PIN_LED_BATT2 = 10;		// OK battery (ORANGE)
const int PIN_LED_BATT3 = 11;		// Full battery (GREEN)
const int PIN_LED_RC = 12;			// RC inverse connection status (RED)
const int PIN_LED_STATUS = 13;		// Status LED = (internal) LED

//DIP switches
const int PIN_DIP_VOLT_SELECT = A1;	// Switch between 12V and 36V
const int PIN_DIP_LED_OFF = A2;		// Don't power status or battery LEDs
const int PIN_DIP_SLEEP = A3;			// Sleep mode
const int PIN_DIP_DRY_RUN = A4;			// Dry run (don't power the motor)

// Motor Controller
const int PIN_MOT_R_STATUS = A7;	// Current output and status
const int PIN_MOT_L_STATUS = A6;	// Current output and status
const int PIN_MOT_FOR_EN = 8;		// Forward enable pin
const int PIN_MOT_REV_EN = 7;		// Reverse enable pin
const int PIN_MOT_FOR_PWM = 6;		// Forward PWN signal pin
const int PIN_MOT_REV_PWM = 5;		// Reverse PWN signal pin

// Interrupts
const int PIN_RC_PWM = 2;			// RC PWM signal
const int PIN_ROT_ENC = 3;			// RC PWM signal

// OTHER
const int PIN_BATT_VOLT = A0;		// Analog read battery voltage
const int PIN_MAX_SPEED = A5;		// Analog max speed potentiometer


/** Universal constants */



/*********************/
/** Global variables */
/*********************/
volatile uint16_t rotCount = 0;
volatile int rotTickTimestamp = 0;
uint16_t prevRotCount = 0;
int prevRotTickTimestamp = micros();

float minBattVoltage = 0;

float targetSpeedPercentage = 0;
float maxSpeedPercentage = 0;
float targetSpeed = 0;


ServoInputPin<PIN_RC_PWM> RCsignal;

void STOP(){
    // Stop the motor
    digitalWrite(PIN_MOT_FOR_EN, 0);
    digitalWrite(PIN_MOT_REV_EN, 0);
    digitalWrite(PIN_MOT_FOR_PWM, 0);
    digitalWrite(PIN_MOT_REV_PWM, 0);

    // Send robot to an infinite loop of fast blinking status LED
    //TODO add an argument for the blink speed depending on stop reason
    while(true){
        digitalWrite(PIN_LED_STATUS,HIGH);
        delay(200);
        digitalWrite(PIN_LED_STATUS,LOW);
        delay(200);
    }

}

boolean BatteryCheck(){
    //Check if voltage is OK
    //Set battery LED's accordingly
}

void rotationCount(){
    rotCount++;
    rotTickTimestamp = micros();
}

float readMaxSpeed(){
    // Read analog potentiometer for max speed
}

void setup() {
	Serial.begin(115200); // Set up serial communication to PC for debugging

    // RC
    RCsignal.attach();
	while (!RCsignal.available()) {  // if no signal -> wait
		Serial.println("Waiting for servo signals...");
		delay(500);
	}

    // Rot Enc
    attachInterrupt(digitalPinToInterrupt (PIN_ROT_ENC), rotationCount, RISING);

    // PIN setup

}



void loop() {
    /** Safety checks */

	//check battery
    if(!BatteryCheck()){
        STOP();
    }

    if(!RCsignal.available()){
        STOP();
    }


    /** Speed control */
    targetSpeedPercentage = RCsignal.getPercent();  // RC pwm percentage
    maxSpeedPercentage = readMaxSpeed();  // Get maxSpeed from pot as % of 5V
    targetSpeed = targetSpeedPercentage * maxSpeedPercentage; 
    

    //TODO do magic to work out desired PWM
    // Based on targetSpeed and 

    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // code with interrupts blocked (consecutive atomic operations will not get interrupted)
        prevRotCount = rotCount++;
        rotCount = 0;
        prevRotTickTimestamp = rotTickTimestamp;
    }


}
