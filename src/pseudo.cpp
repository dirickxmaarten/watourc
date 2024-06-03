#include <Arduino.h>
#include <ServoInput.h> 
#include <util/atomic.h>
/**************/
/** CONSTANTS */
/**************/

const int MAX_SPEED_RPM = 1000;


// Motor Controller
const int PIN_MOT_R_STATUS = A7;	// Current output and status
const int PIN_MOT_L_STATUS = A6;	// Current output and status
const int PIN_MOT_FOR_EN = 8;		// Forward enable pin
const int PIN_MOT_REV_EN = 7;		// Reverse enable pin
const int PIN_MOT_FOR_PWM = 6;		// Forward PWN signal pin
const int PIN_MOT_REV_PWM = 5;		// Reverse PWN signal pin

// Interrupts
const int PIN_ROT_ENC = 3;			// RC PWM signal

// OTHER
const int PIN_MAX_SPEED = A5;		// Analog max speed potentiometer


/** Universal constants */



/*********************/
/** Global variables */
/*********************/
volatile uint16_t rotCount = 0;
volatile int rotTickTimestamp = 0;
uint16_t prevRotCount = 0;
int prevRotTickTimestamp = micros();

float targetSpeedPercentage = 0;
float maxSpeedPercentage = 0;
float targetSpeed = 0;

void rotationCount(){
    rotCount++;
    rotTickTimestamp = micros();
}

float readMaxSpeed(){
    // Read analog potentiometer for max speed
}

float getRCsignalPercent(){
    // Get RC signal as a percentage from -1.00 to 1.00
}

void setup() {
	Serial.begin(115200); // Set up serial communication to PC for debugging

    // Rot Enc
    attachInterrupt(digitalPinToInterrupt (PIN_ROT_ENC), rotationCount, RISING);

    // PIN setup

}



void loop() {

    /** Speed control */
    targetSpeedPercentage = getRCsignalPercent();  // 
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
