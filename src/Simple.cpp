#include <Arduino.h>
#include <ServoInput.h> 
#include <util/atomic.h>

// Rate of change
int rampRate = 1;

// Motor Controller
const int PIN_MOT_R_STATUS = A7;	// Current output and status 1A = 27 -> 30
const int PIN_MOT_L_STATUS = A6;	// Current output and status 1A = 33 
const int PIN_MOT_FOR_EN = 12;		// Forward enable pin
const int PIN_MOT_REV_EN = 11;		// Reverse enable pin
const int PIN_MOT_FOR_PWM = 10;		// Forward PWN signal pin (980 Hz)
const int PIN_MOT_REV_PWM = 9;		// Reverse PWN signal pin (980 Hz)

// RC
const int PIN_RC_PWM = 2;			// RC PWM signal

// Rotary
const int PIN_ROT_ENC = 3;			// Rotary encoder tick signal

const float RC_CENTER_DEADZONE_PERCENTAGE = 0.05; // 5% around center mapped to 0
int counter, direction, output; 

// Throttle Setup and init of RC listener
const int ThrottlePulseMin = 1000;  // microseconds (us)
const int ThrottlePulseMax = 2000;  // Ideal values for your servo can be found with the "Calibration" environment
ServoInputPin<PIN_RC_PWM> throttle(ThrottlePulseMin, ThrottlePulseMax);

int readRCsignalPWM(){
	int throttlePWM = throttle.mapDeadzone(-255,255,
	 											RC_CENTER_DEADZONE_PERCENTAGE);
    
	if (throttlePWM < 0)
	{
		return 0.0;
	}
	return throttlePWM;
}

void setup() {
	Serial.begin(9600); // Set up serial communication to PC for debugging

	// Motor Controller
	pinMode(PIN_MOT_FOR_PWM, OUTPUT);
	pinMode(PIN_MOT_REV_PWM, OUTPUT);
	pinMode(PIN_MOT_FOR_EN, OUTPUT);
	pinMode(PIN_MOT_REV_EN, OUTPUT);
    digitalWrite(PIN_MOT_FOR_EN, HIGH);
	digitalWrite(PIN_MOT_REV_EN, HIGH);
    digitalWrite(PIN_MOT_FOR_PWM,LOW);
    throttle.attach(); 

    output = 0;
    // analogWrite(PIN_MOT_REV_PWM,counter);
    // delay(2000);
    
}

void loop() {
    
    if (analogRead(PIN_MOT_L_STATUS) > 810){
        counter = 0;
    }
    int throttlePWM = readRCsignalPWM();
    // smoothRC= (30*smoothRC + throttlePercent)/31;

    // Smoothly adjust the setpoint
    if (output < throttlePWM)
    {
        output += rampRate; // Increase setpoint gradually
    }
    else if (output > throttlePWM)
    {
        output -= rampRate*3; // Decrease setpoint gradually
        if (output < 0 ){
            output = 0;
        }
    }
    
    
    
    
    
    analogWrite(PIN_MOT_REV_PWM,output);



    Serial.println(output);
    // Serial.print("cntr: ");
    // Serial.print(counter);
    // Serial.print("\t");
    // Serial.print("Lcur: ");
    // Serial.print(analogRead(PIN_MOT_L_STATUS));
    // Serial.print("\t");
    // Serial.print("out: ");
    // Serial.print(output);
    // Serial.print("\t");
    // Serial.print("throttle: ");
    // Serial.print(throttlePercent);
    // Serial.print("\t");
    // Serial.print("smoothRC: ");
    // Serial.print(smoothRC);
    // Serial.print("\t");
    // Serial.println();
    // Serial.print("Rcur: ");
    


    delay(25);


}