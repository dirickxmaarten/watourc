#include <Arduino.h>
#include <ServoInput.h> 
#include <util/atomic.h>


// Motor Controller
const int PIN_MOT_R_STATUS = A7;	// Current output and status 1A = 27 -> 30
const int PIN_MOT_L_STATUS = A6;	// Current output and status
const int PIN_MOT_FOR_EN = 12;		// Forward enable pin
const int PIN_MOT_REV_EN = 11;		// Reverse enable pin
const int PIN_MOT_FOR_PWM = 10;		// Forward PWN signal pin (980 Hz)
const int PIN_MOT_REV_PWM = 9;		// Reverse PWN signal pin (980 Hz)

int counter, direction;
void setup() {
	Serial.begin(9600); // Set up serial communication to PC for debugging

	// Motor Controller
	pinMode(PIN_MOT_FOR_PWM, OUTPUT);
	pinMode(PIN_MOT_REV_PWM, OUTPUT);
	pinMode(PIN_MOT_FOR_EN, OUTPUT);
	pinMode(PIN_MOT_REV_EN, OUTPUT);
    digitalWrite(PIN_MOT_FOR_EN, HIGH);
	digitalWrite(PIN_MOT_REV_EN, HIGH);
    digitalWrite(PIN_MOT_REV_PWM,LOW);
    counter = 0;
    direction = 1;
    counter = 255;
    analogWrite(PIN_MOT_FOR_PWM,counter);
    delay(2000);
}

void loop() {
    

    // if (counter == 255){
    //     direction = -1;
    // };
    // if (counter == 0){
    //     direction = 1;
    // };
    if (analogRead(PIN_MOT_R_STATUS) > 30){
        counter = 0;
    }
    analogWrite(PIN_MOT_FOR_PWM,counter);
    Serial.print("cntr: ");
    Serial.print(counter);
    Serial.print("\t");
    Serial.print("Lcur: ");
    Serial.print(analogRead(PIN_MOT_R_STATUS));
    Serial.print("\t");
    Serial.println();
    // Serial.print("Rcur: ");

    delay(25);


}