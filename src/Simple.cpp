#include <Arduino.h>
#include <ServoInput.h> 
#include <util/atomic.h>

// Rate of change
int rampRate = 1;

// LEDs
const int PIN_LED_STATUS = 13;		// Status LED = (internal) LED
const int PIN_LED_RC = 6;			// RC inverse connection status (RED)



// Motor Controller
const int PIN_MOT_R_STATUS = A7;	// Current output and status 1A = 27 -> 30
const int PIN_MOT_L_STATUS = A6;	// Current output and status 1A = 33 
const int PIN_MOT_FOR_EN = 12;		// Forward enable pin
const int PIN_MOT_REV_EN = 11;		// Reverse enable pin
const int PIN_MOT_FOR_PWM = 10;		// Forward PWN signal pin (980 Hz)
const int PIN_MOT_REV_PWM = 9;		// Reverse PWN signal pin (980 Hz)

// RC
const int PIN_RC_PWM = 2;			// RC PWM signal

// Battery
const int PIN_LED_BATT0 = 4;		// OK battery (ORANGE)
const int PIN_LED_BATT1 = 5;		// Full battery (GREEN)
const int BATT_VOLTAGE = 18;			   // Voltage of 1 battery
const float MIN_BATT_PERCENTAGE = 3.0/3.6; // cell min voltage divided by normal
const float VOLT_DIV_FACTOR = 47/(4.7+47); // Voltage drop factor by R1 and R2
float minBattVoltage, percentVoltage33, percentVoltage66;
const int PIN_BATT_VOLT = A0;		// Analog read battery voltage
float weightedBatteryVoltage = BATT_VOLTAGE;


// Rotary
const int PIN_ROT_ENC = 3;			// Rotary encoder tick signal

const float RC_CENTER_DEADZONE_PERCENTAGE = 0.05; // 5% around center mapped to 0
int counter, direction, output; 
int maxTicksForOverCurrent = 5;

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

void STOP(){
    // Stop the motor
    digitalWrite(PIN_MOT_FOR_EN, 0);
    digitalWrite(PIN_MOT_REV_EN, 0);
    digitalWrite(PIN_MOT_FOR_PWM, 0);
    digitalWrite(PIN_MOT_REV_PWM, 0);


    while (true) {
        digitalWrite(PIN_LED_STATUS,!digitalRead(PIN_LED_STATUS));
        delay(200);
    }
	

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
    weightedBatteryVoltage = (30*weightedBatteryVoltage+battVoltage)/31;
	if(weightedBatteryVoltage<minBattVoltage){
		return false;
	}
    Serial.println(weightedBatteryVoltage);

    // Set LEDs: bat0 if above 33% of the difference between max and min voltage
    // batt1 LED if above 66% of difference
    // We can use the implicit boolean to int conversion here
    digitalWrite(PIN_LED_BATT0,battVoltage>percentVoltage33);
    digitalWrite(PIN_LED_BATT1,battVoltage>percentVoltage66);

	// Battery is OK!
	return true;
}

void setup() {
	Serial.begin(9600); // Set up serial communication to PC for debugging


	// Motor Controller
	pinMode(PIN_MOT_FOR_PWM, OUTPUT);
	pinMode(PIN_MOT_REV_PWM, OUTPUT);
	pinMode(PIN_MOT_FOR_EN, OUTPUT);
	pinMode(PIN_MOT_REV_EN, OUTPUT);

    pinMode(PIN_LED_STATUS, OUTPUT);
    pinMode(PIN_LED_RC, OUTPUT);
    pinMode(PIN_LED_BATT0, OUTPUT);
	pinMode(PIN_LED_BATT1, OUTPUT);


    digitalWrite(PIN_MOT_FOR_EN, HIGH);
	digitalWrite(PIN_MOT_REV_EN, HIGH);
    digitalWrite(PIN_MOT_FOR_PWM,LOW);

	// Do some calculation that won't change during execution
	// Set battery minimum voltage. Multiply by two if the we have 2 batteries 
	// in series, and multiply it by the minimum percentage for an OK battery
	minBattVoltage = BATT_VOLTAGE * MIN_BATT_PERCENTAGE;
	// Same, but calculate the 33% charge and 66% charge voltages
	percentVoltage33 = (BATT_VOLTAGE - minBattVoltage ) * 
						0.333333 + minBattVoltage;
	percentVoltage66 = (BATT_VOLTAGE - minBattVoltage ) * 
						0.666666 + minBattVoltage;

	// debug
	Serial.println("Minimum battery voltage: " + (String)minBattVoltage);
	Serial.println("33% charge battery voltage: " + (String)percentVoltage33);
	Serial.println("66% charge battery voltage: " + (String)percentVoltage66);

    Serial.println("Current Voltage: " + (String)(analogRead(PIN_BATT_VOLT)/1023.0*50.0/VOLT_DIV_FACTOR));

    delay(1500);

    throttle.attach(); 

    output = 0;
    // delay(2000);
    
}

void loop() {
    int current = analogRead(PIN_MOT_L_STATUS);
    if (current > 810){
        counter++;
        if(counter >= maxTicksForOverCurrent){
            STOP();
        }
    } else {
        counter = 0;
    }

    if (!BatteryCheck()){
        STOP();
    }

    int throttlePWM = readRCsignalPWM();

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
    analogWrite(PIN_LED_RC, output);



    // Serial.println(current);
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