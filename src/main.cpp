#include <Arduino.h>

int LED = 13;						// sets LED to pin 13
int DelayTime = 1500;				// sets delay time in 1/1000ths of a second

void setup()						// declares setup portion of code
{
	pinMode(LED, OUTPUT);			// sets LED pin as an output
}

void Turn_LED_On()					// declares function to turn LED on
{
	digitalWrite(LED, HIGH);		// brings LED pin HIGH, turning it on
}

void Turn_LED_Off()					// declares function to turn LED off
{
	digitalWrite(LED, LOW);			// brings LED pin LOW, turning it off
}

void Wait_Some()					// declares function for delay
{
	delay(DelayTime);				// pauses for DelayTime in 1/1000s sececond
}

void loop()							// declares loop portion of code
									// this is where the program actually runs
{
	Turn_LED_On();					// turns LED on
	Wait_Some();					// waits some
	Turn_LED_Off();					// turns LED off
	Wait_Some();					// waits some more
}