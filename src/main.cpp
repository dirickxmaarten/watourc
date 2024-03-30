/**
 * I'll be quite verbose in my comments and follow Jerry Coffin's advice:
 * "The code is only there to explain the comments to the machine"
 * 
 * GOAL
 * ----
 * This microcontroller code is written to make a big robot move. It does this 
 * by getting 
 * signals from a typical hobby RC transceiver, and translating those to 
 * motor movement.  As this is potentialy dangerous, several fail safes are in 
 * place. Additionaly, there is are some status lights.
 * 
 * INPUT
 * -----
 * Most RC remote controls have self-centering levers. One forward/reverse, and 
 * another for left/right. This mimics a basic joystick and can be seen as such.
 * The receiver yields a 50hz Pulse Width Modulation signal. Of the 20ms pulse,
 * the HIGH signal only last between 1 and 2 milliseconds. The default pulse is
 * 1.5ms. One millisecond means full reverse (or Left), two milliseconds is full
 *  forward (or Right).
 * 
 * OUTPUT
 * ------
 * To be specified 
 * #TODO
 * 
 * FAILSAFE
 * --------
 * The most important fail-safe is the transceiver. If the control PWM signal 
 * disappears, all movement must halt immediately. This is also tied to a 
 * status LED. Furthermore, there is an Emergency Stop button on the robot that 
 * cuts power if something does go wrong. This physical button is NOT part of 
 * the code.
 * 
*/

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