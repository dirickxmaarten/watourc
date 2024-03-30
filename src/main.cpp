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
 * 
 * 
*/

#include <Arduino.h>
#include <ServoInput.h>
/*#include <PinChangeInterrupt.h>*/


int LED = 13;						// sets LED to pin 13
int DelayTime = 1500;				// sets delay time in 1/1000ths of a second



ServoInputPin<2> servo;

void setup() {
	Serial.begin(115200);
	servo.attach();  // attaches the servo input interrupt

	while (servo.available() == false) {
		Serial.println("Waiting for servo signal...");
		delay(500);
	}
}

void loop() {
	float angle = servo.getAngle();  // get angle of servo (0 - 180)
	Serial.println(angle);
}