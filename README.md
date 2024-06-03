# Watou Vooruitgang RC
## GOAL
This Arduino Nano microcontroller code is written to make a big robot move. It does this by getting signals from a typical hobby RC transceiver, and translating those to motor movement. As this is potentially dangerous, several failsafes are in place. Additionally, there are some status lights.

## Failsafes
The most important fail-safe is the transceiver. If the control PWM signal disappears, all movement must halt immediately. This is also tied to a status LED. Furthermore, there is an Emergency Stop button on the robot that cuts power if something does go wrong. This physical button is NOT part of the code and MUST be implemented in your designs! Finally there is an over-current protection for the motor driver. The battery voltage is monitored as well. This prevents us from draining the batteries to an unrecoverable state (hopefully). A second LED is used for battery status.

## CLOSED LOOP SPEED CONTROL

We'll implement a close loop speed control. This means we pick up values from an optical gate driven by a slotted wheel. This allows us to variate the PWM load based on the actual speed.

## INPUT

### RC
Most RC remote controls have self-centering levers. One forward/reverse, and another for left/right. This mimics a basic joystick and can be seen as such. The receiver yields a 50hz Pulse Width Modulation signal. Of the 20ms pulse, the HIGH signal only last between 1 and 2 milliseconds. The default pulse is 1.5ms. One millisecond means full reverse (or Left), two milliseconds is full forward (or Right). That is the guideline. Some calibration is required.
See calibrate.cpp

### Rotary encoder
A light gate rotary encoder is used to keep track of the motor's position. In essence it's a ticking device. You count the ticks, you count the time and you end up with the speed.

### Motor driver power consumption
Our motor driver board has a readout of the used power. This will be used to protect our driver from over-current. We implemented a voltage divider to and a low-pass filter to translate it to smooth voltage between 0 and 5V.

### DIP switches
Several DIP switches are present to select modes and other settings.
These are:
- DIP1: regular vs breathing mode

### Max speed potentiometer
To limit max speed during testing, we added a 10k potentiometer. The readout of the signal limits the max speed.

### Battery level
The battery level (voltage) is monitored as well. We're using Lithium-Ion batteries. They can be damaged when drained too much. A voltage divider with a factor of 10 brings the voltage from up to 50V to 5V.

### Reset button for Arduino  nano
A basic reset button for the Arduino code

## OUTPUT
There is only 1 motor on our robot: forward! This is controlled with a dual H-bridge board: BTS9760 (BTS-2). There are several status LED.

### Motor driver
The motor driver is controlled over PWM as well. The accepted frequency is between 0 and 25kHz. We'll be using the default (audible) 980Hz our Arduino offers on pin 5 and 6. 
### Status LEDs
- RC status LED: blinks when there is no RC connection
- Battery voltage 1: full charge (66% above min voltage)
- Battery voltage 2: half charge (33% above min voltage)
- Status LED: two functions shown by blink sequence
	- low battery: slow blink, robot in Emergency mode
	- Over-current: fast blink, robot in Emergency mode

# Schematics
The schematics are on https://crcit.net/c/63727c09a5674d3b95523fb6a8062823