/*! \class	Motor

    \brief	Motor class for AVR32UC3B offers

		This class uses the PWM class and controls some GPIO-Pins to provide motor control.
		The motor speed can be set between 0 and 255, but is limited to Configuration::PWM_maxPWMRatio.
		The motor direction can be set to forward or backward.
		The motors can be enabled and disabled, which means enabling/disabling the H Bridges of all motors.
		
		In this class "speed" is the same as "PWM": a value from 0 to 255 representing the PWM ratio.
		
		Attention: all motors share the same enable pin.
		Attention: when class is destroyed or cleanUp() is called, the enable pin will be uninititalized.
*/

#ifndef MOTOR_H_
#define MOTOR_H_

#include "../Configuration/Configuration.h"
#include "PWM.h"

class Motor {
private:

    Configuration::s_MotorConfig motorconfig;
    unsigned char LastValidSpeed;
    PWM pwmobjekt;

    void Motor_CleanUp();
public:
	
	static void initEnablePin();
	static void setEnabled( bool enabled );
	static bool getIsEnabled();
	
	Motor();
	~Motor();
	bool init( Configuration::s_MotorConfig* thisMotorConfig_ );
	bool setSpeed( unsigned char ratioOn );
	unsigned char getSpeed();
	void setDirection( bool forward );
};

#endif /* MOTOR_H_ */
