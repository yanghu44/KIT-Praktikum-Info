/*! \class	Segway

    \brief	Contains main segway functionality as there are initialization and the controlling algorithm.

		This class creates and initializes helper objects of sensor, motor and communication classes. Then, it
		uses these objects to execute the segway controlling algorithm.
		Debug data is provided via bluetooth using the UART class.
*/
#ifndef SEGWAY_H_
#define SEGWAY_H_

#include <cstring>
#include "../Timer/Timer.h"
#include "../Antrieb/Motor.h"
#include "../Sensor/ADC.h"
#include "../Sensor/ADCSensor.h"
#include "../Sensor/GPIOSensor.h"

#define IR_FREQUENCY												100 									//IR-Frequenzy = 100 Hz
#define BATTERYCHECKENABLE											1										// 1 - batterycheck enabled, 0 - disabled

#define SYSCTL_RUN_CLOCK_GATING_CONTROL_REGISTER					0x400FE108
#define GPIO_PORTF_DIR_REGISTER										0x40025400
#define GPIO_PORTF_DIGITAL_ENABLE_REGISTER							0x4002551C
#define GPIO_PORTF_DATA_REGISTER									0x400253FC

int rs;
int ls;
int c ;


class Segway {
private:

	//! Object of Timer class to initialize the timer.
	Timer myTimer;
	//! Object of ADC class as a helper for the ADCSensor objects.
	ADC myADC;
	//! Sensor object for the foot switch to detect if a person is standing on the segway.
	GPIOSensor footSwitchSensor;
	//! Sensor object for the accelerometer for balancing.
	ADCSensor orientationAccelerometer;
	//! Sensor object for the gyrometer for balancing.
	ADCSensor orientationGyrometer;
	//! Sensor object for the gyrometer's reference value.
	ADCSensor orientationGyrometerReference;
	//! Sensor object for the potentiometer measuring the steering angle.
	ADCSensor steeringPotentiometer;
	//! Sensor object for measuring battery voltage.
	ADCSensor batteryVoltageSensor;
	//! Representation of the left motor.
	Motor leftMotor;
	//! Representation of the right motor.
	Motor rightMotor;
	
	// Class Functions
	void initHelpers();
	void setCurrentSteeringPositionAsZero();
	void setCurrentOrientationAsZero();
	void setCurrentGyroPositionZero();
	void cleanUp();
	
	void displayError( unsigned char errorCode );
	
	// control algorithm functions
	long expMovingAverage( long oldVal, long newVal, unsigned char weightOfNewVal_permille );
	float expMovingAverage( float oldVal, float newVal, unsigned char weightOfNewVal_permille );
	long limitSteeringPWM( long steeringPotentiometerValue, long driveSum );
	void setMotorSpeed( Motor* motor, long speed );
	float limitSpeedSoft( long driveSum );
	
	// other functions
	void doBatteryCheck();
	void waitForFootSwitchReleased();
	void initStatusLED( Configuration::s_StatusLED* statusLEDConfig );
	void setStatusLED( Configuration::s_StatusLED* statusLEDConfig, bool on );
	
	// Segway State Variables
	long
		//! Variable used for integration. Proportional to the motor's common PWM.
		driveSum,
		//! Array used for calculating an average over the segway's speed for speed limitation.
		//! \sa	limitSpeedSoft
		AVGSpeedLimitDriveSumArray[200],
		//! Array index containing the current position in AVGSpeedLimitDriveSumArray.
		//! \sa	limitSpeedSoft, AVGSpeedLimitDriveSumArray
		AVGSpeedLimitDriveSumArray_index;
		
	unsigned long
		//! Timeout counter when foot switch is released.
		footSwitchTimeout,
		//! Timeout counter to block steering after foot switch was pressed to simplify stepping onto the segway.
		blockSteeringTimeout;
		//! Counter for realizing a reasonable debug data sending rate.
		// debugDataCounter;
		
	float
		//! Current tilt angle of the segway against calibration position. Calculated by a complementary filter.
		CFTiltAngle;
		
	bool
		//! To store if steering is blocked after foot switch was pressed to simplify stepping onto the segway.
		steeringBlocked;
		
	enum {_standby, _run}
		//! Operation mode of the controlling algorithm.
		operationMode;
		

	/*!
	\brief The number of seconds between two timer interrupts
	*/
	static const float TIMER_DELTA_T_SECONDS;

	/*!
	\brief Maximum PWM supported by the PWM class
	*/
	static const unsigned char PWMCLASS_MAX_PWM = 255;

	/*!
	\brief Minimum battery voltage in mV. If voltage is lower than this, nothing happens if the foot switch is pressed for initialization.
	*/
	static const unsigned long MIN_BATTERY_VOLTAGE = 22000;

	/*
		Control algorithm configuration
	*/
	/*!
	\brief Hard speed limit: Maximum PWM (Percent) (excluding steering PWM)
	*/
	static const unsigned char DRIVESPEED_PWM_MAX_PERCENT = 60;

	/*!
	\brief Soft speed limit configuration

	Soft speed limit: Segway is leaning against driver, starting from zero degrees at ALGCFG_SPEEDLIMIT_START_PWM_PERCENT percent PWM to
	ALGCFG_SPEEDLIMIT_LEAN_BACK_MAX_DEGREES degrees at ALGCFG_SPEEDLIMIT_END_PWM_PERCENT percent PWM.
	*/
	static const unsigned char ALGCFG_SPEEDLIMIT_START_PWM_PERCENT = 10;
	/*!
	\brief Soft speed limit configuration

	Soft speed limit: Segway is leaning against driver, starting from zero degrees at ALGCFG_SPEEDLIMIT_START_PWM_PERCENT percent PWM to
	ALGCFG_SPEEDLIMIT_LEAN_BACK_MAX_DEGREES degrees at ALGCFG_SPEEDLIMIT_END_PWM_PERCENT percent PWM.
	*/
	static const unsigned char ALGCFG_SPEEDLIMIT_END_PWM_PERCENT = 20;
	/*!
	\brief Soft speed limit configuration

	Soft speed limit: Segway is leaning against driver, starting from zero degrees at ALGCFG_SPEEDLIMIT_START_PWM_PERCENT percent PWM to
	ALGCFG_SPEEDLIMIT_LEAN_BACK_MAX_DEGREES degrees at ALGCFG_SPEEDLIMIT_END_PWM_PERCENT percent PWM.
	*/
	static const unsigned char ALGCFG_SPEEDLIMIT_LEAN_BACK_MAX_DEGREES = 45;


	/*!
	\brief Number of timer iterations after foot switch is released in run mode before motors are turned off.
	Value: timer frequency is 100Hz, so there's a timeout of 1 second.
	*/
	static const unsigned long ALGCFG_FOOTSWITCHTIMEOUT_CRITICAL = 50;

	/*!
	\brief Weighting factor for the complementary filter to calculate the vehicle's angle
	*/
	static const float ALGCFG_COMPL_FILTER_ACCELANGLE_FACTOR;

	/*!
	\brief Maximum value of DriveSum
	*/
	static const long ALGCFG_DRIVESUM_MAX_VALUE = 55000;

	/*!
	\brief Weighting of angle in the calculation of the correction speed
	*/
	static const unsigned long BALANCEMOMENT_ANGLE_WEIGHT = 3*27;
	/*!
	\brief Weighting of angle rate in the calculation of the correction speed
	*/
	static const unsigned long BALANCEMOMENT_ANGLERATE_WEIGHT = 1*27;

	// Steering configuration. Steering power is linearly decreased with increasing driveSum.
	/*!
	\brief Maximum value given by steeringPotentiometer
	*/
	static const long ALGCFG_STEERING_MAX_STEERINGPOTI_VALUE = 40;
	/*!
	\brief Maximum steering PWM when standing still
	*/
	static const unsigned long ALGCFG_DYNAMIC_STEERING_AT_NO_SPEED = 20;
	/*!
	\brief PWM until which max steering value is decreased linearly
	*/
	static const unsigned char ALGCFG_DYNAMIC_STEERING_MAX_PWM = 15;
	/*!
	\brief Maximum steering PWM between ALGCFG_DYNAMIC_STEERING_MAX_PWM and ALGCFG_BLOCK_STEERING_PWM
	*/
	static const unsigned long ALGCFG_DYNAMIC_STEERING_AT_MAX_PWM = 4;
	/*!
	\brief No steering from this PWM
	*/
	static const unsigned char ALGCFG_BLOCK_STEERING_PWM = 25;

	/*!
	\brief Factor for converting the voltage measured by the battery voltage sensor to the correct voltage.
	*/
	static const float MULTIPLEXSENSOR_BATTERYVOLTAGE_FACTOR_MV;


	/*
		Control algorithm configuration: automatic part, do not change
	*/
	/*!
	\brief Factor to convert PWM to driveSum according to: PWM = driveSum / PWM_TO_DRIVESUM_FACTOR
	*/
	static const float PWM_TO_DRIVESUM_FACTOR;
	/*!
	\brief if driveSum exceeds this value, speed limitation gets active.
	*/
	static const float SPEEDLIMIT_START_DRIVESUM;
	/*!
	\brief if driveSum exceeds this value, speed limit is at it's maximum effect.
	*/
	static const float SPEEDLIMIT_END_DRIVESUM;


	/*
		Error Codes
	*/
	/*!
	\brief Error code: battery low
	*/
	static const unsigned char ERROR_CODE_INIT_LOWBATTERY = 1;
	/*!
	\brief Error code: timer initialization failed
	*/
	static const unsigned char ERROR_CODE_INIT_TIMER = 2;
	/*!
	\brief Error code: ADC initialization failed
	*/
	static const unsigned char ERROR_CODE_INIT_ADC = 3;
	/*!
	\brief Error code: orientation accelerometer initialization failed
	*/
	static const unsigned char ERROR_CODE_INIT_ORIENTACCEL = 4;
	/*!
	\brief Error code: orientation gyrometer initialization failed
	*/
	static const unsigned char ERROR_CODE_INIT_ORIENTGYRO = 5;
	/*!
	\brief Error code: orientation gyrometer reference voltage sensor initialization failed
	*/
	static const unsigned char ERROR_CODE_INIT_ORIENTGYROREFERENCE = 6;
	/*!
	\brief Error code: steering potentiometer initialization failed
	*/
	static const unsigned char ERROR_CODE_INIT_STEERINGPOTI = 7;
	/*!
	\brief Error code: multiplex sensor initialization failed
	*/
	static const unsigned char ERROR_CODE_INIT_BATTERYSENS = 8;
	/*!
	\brief Error code: left motor initialization failed
	*/
	static const unsigned char ERROR_CODE_INIT_LEFTMOTOR = 9;
	/*!
	\brief Error code: right motor initialization failed
	*/
	static const unsigned char ERROR_CODE_INIT_RIGHTMOTOR = 10;

public:
	Segway();
	void timerFunction();
	void main();
};

#endif /* SEGWAY_H_ */
