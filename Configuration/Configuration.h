/*! \class	Configuration

    \brief	This class contains static variables only, which hold the configuration parameters for all other classes used by the segway project.

		In Configuration.h the variables and structs are declared.
		In Configuration.cpp the variables are defined and initialized with zero.
		In init() the variables are set to the configuration values.
		
		This behavior allows calculations to be made within init().
*/


#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#define ADC_NUM_CONFIGURED_CHANNELS 			11



class Configuration {
private:
	static void init_Timer();
	static void init_PWM();
	static void init_Motor();
	static void init_Sensors();
	static void init_Sensor_FootSwitch();
	static void init_Sensor_OrientationAccelerometer();
	static void init_Sensor_OrientationGyrometer();
	static void init_Sensor_OrientationGyrometerReference();
	static void init_Sensor_SteeringPotentiometer();
	static void init_Sensor_BatteryVoltage();
	static void init_Sensor_counterSensHAL1();
	static void init_Sensor_counterSensHAL2();
	static void init_Sensor_curr_Sense_L();
	static void init_Sensor_curr_Sense_R();


	static void init_StatusLEDs();

public:
	struct s_PWMConfig {
		unsigned char channelID;				// Channel of PWM
		unsigned char maxPWMRatio;				//Value between 0 and 255. 255 equals always-on
		unsigned long frequency;				
		unsigned char GPIO_port;				// 0 ~ PA, 1 ~ PB;
		unsigned char GPIO_pin;	
		unsigned char Pin_Mux_Assignement;
		unsigned char pwm_module;
	};
	struct s_MotorConfig {
		unsigned char directionPinPort;			// 0 ~ PA, 1 ~ PB;
		unsigned long directionPinPin;			// Pin for Port PA/PB
		bool directionPinForwardValue;			// 1: high on forward, 0: low on backward
		s_PWMConfig* PWMConfig;					
	};
	struct s_GPIOSensorConfig {
		unsigned char port;
		unsigned long pin;
		bool pullupEnabled;
	};
	struct s_gpioMultiplexData {
		bool configured;
		unsigned char port;
		unsigned long pin;
	};
	struct s_ADCSensorConfig {
		unsigned int ADCChannelID;
		signed long zeroOffset;
		unsigned char HWAverage;
		bool useZeroOffset, useHWAverage;
	};

	struct s_StatusLED {
		unsigned char port;
		unsigned long pin;
	};
	
	
	// MISC	
	static unsigned long
		CPUCLK,									//processor clock
		PWMCLK,									//PWM clock
		ADCCLK;									//ADC clock
	
	// TIMER
	static unsigned char
		Timer_Interrupt_Vector_Nr;
		

	// PWM
	static s_PWMConfig
		leftPWMConfig,
		rightPWMConfig;
		
	// MOTOR
	static s_MotorConfig
		leftMotorConfig,
		rightMotorConfig;
		
	static unsigned char Motor_enabledPinPort;
	static unsigned long Motor_enabledPinPin;
	static bool Motor_enabledPinEnabledValue;				// 1: high on enabled, 0: low on enabled

	
	// GPIOSENSOR
	static s_GPIOSensorConfig
		footSwitchConfig;
	
	// ADC(SENSOR)
	static unsigned long ADC_Internal_Clock;				//ADC internal clock ( 16MHz for 1MSPS Conversion speed)
	static s_gpioMultiplexData
		ADC_gpioMultiplexData[ADC_NUM_CONFIGURED_CHANNELS];
	
	static s_ADCSensorConfig
		orientationAccelerometerConfig,
		orientationGyrometerConfig,
		orientationGyrometerReferenceConfig,
		steeringPotentiometerConfig,
		batteryVoltageSensorConfig,
		counterSensHAL1Config,
		counterSensHAL2Config,
		curr_Sense_LConfig,
		curr_Sense_RConfig;

	// Status LEDs
	static s_StatusLED
		redStatusLEDConfig,
		greenStatusLEDConfig,
		blueStatusLEDConfig;

	static void init();
};

#endif /* CONFIGURATION_H_ */
