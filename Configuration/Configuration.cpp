#include "Configuration.h"

// MISC
unsigned long
	Configuration::CPUCLK = 0,
	Configuration::PWMCLK = 0,
	Configuration::ADCCLK = 0;

// TIMER
unsigned char
	Configuration::Timer_Interrupt_Vector_Nr;

// PWM
Configuration::s_PWMConfig
	Configuration::leftPWMConfig = {},
	Configuration::rightPWMConfig = {};


// MOTOR
Configuration::s_MotorConfig
	Configuration::leftMotorConfig = {},
	Configuration::rightMotorConfig = {};
		
unsigned char
	Configuration::Motor_enabledPinPort = 0;
unsigned long
	Configuration::Motor_enabledPinPin = 0;
bool
	Configuration::Motor_enabledPinEnabledValue = 0; // 1: high is enabled, 0: low  is enabled

// GPIOSENSOR
Configuration::s_GPIOSensorConfig
	Configuration::footSwitchConfig = {};

// ADC(SENSOR)
unsigned long
	Configuration::ADC_Internal_Clock = 0;

Configuration::s_gpioMultiplexData
	Configuration::ADC_gpioMultiplexData[ADC_NUM_CONFIGURED_CHANNELS] = {};
		
Configuration::s_ADCSensorConfig
	Configuration::orientationAccelerometerConfig = {},
	Configuration::orientationGyrometerConfig = {},
	Configuration::orientationGyrometerReferenceConfig = {},
	Configuration::steeringPotentiometerConfig = {},
	Configuration::batteryVoltageSensorConfig = {},
	Configuration::counterSensHAL1Config = {},
	Configuration::counterSensHAL2Config = {},
	Configuration::curr_Sense_LConfig = {},
	Configuration::curr_Sense_RConfig = {};

// Status LEDs
Configuration::s_StatusLED
	Configuration::redStatusLEDConfig,
	Configuration::greenStatusLEDConfig,
	Configuration::blueStatusLEDConfig;

/*! \brief	Initializes all configuration variables.
*/
void Configuration::init() {
	//
	CPUCLK = 50000000;
	PWMCLK = CPUCLK;
	ADCCLK = 16000000;			// Use PIOSC
	
	init_Timer();
	init_PWM();
	init_Motor();
	init_Sensors();
	init_StatusLEDs();
}

/*! \brief	Initializes timer variables.

		Timer_Channel: selects which of the controller's timer channel is used.
		Timer_Clock_Connection: selects source clock and prescaler.
*/
void Configuration::init_Timer() {
	Timer_Interrupt_Vector_Nr = 35;		// The IR Handler is pre-configured to handle IR on position 35 of NVIC-Table (See tm4c123gh6pm_startup_ccs.c line 105)
}

/*! \brief	Initializes PWM variables for left and right motor.

		channelID: the microcontroller's PWM channel.
		maxPWMRatio: maximum PWM radio allowed (0: always off, 255: always on).
		frequency: PWM frequency.
		GPIO_port: GPIO port, at which the PWM signal is outputted.
		GPIO_pin: GPIO pin, at which the PWM signal is outputted.
*/
void Configuration::init_PWM() {
	leftPWMConfig.channelID = 0;
	leftPWMConfig.maxPWMRatio = 153;
	leftPWMConfig.frequency = 18000;
	leftPWMConfig.GPIO_port = 2; 		//PC4
	leftPWMConfig.GPIO_pin = 4;
	leftPWMConfig.Pin_Mux_Assignement = 4;		// Value has to be filled in the GPIOPCTL Register at the right place to select the right PWM Function
	leftPWMConfig.pwm_module = 0;				// Use PWM Module 0
	
	rightPWMConfig.channelID = 1;
	rightPWMConfig.maxPWMRatio = 153;
	rightPWMConfig.frequency = 18000;
	rightPWMConfig.GPIO_port = 2; 		//PC5
	rightPWMConfig.GPIO_pin = 5;
	rightPWMConfig.Pin_Mux_Assignement = 4;		// Value has to be filled in the GPIOPCTL Register at the right place to select the right PWM Function
	rightPWMConfig.pwm_module = 0;				// Use PWM Module 0
}

/*! \brief	Initializes Motor variables for left and right motor.
		
		Motor_enabledPinEnabledValue: 1: high output = enabled, 0: low output = enabled.
		Motor_enabledPinPort: GPIO port, at which the enabled signal is outputted.
		Motor_enabledPinPin: GPIO pin, at which the enabled signal is outputted.
		
		directionPinForwardValue: 1: high output = forward, 0: low output = forward.
		directionPinPort: GPIO port, at which the direction signal is outputted.
		directionPinPin: GPIO pin, at which the direction signal is outputted.
		PWMConfig: Pointer to the motor's PWM config.
*/
void Configuration::init_Motor() {
	Motor_enabledPinEnabledValue = 1;
	Motor_enabledPinPort = 3; // PD6
	Motor_enabledPinPin = 6;
	
	rightMotorConfig.directionPinForwardValue = 1;
	rightMotorConfig.directionPinPort = 2; 		//PC6
	rightMotorConfig.directionPinPin = 6;
	rightMotorConfig.PWMConfig = &leftPWMConfig;	// Do not Change!!!
	
	leftMotorConfig.directionPinForwardValue = 1;
	leftMotorConfig.directionPinPort = 2; 		//PC7
	leftMotorConfig.directionPinPin = 7;
	leftMotorConfig.PWMConfig = &rightPWMConfig;	// Do not Change!!!
}

/*! \brief	Initializes sensor variables.
		
		ADC_Internal_Clock:	Target value for ADC internal clock (Clock used for the AD conversion).
							For 10 bit conversion this clock must not exceed 5 MHz.
							The prescaler is automatically chosen by the ADC class to set this clock
							to the closest value available, which is lower than ADC_Internal_Clock.
*/
void Configuration::init_Sensors() {
	// GPIO-Sensors
	init_Sensor_FootSwitch();
	
	// ADC-Sensors
	 ADC_Internal_Clock = 16000000;
	
	init_Sensor_OrientationAccelerometer();
	init_Sensor_OrientationGyrometer();
	init_Sensor_OrientationGyrometerReference();
	init_Sensor_SteeringPotentiometer();
	init_Sensor_BatteryVoltage();
	init_Sensor_counterSensHAL1();
	init_Sensor_counterSensHAL2();
	init_Sensor_curr_Sense_R();
	init_Sensor_curr_Sense_L();
}

/*! \brief	Initializes footswitch sensor variables.
		
		port: GPIO port, at which the foot switch signal is measured.
		pin: GPIO pin, at which the foot switch signal is measured.
		pullupEnabled: Enable GPIO pull up resistor..
*/
void Configuration::init_Sensor_FootSwitch() {
	footSwitchConfig.port = 0; 					//PA2
	footSwitchConfig.pin = 2;
	footSwitchConfig.pullupEnabled = true;
}

/*! \brief	Initializes orientationGyrometer-related configuration variables.
		
		ADC_gpioMultiplexData[x]: configure x-th channel of ADC Analog input
		
		configured: is configured.
		port: GPIO port, at which the signal is measured.
		pin: GPIO pin, at which the signal is measured.
		
		ADCChannelID: select ADC channel id and aequivalent Analog Input Channel
		useHWAverage: enable use of Hardware-Averaging in ADC Module
		HWAverage: Set Value for HW-Averaging (0, 2, 4, 8, .... 64)
		useZeroOffset: enable the subtraction of zeroOffset from the measured value before slopeFactor is applied.
		zeroOffset: offset to subtract from the measured value.
*/
void Configuration::init_Sensor_OrientationGyrometer() {
	ADC_gpioMultiplexData[0].configured = true;
	ADC_gpioMultiplexData[0].port = 4;			// PE3
	ADC_gpioMultiplexData[0].pin = 3;
	
	orientationGyrometerConfig.ADCChannelID = 0;	// AIN0
	orientationGyrometerConfig.useHWAverage = true;
	orientationGyrometerConfig.HWAverage = 2;
	orientationGyrometerConfig.useZeroOffset = false;
	orientationGyrometerConfig.zeroOffset = 0;
}
/*! \brief	Initializes orientationGyrometerReference (reference voltage for the orientationGyrometer)-related configuration variables.
		
		ADC_gpioMultiplexData[x]: configure x-th channel of ADC Analog input
		
		configured: is configured.
		port: GPIO port, at which the signal is measured.
		pin: GPIO pin, at which the signal is measured.
		
		ADCChannelID: select ADC channel id and aequivalent Analog Input Channel
		useHWAverage: enable use of Hardware-Averaging in ADC Module
		HWAverage: Set Value for HW-Averaging (0, 2, 4, 8, .... 64)
		useZeroOffset: enable the subtraction of zeroOffset from the measured value before slopeFactor is applied.
		zeroOffset: offset to subtract from the measured value.
*/
void Configuration::init_Sensor_OrientationGyrometerReference() {
	ADC_gpioMultiplexData[1].configured = true;
	ADC_gpioMultiplexData[1].port = 4;		// PE2
	ADC_gpioMultiplexData[1].pin = 2;
	
	orientationGyrometerReferenceConfig.ADCChannelID = 1;  // AIN1
	orientationGyrometerReferenceConfig.useHWAverage = true;
	orientationGyrometerReferenceConfig.HWAverage = 2;
	orientationGyrometerReferenceConfig.useZeroOffset = false;
	orientationGyrometerReferenceConfig.zeroOffset = 0;
}
/*! \brief	Initializes orientationAccelerometer-related configuration variables.

		ADC_gpioMultiplexData[x]: configure x-th channel of ADC Analog input

		configured: is configured.
		port: GPIO port, at which the signal is measured.
		pin: GPIO pin, at which the signal is measured.

		ADCChannelID: select ADC channel id and aequivalent Analog Input Channel
		useHWAverage: enable use of Hardware-Averaging in ADC Module
		HWAverage: Set Value for HW-Averaging (0, 2, 4, 8, .... 64)
		useZeroOffset: enable the subtraction of zeroOffset from the measured value before slopeFactor is applied.
		zeroOffset: offset to subtract from the measured value.
*/
void Configuration::init_Sensor_OrientationAccelerometer() {
	ADC_gpioMultiplexData[2].configured = true;
	ADC_gpioMultiplexData[2].port = 4; 	//PE1
	ADC_gpioMultiplexData[2].pin = 1;

	orientationAccelerometerConfig.ADCChannelID = 2;	//AIN2
	orientationAccelerometerConfig.useHWAverage = true;
	orientationAccelerometerConfig.HWAverage = 2;
	orientationAccelerometerConfig.useZeroOffset = false;
	orientationAccelerometerConfig.zeroOffset = 0;
}

/*! \brief	Initializes steeringPotentiometer-related configuration variables.
		
		ADC_gpioMultiplexData[x]: configure x-th channel of ADC Analog input
		
		configured: is configured.
		port: GPIO port, at which the signal is measured.
		pin: GPIO pin, at which the signal is measured.
		
		ADCChannelID: select ADC channel id and aequivalent Analog Input Channel
		useHWAverage: enable use of Hardware-Averaging in ADC Module
		HWAverage: Set Value for HW-Averaging (0, 2, 4, 8, .... 64)
		useZeroOffset: enable the subtraction of zeroOffset from the measured value before slopeFactor is applied.
		zeroOffset: offset to subtract from the measured value.
*/
void Configuration::init_Sensor_SteeringPotentiometer() {
	ADC_gpioMultiplexData[4].configured = true;
	ADC_gpioMultiplexData[4].port = 3;		// PD3
	ADC_gpioMultiplexData[4].pin = 3;
	
	steeringPotentiometerConfig.ADCChannelID = 4;		// AIN4
	steeringPotentiometerConfig.useHWAverage = true;
	steeringPotentiometerConfig.HWAverage = 2;
	steeringPotentiometerConfig.useZeroOffset = false;
	steeringPotentiometerConfig.zeroOffset = 0;
}

/*! \brief	Initializes batteryVoltageSensor-related configuration variables.
		
		ADC_gpioMultiplexData[x]: configure x-th channel of ADC Analog input
		
		configured: is configured.
		port: GPIO port, at which the signal is measured.
		pin: GPIO pin, at which the signal is measured.
		
		ADCChannelID: select ADC channel id and aequivalent Analog Input Channel
		useHWAverage: enable use of Hardware-Averaging in ADC Module
		HWAverage: Set Value for HW-Averaging (0, 2, 4, 8, .... 64)
		useZeroOffset: enable the subtraction of zeroOffset from the measured value before slopeFactor is applied.
		zeroOffset: offset to subtract from the measured value.
*/
void Configuration::init_Sensor_BatteryVoltage() {
	ADC_gpioMultiplexData[5].configured = true;
	ADC_gpioMultiplexData[5].port = 3;		// PD2
	ADC_gpioMultiplexData[5].pin = 2;
	
	batteryVoltageSensorConfig.ADCChannelID = 5;	// AIN5
	batteryVoltageSensorConfig.useHWAverage = true;
	batteryVoltageSensorConfig.HWAverage = 2;
	batteryVoltageSensorConfig.useZeroOffset = false;
	batteryVoltageSensorConfig.zeroOffset = 0;
}

/*! \brief	Initializes counterSensHAL1-related configuration variables.

		ADC_gpioMultiplexData[x]: configure x-th channel of ADC Analog input

		configured: is configured.
		port: GPIO port, at which the signal is measured.
		pin: GPIO pin, at which the signal is measured.

		ADCChannelID: select ADC channel id and aequivalent Analog Input Channel
		useHWAverage: enable use of Hardware-Averaging in ADC Module
		HWAverage: Set Value for HW-Averaging (0, 2, 4, 8, .... 64)
		useZeroOffset: enable the subtraction of zeroOffset from the measured value before slopeFactor is applied.
		zeroOffset: offset to subtract from the measured value.
*/
void Configuration::init_Sensor_counterSensHAL1() {
	ADC_gpioMultiplexData[6].configured = true;
	ADC_gpioMultiplexData[6].port = 3;		// PD1
	ADC_gpioMultiplexData[6].pin = 1;

	counterSensHAL1Config.ADCChannelID = 6;	// AIN6
	counterSensHAL1Config.useHWAverage = true;
	counterSensHAL1Config.HWAverage = 2;
	counterSensHAL1Config.useZeroOffset = false;
	counterSensHAL1Config.zeroOffset = 0;
}

/*! \brief	Initializes counterSensHAL2-related configuration variables.

		ADC_gpioMultiplexData[x]: configure x-th channel of ADC Analog input

		configured: is configured.
		port: GPIO port, at which the signal is measured.
		pin: GPIO pin, at which the signal is measured.

		ADCChannelID: select ADC channel id and aequivalent Analog Input Channel
		useHWAverage: enable use of Hardware-Averaging in ADC Module
		HWAverage: Set Value for HW-Averaging (0, 2, 4, 8, .... 64)
		useZeroOffset: enable the subtraction of zeroOffset from the measured value before slopeFactor is applied.
		zeroOffset: offset to subtract from the measured value.
*/
void Configuration::init_Sensor_counterSensHAL2() {
	ADC_gpioMultiplexData[7].configured = true;
	ADC_gpioMultiplexData[7].port = 3;		// PD0
	ADC_gpioMultiplexData[7].pin = 0;

	counterSensHAL2Config.ADCChannelID = 7;	// AIN7
	counterSensHAL2Config.useHWAverage = true;
	counterSensHAL2Config.HWAverage = 2;
	counterSensHAL2Config.useZeroOffset = false;
	counterSensHAL2Config.zeroOffset = 0;
}

/*! \brief	Initializes Current_Sensor_R - related configuration Variables.

		ADC_gpioMultiplexData[x]: configure x-th channel of ADC Analog input

		configured: is configured.
		port: GPIO port, at which the signal is measured.
		pin: GPIO pin, at which the signal is measured.

		ADCChannelID: select ADC channel id and aequivalent Analog Input Channel
		useHWAverage: enable use of Hardware-Averaging in ADC Module
		HWAverage: Set Value for HW-Averaging (0, 2, 4, 8, .... 64)
		useZeroOffset: enable the subtraction of zeroOffset from the measured value before slopeFactor is applied.
		zeroOffset: offset to subtract from the measured value.
*/
void Configuration::init_Sensor_curr_Sense_R () {
	ADC_gpioMultiplexData[8].configured = true;
	ADC_gpioMultiplexData[8].port = 4;		// PE5
	ADC_gpioMultiplexData[8].pin = 5;

	curr_Sense_RConfig.ADCChannelID = 8;	// AIN8
	curr_Sense_RConfig.useHWAverage = true;
	curr_Sense_RConfig.HWAverage = 2;
	curr_Sense_RConfig.useZeroOffset = false;
	curr_Sense_RConfig.zeroOffset = 0;
}

/*! \brief	Initializes Current_Sensor_L - related configuration Variables.

		ADC_gpioMultiplexData[x]: configure x-th channel of ADC Analog input

		configured: is configured.
		port: GPIO port, at which the signal is measured.
		pin: GPIO pin, at which the signal is measured.

		ADCChannelID: select ADC channel id and aequivalent Analog Input Channel
		useHWAverage: enable use of Hardware-Averaging in ADC Module
		HWAverage: Set Value for HW-Averaging (0, 2, 4, 8, .... 64)
		useZeroOffset: enable the subtraction of zeroOffset from the measured value before slopeFactor is applied.
		zeroOffset: offset to subtract from the measured value.
*/
void Configuration::init_Sensor_curr_Sense_L () {
	ADC_gpioMultiplexData[9].configured = true;
	ADC_gpioMultiplexData[9].port = 4;		// PE4
	ADC_gpioMultiplexData[9].pin = 4;

	curr_Sense_LConfig.ADCChannelID = 9;	// AIN9
	curr_Sense_LConfig.useHWAverage = true;
	curr_Sense_LConfig.HWAverage = 2;
	curr_Sense_LConfig.useZeroOffset = false;
	curr_Sense_LConfig.zeroOffset = 0;
}

/*! \brief	Initializes status LED configuration variables.
		
		port: GPIO port
		pin: GPIO pin
*/
void Configuration::init_StatusLEDs() {
	// Status LEDs
	redStatusLEDConfig.port = 5;
	redStatusLEDConfig.pin = 1;
	blueStatusLEDConfig.port = 5;
	blueStatusLEDConfig.pin = 2;
	greenStatusLEDConfig.port = 5;
	greenStatusLEDConfig.pin = 3;

}
