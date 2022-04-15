#include "Segway.h"


//const char* Segway::BLUETOOTH_NAME = "TivSeg13";
const float Segway::TIMER_DELTA_T_SECONDS = 0.01;								// Time between timer-IRs
const float Segway::ALGCFG_COMPL_FILTER_ACCELANGLE_FACTOR = 0.02;
const float Segway::MULTIPLEXSENSOR_BATTERYVOLTAGE_FACTOR_MV = ( 33000 / 1023.0 );
const float Segway::PWM_TO_DRIVESUM_FACTOR = ( ALGCFG_DRIVESUM_MAX_VALUE / ( PWMCLASS_MAX_PWM * DRIVESPEED_PWM_MAX_PERCENT / 100 )); // 359,477
const float Segway::SPEEDLIMIT_START_DRIVESUM = ( PWM_TO_DRIVESUM_FACTOR * PWMCLASS_MAX_PWM * ALGCFG_SPEEDLIMIT_START_PWM_PERCENT / 100 );
const float Segway::SPEEDLIMIT_END_DRIVESUM = ( PWM_TO_DRIVESUM_FACTOR * PWMCLASS_MAX_PWM * ALGCFG_SPEEDLIMIT_END_PWM_PERCENT / 100 );

/*! \brief	Function called by the timer. It contains the control algorithm.

		The function first clears IR-flag of the timer, triggers ADC conversion next and then receives the sensor values and calculates and sets the motor's PWM
		according to the controlling algorithm.
*/
void Segway::timerFunction() {
	// Trigger ADC.

	myTimer.resetInterruptFlag();									// Reset Interrupt Flag of the timer at first.

	myADC.clearIntFlag();											// Clear last ADC Int Flag to trigger new Conversion
	myADC.prozessorTriggerConverion();								// Trigger ADC Conversion(´s)
	while(!myADC.IsConversionFinished())
	{
		// Wait until ADC(´s) finished conversions
	}
	myADC.ADCUpdateDataValues();									// Read out FIFO of ADC Sample Sequencer(´s)

	// get sensor values
	bool footSwitchPressed = footSwitchSensor.getValue();
	long orientationGyrometerValue = orientationGyrometer.getIntegerValue()  - orientationGyrometerReference.getIntegerValue(); // orientationGyrometer does not have to be calibrated as it has it's own reference voltage
	long orientationAccelerometerValue = orientationAccelerometer.getIntegerValue();
	long steeringPotentiometerValue = 0 - steeringPotentiometer.getIntegerValue();

//	// to test ADC - Values, you can use these variables. Watch them in the debugger, set Breakpoint at the end of the timerFunction and choose "refresh all windows" instead of disrupting.
//	bool fP = footSwitchSensor.getValue();
//	long oGV = orientationGyrometer.getIntegerValue();
//	long oGR = orientationGyrometerReference.getIntegerValue(); // orientationGyrometer does not have to be calibrated as it has it's own reference voltage
//	long oAV = orientationAccelerometer.getIntegerValue();
//	long BVoltage = batteryVoltageSensor.getIntegerValue() * MULTIPLEXSENSOR_BATTERYVOLTAGE_FACTOR_MV;
//	long sPV = steeringPotentiometer.getIntegerValue();

	// calculate the angle rate in °/s from the sensor value (+- 1000mV = +- 100°/s)
 	float realAngleRate = 0.0 - (((float) orientationGyrometerValue)  * (100.0 / 310.0));
	
	// calculate the acceleration g from the sensor value (+- 800mV = +- 1g)

	// float acceleration = ((float) orientationAccelerometerValue ) / 248.0;						+++
	
	// calculate the (tilt) angle of the segway.
	// sin(a) = acceleration / 1g
	// a in rad = acceleration / 1g
	// a in ° = acceleration / 1g * 360/2pi
	// Approximation for small values
	float accelerometerAngle = ((float) orientationAccelerometerValue) * (57.0 / 248.0);
	
	// Apply complementary filter to the measured angle and the measured angle rate to get the real angle.
	// angle = (0.98)*(angle + gyro * dt) + (0.02)*(x_acc);
	CFTiltAngle = ( 1 - ALGCFG_COMPL_FILTER_ACCELANGLE_FACTOR ) * ( CFTiltAngle + (realAngleRate * TIMER_DELTA_T_SECONDS) ) + ( ALGCFG_COMPL_FILTER_ACCELANGLE_FACTOR )*( accelerometerAngle );
	
	// Calculate correction angle for soft speed limit
	float speedLimitTiltAngle = limitSpeedSoft( driveSum );
	
	// Correction term to be added to the driveSum.
	float balanceMoment = (BALANCEMOMENT_ANGLERATE_WEIGHT * realAngleRate) + BALANCEMOMENT_ANGLE_WEIGHT * ( CFTiltAngle + speedLimitTiltAngle );
	
	// Integration
	driveSum = driveSum + balanceMoment;
	
	// Hard Limit Speed
	if( driveSum > ALGCFG_DRIVESUM_MAX_VALUE ) {
		driveSum = ALGCFG_DRIVESUM_MAX_VALUE;
	} else if( driveSum < -ALGCFG_DRIVESUM_MAX_VALUE ) {
		driveSum = -ALGCFG_DRIVESUM_MAX_VALUE;
	}
	
	
	// Convert driveSum to PWM value
	long driveSpeedPWM = driveSum / PWM_TO_DRIVESUM_FACTOR;
	
	// Calculate steering PWM if not blocked.
	long steeringPWM = 0;
	if( !steeringBlocked ) {
		steeringPWM = limitSteeringPWM( steeringPotentiometerValue, driveSum );
	}
		
	long leftSpeed = driveSpeedPWM;
	long rightSpeed = driveSpeedPWM;
	
	
	if( operationMode == _standby )
	{
		// If in standby mode, don't move
		driveSum = 0;
		leftSpeed = 0;
		rightSpeed = 0;
		
		if( footSwitchPressed )
		{
			// If foot switch is pressed in standby mode, take current position as zero position, set up blocking
			// of the steering for 3 seconds, do a battery check and then enter run mode.
			setCurrentOrientationAsZero();
			setCurrentSteeringPositionAsZero();

			footSwitchTimeout = 0;
			CFTiltAngle = 0;
			// Reset AVGSpeedLimitDrieveSumArray elements
			for(unsigned int i = 0; i < 200; i++)
			{
				AVGSpeedLimitDriveSumArray[i] = 0;
			}
			AVGSpeedLimitDriveSumArray_index = 0;
			
			// prevent Steering within the first 3 seconds
			blockSteeringTimeout = 300;
			steeringBlocked = true;
			
			// Check battery voltage, block if voltage is too low.
			if(BATTERYCHECKENABLE){
			doBatteryCheck();
			}
			leftMotor.setEnabled( true );
			rightMotor.setEnabled( true );

			operationMode = _run;
		}
	}
	
	if( operationMode == _run ) {

		setStatusLED( &Configuration::greenStatusLEDConfig, true );
		setStatusLED( &Configuration::blueStatusLEDConfig, false );
		// If in run mode, control motors.
		if( footSwitchPressed ) {
			// If foot switch is pressed, continue normally.
			footSwitchTimeout = 0;
			
			if( !steeringBlocked ) {
				// If steering is not blocked, add steering PWM to motor speeds.
				leftSpeed += steeringPWM;
				rightSpeed -= steeringPWM;
			} else {
				// If steering is blocked, wait decrease timeout and enable the steering when the timeout value reaches zero.
				if( blockSteeringTimeout > 0 ) {
					blockSteeringTimeout--;
				} else {
					if( steeringBlocked ) {
						if( steeringPWM == 0 && steeringPotentiometerValue == 0 ) {
							// Blocking time elapsed and steeringPWM is zero => activate steering
							steeringBlocked = false;
							blockSteeringTimeout = 0;
						}
					}
				}
			}
		} else {
			// If foot switch is not pressed, start timeout.
			footSwitchTimeout++;
			if( footSwitchTimeout >= ALGCFG_FOOTSWITCHTIMEOUT_CRITICAL ) {
				// Foot switch released too long -> go to standby
				leftMotor.setEnabled( false );
				rightMotor.setEnabled( false );
				setStatusLED( &Configuration::greenStatusLEDConfig, false );
				setStatusLED( &Configuration::blueStatusLEDConfig, true );
				
				operationMode = _standby;
			}
		}
	}
	
	
	// Apply an offset to the motor speeds to face the problem that the wheels - when standing still - won't start
	// turning until a PWM of about 12 (of 255) is reached.
	// If the target speed is lower than 5, set it to zero. Otherwise add/subtract 3 according to the direction to
	// get closer towards 12.
	if( leftSpeed > -5 && leftSpeed < 5 ) {
		leftSpeed = 0;
	} else {
		if( leftSpeed > 0 ) {
			leftSpeed += 3;
		} else {
			leftSpeed -= 3;
		}
	}
	if( rightSpeed > -5 && rightSpeed < 5 ) {
		rightSpeed = 0;
	} else {
		if( rightSpeed > 0 ) {
			rightSpeed += 3;
		} else {
			rightSpeed -= 3;
		}
	}

//	Little code here can be used, to test only the motors and PWM modules for proper working.
//	if(rs > 100){
//		c = -1;
//	}
//	if (rs < -100){
//		c = 1;
//	}
//	rs += c;
//	ls -= c;
//	setMotorSpeed( &leftMotor, ( ls));
//	setMotorSpeed( &rightMotor, ( rs));



	//  Set the motor's speed:
	// 1.741 is a correction factor resulting from fixing a rounding error in the PWM class as the
	// other parameters of the control algorithm were adjusted to the erroneous PWM class.
	setMotorSpeed( &leftMotor,  float(leftSpeed / 1.741));
	setMotorSpeed( &rightMotor, float(rightSpeed / 1.741));

}


/*! \brief	Constructor. Does minimal initialization.
*/
Segway::Segway() {
	
	CFTiltAngle = 0;
	
	// Reset AVGSpeedLimitDrieveSumArray elements
	for(unsigned int i = 0; i < 200; i++)
	{
		AVGSpeedLimitDriveSumArray[i] = 0;
	}
	AVGSpeedLimitDriveSumArray_index = 0;
}


/*! \brief	Calculates an exponential moving average for data type long.

		An exponential moving average is calculated as follows:
		new_averaged_value = weight * new_input_value + (1-weight) * old_averaged_value
		
		The weight factor has to be given in permille, so 500 means that newVal is weighted 50%.
		
	\param	oldVal	[in] old/current average
	\param	newVal	[in] measured value
	\param	weightOfNewVal_permille	[in] weight factor according to function description in permille.
	\return	Exponential moving average
*/
long Segway::expMovingAverage( long oldVal, long newVal, unsigned char weightOfNewVal_permille ) {
	return (( 1000 - weightOfNewVal_permille ) * oldVal + weightOfNewVal_permille * newVal ) / 1000;
}

/*! \brief	Calculates an exponential moving average for data type float.

		An exponential moving average is calculated as follows:
		new_averaged_value = weight * new_input_value + (1-weight) * old_averaged_value
		
		The weight factor has to be given in permille, so 500 means that newVal is weighted 50%.
		
	\param	oldVal	[in] old/current average
	\param	newVal	[in] measured value
	\param	weightOfNewVal_permille	[in] weight factor according to function description in permille.
	\return	Exponential moving average
*/
float Segway::expMovingAverage( float oldVal, float newVal, unsigned char weightOfNewVal_permille ) {
	return ( float( 1000 - weightOfNewVal_permille ) * oldVal + weightOfNewVal_permille * newVal ) / 1000.0;
}

/*! \brief	Calculates the PWM values for steering.

		The steering PWM is calculated by calculating the maximum steering PWM allowed at current speed
		and by calculating the current steering handle position.
		The maximum allowed steering PWM at no speed is ALGCFG_DYNAMIC_STEERING_AT_NO_SPEED. It linearily decreases
		to ALGCFG_DYNAMIC_STEERING_AT_MAX_PWM with an increasing Speed-PWM from 0 to ALGCFG_DYNAMIC_STEERING_MAX_PWM.
		If speed is increased further, the maximum allowed PWM remains constant until ALGCFG_BLOCK_STEERING_PWM is reached.
		If ALGCFG_BLOCK_STEERING_PWM is reached, the maximum allowed steering PWM is 0.
		
		To smooth the segway's behavior, the max allowed steering PWM ist filtered using an exponential moving average.
				
	\param	steeringPotentiometerValue	[in] value returned by steering potentiometer (signed, zero means no steering)
	\param	driveSum	[in] current Speed (driveSum value in timerFunction)
	\return	steering PWM
*/
long Segway::limitSteeringPWM( long steeringPotentiometerValue, long driveSum ) {
	// Limit steering potentiometer value to what is expected at maximum (ALGCFG_STEERING_MAX_STEERINGPOTI_VALUE)
	if( steeringPotentiometerValue > ALGCFG_STEERING_MAX_STEERINGPOTI_VALUE ) {
		steeringPotentiometerValue = ALGCFG_STEERING_MAX_STEERINGPOTI_VALUE;
	} else if( steeringPotentiometerValue < -ALGCFG_STEERING_MAX_STEERINGPOTI_VALUE ) {
		steeringPotentiometerValue = -ALGCFG_STEERING_MAX_STEERINGPOTI_VALUE;
	}

	// Calculate the absolute value of the current drive speed.
	long absDriveSum;
	if( driveSum < 0 ) {
		absDriveSum = -driveSum;
	} else	
	{
		absDriveSum = driveSum;
	}
	
	static float MAMaxSteeringPWM = 0;
	
	if( absDriveSum > ALGCFG_BLOCK_STEERING_PWM * PWM_TO_DRIVESUM_FACTOR ) {
		// If drive speed PWM exceeds ALGCFG_BLOCK_STEERING_PWM, block steering.
		return 0;
	} else if( absDriveSum > ALGCFG_DYNAMIC_STEERING_MAX_PWM * PWM_TO_DRIVESUM_FACTOR ) {
		// If drive speed PWM is lower than ALGCFG_BLOCK_STEERING_PWM but over ALGCFG_DYNAMIC_STEERING_MAX_PWM,
		// allow minimal steering.
		MAMaxSteeringPWM = expMovingAverage( MAMaxSteeringPWM, float( ALGCFG_DYNAMIC_STEERING_AT_MAX_PWM ), 20 );
	} else {
		// steeringSpeed decreases from ALGCFG_DYNAMIC_STEERING_AT_NO_SPEED to ALGCFG_DYNAMIC_STEERING_AT_MAX_PWM with increasing PWM from 0 to ALGCFG_DYNAMIC_STEERING_MAX_PWM
		MAMaxSteeringPWM = expMovingAverage( MAMaxSteeringPWM, float( ALGCFG_DYNAMIC_STEERING_AT_MAX_PWM + ( ALGCFG_DYNAMIC_STEERING_AT_NO_SPEED - ALGCFG_DYNAMIC_STEERING_AT_MAX_PWM ) * ( 1 - absDriveSum / ( ALGCFG_DYNAMIC_STEERING_MAX_PWM * PWM_TO_DRIVESUM_FACTOR ))), 20 );
	}
	
	// If the maximum Steering PWM exceeds ALGCFG_DYNAMIC_STEERING_AT_NO_SPEED (should not happen ), limit it to that.
	if( MAMaxSteeringPWM > ALGCFG_DYNAMIC_STEERING_AT_NO_SPEED ) {
		MAMaxSteeringPWM = ALGCFG_DYNAMIC_STEERING_AT_NO_SPEED;
	}
	
	return ( MAMaxSteeringPWM * steeringPotentiometerValue ) / float( ALGCFG_STEERING_MAX_STEERINGPOTI_VALUE );
}

/*! \brief	Calculates the speed limit angle

		As a speed limit, the segway is intended to lean back against the driver if it runs too fast. The correction
		angle, which is returned by this function, is the angle to which the segway leans against the driver.
		
		The angle is zero while driveSum is lower than SPEEDLIMIT_START_DRIVESUM. Then, it increases linearly to
		ALGCFG_SPEEDLIMIT_LEAN_BACK_MAX_DEGREES with an increasing driveSum to SPEEDLIMIT_END_DRIVESUM. At higher
		speed, the angle remains at ALGCFG_SPEEDLIMIT_LEAN_BACK_MAX_DEGREES.
				
	\param	driveSum	[in] current Speed (driveSum value in timerFunction)
	\return	correction angle for the speed limit
*/
float Segway::limitSpeedSoft( long driveSum ) {
	// Store current driveSum in an array for averaging.
	if( AVGSpeedLimitDriveSumArray_index >= 200 ) {
		AVGSpeedLimitDriveSumArray_index = 0;
	}
	AVGSpeedLimitDriveSumArray[AVGSpeedLimitDriveSumArray_index] = driveSum;
	AVGSpeedLimitDriveSumArray_index++;
	
	// If speed is low, reset speed limit array data
	if( driveSum < 2 * PWM_TO_DRIVESUM_FACTOR && driveSum > -2 * PWM_TO_DRIVESUM_FACTOR ) {
		// Reset AVGSpeedLimitDrieveSumArray elements
		for(unsigned int i = 0; i < 200; i++)
		{
			AVGSpeedLimitDriveSumArray[i] = 0;
		}
		AVGSpeedLimitDriveSumArray_index = 0;
	}

	// Calculate average driveSum using the array values.
	long AVGSpeedLimitDriveSum = 0;
	for( int i = 0; i < 200; i++ ) {
		AVGSpeedLimitDriveSum += AVGSpeedLimitDriveSumArray[i];
	}
	AVGSpeedLimitDriveSum /= 200;

	if( AVGSpeedLimitDriveSum > SPEEDLIMIT_END_DRIVESUM ) {
		// maximum leaning back
		return ALGCFG_SPEEDLIMIT_LEAN_BACK_MAX_DEGREES;
	} else if( AVGSpeedLimitDriveSum > SPEEDLIMIT_START_DRIVESUM ) {
		// leaning back according to the speed
		return ALGCFG_SPEEDLIMIT_LEAN_BACK_MAX_DEGREES * ( AVGSpeedLimitDriveSum - SPEEDLIMIT_START_DRIVESUM ) / ( SPEEDLIMIT_END_DRIVESUM-SPEEDLIMIT_START_DRIVESUM );
	} else if( AVGSpeedLimitDriveSum < -SPEEDLIMIT_END_DRIVESUM ) {
		// maximum leaning ahead
		return -ALGCFG_SPEEDLIMIT_LEAN_BACK_MAX_DEGREES;
	} else if( AVGSpeedLimitDriveSum < -SPEEDLIMIT_START_DRIVESUM ) {
		// leaning ahead according to the speed
		return -ALGCFG_SPEEDLIMIT_LEAN_BACK_MAX_DEGREES * ( -AVGSpeedLimitDriveSum - SPEEDLIMIT_START_DRIVESUM ) / ( SPEEDLIMIT_END_DRIVESUM-SPEEDLIMIT_START_DRIVESUM );
	}
	
	return 0;
}


/*! \brief	Sets the motor's direction and speed to a given PWM value.
	\param	motor	[in] pointer to the motor object
	\param	speed	[in] signed PWM value (-255 to 255) representing the motor speed
*/
void Segway::setMotorSpeed( Motor* motor, long speed ) {
	if( speed < 0 ) {
		motor->setDirection( false );
		motor->setSpeed( -speed );
	} else {
		motor->setDirection( true );
		motor->setSpeed( speed );
	}
}


/*! \brief	Checks the battery voltage and blocks execution if it is lower then MIN_BATTERY_VOLTAGE.

		Blocking is done via endl. loop for Debugging
*/
void Segway::doBatteryCheck() {
	 // Check battery voltage
	if( batteryVoltageSensor.getIntegerValue() * MULTIPLEXSENSOR_BATTERYVOLTAGE_FACTOR_MV < MIN_BATTERY_VOLTAGE )
	{
		while(1)
		{
			displayError(ERROR_CODE_INIT_LOWBATTERY);
		}
	}
}

/*! \brief	Main loop.

		Initializes helper objects, then starts the timer containing the controlling algorithm.
		Before starting the timer, it is made sure that the foot switch is not pressed.
		
		While running, this function sends debug data via the bluetooth interface.
		Different debug values can be enabled by uncommenting them here and in timerFunction().
*/

void Segway::main() {

	initHelpers();
	
	// Footswitch pressed on start -> don't continue until foot switch is released
	waitForFootSwitchReleased();

	// Calibrate steering
	setCurrentSteeringPositionAsZero();
	setCurrentOrientationAsZero();
	
	operationMode = _standby;
	
	// Disable motors and enable timer
	leftMotor.setEnabled( false );
	rightMotor.setEnabled( false );
	setStatusLED( &Configuration::blueStatusLEDConfig, true );
	
	c = 1;

	myTimer.setIsTimerEnabled( true );

	while( true ) {
		// Endless loop
	}
}


/*! \brief	Initializes the helper objects. Blocks execution on error.
*/
void Segway::initHelpers() {
	// Initialize status LEDs
	initStatusLED( &Configuration::redStatusLEDConfig );
	initStatusLED( &Configuration::greenStatusLEDConfig );
	initStatusLED( &Configuration::blueStatusLEDConfig );

	// Initialize Timer at 100Hz, will be in deactivated state.
	if( !myTimer.initTimer( IR_FREQUENCY )) {
		while(1){
			displayError(ERROR_CODE_INIT_TIMER);
		}
	}
	
	// Initialize the ADC, so it can be used by the ADCSensors
	if( !myADC.init()) {
		while(1)
		{
			displayError(ERROR_CODE_INIT_ADC);
		}
	}
		
	footSwitchSensor.init( &Configuration::footSwitchConfig );
		
	if( !orientationAccelerometer.init( &Configuration::orientationAccelerometerConfig, &myADC )) {
		while(1)
		{
			displayError(ERROR_CODE_INIT_ORIENTACCEL);
		}
	}
	if( !orientationGyrometer.init( &Configuration::orientationGyrometerConfig, &myADC )) {		// Debug here,-> ChannelID is going round...
		while(1)
		{
			displayError(ERROR_CODE_INIT_ORIENTGYRO);
		}
	}
	if( !orientationGyrometerReference.init( &Configuration::orientationGyrometerReferenceConfig, &myADC )) {
		while(1)
		{
			displayError(ERROR_CODE_INIT_ORIENTGYROREFERENCE);
		}
	}
	if( !steeringPotentiometer.init( &Configuration::steeringPotentiometerConfig, &myADC )) {
		while(1)
		{
			displayError(ERROR_CODE_INIT_STEERINGPOTI);
		}
	}
	if( !batteryVoltageSensor.init( &Configuration::batteryVoltageSensorConfig, &myADC )) {
		while(1)
		{
			displayError(ERROR_CODE_INIT_BATTERYSENS);
		}
	}
	
		Motor::initEnablePin();

	if( !leftMotor.init( &Configuration::leftMotorConfig )) {
		while(1)
		{
			displayError(ERROR_CODE_INIT_LEFTMOTOR);
		}
	}
	if( !rightMotor.init( &Configuration::rightMotorConfig )) {
		while(1)
		{
			displayError(ERROR_CODE_INIT_RIGHTMOTOR);
		}
	}
	
	// Wait some time for initialisation of HW
	for( int i = 0; i < 500000; i++ ) {

	}
}

/*! \brief	Configures the steering potentiometer object so that the current steering potentiometer position is the new zero position.

		If there is an offset set currently, the offset has to be added to the currently measured sensor value as the sensor
		subtracted this offset directly after measuring the value. So the new offset value is the current value + the old offset value.
	
		If there is no offset set currently, use the current value as the new offset value.
*/
void Segway::setCurrentSteeringPositionAsZero() {
	steeringPotentiometer.setZeroOffset( true,
							( steeringPotentiometer.getZeroOffsetIsActive() ?
								  ( steeringPotentiometer.getIntegerValue() + steeringPotentiometer.getZeroOffset())
								: steeringPotentiometer.getIntegerValue()));
}

/*! \brief	Configures the orientation accelerometer object so that the current steering potentiometer position is the new zero position.

		If there is an offset set currently, the offset has to be added to the currently measured sensor value as the sensor
		subtracted this offset directly after measuring the value. So the new offset value is the current value + the old offset value.

		If there is no offset set currently, use the current value as the new offset value.
*/
void Segway::setCurrentOrientationAsZero() {
	orientationAccelerometer.setZeroOffset( true,
							( orientationAccelerometer.getZeroOffsetIsActive() ?
								  ( orientationAccelerometer.getIntegerValue() + orientationAccelerometer.getZeroOffset())
								: orientationAccelerometer.getIntegerValue()));
}
/*! \brief	Configures the Gyrometer object so that the current position is the new zero position.

		If there is an offset set currently, the offset has to be added to the currently measured sensor value as the sensor
		subtracted this offset directly after measuring the value. So the new offset value is the current value + the old offset value.

		If there is no offset set currently, use the current value as the new offset value.
*/
void Segway::setCurrentGyroPositionZero() {
	orientationGyrometer.setZeroOffset( true,
							( orientationGyrometer.getZeroOffsetIsActive() ?
								  ( orientationGyrometer.getIntegerValue() + orientationGyrometer.getZeroOffset())
								: orientationGyrometer.getIntegerValue()));
}

/*! \brief	Cleans up before class is destroyed. Disables the timer.
*/
void Segway::cleanUp() {
	myTimer.setIsTimerEnabled( false );
	myTimer.setIsTimerInterruptEnabled( false );
}

/*! \brief	ERROR. Blocks execution.

		RED Led blinks
*/
void Segway::displayError( unsigned char errorCode ) {
	// turn off all LEDs
	setStatusLED( &Configuration::greenStatusLEDConfig, false );
	setStatusLED( &Configuration::redStatusLEDConfig, false );
	setStatusLED( &Configuration::blueStatusLEDConfig, false );

	while( true )
	{
		for( unsigned char i = 0; i < errorCode; i++ )
		{
			// blink red LED error code times then pause
			setStatusLED( &Configuration::redStatusLEDConfig, true );
			for( unsigned long wait = 0; wait < 1000000; wait++ )
			{
			}
			setStatusLED( &Configuration::redStatusLEDConfig, false );
			for( unsigned long wait = 0; wait < 1000000; wait++ )
			{
			}
		}
		for( unsigned long wait = 0; wait < 5000000; wait++ )
			{
			}
	}
}


/*! \brief	Initializes the status LEDs.
	\param	statusLEDConfig	[in] Pointer to configuration struct for the LED to initialize.
*/
void Segway::initStatusLED( Configuration::s_StatusLED* statusLEDConfig ) {
	// Use Onbaord LEDs

	*(volatile unsigned int *)SYSCTL_RUN_CLOCK_GATING_CONTROL_REGISTER |= (1u << statusLEDConfig->port);			// enable clock for Port F

	*(volatile unsigned int *)GPIO_PORTF_DIR_REGISTER |= (1u << statusLEDConfig->pin);								// Set Port F LED Pins as outputs

	*(volatile unsigned int *)GPIO_PORTF_DIGITAL_ENABLE_REGISTER |= (1u << statusLEDConfig->pin);					// enable digital PORT F

	*(volatile unsigned int *)GPIO_PORTF_DATA_REGISTER &= ~(1u << statusLEDConfig->pin);							// Clear Port F LED Pins
}


/*! \brief	Enables/Disables the status LEDs.
	\param	statusLEDConfig	[in] Pointer to the configuration struct for the LED to initialize.
	\param	on	[in] true = LED on, false = LED off
*/
void Segway::setStatusLED( Configuration::s_StatusLED* statusLEDConfig, bool on ) {
	if( on )
	{
		 *(volatile unsigned int *)GPIO_PORTF_DATA_REGISTER |= (1u << statusLEDConfig->pin);

	} else
	{
		*(volatile unsigned int *)GPIO_PORTF_DATA_REGISTER &= ~(1u << statusLEDConfig->pin);
	}
}

/*! \brief	Blocks execution until the foot switch is released.
*/
void Segway::waitForFootSwitchReleased() {
	// Footswitch pressed -> don't continue until foot switch is released
	if( footSwitchSensor.getValue())
	{
		setStatusLED( &Configuration::redStatusLEDConfig, true );
		// Block execution until the foot switch sensor is released.
		while( footSwitchSensor.getValue())
		{
			for( unsigned long wait = 0; wait < 1000000; wait++ )
			{

			}
		}
		setStatusLED( &Configuration::redStatusLEDConfig, false );
	}
}
