#include "Simulation.h"

/*! \brief	Sets the motor's direction and speed to a given PWM value.
	\param	motor	[in] pointer to the motor object
	\param	speed	[in] signed PWM value (-255 to 255) representing the motor speed
*/
void Simulation::setMotorSpeed( Motor* motor, long speed ) {
	if( speed < 0 ) {
		motor->setDirection( false );
		motor->setSpeed( -speed );
	} else {
		motor->setDirection( true );
		motor->setSpeed( speed );
	}
}


void Simulation::main() {
	for( unsigned int i = 0; i < 100; i++ ) {

	}
	
	//BP1

	initHelpers();
		
	// Disable motors and enable timer
	leftMotor.setEnabled( false );
	rightMotor.setEnabled( false );
	myTimer.setIsTimerEnabled( true );
	
	setStatusLED( &Configuration::greenStatusLEDConfig, true );
	
	for( unsigned int i = 0; i < 100; i++ ) {

	}
	
	//BP2

	
	leftMotor.setEnabled( true );
	leftMotor.setSpeed( 15 );
	rightMotor.setEnabled( true );
	rightMotor.setSpeed( 15 );
	
	for( unsigned int i = 0; i < 100; i++ ) {

	}
	
	//BP3
	
	

	while( true ) {
	}
//
//	cleanUp();
}


/*! \brief	Initializes the helper objects. Blocks execution on error.
*/
void Simulation::initHelpers() {
	// Initialize status LEDs
	initStatusLED( &Configuration::redStatusLEDConfig );
	setStatusLED( &Configuration::redStatusLEDConfig, false );
	initStatusLED( &Configuration::greenStatusLEDConfig );
	setStatusLED( &Configuration::greenStatusLEDConfig, false );
	// Initialize Timer at 100Hz, will be in deactivated state.
	if( !myTimer.initTimer( TIMER_FREQUENZYHHH )) {
		displayError( ERROR_CODE_INIT_TIMER );
	}
	
	// Initialize the ADC, so it can be used by the ADCSensors
	if( !myADC.init()) {
		displayError( ERROR_CODE_INIT_ADC );
	}
	
	footSwitchSensor.init( &Configuration::footSwitchConfig );
		
	if( !orientationAccelerometer.init( &Configuration::orientationAccelerometerConfig, &myADC )) {
		while(1){
		}
	}
	if( !orientationGyrometer.init( &Configuration::orientationGyrometerConfig, &myADC )) {
		while(1){
				}
	}
	if( !orientationGyrometerReference.init( &Configuration::orientationGyrometerReferenceConfig, &myADC )) {
		while(1){
				}
	}
	if( !steeringPotentiometer.init( &Configuration::steeringPotentiometerConfig, &myADC )) {
		while(1){
				}
	}
	if( !batteryVoltageSensor.init( &Configuration::batteryVoltageSensorConfig, &myADC )) {
		while(1){
				}
	}

// ADC Multiplexer
	
	Motor::initEnablePin();

	if( !leftMotor.init( &Configuration::leftMotorConfig )) {
		while(1){
				}
	}
	if( !rightMotor.init( &Configuration::rightMotorConfig )) {
		while(1){
				}
	}
}


/*! \brief	Cleans up before class is destroyed. Disables the timer.
*/
void Simulation::cleanUp() {
	myTimer.setIsTimerEnabled( false );
	myTimer.setIsTimerInterruptEnabled( false );
}


/*! \brief	Displays error code. Blocks execution.

		Green LED is off. Red LED blinks errorCode times, then stays off some time and restarts.
*/
void Simulation::displayError( unsigned char errorCode ) {
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
void Simulation::initStatusLED( Configuration::s_StatusLED* statusLEDConfig ) {
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
void Simulation::setStatusLED( Configuration::s_StatusLED* statusLEDConfig, bool on ) {
	if( on )
	{
		 *(volatile unsigned int *)GPIO_PORTF_DATA_REGISTER |= (1u << statusLEDConfig->pin);

	} else
	{
		*(volatile unsigned int *)GPIO_PORTF_DATA_REGISTER &= ~(1u << statusLEDConfig->pin);
	}
}
