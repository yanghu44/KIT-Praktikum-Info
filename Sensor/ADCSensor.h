#include "../Configuration/Configuration.h"
#include "ADC.h"
/*
	Requires the ADC to be initialized! Use this class for implementation of the Sensor Objects
*/

#ifndef ADCSENSOR_H_
#define ADCSENSOR_H_

class ADCSensor {
private:
	void cleanUp();

	Configuration::s_ADCSensorConfig ADC_info;
	ADC ADC_controller;


public:
	ADCSensor();
	~ADCSensor();
	bool init( Configuration::s_ADCSensorConfig* thisADCSensorConfig_, ADC* ADCController_ );
	long getIntegerValue ( );
	void setZeroOffset( bool active, signed long offset );
	bool getZeroOffsetIsActive();
	signed long getZeroOffset();
};

#endif  /*ADCSENSOR_H_ */
