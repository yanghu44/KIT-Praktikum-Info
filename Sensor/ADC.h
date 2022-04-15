/*
	use this class for ADC implementation -> configuring the ADC module of the microcontroller and its object
*/

#ifndef ADC_H_
#define ADC_H_
#include "../Configuration/Configuration.h"

#define ADC_NUM_CHANNELS 8

class ADC {
private:

    long ui32ADC0Value[9];
    volatile long value_adc;
    volatile long unimportant_values;


public:

	ADC();
	~ADC();
	bool init();
	void clearIntFlag( void );
	void prozessorTriggerConverion( void );
	bool IsConversionFinished( void );
	unsigned long getChannelValue( unsigned long channelID );
	void ADCUpdateDataValues( void );
	
	void cleanUpADC( void );
};

#endif /* ADC_H_ */
