/*
 *  Use this class for the implementation of the GPIO (digital) Sensor - Object
*/

#ifndef GPIOSENSOR_H_
#define GPIOSENSOR_H_

#include "../Configuration/Configuration.h"

class GPIOSensor {
	
private:
    Configuration::s_GPIOSensorConfig* cnfgGPIO;

public:
	GPIOSensor();
	~GPIOSensor();

	void init( Configuration::s_GPIOSensorConfig* thisGPIOSensorConfig_ );
	bool getValue();
	void cleanUp();
};

#endif /* GPIOSENSOR_H_ */

