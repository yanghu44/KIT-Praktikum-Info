/*  */

#ifndef PWM_H_
#define PWM_H_

#include "../Configuration/Configuration.h"

class PWM {
private:
    Configuration::s_PWMConfig pwmconfig;
    unsigned int PWMSpeed;
public:
	PWM();
	~PWM();
	bool init( Configuration::s_PWMConfig* PWMConfig);
	
	bool setChannelPWMRatio( unsigned char ratioOn );
	bool setChannelsEnabled( bool enabled );
	
	void cleanUp();
};

#endif /* PWM_H_ */
