/*
	Timer class for AVR32UC3B
	
	DEPENDENCIES:
	An interrupt handling system has to be set up (it is already). This class only manages the timer registers.
	
	EXAMPLE INTERRUPT HANDLING SYSTEM:
	See tm4c123gh6pm_startup_ccs.c file for NVIC vector table. (-> Entry for Timer0IntHandler)
 */

#ifndef TIMER_H_
#define TIMER_H_

class Timer {
private:
	void cleanUp( void );
	
public:
	Timer();
	~Timer();
	static void resetInterruptFlag( void );
	bool initTimer( unsigned long frequency );
	void enableTimerBlockClock(void);
	void setIsTimerEnabled( bool enabled );
	void setIsTimerInterruptEnabled( bool enabled );
};

#endif /* TIMER_H_ */


