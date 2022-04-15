#include "Timer.h"
#include "../Configuration/Configuration.h"

#define GPTMCTL         0x00C
#define GPTMCFG         0x000
#define Timer_0_base    0x40030000
#define GPTMTAMR        0x004
#define GPTMIMR         0x018
#define GPTMICR         0x024
#define GPTMTAILR       0x028
#define RCGCTIMER       0x400FE604
#define NVIC_EN0        0xE000E100
#define NVIC_DIS0       0xE000E180

Timer::Timer() {
}

Timer::~Timer() {
	Timer::cleanUp();
}

void Timer::cleanUp( void ){
    //Ensure the timer is disabled (the TnEN bit in the GPTMCTL register is cleared)
    //reset GPTMCTL (S.737)
    *(volatile unsigned int*)(Timer_0_base + GPTMCTL) = (0x00000000);
}

bool Timer::initTimer( unsigned long frequency ) {
    // Wenn Timer-Modul initialisiert ist, Interrupt aktiviert

    this->enableTimerBlockClock();

    //Ensure the timer is disabled (the TnEN bit in the GPTMCTL register is cleared)
    this->cleanUp();

    //Write the GPTM Configuration Register (GPTMCFG) with a value of 0x0000.0000
    *(volatile unsigned int*)(Timer_0_base + GPTMCFG) = (0x00000000);

    //Configure the TnMR field in the GPTM Timer n Mode Register (GPTMTnMR)
    //Write a value of 0x2 for Periodic mode
    *(volatile unsigned int*)(Timer_0_base + GPTMTAMR) = (0x2);

    //The Timer count down (S.731)
    *(volatile unsigned int*)(Timer_0_base + GPTMTAMR) &= ~(1<<4);

    //Load the start value into the GPTM Timer n Interval Load Register (GPTMTnILR)
    *(volatile unsigned int*)(Timer_0_base + GPTMTAILR) = ( (unsigned int)((Configuration::CPUCLK)/100) - 1);

    //set the appropriate bits in the GPTM Interrupt Mask Register (GPTMIMR) (S.745)
    *(volatile unsigned int*)(Timer_0_base + GPTMIMR) |= (1<<0);

    //Set the TnEN bit in the GPTMCTL register to enable the timer and start counting (S.737)
    //*(volatile unsigned int*)(Timer_0_base + GPTMCTL) |= (1<<0);

    //this->setIsTimerInterruptEnabled(true);
	return true;
}

void Timer::setIsTimerEnabled( bool enabled ) {
    //If enabled -> Timer aktivieren; else -> Timer deaktivieren
    if(enabled == true){
        *(volatile unsigned int*)(Timer_0_base + GPTMCTL) |= (1<<0);
    }
    else{
        *(volatile unsigned int*)(Timer_0_base + GPTMCTL) &= ~(1<<0);
    }

}

void Timer::enableTimerBlockClock(){
    // enable 16/32-bit timer modules in Run mode
    *(volatile unsigned int*)(RCGCTIMER) |= (1<<0);
}

void Timer::setIsTimerInterruptEnabled( bool enable ) {
//    if(enable){      *(volatile unsigned int*)(Timer_0_base + GPTMIMR) |= (1<<0);}
//    else{*(volatile unsigned int*)(Timer_0_base + GPTMIMR) &= ~(1<<0);}
   if(enable == true){
       *(volatile unsigned int*)(NVIC_EN0) = (1);
   }
       else{
           *(volatile unsigned int*)(NVIC_DIS0) = (1);
       }
}

void Timer::resetInterruptFlag() {
    //Setzt im TIMER-ICR(GPTMICR)(S.754) Register das entsprechende Bit um den Interrupt Flag zu löschen
    *(volatile unsigned int*)(Timer_0_base + GPTMICR) = (0x01);
}

