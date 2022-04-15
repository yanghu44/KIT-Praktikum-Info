#include "PWM.h"
#include "../Configuration/Configuration.h"

#define GPIODIR    0x400
#define GPIODEN    0x51C
#define GPIODATA   0x000
#define GPIOPCTL   0x52C
#define RCGC0      0x100
#define RCGC2      0x108
#define GPIOAFSEL  0x420
#define PWM0CTL    0x040
#define PWM0GENA   0x060
#define PWM0GENB   0x064
#define PWM0LOAD   0x050
#define PWM0CMPA   0x058
#define PWM0CMPB   0x05C
#define PWMENABLE  0x008
#define RCGCPWM    0x640
#define RCGCGPIO   0x608
#define GPIODR2R   0x500

#define GPIO_Clock  0x400FE608
#define GPIO_Port_C 0x40006000
#define GPIO_Port_D 0x40007000
#define BaseRCGC    0x400FE000
#define PWM0_base   0x40028000//0x40029000 für PWM1

PWM::PWM() {
}

PWM::~PWM() {
	cleanUp();
}

bool PWM::init( Configuration::s_PWMConfig* PWMConfig ) {

    this->pwmconfig = *PWMConfig;

    *(volatile unsigned int*) ( BaseRCGC | RCGCPWM ) |= (1 << this->pwmconfig.pwm_module);
    // PWM-Clock aktivieren (RCGCPWM)

    *(volatile unsigned int*) ( BaseRCGC | RCGCGPIO) |= (1 << this->pwmconfig.GPIO_port);
    // Clock für GPIO-Modul C aktivieren (RCGCGPIO)

    *(volatile unsigned int*) ((0x40004000 + this->pwmconfig.GPIO_port * 0x1000) | GPIODEN) |= (1 << this->pwmconfig.GPIO_pin);
    // Aktiviere die Digitallogik
    
    *(volatile unsigned int*) ((0x40004000 + this->pwmconfig.GPIO_port * 0x1000) | GPIODR2R) |= (1 << this->pwmconfig.GPIO_pin);
    // "Aktiviere" den Strom von 2mA
    
    *(volatile unsigned int*) ((0x40004000 + this->pwmconfig.GPIO_port * 0x1000) | GPIOAFSEL) |= (1 << this->pwmconfig.GPIO_pin);
    // Aktiviere alternative Funktion der PINs

    *(volatile unsigned int*) ((0x40004000 + this->pwmconfig.GPIO_port * 0x1000) | GPIOPCTL) |= ((pwmconfig.Pin_Mux_Assignement) << this->pwmconfig.GPIO_pin * 4);
    // Führe PWM Signal auf die richtigen PINs

    *(volatile unsigned int*) (BaseRCGC | PWM0GENA) &= ~((1 << 17) | (1 << 18) | (1 << 19) | (1 << 20));
    // Run-Mode-Clock Konfiguration (PINs 17-20 in RCC auf 0)

    *(volatile unsigned int*) (PWM0_base | PWM0CTL) = (0x00000000);
    //Konfiguriere den PWM generator (Beende den Timer)

    *(volatile unsigned int*) (PWM0_base | PWM0GENA) |= (0x0000008C);
    //Konfiguriere den PWM generator für den countdown mode

    *(volatile unsigned int*) (PWM0_base | PWM0GENB) |= (0x0000080C);
    //Konfiguriere den PWM generator für den countdown mode

    *(volatile unsigned int*) (PWM0_base | PWM0LOAD) |= (0x00000AD9);// (1/(this->pwmconfig.frequency))*Configuration::CPUCLK
    //Konfiguriere den PWM generator auf 18KHz

//    if (pwmconfig.channelID == 1){
//        *(volatile unsigned int*) (PWM0_base | PWM0CMPB) = (PWMSpeed);
//    }
//        //Konfiguriere den Duty Cycle des PWM generator
//    else if (pwmconfig.channelID == 0){
//        *(volatile unsigned int*) (PWM0_base | PWM0CMPA) = (PWMSpeed);
//        //Konfiguriere den Duty Cycle des PWM generator
//    }
    *(volatile unsigned int*) (PWM0_base | PWM0CTL) = (0x00000001);
    //Konfiguriere den PWM generator (Starte den Timer)

	return true;
}

bool PWM::setChannelPWMRatio( unsigned char ratioOn ){
    if (ratioOn == 0){
    return true;
    //Abfrage, ob die Geschwindigkeit 0 ist, da dies zu Fehlern im Simulator führt
    }
    PWMSpeed = 2777 - ratioOn * 2777 / 255;
    //Berechnung des Wertes für die Compare-Register

    if (pwmconfig.channelID == 1){
        *(volatile unsigned int*) (PWM0_base | PWM0CMPB) = (PWMSpeed);
    }
        //Konfiguriere den Duty Cycle des PWM generator
    else if (pwmconfig.channelID == 0){
        *(volatile unsigned int*) (PWM0_base | PWM0CMPA) = (PWMSpeed);
        //Konfiguriere den Duty Cycle des PWM generator
    }

	return true;
}

bool PWM::setChannelsEnabled( bool enabled ) {
    if (enabled == true){
        *(volatile unsigned int*) (PWM0_base | PWMENABLE) |= (1 << (this->pwmconfig.channelID + 6));
        return true;
        //pwmA/B-Signal wird auf die Output-Stage geleitet
    }

    *(volatile unsigned int*) (PWM0_base | PWMENABLE) &= ~(1<<(this->pwmconfig.channelID + 6));
    return true;
    //pwmA/B-Signal wird nicht auf die Output-Stage geleitet
}

void PWM::cleanUp() {}
