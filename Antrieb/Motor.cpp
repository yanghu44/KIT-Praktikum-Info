#include "Motor.h"

#define GPIODIR    0x400
#define GPIODEN    0x51C
#define GPIODATA   0x000

#define GPIO_Clock  0x400FE608
#define GPIO_Port_C 0x40006000
#define GPIO_Port_D 0x40007000

Motor::Motor() {
}

Motor::~Motor() {
}

void Motor::initEnablePin() {

    *(volatile unsigned int*) (GPIO_Clock) |= (1 << Configuration::Motor_enabledPinPort);
    // Clock für Port D aktivieren
    *(volatile unsigned int*) (GPIO_Clock) |= (1 << 2);
    // Clock für Port C aktivieren (1 << this->motorconfig.directionPinPort)

    *(volatile unsigned int*) ((0x40004000 + Configuration::Motor_enabledPinPort * 0x1000)| GPIODIR) |= (1 << Configuration::Motor_enabledPinPin);
    // Pin PD6 als Ausgang setzen (MotorEnable)
    *(volatile unsigned int*) ((0x40004000 + Configuration::Motor_enabledPinPort * 0x1000) | GPIODEN) |= (1 << Configuration::Motor_enabledPinPin);
    // Pin PD6 mit Digitallogik verwenden (MotorEnable)
    *(volatile unsigned int*) ((0x40004000 + Configuration::Motor_enabledPinPort * 0x1000) | GPIODATA) &= ~(1 << Configuration::Motor_enabledPinPin);
    // Pin PD6 auf Low setzen

}

bool Motor::init( Configuration::s_MotorConfig* MotorConfig ) {

    this->motorconfig = *MotorConfig;

    *(volatile unsigned int*) ((0x40004000 + this->motorconfig.directionPinPort * 0x1000) | GPIODEN) |= (1 << this->motorconfig.directionPinPin);
    // Pin PC6,7 (Port C) mit Digitallogik verwenden
    *(volatile unsigned int*) ((0x40004000 + this->motorconfig.directionPinPort * 0x1000) | GPIODIR) |= (1 << this->motorconfig.directionPinPin);
    // Pin PC6,7 (Port C) als Ausgang setzen

    pwmobjekt.init(MotorConfig ->PWMConfig);
    pwmobjekt.setChannelsEnabled(true);
    return true;
}

bool Motor::setSpeed( unsigned char ratioOn ) {

    if (ratioOn > this->motorconfig.PWMConfig->maxPWMRatio){
        return true;
        //Abfrage, ob das PWM-Signal größer als der erlaubte Maximalwert ist
    }
    if (ratioOn == 0){
        return true;
        //Abfrage, ob das PWM-Signal 0 ist, da dies Fehler im Simulator erzeugt
    }
    LastValidSpeed = ratioOn;
    pwmobjekt.setChannelPWMRatio(ratioOn);
    return true;
}

unsigned char Motor::getSpeed() {
    return LastValidSpeed;
    //Ausgabe der letzten korrekten Geschwindigkeit
}

void Motor::setDirection( bool forward ) {
    volatile unsigned int dirPin = (1 << this->motorconfig.directionPinPin);
    if (forward == this->motorconfig.directionPinForwardValue)
    {
        *(volatile unsigned int*) (0x40004000 + GPIODATA + (dirPin << 2)) |= dirPin;
            // Pin PC6,7 auf Rückwärts schalten (Motorrichtung auf High)
    }
    *(volatile unsigned int*) (0x40004000 + GPIODATA + (dirPin << 2)) &= ~dirPin;
    // Pin PC6,7 auf Vorwärts schalten (Motorrichtung auf Low)


}

void Motor::setEnabled( bool enabled ) {

    if(enabled == 0){
        *(volatile unsigned int*) ((0x40004000 + Configuration::Motor_enabledPinPort * 0x1000) | GPIODATA) &= ~(1 << Configuration::Motor_enabledPinPin);
        return;
        // Pin PD6 auf Low setzen
    }
    *(volatile unsigned int*) ((0x40004000 + Configuration::Motor_enabledPinPort * 0x1000) | GPIODATA) |= (1 << Configuration::Motor_enabledPinPin);
        // Pin PD6 auf High setzen
}

bool Motor::getIsEnabled() {
	return ((0x40004000 + Configuration::Motor_enabledPinPort * 0x1000) | GPIODATA) & (1 << Configuration::Motor_enabledPinPin);
	// Auslesen des 6.Bit in Register (GPIO_PORT_D | GPIODATA)
}
void Motor::Motor_CleanUp(){}
