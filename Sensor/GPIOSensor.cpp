#include "GPIOSensor.h"

// Registerdefinitionen
#define GPIO_Port_A_Base    0x40058000
#define GPIORCGC            0x400FE608
#define GPIOHBCTL           0x400FE06C
#define GPIODATA            0x000
#define GPIODIR             0x400
#define GPIODEN             0x51C
#define GPIOPUR             0x510
#define GPIODR2R            0x500

GPIOSensor::GPIOSensor() {
}

GPIOSensor::~GPIOSensor() {
}

void GPIOSensor::init( Configuration::s_GPIOSensorConfig* thisGPIOSensorConfig_ ) {
    this->cnfgGPIO = thisGPIOSensorConfig_;

    // 00100100
    // 01000000
    // 01100100
    //&= ~(1<<6)

    //The aperture enabled for a given GPIO port is controlled by the appropriate bit in the GPIOHBCTL register
    * (volatile unsigned int*) (GPIOHBCTL) |= (1<<this->cnfgGPIO->port);

    // Clock für GPIO Port A aktivieren
    * (volatile unsigned int*) (GPIORCGC) |= (1<<this->cnfgGPIO->port);

    // Fussschalter als Eingang initialisieren PA2
    // -> S.663: GPIODIR-Register: Pin2 (Port A) auf 0 setzen
    * (volatile unsigned int*) ((GPIO_Port_A_Base + this->cnfgGPIO->port * 0x1000) + GPIODIR) &= ~(1 << this->cnfgGPIO->pin);

    * (volatile unsigned int*) ((GPIO_Port_A_Base + this->cnfgGPIO->port * 0x1000) + GPIODR2R) |= (1 << this->cnfgGPIO->pin);

    //Pull-Up Register(S.677) aktivieren
   //if(this->cnfgGPIO->pullupEnabled == true)
    * (volatile unsigned int*) ((GPIO_Port_A_Base + this->cnfgGPIO->port * 0x1000) + GPIOPUR) |= (1 << this->cnfgGPIO->pin);

    //To enable GPIO pins as digital I/Os (S.682)
    * (volatile unsigned int*) ((GPIO_Port_A_Base + this->cnfgGPIO->port * 0x1000) + GPIODEN) |= (1 << this->cnfgGPIO->pin);

   }
bool GPIOSensor::getValue() {
    //Rückgabewert true heißt, dass der Anschlusspin High-Level hat
    // -> S.662: GPIODATA-Register: Pin2 (Port A) auf 1 setzen
    if (*(volatile unsigned int*)((GPIO_Port_A_Base + this->cnfgGPIO->port * 0x1000) + GPIODATA) & (1<<this->cnfgGPIO->pin)){
        return true;}
    else{
	return false;}
}

void GPIOSensor::cleanUp(){
    // Ausgangspins deaktivieren und Variablen löschen
    // reset GPIODATA
    * (volatile unsigned int*)((GPIO_Port_A_Base + this->cnfgGPIO->port * 0x1000) + GPIODATA) |= (0x00000000);
    //delete GPIODATA;
};

