#include "ADCSensor.h"
#include "ADC.h"
#include "../Configuration/Configuration.h"
#include <stdio.h>
#include <stdlib.h>

ADCSensor::ADCSensor() {
}

ADCSensor::~ADCSensor() {
}

bool ADCSensor::init( Configuration::s_ADCSensorConfig* thisADCSensorConfig_, ADC* ADCController_ ) {

    // Speichert für jedes Sensor-Objekt die, durch den Zeiger im Übergabewert übergeben
    // Parameter von ADCSensrCfg in eigenen Variablen/Konstanten.

    this->ADC_info = *thisADCSensorConfig_;
    ADC_controller = *ADCController_;

    return true;


}

signed long ADCSensor::getIntegerValue() {
    // Gibt den Wert des jeweiligen ADC-Kanals/Sensor-Objekts zurück. Wenn ein Offset
    // verwendet wird, so wird dieser vor der Rückgabe des Messewerts abgezogen.


        unsigned long result = ADC_controller.getChannelValue(ADC_info.ADCChannelID);

        if(ADC_info.useZeroOffset == true)
            {
                return (result - ADC_info.zeroOffset);
            }
        else
            {
                return result;
            }



}


void ADCSensor::setZeroOffset( bool active, signed long offset ) {

    // aktiviert oder deaktiviert die Verwendung eines Offset mit active und setzt den Offset
    // auf den Übergabeparameter offset .


    if(active==true)
    {
        ADC_info.useZeroOffset = true;
        ADC_info.zeroOffset = offset;
    }
    else if(active==false)
        {
            ADC_info.useZeroOffset = false;
        }



}

bool ADCSensor::getZeroOffsetIsActive() {

    //Gibt zurück ob ein Offset verwendet wird.

    if(ADC_info.useZeroOffset == true)
    {
        return ADC_info.useZeroOffset;
    }
    else
    {
        return false;
    }

}

signed long ADCSensor::getZeroOffset() {

    //Gibt den Offset-Wert zurück.
	return ADC_info.zeroOffset;
}
void ADCSensor::cleanUp(){

    // Wird vom Destruktor genutzt, um Ausgangspins zu deaktivieren und Variablen zu löschen.

}
