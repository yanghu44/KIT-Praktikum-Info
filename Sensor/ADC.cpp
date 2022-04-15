#include "ADC.h"
#include <stdio.h>
#include <stdlib.h>

ADC::ADC() {
}
ADC::~ADC() {
}


/*
 * PE3      Gyrometer Y                     AIN0        6
 * PE2      Gyrometer Referenzspannung      AIN1        7
 * PE1      Accelerometer X                 AIN2        8
 * PD3      Potenziometer Lenkausschlag     AIN4        64
 * PD2      Spannungsteiler Batterie        AIN5        63
 * PD1      Drehratensensor 1               AIN6        62
 * PD0      Drehratensensor 2               AIN7        61
 * PE5      Strommessung rechter Motor      AIN8        60
 * PE4      Strommessung linker Motor       AIN9        59
 *
 */

// ADC0, SS0, MUX0=AIN0
// ADC0, SS0, MUX1=AIN1
// ADC0, SS0, MUX2=AIN2
// ADC0, SS0, MUX3=AIN3 --> egal!
// ADC0, SS0, MUX4=AIN4
// ADC0, SS0, MUX5=AIN5
// ADC0, SS0, MUX6=AIN6
// ADC0, SS0, MUX7=AIN7
// ADC1, SS0, MUX0=AIN8
// ADC1, SS0, MUX1=AIN9


#define RCGC            0x400FE000
#define GPIOD           0x40007000 //APB
#define GPIOE           0x40024000 //APB
#define GPIOB           0x40005000 //APB

#define ADC_INT_0       0x40038000 //ADC0
#define ADC_INT_1       0x40039000 //ADC1

#define RCGCADC         0x638
#define RCGCGPIO        0x608
#define GPIOAFSEL       0x420
#define GPIODEN         0x51C
#define GPIOAMSEL       0x528
#define ADC_INT_OFF     0x004
#define ADC_INT_CLEAR   0x00C
#define ADCADCTSS       0x000
#define ADCSSMUX0       0x040
#define ADCSSCTL0       0x044
#define ADCPSSI         0x028
#define ADCSSFIFO0      0x048
#define ADCIM           0x008
#define ADCRIS          0x004
#define ADCSSFSTAT0     0x04C
#define ADCSSOP0        0x050
#define GPIOODR         0x50C
#define GPIOUR          0x510
#define GPIOPR          0x514




bool ADC::init() {

    // Diese Funktion initialisiert ein oder mehrere ADC-Module und deren Eingangspins.
    // Sie konfiguriert Sample-Sequenzer und setzt die Reihenfolge des Multiplexings der
    // Eingänge, wenn eine ADC-Conversion stattfinden soll. Desweiteren aktiviert sie op-
    // tionale ADC-Funktionen wie Hardware-Averaging, falls in der configuration.cpp
    // vorgegeben.



    // Enable the ADC clock using the RCGCADC register (see page 352).
    // RCGCADC, Base 0x400F.E000 offset 0x638
    // Bit 0 = 1 Enable ADC Module 0 Run Mode Clock

    * (volatile unsigned int*) (RCGC | RCGCADC) |= (1 << 0);  //ADCO
    * (volatile unsigned int*) (RCGC | RCGCADC) |= (1 << 1);  //ADC1

    // Enable the clock to the appropriate GPIO modules via the RCGCGPIO register (see page 340). To find out which GPIO ports to enable, refer to “Signal Description” on page 801.
    // Enable GPIO Ports F (Bit 5) und E (Bite 4)
    // Base 0x400F.E000, Offset 0x608

    * (volatile unsigned int*) (RCGC | RCGCGPIO) |= (1 << 4) | (1 << 5);



    // Set the GPIO AFSEL bits for the ADC input pins (see page 671). To determine which GPIOs to configure, see Table 23-4 on page 1344.
    // GPIO Port D (AHB) base: 0x4005.B000 (D0-D3)
    // GPIO Port E (AHB) base: 0x4005.C000 (E1-E5)
    // Offset 0x420
    * (volatile unsigned int*)( GPIOB | GPIOAFSEL ) &= ~(1 << 5);
    * (volatile unsigned int*) (GPIOD | GPIOAFSEL) &= ~((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3));
    * (volatile unsigned int*) (GPIOE | GPIOAFSEL) &= ~((1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5)) ;

    // Configure the AINx signals to be analog inputs by clearing the corresponding DEN bit in the GPIO Digital Enable (GPIODEN) register (see page 682).
    // Offset 0x51C
    // GPIO Port D (AHB) base: 0x4005.B000 (D0-D3)
    // GPIO Port E (AHB) base: 0x4005.C000 (E1-E5)

    * (volatile unsigned int*)( GPIOB | GPIODEN ) &= ~(1 << 5);
    * (volatile unsigned int*) (GPIOD | GPIODEN)  &= ~((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3));
    * (volatile unsigned int*) (GPIOE | GPIODEN)  &= ~((1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5)) ;

    // Disable the analog isolation circuit for all ADC input pins that are to be used by writing a 1 to the appropriate bits of the GPIOAMSEL register (see page 687) in the associated GPIO block.
    // offset 0x528
    // GPIO Port D (AHB) base: 0x4005.B000 (D0-D3)
    // GPIO Port E (AHB) base: 0x4005.C000 (E1-E5)

    * (volatile unsigned int*)( GPIOB | GPIOAMSEL) |= (1 << 5);
    * (volatile unsigned int*) (GPIOD | GPIOAMSEL) |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
    * (volatile unsigned int*) (GPIOE | GPIOAMSEL) |= (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) ;

    //deactivate OpenDrain resistance
            *(volatile unsigned int*)(GPIOB | GPIOODR) &= 0x0;
            * (volatile unsigned int*)(GPIOD | GPIOODR) &= 0x0;
            * (volatile unsigned int*)(GPIOE | GPIOODR) &= 0x0;
                //deactivate Pulldown and PullUp resistance
            * (volatile unsigned int*)(GPIOB | GPIOUR) &= 0x0;
            * (volatile unsigned int*)(GPIOD | GPIOUR) &= 0x0;
            * (volatile unsigned int*)(GPIOE | GPIOUR) &= 0x0;
            * (volatile unsigned int*)(GPIOB | GPIOPR) &= 0x0;
            * (volatile unsigned int*)(GPIOD | GPIOPR) &= 0x0;
            * (volatile unsigned int*)(GPIOE | GPIOPR) &= 0x0;


    // If required by the application, reconfigure the sample sequencer priorities in the ADCSSPRI register. The default configuration has Sample Sequencer 0 with the highest priority and Sample Sequencer 3 as the lowest priority.


    //Ursprünglich in prozessorTriggerconverion:

    // If required by the application, reconfigure the sample sequencer priorities in the ADCSSPRI register. The default configuration has Sample Sequencer 0 with the highest priority and Sample Sequencer 3 as the lowest priority.

    // Ensure that the sample sequencer is disabled by clearing the corresponding ASENn bit in the ADCACTSS register. Programming of the sample sequencers is allowed without having them enabled. Disabling the sequencer during programming prevents erroneous execution if a trigger event were to occur during the configuration process.
    // offset 0x000

    * (volatile unsigned int*) (ADC_INT_0 | ADCADCTSS) &= ~((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3));
    * (volatile unsigned int*) (ADC_INT_1 | ADCADCTSS) &= ~((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3));


    // When using a PWM generator as the trigger source, use the ADC Trigger Source Select
    // (ADCTSSEL) register to specify in which PWM module the generator is located. The default
    // register reset selects PWM module 0 for all generators.
    // --> wird nicht genutzt


    // For each sample in the sample sequence, configure the corresponding input source in the
    // ADCSSMUXn register.
    // (ADCSSMUX0), offset 0x040; (ADCSSMUX1), offset 0x060;  (ADCSSMUX2), offset 0x080

    // ADC0, SS0, MUX0=AIN0
    // ADC0, SS0, MUX1=AIN1
    // ADC0, SS0, MUX2=AIN2
    // ADC0, SS0, MUX3=AIN3 --> egal
    // ADC0, SS0, MUX4=AIN4
    // ADC0, SS0, MUX5=AIN5
    // ADC0, SS0, MUX6=AIN6
    // ADC0, SS0, MUX7=AIN7



    * (volatile unsigned int*) (ADC_INT_0 | ADCSSMUX0) &= ~( (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 8)   | (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 15)  | (1 << 16) | (1 << 17) | (1 << 19) | (1 << 21) | (1 << 23) | (1 << 24) | (1 << 27) | (1 << 31) );
    * (volatile unsigned int*) (ADC_INT_0 | ADCSSMUX0) |= ((1 << 4) | (1 << 9) | (1 << 14) |  (1 << 18) | (1 << 20) | (1 << 22) | (1 << 25 ) | (1 << 26) | (1 << 28) | (1 << 29) | (1 << 30) );

    // ADC1, SS0, MUX0=AIN8
    // ADC1, SS0, MUX0=AIN9

    * (volatile unsigned int*) (ADC_INT_1 | ADCSSMUX0) &= ~( (1 << 0)  | (1 << 1) | (1 << 2) | (1 << 5) | (1 << 6) );
    * (volatile unsigned int*) (ADC_INT_1 | ADCSSMUX0) |= ((1 << 3) | (1 << 4)  | (1 << 7)  );


    // For each sample in the sample sequence, configure the sample control bits in the corresponding
    // nibble in the ADCSSCTLn register. When programming the last nibble, ensure that the END bit
    // is set. Failure to set the END bit causes unpredictable behavior.


    * (volatile unsigned int*) (ADC_INT_0 | ADCSSCTL0) |= ((1 << 29));
    * (volatile unsigned int*) (ADC_INT_1 | ADCSSCTL0) |= ((1 << 5));

    * (volatile unsigned int*) (ADC_INT_0 | ADCSSCTL0) |= ((1 << 30));
    * (volatile unsigned int*) (ADC_INT_1 | ADCSSCTL0) |= ((1 << 6));

    // Bits 3,7,11,...(TS) auf 0 setzen um MUX als Quelle zu wählen
    * (volatile unsigned int*) (ADC_INT_0 | ADCSSCTL0) &= ~((1 << 3) | (1 << 7) | (1 << 11) | (1 << 15) | (1 << 19) | (1 << 23) | (1 << 27) | (1 << 31));
    * (volatile unsigned int*) (ADC_INT_1 | ADCSSCTL0) &= ~((1 << 3) | (1 << 7) | (1 << 11) | (1 << 15) | (1 << 19) | (1 << 23) | (1 << 27) | (1 << 31));

    // SSOP0 -> 1 samples are sent to the digital comparator; samples are sent to the FIFO -> 0
    * (volatile unsigned int*)( ADC_INT_0 | ADCSSOP0 ) &= ~((1 << 0) | (1 << 4) | (1 << 8) | (1 << 12) | (1 << 16) | (1 << 20) | (1 << 24) | (1 << 28));
    * (volatile unsigned int*)( ADC_INT_1 | ADCSSOP0 ) &= ~((1 << 0) | (1 << 4) | (1 << 8) | (1 << 12) | (1 << 16) | (1 << 20) | (1 << 24) | (1 << 28));

    // If interrupts are to be used, set the corresponding MASK bit in the ADCIM register.

    * (volatile unsigned int*) (ADC_INT_0 | ADCIM) |= ((1 << 0));
    * (volatile unsigned int*) (ADC_INT_1 | ADCIM) |= ((1 << 0));

    //bis hier in prozTrigconv


    // Enable the sample sequencer logic by setting the corresponding ASENn bit in the ADCACTSS register.

    * (volatile unsigned int*) (ADC_INT_0 | ADCADCTSS) |= ((1 << 0));
    * (volatile unsigned int*) (ADC_INT_1 | ADCADCTSS) |= ((1 << 0));

	clearIntFlag();
	return true;
}

void ADC::clearIntFlag() {

    // Diese Funktion löscht den Interrupt flag, welcher verwendet werden kann um anzu-
    // zeigen, dass eine Conversion abgeschlossen wurde. Hierbei handelt es sich jedoch um
    // einen Interrupt der nicht an die NVIC (Nested Vectored Interrupt Table) weitergegeben wird!


    //ADC Raw Interrupt Status (ADCRIS), offset 0x004
    // Bit 16, INRDC, Type: Read only --> Value 1: At least one bit in the ADCDCISC register is set, meaning that a digital comparator interrupt has occurred.
    //int check = 0x10000 && (ADC_INT_0 | ADC_INT_OFF);
    //bool (* (volatile unsigned int*) (ADC_INT_0 | ADC_INT_OFF) & (1<<15) )

    //int check2 = 0x10000 && (ADC_INT_1 | ADC_INT_OFF);


    * (volatile unsigned int*) (ADC_INT_0 | ADC_INT_CLEAR) |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
    * (volatile unsigned int*) (ADC_INT_1 | ADC_INT_CLEAR) |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);





}

void ADC::prozessorTriggerConverion() {

    // Startet einen oder mehrere Sample-Sequenzer- und damit ADC-Conversions gleichzeitig.
    // Wie Triggern (Prozessor, GPIO Signal, analog comparators?
    // Sollen die verschiedenen Pins  in einer Endlosschleife stets abgefragt werden oder nur zu bestimmten Zeitpunkten?
    // Muss eine Mittelwertbildung erfolgen?
    // Bei der Initialisierung des Sample Sequencers ist ein Schritt: "For each sample in the sample sequence, configure the corresponding input source in the ADCSSMUXn register." Dieses Register zeigt doch aber einfach nur an, welcher AIN-Eingang genutzt wurde, oder? Dann muss ich da doch nichts konfigurieren, sondern nur auslesen. Oder schreibe ich da aktiv rein, welcher Eingang wann ausgelesen wird?

    // Configure the trigger event for the sample sequencer in the ADCEMUX register.
    // offset 0x014
    // Auf Processor (default) gelassen!

    // The trigger is initiated by setting the SSn bit in the ADCPSSI register.
    // Offset 0x028

    * (volatile unsigned int*) (ADC_INT_0 | ADCPSSI) |= (1 << 0)| (1 << 27);
    * (volatile unsigned int*) (ADC_INT_1 | ADCPSSI) |= (1 << 0)| (1 << 31);



}

bool ADC::IsConversionFinished() {

    // Gibt true zurück, wenn eine Conversion abgeschlossen wurde und false , wenn noch Conversions ablaufen.




    if((* (volatile unsigned int*) (ADC_INT_0 | ADCRIS) & (1<<0) ) == 0){

        return true;
    }

    if((* (volatile unsigned int*) (ADC_INT_1 | ADCRIS) & (1<<0) ) == 0){

        return true;
    }
    else
    {
        return false;
    }


}

unsigned long ADC::getChannelValue( unsigned long channelID ) {

    // Diese Funktion gibt den im Buffer gespeicherten Wert der ADC-Conversion, abhängig
    // von der übergebenen channelID, zurück. Vor der Rückgabe wird dieser Wert (12 bit
    // ADC-Ergebnis) jedoch auf das Ergebnis eines 10 bit ADC zurückgerechnet, damit der
    // Regler damit arbeiten kann.


    //     - Die Methode "unsigned long ADC::getChannelValue( unsigned long channelID )"
    // wird dann vom ADCSensor Objekt aufgerufen und gibt den Wert des entsprechenden
    // AINn Kanals, wobei n=channelID gilt, zurück

    // ADC Sample Sequence Result FIFO 0 (ADCSSFIFO0), offset 0x048


/*
    if(channelID==9)
    {
        return ui32ADC0Value[3];

    }
    else
    {
        return ui32ADC0Value[channelID];
    }*/

    return (ui32ADC0Value[channelID]/4.0);

}

 void ADC::ADCUpdateDataValues() {

     // Hier wird das FIFO (First in first out) Register der verwendeten Sample Sequenzer
     // ausgelesen und zwischengespeichert.

     //- Diese Sequenz sollst du in der Methode "void ADC::ADCUpdateDataValues()"
     // auslesen (hierfür SSFIFOn-Register benutzen) und in der Klasse ADC in einer
     //  Klassenvariablen speichern
   int i = 0;
  /*
     while(IsConversionFinished() == 0 && i<9)
     {
         ui32ADC0Value[i] = (*(volatile unsigned int*) (ADC_INT_0 | ADCSSFIFO0) & 0b111111111111)/4;
         ui32ADC0Value[8] = (*(volatile unsigned int*) (ADC_INT_1 | ADCSSFIFO0) & 0b111111111111)/4;
         i++;
     }

*/



// solange FIFO nicht leer ist
   while (i < 8)
   {
               //if (i<8)
               //{
                   if (i == 3)
                   { // AIN3 not configured / not used
					   ui32ADC0Value[i] = *(volatile unsigned int*)(ADC_INT_0 | ADCSSFIFO0);
                       ui32ADC0Value[i] = 0;
                       ++i;
                   }
                   ui32ADC0Value[i] = *(volatile unsigned int*)(ADC_INT_0 | ADCSSFIFO0);
               /*}
               // FIFO muss komplett ausgelesen werden!
               // --> normal ist eig bei bit 8 Schluss, aber sicher ist sicher :P
               else if (i >= 8)
               {
                   unimportant_values = *(volatile unsigned int*)(ADC_INT_0 | ADCSSFIFO0); // Um sicherzugehen, dass FIFO vollständig ausgelesen wird, werden mehr Daten ausgelesen -> Overflow vermeiden
               }*/
               i++;
    }


   // j describing AIN j
   // AIN 8,9
   int j = 8;
   while(j < 16) {
	   if( j <10 ){
		   ui32ADC0Value[j] = * (volatile unsigned int*)(ADC_INT_1 | ADCSSFIFO0);
	   }
	   else if( j >= 10 ){
		   unimportant_values = * (volatile unsigned int*)(ADC_INT_1 | ADCSSFIFO0);
	   }
	   j++;
    }






}


void ADC::cleanUpADC() {


    // Wird vom Destruktor genutzt um Ausgangspins zu deaktivieren und Variablen zu löschen.

    // * (volatile unsigned int*) (RCGC | RCGCADC) &= ~(1 << 0);

    // Eventuell nicht nötig. --> Destruktor

}







