// PIC16F886 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown Out Reset Selection bits (BOR enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#include <stdio.h>
#include <stdlib.h>
#define _XTAL_FREQ 4000000 //Crystal Oscillator Frequency


#include <xc.h>

// Status ADC
#define GO 1
#define DONE 0
#define ENABLE 1
#define DISABLE 0

// ADCON0
#define ADCS_SHIFT 6 
#define CHS_SHIFT 2
#define GO_DONE_SHIFT 2
#define ADON_SHIFT 0

#define ADCS_MASK 384 
#define CHS_MASK 120
#define GO_DONE_MASK 2
#define ADON_MASK 1

// ADC Conversion Clock
#define FRC 3 // 0b11

// Analog Channel Select Bit (CHS)
#define AN5 5 // 0b0101

// TRISB
#define TRISB5_SHIFT 5
#define TRISB5_MASK 32
#define INPUT 1
#define OUTPUT 0

// ANSEL Shifts
#define ANSEL5_SHIFT 5
#define ANSEL5_MASK 32
#define ANALOG_IN 1
#define DIGITAL_IO 0


void main(void) {
    // Clear all registers
    TRISB = 0x0;
    ANSEL = 0x0;
    ANSELH = 0x0;
    PORTB = 0x0;

    // RB5 = RP1 = potentiometer
        
    // 1 Configuration Port
    
        // Disable pin output driver (See TRIS register)
        TRISB |= (OUTPUT << TRISB5_SHIFT);
        
        // Configuration pin as analog
        ANSEL |= (ANALOG_IN << ANSEL5_SHIFT);

    // 2 configure the ADC module
        
        // select adc conversion clock
        ADCON0 |= (FRC << ADCS_SHIFT); 
        
        // configure voltage reference
        
        
        // select adc input channel
        ADCON0 |= (AN5 << CHS_SHIFT);
        
        // select result format
        
        
        // turn on adc module
        ADCON1 |= (ADON << ENABLE);
        
    // 3 configure adc interrupt (optional)
        // clear adc interrupt flag 
        // enable adc interrupt
        // enable peripheral interrupt
        // enable global interrupt
    // 4 wait the required acquisition time
    // 5 start conversion by setting the GO/DONE bit
        ADCON1 |= (GO << GO_DONE_SHIFT);
    // 6 wait for adc conversion to complete by one of the following
        // polling the GO/DONE bit
        // waiting for the adc interrupt (interrupts enabled)
    // 7 read adc result
    // 8 clear the adc interrupt flag (required if interrupt is enabled)
        
    
    return;
}
