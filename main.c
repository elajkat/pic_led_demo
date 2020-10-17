/*
 * File:   main.c
 * Author: lajoskatona
 *
 * Created on September 19, 2020, 9:03 AM
 */


#include <xc.h>
#include <pic.h>

#pragma config FOSC = INTRCIO  // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF      // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF     // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF        // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF       // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = OFF     // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config IESO = OFF      // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = OFF     // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

#define _XTAL_FREQ 8000000

unsigned char count = 0;
unsigned char max_count = 60;
unsigned int max_light = 4;
unsigned char toggle_rc3 = 0;
unsigned char write_to_rc3 = 0;
unsigned char led_output = 0b00000000;

//TIMER0    8-bit    $$RegValue = 256-((Delay * Fosc)/(Prescalar*4))  delay in sec and Fosc in hz
//FORMULA to calculate Delay
//Delay = ((256-REG_val)*(Prescal*4))/Fosc
// Fosc = 8MHz = 8000000Hz
// 256 - ((Delay * 8000000) / (64 * 4))
// Delay = ((256 - reg) * (64 * 4)) / 8000000
void __interrupt() tc_int(void)
{
    GIE = 0;   // Disable interrupts to be sure that no
               // new interrupt interrupts use here
    if(T0IE && T0IF) // Timer flag has been triggered due to timer overflow
    {
        TMR0 = 1;    // Load the timer Value
        count++;
        T0IF = 0;    // Clear timer interrupt flag
    }
    if (IOCA5 && RABIF) // RA5 interrupt flag and RABIF in INTCON changed
    {
        if (RA5 == 0)
            toggle_rc3 ^= 1;
        if (toggle_rc3)
                write_to_rc3 = 0b00001000;
        else
            write_to_rc3 = 0b00000000;
        PORTC = write_to_rc3 | led_output;
        RABIF = 0;
    }
    if (ADIE && ADIF && !GO_nDONE) // AD interrupt enabled and conversion finished
    {
        max_light = ((ADRESH << 8) + ADRESL) / 20;
        ADIF = 0;
        ADIE = 1;
        GO_nDONE = 1;
    }
    GIE = 1;    // Don't forget to enable again all interrupts!
}

void main(void) {
    OPTION_REG = 0b00000111;  // Timer0 Internal instruction cycle clock (F OSC /4)
                              // and 256 as prescalar
                              // Also Enables PULL UPs

    // Setup timer0 with interrupt
    TMR0 = 1;  // Load the time value for 0.0019968s; delayValue can be between 0-256 only
    T0IE = 1;  // Enable timer interrupt bit in INTCON register
    T0IF = 0;  // Clear Overflow bit (must be cleared in software)

    // Setup Portc as output
    TRISC = 0x00;
    PORTC = 0b00000000; //Initialize all pins to 0

    // Setup Porta5 as input with interrupt-on-change
    TRISA5 = 1;         // RA5 input
    IOCA5 = 1;          // RA5 interrupt enabled

    // Setup ADC with interrupt
    TRISA2 = 1;
    ANS2 = 1;           // RA2 analog input
    ADCON0 = 0b10001001;// Right justified, AN2 channel, AD enabled
    ADCON1 = 0b00100000;// A/D Conversion Clock FOSC/32
    ADIF = 0;           // Clear AD conversion flag bit, PIR1 register
    ADIE = 1;           // A/D Converter (ADC) Interrupt Enable PIE1 register

    PEIE = 1;             //Enable the Peripheral Interrupt
    GIE = 1;              //Enable Global Interrupt
    GO_nDONE = 1;

    unsigned char loop_counter = 0;

    while(1)
    {
        if (count >= max_light)
        {
            count = 0;
            // This was the simplest way I found to use
            // onyly 3 LEDs from the 4 tat is on the board,
            // and use the 4th for something else
            switch(loop_counter)
            {
                case 0:
                    led_output = 0b00000001;
                    break;
                case 1:
                    led_output = 0b00000010;
                    break;
                case 2:
                    led_output = 0b00000100;
                    break;
                case 3:
                    led_output = 0b00000010;
                    break;
                case 4:
                    led_output = 0b00000001;
                    break;

            }
            loop_counter++;
            if (loop_counter > 4)
                loop_counter = 0;
            if (toggle_rc3)
                write_to_rc3 = 0b00001000;
            else
                write_to_rc3 = 0b00000000;
            PORTC = write_to_rc3 | led_output;
        }
    }
}

