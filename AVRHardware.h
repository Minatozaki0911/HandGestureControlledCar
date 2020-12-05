#ifndef AVRHardware
#define AVRHardware
#endif
#include <stdint.h> 
#include "atmega328p.h"
#include "avr/pgmspace.h"
#define _BV(bit) (1<<(bit))

//All of this will be stored in Flash memory so don't worry about your petty 2KB of RAM
#define F_CPU 16000000L //As specified in board.txt config

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define NOT_A_PIN 0
#define NOT_A_PORT 0
#define NOT_ON_TIMER 0

#define PIN_SPI_SS    (10)
#define PIN_SPI_MOSI  (11)
#define PIN_SPI_MISO  (12)
#define PIN_SPI_SCK   (13)
static const uint8_t SS = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK = PIN_SPI_SCK;

#define PIN_WIRE_SDA        (18)
#define PIN_WIRE_SCL        (19)
static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;


volatile unsigned long timer0_overflow_count = 0;


#define digitalPinToBitMask(P) ( pgm_read_byte( digital_pin_to_bit_mask_PGM + (P) ) )
const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
        _BV(0), /*0, port D*/   //0b00000001
        _BV(1),                 //0b00000010
        _BV(2),                 //0b00000100
        _BV(3),                 //0b00001000
        _BV(4),                 //0b00010000
        _BV(5),                 //0b00100000
        _BV(6),                 //0b01000000
        _BV(7),                 //0b10000000
        _BV(0), /* 8, port B */
        _BV(1),
        _BV(2), //10    SS
        _BV(3), //11    MOSI
        _BV(4), //12    MISO
        _BV(5), //13    SCK
        _BV(0), /* 14, port C */
        _BV(1),
        _BV(2),
        _BV(3),
        _BV(4), //SDA
        _BV(5), //SCL
};
#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )
const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
        PD, /* 0 */
        PD,
        PD,
        PD,
        PD,
        PD,
        PD,
        PD,
        PB, /* 8 */
        PB,
        PB,
        PB,
        PB,
        PB,
        PC, /* 14 */
        PC,
        PC,
        PC,
        PC,
        PC,
};
#define digitalPinToTimer(P) ( pgm_read_byte( digital_pin_to_timer_PGM + (P) ) )
const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
    NOT_ON_TIMER, /* 0 - port D */
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    TIMER2B,
    NOT_ON_TIMER,
    TIMER0B,
    TIMER0A,
    NOT_ON_TIMER,
    NOT_ON_TIMER, /* 8 - port B */
    TIMER1A,
    TIMER1B,
    TIMER2A,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER, /* 14 - port C */
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
};//Compatible with ATMEGA328P. I've looked it up in manual
#define portModeRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_mode_PGM + (P))) )
const uint16_t PROGMEM port_to_mode_PGM[] = {
        NOT_A_PORT,
        NOT_A_PORT,
        (uint16_t)&DDRB,
        (uint16_t)&DDRC,
        (uint16_t)&DDRD,
};
#define portOutputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_output_PGM + (P))) )
const uint16_t PROGMEM port_to_output_PGM[] = {
        NOT_A_PORT,
        NOT_A_PORT,
        (uint16_t)&PORTB,
        (uint16_t)&PORTC,
        (uint16_t)&PORTD,
};
#define portInputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_input_PGM + (P))) )
const uint16_t PROGMEM port_to_input_PGM[] = {
        NOT_A_PORT,
        NOT_A_PORT,
        (uint16_t)&PINB,
        (uint16_t)&PINC,
        (uint16_t)&PIND,
};
#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
//
//
//for familiarity's sake I use the same name
void pinMode(uint8_t pin, uint8_t mode)
{
    uint8_t bit = digitalPinToBitMask(pin); //which bit in PORTx
    uint8_t port = digitalPinToPort(pin);   //return the according port of the specified pin
                                            //example: pin 5, bit = 

    volatile uint8_t* reg, * out;

    if (port == NOT_A_PIN) return;      //If not an existed pin, exit this func
   
    reg = portModeRegister(port);       /*! @result address of where that pin is from */
    out = portOutputRegister(port);     

    if (mode == INPUT) {
        uint8_t oldSREG = SREG; // doing this to preserve the STATUS REGISTER data
        cli();                  //turn of interrupt
        *reg &= ~bit;           //access that memory address and clear that bit
        *out &= ~bit;
        SREG = oldSREG;
    }
    else if (mode == INPUT_PULLUP) {
        uint8_t oldSREG = SREG;
        cli();
        *reg &= ~bit;
        *out |= bit;            //turn on pullup resistor
        SREG = oldSREG;
    }
    else {
        uint8_t oldSREG = SREG;
        cli();
        *reg |= bit;
        SREG = oldSREG;
    }
}
static void turnOffPWM(uint8_t timer)
{
    switch (timer)
    {
#if defined(TCCR1A) && defined(COM1A1)
    case TIMER1A:   cbi(TCCR1A, COM1A1);
        break;
#endif
#if defined(TCCR1A) && defined(COM1B1)
    case TIMER1B:   cbi(TCCR1A, COM1B1);
        break;
#endif
#if defined(TCCR0A) && defined(COM0A1)
    case  TIMER0A:  cbi(TCCR0A, COM0A1);
        break;
#endif
#if defined(TCCR0A) && defined(COM0B1)
    case  TIMER0B:  cbi(TCCR0A, COM0B1);
        break;
#endif
#if defined(TCCR2A) && defined(COM2A1)
    case  TIMER2A:  cbi(TCCR2A, COM2A1);
        break;
#endif
#if defined(TCCR2A) && defined(COM2B1)
    case  TIMER2B:  cbi(TCCR2A, COM2B1);
        break;
#endif
    }
}
void digitalWrite(uint8_t pin, uint8_t val)
{
    uint8_t timer = digitalPinToTimer(pin); //PWM check ? 
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t* out;

    if (port == NOT_A_PIN) return;

    // If the pin that support PWM output, turn it off before doing a digital write.
    if (timer != NOT_ON_TIMER) turnOffPWM(timer);

    out = portOutputRegister(port);

    uint8_t oldSREG = SREG;
    cli();

    if (val == LOW) {
        *out &= ~bit;
    }
    else {
        *out |= bit;
    }

    SREG = oldSREG;
}
int digitalRead(uint8_t pin)
{
    uint8_t timer = digitalPinToTimer(pin);
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);

    if (port == NOT_A_PIN) return LOW;

    // If the pin that support PWM output, we need to turn it off
    // before getting a digital reading.
    if (timer != NOT_ON_TIMER) turnOffPWM(timer);

    if (*portInputRegister(port) & bit) return HIGH;
    return LOW;
}
//return 1 clock per 1 microSecond
unsigned long micros() { 
    unsigned long m;
    uint8_t oldSREG = SREG;     //reserve status register
    uint8_t t;

    cli();
    m = timer0_overflow_count;  
    t = TCNT0;
    if ((TIFR0 & _BV(TOV0)) && (t < 255)) //wait for TOV0 flag
        m++;

    SREG = oldSREG;

    return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
}
