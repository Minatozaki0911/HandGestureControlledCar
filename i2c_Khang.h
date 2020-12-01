#include <inttypes.h>
#include "AVRHardware.h"
///Macro section
#ifndef i2c_Khang.h
#define i2c_Khang.h
#endif

#ifndef BUFFER_LENGTH
#define BUFFER_LENGTH 32;
#endif // !BUFFER_LENGTH


#ifndef TWI_FREQ
#define TWI_FREQ 100000L
#endif
#define TWI_READY 0
/// Variables Declaring section
uint8_t rxBuffer[BUFFER_LENGTH];
uint8_t rxBufferIndex = 0;
uint8_t rxBufferLength = 0;

uint8_t txAddress = 0;
uint8_t txBuffer[BUFFER_LENGTH];
uint8_t txBufferIndex = 0;
uint8_t txBufferLength = 0;

static volatile uint8_t i2c_state; //State of transmission
static volatile uint8_t i2c_sendStop;	//Stop bit or not for 
static volatile uint8_t i2c_inRepStart;	//repeating start

static volatile uint32_t i2c_timeout_us = 0ul;
static volatile bool i2c_timed_out_flag = false;  // a timeout has been seen
static volatile bool i2c_do_reset_on_timeout = false;  // reset the TWI registers on timeout

static void (*user_onRequest)(void);
static void (*user_onReceive)(int);

static void (*i2c_onSlaveTransmit)(void);
static void (*i2c_onSlaveReceive)(uint8_t*, int);

static uint8_t transmitting = 0;
/*
 * Function twi_attachSlaveRxEvent
 * Desc     sets function called before a slave read operation
 * Input    function: callback function to use
 * Output   none
 */

void i2c_attachSlaveRxEvent(void (*function)(uint8_t*, int))
{
    i2c_onSlaveReceive = function;
}

/*
 * Function twi_attachSlaveTxEvent
 * Desc     sets function called before a slave write operation
 * Input    function: callback function to use
 * Output   none
 */
void i2c_attachSlaveTxEvent(void (*function)(void))
{
    i2c_onSlaveTransmit = function;
}

/// <summary>
/// Initate I2C lines
/// </summary>
/// <param name="">There is no param tho :) </param>
/// 
void i2c_init(void) {
	rxBufferIndex = 0;
	rxBufferLength = 0;

	txBufferIndex = 0;
	txBufferLength = 0;
	
	i2c_state = TWI_READY;
	i2c_sendStop = true;
	i2c_inRepStart = false;

	// activate internal pullups for i2C
	digitalWrite(SDA, 1);
	digitalWrite(SCL, 1);

	// initialize I2C prescaler and bit rate
	cbi(TWSR, TWPS0);
	cbi(TWSR, TWPS1);
	//SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR))
	TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;
    //Enable, ACK, Interrupt
	TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
    //Default callback function
    i2c_attachSlaveRxEvent();
    i2c_attachSlaveTxEvent();

}
void i2c_disable(void)
{
	// disable twi module, acks, and twi interrupt
	TWCR &= ~(_BV(TWEN) | _BV(TWIE) | _BV(TWEA));

	// deactivate internal pullups for twi.
	digitalWrite(SDA, 0);
	digitalWrite(SCL, 0);
}
uint8_t i2c_readFrom(uint8_t address, uint8_t* data, uint8_t length, uint8_t sendStop)
{
    uint8_t i;

    // ensure data will fit into buffer
    if (TWI_BUFFER_LENGTH < length) {
        return 0;
    }

    // wait until twi is ready, become master receiver
    uint32_t startMicros = micros();
    while (TWI_READY != twi_state) {
        if ((twi_timeout_us > 0ul) && ((micros() - startMicros) > twi_timeout_us)) {
            twi_handleTimeout(twi_do_reset_on_timeout);
            return 0;
        }
    }
    twi_state = TWI_MRX;
    twi_sendStop = sendStop;
    // reset error state (0xFF.. no error occured)
    twi_error = 0xFF;

    // initialize buffer iteration vars
    twi_masterBufferIndex = 0;
    twi_masterBufferLength = length - 1;  // This is not intuitive, read on...
    // On receive, the previously configured ACK/NACK setting is transmitted in
    // response to the received byte before the interrupt is signalled. 
    // Therefor we must actually set NACK when the _next_ to last byte is
    // received, causing that NACK to be sent in response to receiving the last
    // expected byte of data.

    // build sla+w, slave device address + w bit
    twi_slarw = TW_READ;
    twi_slarw |= address << 1;

    if (true == twi_inRepStart) {
        // if we're in the repeated start state, then we've already sent the start,
        // (@@@ we hope), and the TWI statemachine is just waiting for the address byte.
        // We need to remove ourselves from the repeated start state before we enable interrupts,
        // since the ISR is ASYNC, and we could get confused if we hit the ISR before cleaning
        // up. Also, don't enable the START interrupt. There may be one pending from the 
        // repeated start that we sent ourselves, and that would really confuse things.
        twi_inRepStart = false;			// remember, we're dealing with an ASYNC ISR
        startMicros = micros();
        do {
            TWDR = twi_slarw;
            if ((twi_timeout_us > 0ul) && ((micros() - startMicros) > twi_timeout_us)) {
                twi_handleTimeout(twi_do_reset_on_timeout);
                return 0;
            }
        } while (TWCR & _BV(TWWC));
        TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);	// enable INTs, but not START
    }
    else {
        // send start condition
        TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
    }

    // wait for read operation to complete
    startMicros = micros();
    while (TWI_MRX == twi_state) {
        if ((twi_timeout_us > 0ul) && ((micros() - startMicros) > twi_timeout_us)) {
            twi_handleTimeout(twi_do_reset_on_timeout);
            return 0;
        }
    }

    if (twi_masterBufferIndex < length) {
        length = twi_masterBufferIndex;
    }

    // copy twi buffer to data
    for (i = 0; i < length; ++i) {
        data[i] = twi_masterBuffer[i];
    }

    return length;
}


