#include <WProgram.h>
#include <stdbool.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "config.hpp"
#include "encoder.hpp"

static volatile uint8_t last_enc1a = 0;
static volatile uint8_t last_enc1b = 0;
static volatile uint8_t last_enc2a = 0;
static volatile uint8_t last_enc2b = 0;

volatile int32_t motor1_ticks = 0;
volatile int32_t motor2_ticks = 0;

void encoder_init(void)
{
	//************* Intialize encoders	*********************
	// Initialize 
	DDRD &=  ~_BV(PIN2) & ~_BV(PIN3)& ~_BV(PIN4)& ~_BV(PIN5);
	PORTD = _BV(PIN2) | _BV(PIN3) |  _BV(PIN4) | _BV(PIN5);

	cli();
	PCICR  |= _BV(PCIE2);
	PCMSK2 = _BV(PCINT18) | _BV(PCINT19) | _BV(PCINT20) | _BV(PCINT21);
	sei();
}

static inline void encoder_tick(volatile uint8_t &priorA,   volatile uint8_t &priorB,
                                uint8_t           channelA, uint8_t           channelB,
                                volatile int32_t &encoderCount)
{
	   if (!(priorA ) && (channelA)) {
	     if (!channelB) {
	       encoderCount--;
	     } else {
	       encoderCount++;
	     }
	   }
	   if ((priorA) && !(channelA)) {
	     if (!channelB ) {
	       encoderCount++;
	     } else {
	       encoderCount--;
	     }
	   }

	   priorA  = channelA;

	  if (!(priorB) && (channelB)) {
	   	     if (!channelA) {
	   	       encoderCount++;
	   	     } else {
	   	       encoderCount--;
	   	     }
	   	   }
	   	   if ((priorB) && (!channelB)) {
	   	     if (!channelA ) {
	   	       encoderCount--;
	   	     } else {
	   	       encoderCount++;
	   	     }
	   	   }
	   priorB = channelB;
}

ISR(PCINT2_vect)
{
	encoder_tick(last_enc1a, last_enc1b, bit_is_set(PIND, PIN5), bit_is_set(PIND, PIN4), motor1_ticks);
	encoder_tick(last_enc2a, last_enc2b, bit_is_set(PIND, PIN2), bit_is_set(PIND, PIN3), motor2_ticks); // BROKEN
}
