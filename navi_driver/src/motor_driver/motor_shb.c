
#include <stdint.h>
#include "penny.h"
#include "muc_timer.h"


struct pin {
	uint8_t volatile *port;
	uint8_t mask;
};

struct pwm16_out {
	uint16_t volatile *mreg; /* the match compare register */
	struct pin p;
};

struct mshb {
	struct pwm16_out pwma;
	struct pin b;
	struct pin enable;
};

#define PIN_INITIALIZER(port_, idx) { .port = &(port_), .mask = (1 << (idx)) }
#define PWM_INITIALIZER(reg, pin) { .mreg = &(reg), .p = pin }
#define MSHB_INITIALIZER(pa, pb, en)				\
		{ .pwma = pa, .b = pb, .enable = en }

#define PD_4 PIN_INITIALIZER(PORTD, 4) // arduino  4
#define PD_5 PIN_INITIALIZER(PORTD, 5) // arduino  5
#define PD_6 PIN_INITIALIZER(PORTD, 6) // arduino  6
#define PD_7 PIN_INITIALIZER(PORTD, 7) // arduino  7

#define PB_1 PIN_INITIALIZER(PORTB, 1) // arduino  9
#define PB_2 PIN_INITIALIZER(PORTB, 2) // arduino 10

#define PC_2 PIN_INITIALIZER(PORTC, 2) // arduino a2
#define PC_3 PIN_INITIALIZER(PORTC, 3) // arduino a3
#define PC_4 PIN_INITIALIZER(PORTC, 4) // arduino a4
#define PC_5 PIN_INITIALIZER(PORTC, 5) // arduino a5

#define TMR0_PWMA PWM_INITIALIZER(OCR0A, PD_6)
#define TMR0_PWMB PWM_INITIALIZER(OCR0B, PD_5)
#define TMR1_PWMA PWM_INITIALIZER(OCR1A, PB_1)
#define TMR1_PWMB PWM_INITIALIZER(OCR1B, PB_2)

static struct mshb mshb_d [] = {
	MSHB_INITIALIZER(TMR1_PWMA, PD_6, PD_4), // A= 9, B=6, INH=4 :: left
	MSHB_INITIALIZER(TMR1_PWMB, PD_5, PD_7)  // A=10, B=5, INH=7 :: right
};

#define PIN_INIT_OUT(pin) do {					\
	PIN_SET_LOW(pin);					\
	*((pin).port - 1) |= (pin).mask;			\
} while(0)

#define PIN_SET_HIGH(pin) do {					\
	*((pin).port) |= (pin).mask;				\
} while(0)

#define PIN_SET_LOW(pin) do {					\
	*((pin).port) &= (uint8_t)~(pin).mask;			\
} while(0)

#define PWM_INIT(pwm) do {					\
	PIN_INIT_OUT((pwm).p);					\
} while(0)

#define MSHB_INIT(mshb) do {					\
	PWM_INIT((mshb).pwma);					\
	PIN_INIT_OUT((mshb).enable);				\
	PIN_INIT_OUT((mshb).b);					\
} while(0)


void pwm16_set(struct pwm16_out pwm, uint16_t val15)
{
	*(pwm.mreg) = val15;
}

void mshb_init(void)
{
	/* Pin mappings:
	 * Digital 10 / OC1B / PB2 => PA / PWMA / IN (A)
	 * Digital  9 / OC1A / PB1 => PB / PWMB / IN (B)
	 * Digital  7 / PD7 => ENA / ENB / INH (A) / INH (B)
	 */
	TIMER1_INIT_PWM(INT16_MAX);
	uint8_t i;
	for (i = 0; i < ARRAY_SIZE(mshb_d); i++) {
		MSHB_INIT(mshb_d[i]);
	}

}

void mshb_enable(uint8_t i)
{
	PIN_SET_HIGH(mshb_d[i].enable);
}

void mshb_disable(uint8_t i)
{
	PIN_SET_LOW(mshb_d[i].enable);
}

void mshb_set(uint8_t i, int16_t speed)
{
	if (speed > 0) {
		pwm16_set(mshb_d[i].pwma, speed);
		PIN_SET_LOW(mshb_d[i].b);
	} else {
		pwm16_set(mshb_d[i].pwma, INT16_MAX + speed);
		PIN_SET_HIGH(mshb_d[i].b);
	}
}
