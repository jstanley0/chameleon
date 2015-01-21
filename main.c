// chameleon
// based on AVR ATTINY44
// build with avr-gcc
//
// Source code (c) 2015 by Jeremy Stanley
// Licensed under GNU GPL v2 or later

// inputs:
// pin 13. PA0 (ADC0) = photoresistor

// target illuminator:
// common-cathode RGB LED
// pin 10. PA3 = red
// pin  9. PA4 = green
// pin  8. PA5 = blue
#define ILLUMINATOR_PORT PORTA
#define ILLUMINATOR_MASK 0b00111000
#define ILLUMINATOR_BIT(index) (0b1000 << (index))

// display:
// common-cathode RGB LED
// pin 7. PA6 (OC1A) = blue
// pin 6. PA7 (OC0B) = green
// pin 5. PB2 (OC0A) = red
#define R_INTENSITY OCR0A
#define G_INTENSITY OCR0B
#define B_INTENSITY OCR1A


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

void init_adc()
{
	// select AVCC reference, ADC1 source
	ADMUX = 0;
	
	// enable ADC and start conversions at 1/128 prescaler
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

volatile uint8_t illum = 0;

void init_illum()
{
	ILLUMINATOR_PORT |= ILLUMINATOR_BIT(illum);
}

void cycle_illum()
{
	ILLUMINATOR_PORT &= ~ILLUMINATOR_MASK;
	if (++illum == 3)
		illum = 0;
	ILLUMINATOR_PORT |= ILLUMINATOR_BIT(illum);
}

void init_pwm()
{
	R_INTENSITY = 0;
	G_INTENSITY = 0;
	B_INTENSITY = 0;

	// timer0
	// select non-inverting fast PWM mode for red (0A) and green (0B)
	TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (1 << CS01);	// 1/8 prescaler
	
	// timer1
	// select non-inverting 8-bit fast PWM mode for blue (1A)
	TCCR1A = (1 << COM1A1) | (1 << WGM10);
	TCCR1B = (1 << WGM12) | (1 << CS11); // 1/8 prescaler
}

uint8_t clamp(int16_t val)
{
	if (val < 0)
		return 0;
	else if (val > 255)
		return 255;
	else
		return val;
}

// result is the brightness result for the color given by illum
// (0 == full brightness; 1023 = total darkness)
void update_pwm(uint16_t result)
{
	static uint8_t rgb[3] = {0, 0, 0};
	uint8_t sat[3];
	uint8_t i, tc = (1023 - result) >> 2;

	rgb[illum] = tc;
	
	// a quick-and-dirty way to amp up the saturation
	uint8_t avg = (rgb[0] + rgb[1] + rgb[2]) / 3;
	for(i = 0; i < 3; ++i) {
		int16_t d = 6 * (rgb[i] - avg);
		sat[i] = clamp(d + rgb[i]);
	}

	R_INTENSITY = sat[0];
	G_INTENSITY = sat[1];
	B_INTENSITY = sat[2];
}

ISR(ADC_vect)
{
	// average a whole bunch of readings
	static uint32_t accum = 0;
	static uint16_t count = 0;
	
	accum += ADC;
	if (++count == 256) {
		uint16_t result = accum >> 8;
		count = 0;
		accum = 0;
		update_pwm(result);
		cycle_illum();
	}
}

int main(void)
{
	// Initialize I/O
	DDRA  = 0b11111000;
	PORTA = 0b00000000;
	DDRB  = 0b00000100;
	PORTB = 0b00000000;
    
	init_illum();
	init_adc();
	init_pwm();

	// Enable interrupts
	sei();

	// Interrupts run the show here.
	for(;;)
	{
		set_sleep_mode(SLEEP_MODE_IDLE);
		sleep_mode();
 	}
}

