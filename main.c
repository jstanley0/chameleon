// chameleon, breadboard test
// based on AVR ATMEGA168
// build with avr-gcc
//
// Source code (c) 2007-2015 by Jeremy Stanley
// Licensed under GNU GPL v2 or later

// inputs:
// PC0 (input)  = teh button
// PC1 (input)  = photoresistor

// target illuminator:
// PC3 (output) = red LED
// PC4 (output) = green LED
// PC5 (output) = blue LED

// display:
// PD3 (OC2B) = blue LED
// PD5 (OC0B) = green LED
// PD6 (OC0A) = red LED

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

void init_adc()
{
	// power on the ADC
	PRR &= ~(1 << PRADC);
	
	// select AVCC reference, ADC1 source
	ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX0);
	
	// enable ADC and start conversions at 1/128 prescaler
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

volatile uint8_t illum = 0;

void init_illum()
{
	PORTC |= (0b1000 << illum);
}

void cycle_illum()
{
	PORTC &= ~(0b1000 << illum);
	if (++illum == 3)
		illum = 0;
	PORTC |= (0b1000 << illum);
}

void init_pwm()
{
	// timer0
	// select non-inverting fast PWM mode for red (0A) and green (0B)
	OCR0A = 0;
	OCR0B = 0;
	TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (1 << CS02);	// 1/256 prescaler
	
	// timer2
	OCR2B = 0;
	TCCR2A = (1 << COM2B1) | (1 << WGM01) | (1 << WGM00);
	TCCR2B = (1 << CS22) | (1 << CS21);	// 1/256 prescaler
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

	OCR0A = sat[0];
	OCR0B = sat[1];
	OCR2B = sat[2];
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
	DDRC  = 0b00111000;
	PORTC = 0b00000011;	// red illuminator starts on
	DDRD  = 0b01101000;
	PORTD = 0b00000000;
    
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

