// chameleon
// based on AVR ATTINY44
// build with avr-gcc
//
// Source code (c) 2015 by Jeremy Stanley
// Licensed under GNU GPL v2 or later

// if defined, blink out the high 4 bits of RGB values instead of mixing colors
// (bright blink = 1, low blink = 0)
//#define DIAGNOSTIC

// inputs:
// pin 13. PA0 (ADC0) = photoresistor

// target illuminator:
// common-cathode RGB LED
// pin 12. PA1 = blue
// pin 11. PA2 = green
// pin 10. PA3 = red
#define ILLUMINATOR_PORT PORTA
#define ILLUMINATOR_MASK 0b00001110
#define ILLUMINATOR_BIT(index) (0b1000 >> (index))

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
#include <util/delay.h>

void init_adc()
{
	// select AVCC reference, ADC1 source
	ADMUX = 0;
	
	// left-align results
	ADCSRB = (1 << ADLAR);
	
	// enable ADC and start conversions at 1/128 prescaler
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

volatile uint8_t illum = 0;

#ifdef DIAGNOSTIC

void display_color(uint8_t color, uint8_t intensity)
{
	R_INTENSITY = 255 - ((color == 0) ? intensity : 0);
	G_INTENSITY = 255 - ((color == 1) ? intensity : 0);
	B_INTENSITY = 255 - ((color == 2) ? intensity : 0);
}

#define BIT 300
#define INTERBIT 150
#define INTERBYTE 500

void display_bit(uint8_t bit)
{
	display_color(illum, bit ? 255 : 7);
	_delay_ms(BIT);
	display_color(4, 0);
	_delay_ms(INTERBIT);
}

void display_byte(uint8_t x)
{
	uint8_t bit = 0x80;
	
	cli();
	while(bit > 0x8)
	{
		display_bit(x & bit);
		bit >>= 1;
	}
	_delay_ms(INTERBYTE);
	sei();
}

#endif	// DIAGNOSTIC

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
	// we use 0 = max brightness; 255 = off
	// because otherwise we cannot get a 0% duty cycle for a color
	// (and 255/256 is indistinguishable from full-on)
	R_INTENSITY = 255;
	G_INTENSITY = 255;
	B_INTENSITY = 255;

	// timer0
	// select inverting fast PWM mode for red (0A) and green (0B)
	TCCR0A = (1 << COM0A1) | (1 << COM0A0) | (1 << COM0B1) | (1 << COM0B0) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (1 << CS01);	// 1/8 prescaler
	
	// timer1
	// select inverting 8-bit fast PWM mode for blue (1A)
	TCCR1A = (1 << COM1A1) | (1 << COM1A0) | (1 << WGM10);
	TCCR1B = (1 << WGM12) | (1 << CS11); // 1/8 prescaler
}

uint8_t clamp(int val)
{
	if (val < 0)
		return 0;
	else if (val > 255)
		return 255;
	else
		return val;
}

// result is the brightness result for the color given by illum
// (0 == full brightness; 255 = total darkness)
void update_pwm(uint8_t result)
{
	static uint8_t rgb[3] = {0, 0, 0};
	static const int fudge[3] = {13, 12, 11};
	uint8_t sat[3];
	uint8_t i, tc = result;
#ifdef DIAGNOSTIC
	display_byte(tc); return;
#else

	// fix color balance
	rgb[illum] = clamp((tc * fudge[illum]) / 11);

	// amp up the saturation
	uint8_t avg = (rgb[0] + rgb[1] + rgb[2]) / 3;
	for(i = 0; i < 3; ++i) {
		int16_t d = 8 * (rgb[i] - avg);
		sat[i] = clamp(d + rgb[i]);
	}

	R_INTENSITY = sat[0];
	G_INTENSITY = sat[1];
	B_INTENSITY = sat[2];
#endif
}

ISR(ADC_vect)
{
	// average a whole bunch of readings
	static int16_t accum = 0;
	static uint8_t count = 0;
	
	accum += ADCH;
	if (++count == 128) {
		uint16_t result = accum >> 7;
		count = 0;
		accum = 0;
		update_pwm(result);
		cycle_illum();
	}
}

int main(void)
{
	// Initialize I/O
	DDRA  = 0b11001110;
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

