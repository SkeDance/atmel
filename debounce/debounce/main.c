/*
 * vneshn_preriv.c
 *
 * Created: 17.05.2024 14:39:06
 * Author : alejn
 */ 

#define  F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>

volatile int millis = 0;

int main(void)
{
	
	//таймер
	TIMSK = (1<<OCIE0);
	TCCR0 = (1<<CS00)|(1<<CS01)|(1<<CS02)|(0<<WGM00)|(1<<WGM01);
	OCR0 = 7;
	
	//прерывание на кнопке
	EICRA = 0b00000010; //спад
	EIMSK = 0b00000001;
	
	//включение глобальных прерываний
	SREG |= (1<<7);
	

	/*
	D0 - button - input - HIGH
	D1 - diode 1 - output - LOW
	D4 - diode 2 - output - LOW
	D5 - diode 3 - output -HIGH
	HIGH LEVEL SIGNAL on D0
	LOW LEVEL SIGNAL on D1 and D4
	*/
	
	DDRD |= ((1<<1)|(1<<4)|(1<<5));
	DDRD &=~ (1<<0);
	PORTD |= ((1<<0)|(1<<5));// кнопка к подт€гивающему резистору, на светодиодах по 0, кроме 5-го
	PORTD &=~ ((1<<1)|(1<<4));

	/* Replace with your application code */
    while (1) 
    {

	}
}


ISR(TIMER0_COMP_vect)
{
	millis++;
}


ISR(INT0_vect)
{
	if (millis > 1000){
		PORTD ^= ((1<<1)|(1<<4)|(1<<5));
		//PORTD |= ((1<<1)|(1<<4));
		millis = 0;
	}
}




