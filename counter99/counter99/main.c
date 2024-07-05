/*
 * test1.c
 *
 * Created: 15.03.2024 13:10:00
 * Author : Zaxar
 */ 

#define F_CPU	16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>
// (1 << 7) | (1 << 6) |
	volatile static int z = 0;
	volatile static int t = 0;
	volatile static int e = 0;

void count0 (){
	
	switch (e){
		case 0:{
			PORTE = (1 << 1) | (1 << 0) | (1 << 2); 
			PORTB = (1 << 6) | (1 << 2) | (1 << 3) | (0 << 5); //0
			break;
		}
		case 1:{
			PORTE = (0 << 1) | (1 << 0) | (1 << 2);
			PORTB = (0 << 6) | (0 << 2) | (0 << 3) | (0 << 5); //1
			break;
		}
		case 2:{
			PORTE = (1 << 1) | (1 << 0) | (0 << 2); 
			PORTB = (0 << 6) | (1 << 2) | (1 << 3) | (1 << 5); //2 
			break;
		}
		case 3:{
			PORTE = (1 << 1) | (1 << 0) | (1 << 2); 
			PORTB = (0 << 6) | (0 << 2) | (1 << 3) | (1 << 5); //3
			break;
		}
		case 4:{
			PORTE = (0 << 1) | (1 << 0) | (1 << 2);
			PORTB = (1 << 6) | (0 << 2) | (0 << 3) | (1 << 5); //4
			break;
		}
		case 5:{
			PORTE = (1 << 1) | (0 << 0) | (1 << 2); 
			PORTB = (1 << 6) | (0 << 2) | (1 << 3) | (1 << 5); //5
			break;
		}
		case 6:{
			PORTE = (1 << 1) | (0 << 0) | (1 << 2); 
			PORTB = (1 << 6) | (1 << 2) | (1 << 3) | (1 << 5); //6
			break;
		}
		case 7:{
			PORTE = (1 << 1) | (1 << 0) | (1 << 2); 
			PORTB = (0 << 6) | (0 << 2) | (0 << 3) | (0 << 5); //7
			break;
		}
		case 8:{
			PORTE = (1 << 1) | (1 << 0) | (1 << 2); 
			PORTB = (1 << 6) | (1 << 2) | (1 << 3) | (1 << 5); //8
			break;
		}
		case 9:{
			PORTE = (1 << 1) | (1 << 0) | (1 << 2); 
			PORTB = (1 << 6) | (0 << 2) | (1 << 3) | (1 << 5); //9
			break;
		}
	}	
}

int main(void)
{

	DDRE = (1 << DDE6) | (1 << DDE7) | (1 << DDE1) | (1 << DDE0) | (1 << DDE2) ;
	DDRB = (1 << DDB6) | (1 << DDB2) | (1 << DDB3) | (1 << DDB5) | (1 << DDB0);
	
	TIMSK = (1 << OCIE2) | (1 << OCIE0);
	

	// первый счётчик
	TCCR0 = (1 << CS00) | (1 << CS01) | (1 << CS02) |
	(0 << WGM00) | (1 << WGM01);
	OCR0 = 7;

	
	// второй счётчик
	TCCR2 = (1 << CS20) | (0 << CS21) | (1 << CS22) |
	(0 << WGM20) | (1 << WGM21);
	OCR2 = 7;


	sei();
	
	/* Replace with your application code */
	while (1)
	{
		
	}
}

ISR(TIMER0_COMP_vect)
{
	z++;
	if (z == 2000){
		++t;
		z = 0;
	}
	if (t == 100){
		t = 0;
	}
}


ISR(TIMER2_COMP_vect)
{
	static int m = 0;
	PORTE &=~ ((1<<7)|(1<<6));
	if (m == 0){
		e = t % 10;
		count0(e);
		PORTE |=(1<<6);
	}
	if (m == 1){
		e = t / 10;
		count0(e);
		PORTE |=(1<<7);
	}
	m++;
	if (m == 2){
		m = 0;
	}
}

