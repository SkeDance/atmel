/*
 * ADC.c
 *
 * Created: 26.07.2024 14:41:52
 * Author : alejn
 */ 

#include <avr/io.h>


int main(void)
{
	
	DDRB |= (1 << 2) | (1 << 1) | (1 << 0);
	PORTB &=~ ((1 << 2) | (1 << 1) | (1 << 0));
	DDRC &=~ (1 << 1);
	
	ADCSRA |= (1 << ADEN);//разрешаем работу АЦП
	ADCSRA |= (1 << ADFR);//режим непрерывного преоразования
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1);//частота дискретизации 125 кГЦ, делитель 64
	ADMUX |= (1 << REFS1) | (1 << REFS0);//внутренний ИОН 2.56 В
	
	//for c_ex ADMUX |= (1 << REFS0);
	ADMUX &=~ (1 << ADLAR);//po pravoi storone
	
	//vivod ADC1
	ADMUX |= (1 << MUX0); 
	ADMUX &=~ (1 << MUX3) | (1 << MUX2) | (1 << MUX1);
	//
	
	ADCSRA |= (1 << ADSC);//zapusk ADC
	
	//adc power connections datasheet figure 96 
    while (1){
		if(ADCSRA & (1 << 4)){
			if(ADC >= 960){
				PORTB |= (1 << 0);
				PORTB &=~ (1 << 1);
				PORTB &=~ (1 << 2);
			}
			if(ADC >= 560 && ADC < 680){
				PORTB &=~ (1 << 0);
				PORTB |= (1 << 1);
				PORTB &=~ (1 << 2);
			}
			if(ADC < 400){
				PORTB &=~ (1 << 0);
				PORTB &=~ (1 << 1);
				PORTB |= (1 << 2);
			}	
		ADCSRA |= (1 << 4);//возврат флага прерывания на АЦП в 1, необходимо для работы цикла if
		}
	}
}

