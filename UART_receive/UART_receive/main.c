#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <string.h>

#define F_CPU 8000000UL
#define BAUD 9600L
#define UBRRL_value (F_CPU/(BAUD*16))-1

//посылка $01$0B$B8$78, где 0x01 - адрес устройства, 0x0B и 0xB8 - уставка 3000 мВ в форме uint16_t, 0x78 - crc8
void init_pin(void);

#define LED1_on() PORTB |= (1<<2)
#define LED1_off() PORTB &=~ (1<<2)
#define DEVICE_ADDRESS 0x01//адрес устройства

volatile uint8_t received_data[3];
volatile uint8_t received_index = 0;
volatile uint8_t led_on = 0;

//crc chat-gpt version
//work with message $01$01$01$01$01$01$01$79
//check out code for calculate crc8
uint8_t crc8(const uint8_t *data, uint8_t len){
	uint8_t crc = 0x00;
	while(len--){
		uint8_t extract = *data++;
		for(uint8_t tempI = 8; tempI; tempI--){
			uint8_t sum = (crc ^ extract) & 0x01;
			crc >>= 1;
			if(sum){
				crc ^= 0x8C;
			}
			extract >>= 1;
		}
	}
	return crc;
}

ISR(USART_RXC_vect){
	received_data[received_index] = UDR;
	received_index++;
	
	if(received_index == 4){
		
		if(received_data[0] == DEVICE_ADDRESS){
			
			if(crc8(received_data, 3) == received_data[3]){
				led_on = 1;
				LED1_on();//светодиод - индикатор успешного приема посылки
				TCNT1 = 0;//сброс таймера
				uint16_t voltage = (received_data[1] << 8) | received_data[2];//декодирование принимаемой уставки из посылки
				if(voltage == 3000){
					PORTB ^= (1 << 0);//проверка правильности декодирования уставки
				}
			}
		}
		memset(received_data, 0, sizeof(received_data));//зануление массива
		received_index = 0;//сброс индекса принимаемого байта посылки
		PORTB ^= (1 << 1); //проверка захода в цикл
	}
}


void init_USART(){
	UBRRL = UBRRL_value;
	UBRRH = UBRRL_value >> 8;
	UCSRB |= (1 << TXEN)|(1 << RXEN);
	UCSRC |= (1 << URSEL)|(1 << UCSZ0)|(1 << UCSZ1);
	UCSRB |= (1 << RXCIE);
	sei();
}

//16-bit Timer1
void init_Timer1(){
	TCCR1B |= (1 << WGM12) | (1 << CS12) | (1 << CS10);// TC режим, предделитель 1024
	OCR1A = 7812;
	TIMSK |= (1 << OCIE1A);//Включение прерывания по совпадению
}




int main(void)
{
	init_Timer1();
	init_pin();
	init_USART();
	sei();
	
	while(1){

	}
}

ISR(TIMER1_COMPA_vect) {
	if (led_on) {
		led_on = 0;
		LED1_off();
	}
}

void init_pin(void){
	DDRB |= (1 << 2) | (1 << 1) | (1 << 0);
	PORTB &=~ (1 << 2);
}