#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <string.h>

#define F_CPU 8000000UL
#define BAUD 9600L
#define UBRRL_value (F_CPU/(BAUD*16))-1

//������� $01$0B$B8$78, ��� 0x01 - ����� ����������, 0x0B � 0xB8 - ������� 3000 �� � ����� uint16_t, 0x78 - crc8
void init_pin(void);

#define LED1_on() PORTB |= (1<<2)
#define LED1_off() PORTB &=~ (1<<2)
#define DEVICE_ADDRESS 0x01//����� ����������

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
				LED1_on();//��������� - ��������� ��������� ������ �������
				TCNT1 = 0;//����� �������
				uint16_t voltage = (received_data[1] << 8) | received_data[2];//������������� ����������� ������� �� �������
				if(voltage == 3000){
					PORTB ^= (1 << 0);//�������� ������������ ������������� �������
				}
			}
		}
		memset(received_data, 0, sizeof(received_data));//��������� �������
		received_index = 0;//����� ������� ������������ ����� �������
		PORTB ^= (1 << 1); //�������� ������ � ����
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
	TCCR1B |= (1 << WGM12) | (1 << CS12) | (1 << CS10);// TC �����, ������������ 1024
	OCR1A = 7812;
	TIMSK |= (1 << OCIE1A);//��������� ���������� �� ����������
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