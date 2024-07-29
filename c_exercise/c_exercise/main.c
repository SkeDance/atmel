#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <string.h>

#define F_CPU 8000000UL
#define PWM_FREQ 2000
#define BAUD 9600L
#define UBRRL_value (F_CPU/(BAUD*16))-1

#define LED1_on() PORTD |= (1 << 5)
#define LED1_off() PORTD &=~ (1 << 5)
#define DEVICE_ADDRESS 0x01// ����� ����������

volatile uint8_t received_data[3];
volatile uint8_t received_index = 0;
volatile uint8_t led_on = 0;
volatile uint16_t ms_count = 0;//������� ��� ������������
volatile uint16_t ms_count2 = 0;//������� ��� ����������
volatile uint16_t voltage;//����������� �������
volatile int z = 0;
volatile int msg_rec;//���� ������ �������
float Kp = 0.1; // �������� ����������������� ������������

//message $01$03$E8$D5

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
				msg_rec = 1;
				LED1_on();// ��������� ��������� ������ �������
				TCNT0 = 0;// ����� �������
				voltage = (received_data[1] << 8) | received_data[2];// ������������� ����������� ������� �� �������
				/* �������� ������������ ������������� �������
				if(voltage == 1000){
					PORTB ^= (1 << 2);
				}
				*/
			}
		}
		memset(received_data, 0, sizeof(received_data));//��������� �������
		received_index = 0;//����� ������� ������������ ����� �������
		//PORTB ^= (1 << 2); //�������� ������ � ����
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

void timer0_init() {
	TCCR0 = 0b00000101;
	TCNT0 = 0;
	TIMSK |= (1 << TOIE0);
}


ISR(TIMER0_OVF_vect) {
	if (led_on) {
		z++;
		if(z == 10){
			led_on = 0;
			LED1_off();
			//PORTB ^= (1 << 2);
			z = 0;
		}
	}
}

void init_timer2() {
	// ��������� ������� 2 ��� ��������� ���������� ������ 1 ��
	TCCR2 |= (1 << WGM21);
	OCR2 = 124; // ������������ �������� (��� 1 �� ��� �������� ������� 8 ��� � ������������ 64)
	TIMSK |= (1 << OCIE2); // ��������� ���������� ��� ���������� � OCR2
	TCCR2 |= (1 << CS22); // ������������ 64 (��� 8 ��� ��� ���� 1 ��)
}

ISR(TIMER2_COMP_vect) {
	ms_count++;
	ms_count2++;
}

void delay_ms(uint16_t ms) {
	ms_count = 0;
	while (ms_count < ms);
}

void init_adc() {
	//ADC1 input
	DDRC &=~ (1 << 1);
	ADMUX |= (1 << MUX0);
	ADMUX &=~ (1 << MUX3) | (1 << MUX2) | (1 << MUX1);

	ADCSRA |= (1 << ADFR);// ����� ������������ �������������
	ADMUX |= (1 << REFS1) | (1 << REFS0);// ���������� ��� 2.56 �
	ADMUX &=~ (1 << ADLAR);// po pravoi storone
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

uint16_t read_adc() {
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;

}

void init_led() {
	DDRB |= (1 << PB0); // ��������� PB0 ��� �����
	DDRD |= (1 << PD7) | (1 << PD5);
	DDRB |= (1 << 2);
	
}

void init_pwm() {
	
	//PWM Frequency
	ICR1 = F_CPU / (PWM_FREQ * 8 * 2);//250
	
	//prescaler 8
	TCCR1B |= (1<<CS11);
	TCCR1B &=~ (1<<CS10);
	TCCR1B &=~ (1<<CS12);
	
	//PWM OC1A
	TCCR1A |= (1<<COM1A1);
	TCCR1A &=~ (1<<COM1A0);
	
	//Phase and Frequency Correct PWM
	TCCR1A &=~ (1<<WGM10);
	TCCR1A |= (1<<WGM11);
	TCCR1B |= (1<<WGM12);
	TCCR1B |= (1<<WGM13);
	
	//PB1 output
	DDRB |= (1 << 1);
	
	//Low level signal PB1
	PORTB &=~ (1<<1);
}

void set_pwm(uint16_t value) {
	OCR1A = value; // ��������� �������� ���
}

int main(void) {
	uint16_t setpoint = (1023 * (voltage / 1000)) / 2.56; // ����������� �������
	uint16_t current_value;
	int16_t error;
	int16_t control_signal;

	init_adc();
	init_pwm();
	init_timer2();
	timer0_init();
	init_led();
	init_USART();
	
	sei();

	while (1) {
		current_value = read_adc();
		error = setpoint - current_value; // ���������� ������
		control_signal = Kp * error; // ���������� ������������ �����������
		
		// ����������� ������������ ������� � �������� ���������� ��������
		if (control_signal < 0) {
			
			control_signal = 0;
			} else if (control_signal > 1023) {
			control_signal = 1023;
		}
		
		set_pwm((uint16_t)control_signal); // ���������� ������������ �����������

		// �������� �������� ��� � ������� ����������
		uint16_t lower_bound = setpoint - (setpoint / 10);
		uint16_t upper_bound = setpoint + (setpoint / 10);
		if(msg_rec == 1){
			//msg_rec = 0;
			if (current_value >= lower_bound && current_value <= upper_bound) {
				PORTB |= (1 << 2);
				/*if(ms_count2 == 100){
					PORTB ^= (1 << 2);
					ms_count2 = 0;
				}*/
			}
			else {
				PORTB &=~ (1 << 2);
				/*if(ms_count2 == 10){
					PORTB ^= (1 << 2);
					ms_count2 = 0;
				}*/
			}
		}
		delay_ms(100); // �������� ��� ������������
	}
	return 0;
}

//timer1 used for PWM
//timer0 and timer2 used to count ms for indication