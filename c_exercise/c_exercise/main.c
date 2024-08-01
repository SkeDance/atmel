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
#define DEVICE_ADDRESS 0x01// адрес устройства

volatile uint8_t received_data[3];
volatile uint8_t received_index = 0;
volatile uint8_t led_on = 0;
volatile uint16_t ms_count = 0;//счетчик для стабилизации
volatile uint16_t ms_count2 = 0;//счетчик для светодиода
volatile uint16_t voltage;//принимаемая уставка
volatile int z = 0;
volatile int msg_rec;//флаг приема посылки
volatile uint16_t control_signal;
volatile uint16_t setpoint;
float Kp = 0.02; // значение пропорционального коэффициента

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
			
			//if(crc8(received_data, 3) == received_data[3])
			{
				led_on = 1;
				msg_rec = 1;
				LED1_on();// индикатор успешного приема посылки
				TCNT0 = 0;// сброс таймера
				voltage = (received_data[1] << 8) | received_data[2];// декодирование принимаемой уставки из посылки
				/* проверка правильности декодирования уставки
				if(voltage == 1000){
					PORTB ^= (1 << 2);
				}
				*/
			}
		}
		memset(received_data, 0, sizeof(received_data));//зануление массива
		received_index = 0;//сброс индекса принимаемого байта посылки
		//PORTB ^= (1 << 2); //проверка захода в цикл
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
	// Настройка таймера 2 для генерации прерываний каждые 1 мс
	TCCR2 |= (1 << WGM21);
	OCR2 = 124; // Сравниваемое значение (для 1 мс при тактовой частоте 8 МГц и предделителе 64)
	TIMSK |= (1 << OCIE2); // Включение прерывания при совпадении с OCR2
	TCCR2 |= (1 << CS22); // Предделитель 64 (для 8 МГц это дает 1 мс)
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
	
	
	ADCSRA |= (1 << ADEN);//ADC enable
	ADCSRA |= (1 << ADFR);// режим непрерывного преоразования
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0);
	ADMUX |= (1 << REFS1) | (1 << REFS0);// внутренний ИОН 2.56 В
	ADMUX &=~ (1 << ADLAR);// po pravoi storone
	ADMUX |= (1 << MUX0);
	ADMUX &=~ (1 << MUX3) | (1 << MUX2) | (1 << MUX1);
	ADCSRA |= (1 << ADSC);
	//ADMUX |= (0 << REFS1) | (1 << REFS0);
	
		
	/*Maksim
		//ADC1 input
		//DDRC &=~ (1 << 1);
		//ADMUX |= (1 << MUX0);
		//ADMUX &=~ (1 << MUX3) | (1 << MUX2) | (1 << MUX1);

		//ADCSRA |= (1 << ADFR);// режим непрерывного преоразования
		//ADMUX |= (1 << REFS1) | (1 << REFS0);// внутренний ИОН 2.56 В
		ADMUX |= (0 << REFS1) | (1 << REFS0);
		//ADMUX &=~ (1 << ADLAR);// po pravoi storone
		ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
		*/
}

uint16_t read_adc() {
	while(1){
		if(ADCSRA & (1 << 4)){
			ADCSRA |= (1 << 4);	
		}
	return ADC;	
	}
	/*Maksim (uint8_t channel)
	ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
	*/
}

void init_led() {
	DDRB |= (1 << PB0); // Настройка PB0 как выход
	DDRD |= (1 << PD7) | (1 << PD5);
	DDRB |= (1 << 2);
	
}

void init_pwm() {

	TCCR1A = (1 <<COM1A1) | (0 << COM1A0) | (1 << WGM11) | (1 << WGM10);
	TCCR1B = (1 <<CS11);
	
	
	/* MAKSIM invert
	TCCR1A = (1 <<COM1A1) | (1 << COM1A0) | (1 << WGM11) | (1 << WGM10);
	TCCR1B = (1 <<CS11);
	*/
	
	/*
	ICR1 = F_CPU / (PWM_FREQ * 2);//2000
	
	//PWM Frequency
	ICR1 = F_CPU / (PWM_FREQ * 2);//2000
	
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
	*/
	//PB1 output
	DDRB |= (1 << 1);
	
	//Low level signal PB1
	PORTB &=~ (1<<1);
	
}

void set_pwm() {
	OCR1A = control_signal; // Установка значения ШИМ
}

int main(void) {
	
	uint16_t current_value;
	int16_t error;
	
	//int16_t control_signal;

	init_adc();
	init_pwm();
	init_timer2();
	timer0_init();
	init_led();
	init_USART();
	
	sei();

	while (1) {
		setpoint = (float)1023 * voltage / (float)2560; // принимаемая уставка
		current_value = read_adc();
		if(setpoint > current_value){
			error = setpoint - current_value; 	
			control_signal += (Kp * error); 
		}
		else{
			error = current_value - setpoint;
			control_signal -= (Kp * error);
		}
		
		// Вычисление ошибки
		//control_signal += (Kp * error); // Вычисление управляющего воздействия
		
		// Ограничение управляющего сигнала в пределах допустимых значений
		if (control_signal < 0) {
			
			control_signal = 0;
			} else if (control_signal > 1023) {
			control_signal = 1023;
		}
		
		set_pwm(control_signal); // Применение управляющего воздействия

		// Проверка значения АЦП и мигание светодиода
		uint16_t lower_bound = setpoint - (setpoint / 5);
		uint16_t upper_bound = setpoint + (setpoint / 5);
		if(msg_rec == 1){
			//msg_rec = 0;
			if (current_value >= lower_bound && current_value <= upper_bound) {
				//PORTB |= (1 << 2);
				if(ms_count2 >= 500){
					PORTB ^= (1 << 2);
					ms_count2 = 0;
				}
			}
			else {
				//PORTB &=~ (1 << 2);
				if(ms_count2 >= 100){
					PORTB ^= (1 << 2);
					ms_count2 = 0;
				}
			}
		}
		//delay_ms(100); // Задержка для стабилизации
	}
	return 0;
}

//timer1 used for PWM
//timer0 and timer2 used to count ms for indication