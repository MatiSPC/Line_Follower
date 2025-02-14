#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <stdio.h>

volatile uint16_t left_sensor = 0;
volatile uint16_t right_sensor = 0;
volatile uint16_t granica = 0;

// Definicje pr�dko�ci transmisji (Baud Rate)
#define BAUD 9600
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)

void ADC_Init()
{
	// Ustawienie referencji napi�cia
	ADMUX |= (1 << REFS0);
	// W��czenie ADC i ustawienie preskalera na 64
	ADCSRA |= (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t read_left_sensor()
{
	// Wyb�r kana�u ADC - 1
	ADMUX |= (1 << MUX0);
	// Rozpocz�cie konwersji
	ADCSRA |= (1 << ADSC);
	// Oczekiwanie na zako�czenie konwersji
	while (ADCSRA & (1 << ADSC));
	// Zwr�cenie wyniku konwersji
	return ADC;
}

uint16_t read_right_sensor()
{
	// Wyb�r kana�u ADC - 0
	ADMUX &= ~(1 << MUX0);
	// Rozpocz�cie konwersji
	ADCSRA |= (1 << ADSC);
	// Oczekiwanie na zako�czenie konwersji
	while (ADCSRA & (1 << ADSC));
	// Zwr�cenie wyniku konwersji
	return ADC;
}

bool isPressedStartButton()
{
	if(!(PIND & (1 << PIND3)))
	{
		return true;
	}
	return false;
}

bool isPressedPomiarKartka()
{
	if (!(PIND & (1 << PIND5))) {
		return true;
	}
	return false;
}

bool isPressedPomiarTasma()
{
	if (!(PIND & (1 << PIND6))) {
		return true;
	}
	return false;
}

void PWM_init()
{
	// Ustawienie trybu Fast PWM oraz ustawienie preskalera na 128
	TCCR1A |= (1 << WGM10) | (1 << WGM11);
	TCCR1B |= (1 << CS12);
	
	// Ustawienie wyj�cia na pinie OC1A (PB1 oraz PB2)
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
	
}

bool LeftSensorOK(uint16_t sensor)
{
	if(sensor < granica)
	{
		return 1;
	}
	return 0;
}

bool RightSensorOK(uint16_t sensor)
{
	if(sensor < granica)
	{
		return 1;
	}
	return 0;
}

void LeftMotor(uint8_t predkosc)
{
	OCR1A = predkosc; //PB1 D9
	PORTB |= (1 << PORTB0); //PB0 D8
	PORTD &= ~(1 << PORTD7); //PD7 D7
}
void RightMotor(uint8_t predkosc)
{
	OCR1B = predkosc; //PB2 D10
	PORTD |= (1 << PORTD4); //PD4 D4
	PORTD &= ~(1 << PORTD2); //PD2 D2
}
void StopMotors()
{
	PORTB &= ~(1 << PORTB0);
	PORTD &= ~(1 << PORTD7);
	PORTD &= ~(1 << PORTD4);
	PORTD &= ~(1 << PORTD2);
}

// Funkcja do inicjalizacji USART
void USART_Init(unsigned int ubrr) {
	// Ustawienie pr�dko�ci transmisji
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;
	// W��czenie nadajnika i odbiornika
	UCSR0B = (1 << TXEN0) | (1 << RXEN0);
	// Ustawienie formatu ramki: 8 bit�w danych, 1 bit stopu, bez parzysto�ci
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Funkcja do wysy�ania danych przez USART
void USART_Transmit(unsigned char data) {
	// Czekanie a� bufor nadawczy b�dzie pusty
	while (!(UCSR0A & (1 << UDRE0)))
	;
	// Wys�anie danych
	UDR0 = data;
}

// Funkcja do odbierania danych przez USART
unsigned char USART_Receive(void) {
	// Czekanie a� dane zostan� odebrane
	while (!(UCSR0A & (1 << RXC0)));
	// Odczytanie i zwr�cenie odebranych danych
	return UDR0;
}

int main(void)
{
	//definiowanie zmiennych
	uint8_t wypelnienie1 = 200;
	uint8_t wypelnienie2 = 120;
	uint16_t pomiar_kartka = 10;
	uint16_t pomiar_tasma = 10;
	bool isMoving = false;
	bool pom1 = false;
	bool pom2 = false;

	//ustawienie pin�w jako output
	DDRB |= (1 << DDB0);
	DDRB |= (1 << DDB1); //PWM
	DDRB |= (1 << DDB2); //PWM
	DDRD |= (1 << DDD4);
	DDRD |= (1 << DDD7);
	DDRD |= (1 << DDD2);
	
	//ustawienie pin�w jako input - przyciski
	DDRD &= ~(1 << DDD3);
	DDRD &= ~(1 << DDD5);
	DDRD &= ~(1 << DDD6);
	
	//podci�gni�cie rezystor�w pullup - do przycisk�w
	PORTD |= (1 << PORTD3);
	PORTD |= (1 << PORTD5);
	PORTD |= (1 << PORTD6);
	
	//inicjalizacja ADC
	ADC_Init();
	
	//inicjalizacja PWM
	PWM_init();
	
	//w��czenie globalnych przerwa�
	//sei();
	
	// Inicjalizacja USART z wybran� pr�dko�ci� transmisji
	USART_Init(BAUDRATE);

	while (1)
	{
		
		//kalibracja czujnik�w - ustalenie granicy
		if(isPressedPomiarTasma())
		{
			while(isPressedPomiarTasma());
			pomiar_tasma = read_left_sensor();
			pom1 = true;
		}
		
		if(isPressedPomiarKartka())
		{
			while(isPressedPomiarKartka());
			pomiar_kartka = read_right_sensor();
			pom2 = true;
		}
		
		granica = (pomiar_kartka + pomiar_tasma)/2;
		
		if (pom1) {
			char buffer[20];
			sprintf(buffer, "Tasma: %u\r\n", pomiar_tasma);
			for (uint8_t i = 0; buffer[i]; i++) {
				USART_Transmit(buffer[i]);
			}
			pom1 = false;
		}
		
		if (pom2) {
			char buffer[20];
			sprintf(buffer, "Kartka: %u\r\n", pomiar_kartka);
			for (uint8_t i = 0; buffer[i];i++) {
				USART_Transmit(buffer[i]);
			}
			pom2 = false;
		}
		
		//g��wny program - warunki jazdy pojazdu
		
		if(isPressedStartButton() && isMoving == false)
		{
			if(LeftSensorOK(left_sensor) == true && RightSensorOK(right_sensor) == true) //je�li oba czujniki widz� lini�
			{
				//jazda prosto
				LeftMotor(wypelnienie1);
				RightMotor(wypelnienie1);
			}
			else if(LeftSensorOK(left_sensor) == false)
			{
				//jazda po �uku w prawo
				LeftMotor(wypelnienie1);
				RightMotor(wypelnienie2);
			}
			else if(RightSensorOK(right_sensor) == false)
			{
				//jazda po �uku w lewo
				LeftMotor(wypelnienie2);
				RightMotor(wypelnienie1);
			}
			isMoving = true;
		}
		else if(isPressedStartButton() && isMoving == true)
		{
			StopMotors();
			isMoving = false;
		}
	}
}
