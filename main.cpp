#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <stdio.h>

volatile uint16_t left_sensor = 0;
volatile uint16_t right_sensor = 0;
volatile uint16_t granica = 0;

// Definicje prędkości transmisji (Baud Rate)
#define BAUD 9600
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)

void ADCVoltageReferenceSet(){
	ADMUX |= (1 << REFS0);
}

void ADCEnable(){
	ADCSRA |= (1 << ADEN);
}

void ADCInterruptEnable(){
	ADCSRA |= (1 << ADIE);
}

void ADCSetPrescaler(){
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS1);
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

void PWMFastModeSet(){
	TCCR1A |= (1 << WGM10) | (1 << WGM11);
}

void PWMSetPrescaler(){
	TCCR1B |= (1 << CS12);
}

void PWMSetOutputPins(){
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

void USARTSetTransmissionSpeed(unsigned int ubrr){
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;
}

void USARTTransmitterTurnOn(){
	UCSR0B = (1 << TXEN0);
}

void USARTReceiverTurnOn(){
	UCSR0B = (1 << RXEN0);
}

void USARTFrameFormatSetting(){
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Funkcja do wysyłania danych przez USART
void USART_Transmit(unsigned char data) {
	// Czekanie aż bufor nadawczy będzie pusty
	while (!(UCSR0A & (1 << UDRE0)))
	;
	// Wysłanie danych
	UDR0 = data;
}

// Funkcja do odbierania danych przez USART
unsigned char USART_Receive(void) {
	// Czekanie aż dane zostaną odebrane
	while (!(UCSR0A & (1 << RXC0)));
	// Odczytanie i zwrócenie odebranych danych
	return UDR0;
}

void ADCSetChannel(){
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
}

void ADCStartConversion(){
	ADCSRA |= (1 << ADSC);
}

void WaitingForConversionToComplete(){
	while (ADCSRA & (1 << ADSC));
}

class ADCValueReader {
	private:
	uint8_t Sensor;
	
	public:
	ADCValueReader(uint8_t channel) : Sensor(channel) {}
		
	uint16_t ReadSensorMeasurment() {

		ADCSetChannel();	
		ADCStartConversion();
		WaitingForConversionToComplete();

		return ADC;
		}
	};

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

	//ustawienie pinów jako output
	DDRB |= (1 << DDB0);
	DDRB |= (1 << DDB1); //PWM
	DDRB |= (1 << DDB2); //PWM
	DDRD |= (1 << DDD4);
	DDRD |= (1 << DDD7);
	DDRD |= (1 << DDD2);
	
	//ustawienie pinów jako input - przyciski
	DDRD &= ~(1 << DDD3);
	DDRD &= ~(1 << DDD5);
	DDRD &= ~(1 << DDD6);
	
	//podciągnięcie rezystorów pullup - do przycisków
	PORTD |= (1 << PORTD3);
	PORTD |= (1 << PORTD5);
	PORTD |= (1 << PORTD6);
	
	ADCVoltageReferenceSet();
	ADCEnable();
	ADCInterruptEnable();
	ADCSetPrescaler();
	
	ADCValueReader LeftSensor(1);
	ADCValueReader RightSensor(0);
	
	PWMFastModeSet();
	PWMSetPrescaler();
	PWMSetOutputPins();

	USARTSetTransmissionSpeed(BAUDRATE);
	USARTTransmitterTurnOn();
	USARTReceiverTurnOn();
	USARTFrameFormatSetting();

	while (1)
	{
		
		//kalibracja czujników - ustalenie granicy
		if(isPressedPomiarTasma())
		{
			while(isPressedPomiarTasma());
			pomiar_tasma = LeftSensor.ReadSensorMeasurment();
			pom1 = true;
		}
		
		if(isPressedPomiarKartka())
		{
			while(isPressedPomiarKartka());
			pomiar_kartka = RightSensor.ReadSensorMeasurment();
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
		
		//główny program - warunki jazdy pojazdu
		
		if(isPressedStartButton() && isMoving == false)
		{
			if(LeftSensorOK(left_sensor) == true && RightSensorOK(right_sensor) == true) //jeśli oba czujniki widzą linię
			{
				//jazda prosto
				LeftMotor(wypelnienie1);
				RightMotor(wypelnienie1);
			}
			else if(LeftSensorOK(left_sensor) == false)
			{
				//jazda po łuku w prawo
				LeftMotor(wypelnienie1);
				RightMotor(wypelnienie2);
			}
			else if(RightSensorOK(right_sensor) == false)
			{
				//jazda po łuku w lewo
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

