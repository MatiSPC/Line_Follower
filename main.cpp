#define F_CPU 16000000UL
#define BAUD 9600
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <stdio.h>

volatile uint16_t left_sensor = 0;
volatile uint16_t right_sensor = 0;
volatile uint16_t LimitForChangingDirection = 0;
volatile uint16_t MeasurmentOverPaper = 0;
volatile uint16_t MeasurmentOverTape = 0;
uint16_t MotorSpeedFast = 200;
uint16_t MotorSpeedSlow = 120;
bool TapeMeasured = false;
bool PaperMeasured = false;

void UsePinsAsOutput(){
	DDRB |= (1 << DDB0);
	DDRB |= (1 << DDB1); //PWM
	DDRB |= (1 << DDB2); //PWM
	DDRD |= (1 << DDD4);
	DDRB |= (1 << DDB3);
	DDRD |= (1 << DDD2);
}

void UsePinsAsInput(){
	DDRD &= ~(1 << DDD3);
	DDRD &= ~(1 << DDD5);
	DDRD &= ~(1 << DDD6);
}

void PullUpResistors(){
	PORTD |= (1 << PORTD3);
	PORTD |= (1 << PORTD5);
	PORTD |= (1 << PORTD6);
}

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

bool StartButtonPressed(){
	if(!(PIND & (1 << PIND3)))
	{
		return true;
	}
	return false;
}

bool ButtonPressed(uint8_t pin){
	if (!(PIND & (1 << pin))){
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
	if(sensor < LimitForChangingDirection)
	{
		return 1;
	}
	return 0;
}

bool RightSensorOK(uint16_t sensor)
{
	if(sensor < LimitForChangingDirection)
	{
		return 1;
	}
	return 0;
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

void USARTTransmit(unsigned char data){
	while (!(UCSR0A & (1 << UDRE0)))
	;
	UDR0 = data;
}

unsigned char USART_Receive(void){
	while (!(UCSR0A & (1 << RXC0)))
	;
	return UDR0;
}

class ADCValueReader{
	private:
	uint8_t Sensor;
	
	void ADCSetChannel(){
		ADMUX = (ADMUX & 0xF0) | (Sensor & 0x0F);
	}

	void ADCStartConversion(){
		ADCSRA |= (1 << ADSC);
	}

	void WaitingForConversionToComplete(){
		while (ADCSRA & (1 << ADSC));
	}
	
	public:
	ADCValueReader(uint8_t channel) : Sensor(channel){}
		
	uint16_t ReadSensorMeasurment(){
		ADCSetChannel();	
		ADCStartConversion();
		WaitingForConversionToComplete();

		return ADC;
	}
	
	};

uint16_t CalibrateSensors(ADCValueReader LeftSensor, ADCValueReader RightSensor){
	TapeMeasured = false;
	PaperMeasured = false;
	const uint8_t leftButton = PIND5;
	const uint8_t rightButton = PIND6;
	
	if(ButtonPressed(rightButton)){
		while(ButtonPressed(rightButton));
		MeasurmentOverTape = RightSensor.ReadSensorMeasurment();
		TapeMeasured = true;
	}
	
	if(ButtonPressed(leftButton)){
		while(ButtonPressed(leftButton));
		MeasurmentOverPaper = LeftSensor.ReadSensorMeasurment();
		PaperMeasured = true;
	}
	
	return (MeasurmentOverPaper + MeasurmentOverTape)/2;
}

void WriteToConsole(int measurment, bool measured){
	if (measured){
		char buffer[20];
		sprintf(buffer, "Tasma: %u\r\n", measurment);
		for (uint8_t i = 0; buffer[i]; i++) {
			USARTTransmit(buffer[i]);
		}
		measured = false;
	}
}

struct MotorConfig{
	volatile uint8_t *port;
	volatile uint8_t *ddr;
	volatile uint8_t *ocr;
	uint8_t pinFwd;
	uint8_t pinBwd;
};

class MotorController{
	private:
	volatile uint8_t *port;
	volatile uint8_t *ddr;
	volatile uint8_t *ocr;
	uint8_t pinFwd, pinBwd;
	
	public:
	MotorController(const MotorConfig &config)
	: port(config.port), ddr(config.ddr), ocr(config.ocr), pinFwd(config.pinFwd), pinBwd(config.pinBwd){
		*ddr |= (1 << pinFwd) | (1 << pinBwd);
	}
	
	void RotateForward(uint8_t Speed){
		*ocr = Speed;
		*port |= (1 << pinFwd);
		*port &= ~(1 << pinBwd);
	}

	void RotateBackward(uint8_t Speed){
		*ocr = Speed;
		*port |= (1 << pinBwd);
		*port &= ~(1 << pinFwd);
	}

	void Stop(){
		*port &= ~(1 << pinFwd);
		*port &= ~(1 << pinBwd);
	}
	
	};

class DrivingFunctions : public MotorController{
	private:
	MotorController LeftMotorr;
	MotorController RightMotorr;
	
	public:
	DrivingFunctions(const MotorConfig &leftConfig, const MotorConfig &rightConfig)
	: LeftMotorr(leftConfig), RightMotorr(rightConfig){}
	
	void DriveForward(){
		LeftMotorr.RotateBackward(MotorSpeedFast);
		RightMotorr.RotateForward(MotorSpeedFast);
	}
	
	void DriveBackward(){
		LeftMotorr.RotateForward(MotorSpeedSlow);
		RightMotorr.RotateBackward(MotorSpeedSlow);
	}
	
	void StopVehicle(){
		LeftMotorr.Stop();
		RightMotorr.Stop();
	}
	
	void TurnLeft(){
		LeftMotorr.RotateBackward(MotorSpeedSlow);
		RightMotorr.RotateForward(MotorSpeedFast);
	}
	
	void TurnRight(){
		LeftMotorr.RotateBackward(MotorSpeedFast);
		RightMotorr.RotateForward(MotorSpeedSlow);
	}
	
	};

int main(void)
{
	uint8_t wypelnienie1 = 200;
	uint8_t wypelnienie2 = 120;
	bool isMoving = false;

	UsePinsAsOutput();
	UsePinsAsInput();
	PullUpResistors();
	
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

	MotorConfig LeftMotorConfig = { &PORTB, &DDRB, &OCR1A, PORTB0, PORTB3};
	MotorConfig RightMotorConfig = { &PORTD, &DDRD, &OCR1B, PORTD4, PORTD2};

	DrivingFunctions Vehicle(LeftMotorConfig, RightMotorConfig);

	while (1){
		LimitForChangingDirection = CalibrateSensors(LeftSensor, RightSensor);
		
		WriteToConsole(MeasurmentOverTape, TapeMeasured);
		WriteToConsole(MeasurmentOverPaper, PaperMeasured);
		//główny program - warunki jazdy pojazdu
		
		if(StartButtonPressed() && isMoving == false){
			if(LeftSensorOK(left_sensor) == true && RightSensorOK(right_sensor) == true) //jeśli oba czujniki widzą linię
			{
				Vehicle.DriveForward()
			}
			else if(LeftSensorOK(left_sensor) == false){
				Vehicle.TurnRight();
			}
			else if(RightSensorOK(right_sensor) == false){
				Vehicle.TurnLeft();
			}
			isMoving = true;
		}
		else if(StartButtonPressed() && isMoving == true){
			Vehicle.StopVehicle();
			isMoving = false;
		}
	}
}

