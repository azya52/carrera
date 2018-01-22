/*
 * CarAttiny25.c
 *
 * Created: 08.01.2018 15:27:24
 * Author : Alexander Zakharyan
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define TRACK_PIN PB4
#define MOTOR_PIN PB1
#define IRLED_PIN PB0
#define NU_PIN1 PB2
#define NU_PIN2 PB3
#define NU_PIN3 PB5

#define DBLCLICK_DELAY_MS 250 
#define PACKET_LENGTH_MS 75
#define DBLCLICK_DELAY DBLCLICK_DELAY_MS/PACKET_LENGTH_MS

#define TRANSM_FREQ 9800
#define PERIOD_QURT_CICLES F_CPU/TRANSM_FREQ/4

#define PROG_WORD_CHECK 12
#define ACTIV_WORD_CHECK 7
#define CONTROLLER_WORD_CHECK 9

/*
#define PROG_WORD_CHECK 0x1000
#define PACE_WORD_CHECK 0x3C0
#define ACTIV_WORD_CHECK 0x80
#define CONTROLLER_WORD_CHECK 0x200
*/
#define GHOST_CAR_ID 6
#define PACE_CAR_ID 7

#define SHORT_TONE_MS 100
#define LONG_TONE_MS 200

#define MIN_CAR_SPEED_MULTIPLIER 9
#define MAX_CAR_SPEED_MULTIPLIER 14

uint8_t EEMEM eeprom_carID; 
uint8_t EEMEM eeprom_progInNextPowerOn; 
uint8_t EEMEM eeprom_ghostSpeed; 
uint8_t EEMEM eeprom_speedMultiplier; 

uint8_t volatile carID = 255;
uint8_t volatile currentSpeed = 0;
uint8_t volatile sincWordIndex = 0;
uint8_t volatile doubleClickControllerId = 255;
uint8_t volatile countClickSW = 0;
uint8_t volatile countNotClickSW = 0;
uint8_t volatile progMode = 0;
uint8_t volatile progGhostMode = 0;
uint8_t volatile progSpeedMode = 0;
uint8_t volatile ghostSpeed = 0;
uint8_t volatile speedMultiplier = 14;
uint8_t volatile progSpeedSelecter = 0;

void playTone(){
	cli();
	TCCR0B = 0<<WGM02 | 1<<CS01 | 1<<CS00; // div 64
	OCR0B = 240;
	_delay_ms(150);	
	OCR0B = 255;
	TCCR0B = 0<<WGM02 | 1<<CS00; // no div
	sei();
}

void timersInit() {
	// MOTOR PWM
	TCCR0A = 1<<COM0B0 | 1<<COM0B1 | 3<<WGM00;
	TCCR0B = 0<<WGM02 | 1<<CS00; // no div
	OCR0B = 255;

	// LED PWM
	TCCR1 |= 1<<CTC1 | 1<<PWM1A | 1<<COM1A0 | 1<<CS12 | 1<<CS10; // div 16
}

void startLEDPWM() {
	OCR1C = 32*(carID+1);
	OCR1A = OCR1C - 16;
}

void pinsInit() {
	DDRB = 0; //all as input
	DDRB |= (1 << MOTOR_PIN); //MOTOR as output
	DDRB |= (1 << IRLED_PIN); //IRLED as output
	
	PORTB &= (1 << TRACK_PIN); //PullUp TRACK
	PORTB &= (1 << NU_PIN1); //PullUp not used pins
	PORTB &= (1 << NU_PIN2); //PullUp not used pins
	PORTB &= (1 << NU_PIN3); //PullUp not used pins
}

void interruptsInit(){
	GIMSK = (1<<PCIE); //turn on PCINT
	MCUCR = (1<<ISC01) | (0<<ISC00); // falling front
	PCMSK = (1<<TRACK_PIN); //turn on interrupts only on TRACK
	sei();
}

void setCarSpeed(uint8_t speed){
	currentSpeed = speed;
	OCR0B = 255-speed*speedMultiplier;	
}

void setCarID(uint8_t newId){
	carID = newId;
	eeprom_write_byte(&eeprom_carID,newId);
	startLEDPWM();
}

void stopProg() {
	if(progSpeedMode){
		speedMultiplier = MIN_CAR_SPEED_MULTIPLIER + progSpeedSelecter;
		progSpeedMode = 0;
		eeprom_write_byte(&eeprom_speedMultiplier, speedMultiplier);	
	}
	if(progMode){
		progMode = 0;
	}
}

void onDblClick(uint8_t controllerId, uint8_t clickCount){
	if(progGhostMode) {
		ghostSpeed = currentSpeed;
		currentSpeed = 0;
		progGhostMode = 0;
		eeprom_write_byte(&eeprom_ghostSpeed, ghostSpeed);
		setCarID(GHOST_CAR_ID);
		playTone();
	} else
	if(progSpeedMode) {
		playTone();
		progSpeedSelecter++;
		if(progSpeedSelecter==5){
			_delay_ms(50);
			playTone();
			stopProg();
		}
	} else
	if(clickCount>1 && currentSpeed==0){
		playTone();
		if(progMode) {
			_delay_ms(50);
			playTone();
			if(clickCount==2){
				setCarID(controllerId);
			} else
			if(clickCount==3){
				progSpeedMode = 1;
				progSpeedSelecter = 0;
			} else
			if(clickCount==4){
				carID = controllerId;
				progGhostMode = 1;
			}
			progMode = 0;
		} else {
			eeprom_write_byte(&eeprom_progInNextPowerOn,1);
		}
	}
}

void checkDblClick(uint8_t controllerId, uint8_t sw){
	if(sw){ //not pressed
		if(controllerId==doubleClickControllerId){
			if(countNotClickSW==0){
				countClickSW++;
			}
			if(countNotClickSW++>DBLCLICK_DELAY){
				doubleClickControllerId = 255;
				if(countClickSW>0){
					onDblClick(controllerId, countClickSW);
				}
			}
		}
	} else {
		if(controllerId<doubleClickControllerId){
			doubleClickControllerId = controllerId;
			countClickSW = 0;
		}
		if(doubleClickControllerId == controllerId){
			countNotClickSW = 0;
		}
	}
}

void onProgramDataWordReceived(uint16_t word){
}

void onActiveControllerWordReceived(uint16_t word){
	uint8_t anyKeyPressed = word & 0x7F;
	if(anyKeyPressed){
		stopProg();
	}
	uint8_t currentKeyPressed = (word << (carID & 0x0F)) & 0x40;
	if(carID==GHOST_CAR_ID) {
		if((currentSpeed==0) && anyKeyPressed) {
			setCarSpeed(ghostSpeed);
		}
	} else {
		if(currentKeyPressed){
			if(currentSpeed==0){
				setCarSpeed(1);
			}
		} else {
			setCarSpeed(0);
		}
	}
}

void onControllerWordReceived(uint16_t word){
	uint8_t controllerId = (word >> 6) & 0x07;
	uint8_t speed = (word >> 1) & 0x0F;
	if(controllerId==carID){
		setCarSpeed(speed);
	} 
	checkDblClick(controllerId, (word>>5) & 1);
}

void onWordReceived(uint16_t word){
	if((word >> CONTROLLER_WORD_CHECK) == 1){
		if(word >> 6 != 0x0F){
			onControllerWordReceived(word);
		}
	} else
	if((word >> PROG_WORD_CHECK) == 1){
		onProgramDataWordReceived(word);
	} else
	if((word >> ACTIV_WORD_CHECK) == 1){
		onActiveControllerWordReceived(word);
	}
	/*
	if((word & PACE_WORD_CHECK) == CONTROLLER_WORD_CHECK){
		onControllerWordReceived(word);
	} else
	if((word & CONTROLLER_WORD_CHECK) == CONTROLLER_WORD_CHECK){
		onControllerWordReceived(word);
	} else
	if((word & PROG_WORD_CHECK) == PROG_WORD_CHECK){
		onProgramDataWordReceived(word);
	} else
	if((word & ACTIV_WORD_CHECK) == ACTIV_WORD_CHECK){
		onActiveControllerWordReceived(word);
	} 
	*/
	/*
	if((word & PROG_WORD_CHECK) == PROG_WORD_CHECK){
		sincWordIndex = 1;
	}
	if(sincWordIndex>0){
		switch(sincWordIndex){
			case 1: 
				onProgramDataWordReceived(word);
				break;
			case 2:
				break;
			case 3:
			case 9:
				if((word & ACTIV_WORD_CHECK) == ACTIV_WORD_CHECK){
					onActiveControllerWordReceived(word);
				}
				break;
			case 4:
			case 5:
			case 6:
			case 7:
			case 8:
			case 10:
				if((word & CONTROLLER_WORD_CHECK) == CONTROLLER_WORD_CHECK){
					onControllerWordReceived(word);
				}
				break;
			default:
				break;
		}
		sincWordIndex++;
	}
	*/
}

ISR(PCINT0_vect){
	//_delay_us(PERIOD_QURT_CICLES);
	uint8_t start = TCNT0;
	while ((uint8_t)(TCNT0-start) < PERIOD_QURT_CICLES);
	start += PERIOD_QURT_CICLES;
	uint8_t firstHalfCycle = 1;
	uint8_t secondHalfCycle = 0;
	uint16_t receivedValue = 0;
	while(firstHalfCycle != secondHalfCycle) {
		receivedValue = (receivedValue<<1) | firstHalfCycle;
		//_delay_us(PERIOD_HALF_CICLES);

		while ((uint8_t)(TCNT0-start) < PERIOD_QURT_CICLES);
		start += PERIOD_QURT_CICLES;
		while ((uint8_t)(TCNT0-start) < PERIOD_QURT_CICLES);
		start += PERIOD_QURT_CICLES;

		firstHalfCycle = (PINB >> TRACK_PIN) & 1;
		//_delay_us(PERIOD_HALF_CICLES);

		while ((uint8_t)(TCNT0-start) < PERIOD_QURT_CICLES);
		start += PERIOD_QURT_CICLES;
		while ((uint8_t)(TCNT0-start) < PERIOD_QURT_CICLES);
		start += PERIOD_QURT_CICLES;
		
		secondHalfCycle = (PINB >> TRACK_PIN) & 1;
	}
	onWordReceived(receivedValue);
	GIFR = 1 << PCIF;
}


int main(void) {
	carID = eeprom_read_byte(&eeprom_carID);
	ghostSpeed = eeprom_read_byte(&eeprom_ghostSpeed);
	speedMultiplier = eeprom_read_byte(&eeprom_speedMultiplier);
	if(speedMultiplier > MAX_CAR_SPEED_MULTIPLIER){
		speedMultiplier = MAX_CAR_SPEED_MULTIPLIER;
	}
	progMode = eeprom_read_byte(&eeprom_progInNextPowerOn);
	if(progMode){
		if(progMode>1){
			progMode = 0;
		}
		eeprom_write_byte(&eeprom_progInNextPowerOn,0);	
	}
	
	power_adc_disable();
	power_usi_disable();
	//ADCSRA = 0; //disable the ADC

	pinsInit();
	timersInit();
	startLEDPWM();
	interruptsInit();
	
	set_sleep_mode(SLEEP_MODE_IDLE);
	while (1) {
		sleep_enable(); 
		sleep_cpu(); 
	}
}

