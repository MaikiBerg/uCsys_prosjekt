/*
* Prosjekt i uCsys
* Alarmklokke
* Author : Michael
*/

#define F_CPU 16000000UL
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "util/delay.h"
#include "lcdpcf8574/lcdpcf8574.h"

uint8_t hour = 0;
uint8_t minutes = 0;
uint8_t seconds = 0;
uint8_t pot_channel = 0;
uint8_t ldr_channel = 1;

uint8_t alarm_hour = 0;
uint8_t alarm_minutes = 0;

uint8_t state = 0;
uint8_t maxstates = 7;	// 0 - Just count time, 1 - Set hours, 2 - set minutes, 3 - Just count time, 4 - Set alarm hours, 5 - set alarm minutes, 6 - alarm on and count time, 7 alarm off

volatile uint8_t isr_var = 0;

/***********************************************************
*   Functions to be used later in program
*/

// Read potentiometer ADC
uint8_t ReadPOT(){
	ADMUX = (ADMUX & 0xF0) | (pot_channel & 0x0F);	// Read
	ADCSRA |= (1<<ADSC);							// Convert
	// wait until ADC conversion is complete
	do{								// Do nothing
	}while(ADCSRA & (1<<ADSC) );	// While conversion not complete
	return ADCH;					// Return ADCH byte, this will be 0-255 value
}

// Read LightDependentResistor Sensor ADC
uint8_t ReadLDR(){	//200 at dark and 10 at light
	ADMUX = (ADMUX & 0xF0) | (ldr_channel & 0x0F);	//Read
	ADCSRA |= (1<<ADSC);							//Convert
	// wait until ADC conversion is complete
	do{								// Do nothing
	}while(ADCSRA & (1<<ADSC) );	// While conversion not complete
	return ADCH;					// Return ADCH byte, this will be 0-255 value
}

// Write current time to I2C LCD display
void write_time(){
	char buf[3];
	itoa(hour,buf,10);		// From int to str/char
	
	//Print to LCD
	if(hour<10){
		lcd_gotoxy(0,0);
		lcd_puts("0");
		lcd_gotoxy(1,0);
		lcd_puts(buf);
		}else{
		lcd_gotoxy(0,0);
		lcd_puts(buf);
	}
	
	lcd_gotoxy(2,0);
	lcd_puts(":");
	
	char buf1[3];
	itoa(minutes,buf1,10);	// Convert int to str/char
	if(minutes < 10){
		lcd_gotoxy(3,0);
		lcd_puts("0");
		lcd_gotoxy(4,0);
		lcd_puts(buf1);
		}else{
		lcd_gotoxy(3,0);
		lcd_puts(buf1);
	}
	
	lcd_gotoxy(5,0);
	lcd_puts(":");
	
	char buf2[3];
	itoa(seconds,buf2,10);	// Convert int to str/char
	if(seconds < 10){
		lcd_gotoxy(6,0);
		lcd_puts("0");
		lcd_gotoxy(7,0);
		lcd_puts(buf2);
		}else{
		lcd_gotoxy(6,0);
		lcd_puts(buf2);
	}
}

//Function to turn on/off "nightlight", LDR dependent
void night_light(){
	if(ReadLDR() > 100){					// If room is dark
		if((state == 3)|(state == 7)){		// If show time state is current state
			PORTB = (1<<PINB4)|~(1<<PINB2);	// Turn on nightlight and off with alarmlight
		}
		if(state == 6){						// If showtime+alarm state is current state
			PORTB = (1<<PINB4)|(1<<PINB2);	// Turn on nightlight + alarmlight
		}
		}else{					// Room is light
		PORTB &= ~(1<<PINB4);	// Turn off nightlight
	}
}

/*
*	End functions
************************************************************/

/***********************************************************
*   Start main
*/

int main(void){
	
	//Timer1 definitions/settings
	OCR1A = 0b0011110100001000;				// OutputCompareRegister, 16bit, 15624 sykluser => ca 1 sek
	TCCR1B |= (1 << WGM12); 				// OCR1A clear timer on compare CTC mode 4
	TIMSK1 |= (1 << OCIE1A);   				// Set interrupt on compare match
	TCCR1B |= (1 << CS12) | (1 << CS10);  	// 1024 prescale
	
	//LEDs Output
	DDRB = 0b00111111;				// PB0-5 outputs;
	PORTB = 0b00111111;				// Turn off active low leds
	
	//Button interrupt
	PORTD = (1<<PIND2);				// Internal pullup on PD2 (for button)
	EICRA = (1<<ISC01);				// INT0 negative flank, interrupt on falling edge
	EIMSK = (1<<INT0);				// Enable INT0 interrupt
	
	//Set AVcc as ref, and left adjust.
	ADMUX = (1<<REFS0) | (1<<ADLAR);	//0b 0110 0000
	
	//ADcENable, prescale, single conversion and start conversion
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);	//0b 1100 0111 (16mHz/128 = 125kHz)
	
	//I2C
	DDRC |= (1<<DDC5) | (1<<DDC4);	// PC5 and PC4 as outputs for SCL and SDA
	PORTC |= (1<<DDC5) | (1<<DDC4);	// Start as high
	
	//initUSART();	//Used for debugging
	
	lcd_init(LCD_DISP_ON);				// Initialize LCD with display on, no cursor.
	lcd_home();							// Go to start position, alternative: lcd_gotoxy(0,0);
	lcd_led(0);							// Set LCD backlight ON (0)
	lcd_puts("Press button to start");	// Put string onto LCD display
	
	sei();		// interrupt enable
	
/************************************************************
*	Main loop begin
************************************************************/
	while (1)
	{
		//Set time, hours
		if(state == 1){
			lcd_clrscr();		//Clear LCD
			
			while(state == 1){
				lcd_gotoxy(0,0);
				lcd_puts("Set Hours: ");
				char buf[2];
				int hours_int = ReadPOT()/11;	// Get value from potmeter and convert to 0-23

				itoa(hours_int,buf,10);			// int to str/char
				if(hours_int<10){
					lcd_gotoxy(11,0);
					lcd_puts("0");
					lcd_gotoxy(12,0);
					lcd_puts(buf);
					}else{
					lcd_gotoxy(11,0);
					lcd_puts(buf);
				}
				_delay_ms(100);
			}
			hour = ReadPOT()/11;			// Assign hour from potmeter position
		}
		
		//Set time, minutes
		if(state == 2){
			lcd_clrscr(); //Clear LCD
			
			while(state == 2){
				lcd_gotoxy(0,0);
				lcd_puts("Set Minutes: ");
				char buf[2];
				int minutes_int = ReadPOT()/4.3;		//Convert 0-255 reading to 0-60
				
				itoa(minutes_int,buf,10);
				if(minutes_int<10){
					lcd_gotoxy(13,0);
					lcd_puts("0");
					lcd_gotoxy(14,0);
					lcd_puts(buf);
					}else{
					lcd_gotoxy(13,0);
					lcd_puts(buf);
				}
				_delay_ms(100);
			}
			minutes = ReadPOT()/4.3;
		}
		
		//Back to clock
		if(state == 3){
			lcd_clrscr(); //Clear LCD
			
			while(state == 3){
				//Do nothing, keep program from clearing screen and looping in main while loop
			}
		}
		
		//Set alarm hours
		if(state == 4){	
			lcd_clrscr(); //Clear screen
			lcd_gotoxy(0,0);
			lcd_puts("Set Alarm");
			lcd_gotoxy(0,1);
			lcd_puts("Hours: ");
			
			while(state == 4){
				char buf[2];
				int alarm_hours_int = ReadPOT()/11;
				itoa(alarm_hours_int,buf,10);
				
				if(alarm_hours_int<10){
					lcd_gotoxy(6,1);
					lcd_puts("0");
					lcd_gotoxy(7,1);
					lcd_puts(buf);
					}else{
					lcd_gotoxy(6,1);
					lcd_puts(buf);
				}
				_delay_ms(100);
			}
			alarm_hour = ReadPOT()/11;
		}
		
		// Set alarm minutes
		if(state == 5){
			lcd_clrscr();
			lcd_gotoxy(0,0);
			lcd_puts("Set Alarm");
			lcd_gotoxy(0,1);
			lcd_puts("Minutes: ");
			
			while(state == 5){
				char buf[2];
				int alarm_minutes_int = ReadPOT()/4.3;		//Convert 0-255 reading to 0-60
				itoa(alarm_minutes_int,buf,10);
				
				if(alarm_minutes_int<10){
					lcd_gotoxy(8,1);
					lcd_puts("0");
					lcd_gotoxy(9,1);
					lcd_puts(buf);
					}else{
					lcd_gotoxy(8,1);
					lcd_puts(buf);
				}
				_delay_ms(100);
			}

			alarm_minutes = ReadPOT()/4.3;
		}
		
		//Alarm on and set, show time too
		if(state == 6){
			lcd_clrscr();
			lcd_home();				// Return cursor to start pos
			lcd_gotoxy(0,1);
			lcd_puts("Alarm:");
			
			char buf[2];
			itoa(alarm_hour,buf,10);
			if(alarm_hour<10){
				lcd_gotoxy(7,1);
				lcd_puts("0");
				lcd_puts(buf);
				}else{
				lcd_gotoxy(7,1);
				lcd_puts(buf);
			}
			lcd_puts(":");
			char buf1[2];
			itoa(alarm_minutes,buf1,10);
			if(alarm_minutes <10){
				lcd_puts("0");
				lcd_puts(buf1);
				}else{
				lcd_puts(buf1);
			}
			
			while(state == 6){
				//Check alarm
				if(alarm_hour == hour && alarm_minutes == minutes ){
					lcd_led(1);		// Flicker LCD LED as alarm
					_delay_ms(500);
					lcd_led(0);
					_delay_ms(500);
				}
			}
		}
		
		
		if(state == 7){
			lcd_clrscr();
			lcd_led(0);		// Make sure lcd_led is on 
			while(state == 7){
				//Do nothing, let timer1 interrupt write to lcd every 1 sec.
			}
		}
		
	}
}
/*
*	Main end
************************************************************/

/***********************************************************
*   Interrupts
*/

ISR (TIMER1_COMPA_vect){ //Interrupt that happens every second
	
	if (seconds > 59){
		minutes += 1;
		seconds = 0;
	}
	if (minutes > 59){
		hour += 1;
		minutes = 0;
	}
	if (hour > 23){
		hour = 0;
	}
	
	if((state == 3) | (state == 6)|(state == 7)){
		write_time();
		night_light();
	}
	seconds += 1;
}

ISR(INT0_vect)
{
	static uint8_t deb_var=0;	// Better to press button one too many times than have one too many changes ;)
	deb_var++;					// This is to "debounce" 
	if (deb_var >= 2) {
		deb_var=0;
		state++;				
	}
	if (state > maxstates){	// Reset state count
		state = 1;
	}
	
}

/*
*   End of interrupts
************************************************************/
