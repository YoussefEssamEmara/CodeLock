/*******************************************************
This program was created by the
CodeWizardAVR V3.14 Advanced
Automatic Program Generator
� Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : mini_project_finaaaal
Version :
Date    : 24-Apr-2024
Author  :
Company :
Comments:


Chip type               : ATmega32A
Program type            : Application
AVR Core Clock frequency: 11.059200 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 512
*******************************************************/

#include <mega32a.h>
#include <io.h>
#include <eeprom.h>
#include <sleep.h>
#include <interrupt.h>
#include <delay.h>

// Debouncing delay
#define DEBOUNCE_DELAY_MS 20

// Declare your global variables here
#define sbi(port, bit) (port) |= (1 << (bit))
#define cbi(port, bit) (port) &= ~(1 << (bit))

#define bit_is_clear(sfr, bit) (!(sfr & (1 << bit)))

typedef union
{
	struct
	{
		signed char tens;
		signed char units;
	};
	unsigned int value; // alias
} DIAL;					// virtual dial position

volatile DIAL dial;		// virtual dial position
volatile DIAL combo[3]; // combination

typedef enum
{
	STATUS_UNLOCKED,
	STATUS_LOCKED
} STATUS;

STATUS locked; // locked status:  0=unlocked, 1=locked

typedef enum
{
	COMBO_MATCH_0, // no numbers matched - nothing happening
	COMBO_MATCH_1, // 1st number matched
	COMBO_MATCH_2, // 2nd number matched
	COMBO_MATCH_3  // 3rd number matched
} COMBO_STATE;

volatile COMBO_STATE combo_state; // combination state machine variable

typedef enum
{
	PROG_0, // not programming the combination
	PROG_1, // 1st number programmed, checking 2nd
	PROG_2	// 2nd number programmed, checking 3rd
} PROG_STATE;

volatile PROG_STATE prog_state = PROG_0; // combination programming state machine variable

#define SCREENSAVER_TIMEOUT_VALUE 100000 // display blank timeout value in 1/250 second increments (2,000 ~= 8 seconds)
unsigned int screensaver_timeout;

// EEPROM memory map

EEMEM unsigned char eeprom_do_not_use; // .bad luck - do not use the first location in EEPROM
EEMEM unsigned char eeprom_locked;	   // saved locked status:  0=unlocked, 1=locked
EEMEM DIAL eeprom_dial;				   // saved dial position
EEMEM DIAL eeprom_combo[3];			   // programmed combination

// lock functions

#define lock()          \
	cbi(PORTD, PORTD6); \
	locked = STATUS_LOCKED
#define unlock()        \
	sbi(PORTD, PORTD6); \
	locked = STATUS_UNLOCKED

// LED_segment() - decode outputs for seven-segment LED display

typedef enum
{
	SEGMENT_0,
	SEGMENT_1,
	SEGMENT_2,
	SEGMENT_3,
	SEGMENT_4,
	SEGMENT_5,
	SEGMENT_6,
	SEGMENT_7,
	SEGMENT_8,
	SEGMENT_9,
	SEGMENT_BLANK,
	SEGMENT_DASH
} SEGMENT;

SEGMENT LED_segment(unsigned char value)
{

	const unsigned char LED_segment_lookup_table[] = {

		0b0111111, // 0
		0b0000110, // 1
		0b1011011, // 2
		0b1001111, // 3
		0b1100110, // 4
		0b1101101, // 5
		0b1111101, // 6
		0b0000111, // 7
		0b1111111, // 8
		0b1101111, // 9
		0b0000000, // blank
		0b1000000, // dash
	};

	return ~LED_segment_lookup_table[value];
}

// LED_blink() - blink LED display

void LED_blink(unsigned char n)
{

	while (n--)
	{

		cli();				  // disable interrupts (prevents display multiplexing)
		PORTA = 0b11111111;	  // all segments off
		delay_ms(200);		  // ~1 second delay, with display off
		GIFR |= (1 << INTF2); // Clear INT2 flag
		sei();				  // re-enable interrupts
		delay_ms(200);		  // ~1 second delay, with display on
	}
}

typedef enum
{
	MOTION_RIGHT,
	MOTION_LEFT
} MOTION;

// Software debouncing
unsigned int software_debounce = 0x55;

// External Interrupt 0 service routine
interrupt[EXT_INT0] void ext_int0_isr(void)
{
	// Place your code here
	software_debounce = 0xAA;
	MOTION motion; // direction of detected motion

	screensaver_timeout = SCREENSAVER_TIMEOUT_VALUE; // reset screensaver timeout value - turns display on

	// check encoder phase B to determine direction of motion

	delay_ms(DEBOUNCE_DELAY_MS);

	// bit_is_clear(PIND, PORTD3) &&
	if (software_debounce == 0xAA)
	{
		motion = MOTION_RIGHT;
	}
	else
	{
		motion = MOTION_LEFT;
	}

	delay_ms(DEBOUNCE_DELAY_MS);

	if (motion == MOTION_RIGHT)
	{
		dial.units++; // increment dial position
		if (dial.units > 9)
		{
			dial.units = 0; // reset units on overflow
			dial.tens++;	// increment tens digit
			if (dial.tens > 9)
				dial.tens = 0; // reset on overflow
		}
	}
	else
	{
		dial.units--; // decrement dial position
		if (dial.units < 0)
		{
			dial.units = 9; // rollover on underflow
			dial.tens--;	// decrement tens digit
			if (dial.tens < 0)
				dial.tens = 9; // rollover on underflow
		}
	}

	delay_ms(DEBOUNCE_DELAY_MS);

	// combination lock logic

	switch (combo_state)
	{

	case COMBO_MATCH_0:
		// detect retrograde motion
		if (motion == MOTION_LEFT)
		{
			combo_state = COMBO_MATCH_0; // start over
		}
		else
		{
			// look for 1st number match
			if (dial.value == combo[0].value)
			{
				// matched first number of combination
				combo_state = COMBO_MATCH_1; // advance to next state
			}
		}
		break;

	case COMBO_MATCH_1:
		// detect retrograde motion
		if (motion == MOTION_RIGHT)
		{
			combo_state = COMBO_MATCH_0; // start over
		}
		else
		{
			// look for 2nd number match
			if (dial.value == combo[1].value)
			{
				// matched second number of combination
				combo_state = COMBO_MATCH_2; // advance to next state
			}
		}
		break;

	case COMBO_MATCH_2:
		// detect retrograde motion
		if (motion == MOTION_LEFT)
		{
			combo_state = COMBO_MATCH_0; // start over
		}
		else
		{
			// look for 3rd number match
			if (dial.value == combo[2].value)
			{
				unlock();								   // combination satisfied
				eeprom_write_byte(&eeprom_locked, locked); // save unlocked status
				LED_blink(5);
				combo_state = COMBO_MATCH_3; // advance to next state
			}
		}
		break;

	case COMBO_MATCH_3:
		lock();									   // any motion relocks
		eeprom_write_byte(&eeprom_locked, locked); // save locked status
		combo_state = COMBO_MATCH_0;			   // start over
		break;

	default: // ??? unknown/unexpected state
		break;
	}

	GIFR |= (1 << INTF0);
}

// External Interrupt 1 service routine
interrupt[EXT_INT1] void ext_int1_isr(void)
{
	// Place your code here
	software_debounce = 0x55;
}

// External Interrupt 2 service routine
interrupt[EXT_INT2] void ext_int2_isr(void)
{
	// Place your code here
	static DIAL candidate[3]; // holder for candidate combination

	// combination programming logic

	switch (prog_state)
	{

	case PROG_0:						 // begin combination programming sequence
		candidate[0].value = dial.value; // save current dial position
		LED_blink(1);					 // blink display once to acknowledge 1st number saved
		prog_state = PROG_1;			 // advance to next state
		break;

	case PROG_1: // entering 2nd number
		// 2nd number must be different than 1st
		if (dial.value != candidate[0].value)
		{
			candidate[1].value = dial.value; // save current dial position
			LED_blink(2);					 // blink display twice to acknowledge 2nd number saved
			prog_state = PROG_2;			 // advance to next state
		}
		break;

	case PROG_2:
		// 3rd number must be different than 2nd
		if (dial.value != candidate[1].value)
		{
			candidate[2].value = dial.value;							   // save current dial position
			eeprom_write_word(&eeprom_combo[0].value, candidate[0].value); // save 1st number
			eeprom_write_word(&eeprom_combo[1].value, candidate[1].value); // save 2nd number
			eeprom_write_word(&eeprom_combo[2].value, candidate[2].value); // save 3rd number
			combo[0].value = candidate[0].value;						   // the new combination
			combo[1].value = candidate[1].value;						   // the new combination
			combo[2].value = candidate[2].value;						   // the new combination
			LED_blink(3);												   // blink display three times to acknowledge 3rd number saved
			prog_state = PROG_0;										   // start over
		}
		break;

	default: // ??? unknown/unexpected state
		break;
	}

	GIFR |= (1 << INTF2); // Clear INT2 flag
}

typedef enum
{
	DIGIT_LEFT,
	DIGIT_RIGHT
} DIGIT;

// Timer 0 overflow interrupt service routine
interrupt[TIM0_OVF] void timer0_ovf_isr(void)
{
	// Reinitialize Timer 0 value
	TCNT0 = 0x4F;
	// Place your code here
	static unsigned char digit = DIGIT_LEFT; // alternate between left & right digits

	if (screensaver_timeout)
	{

		cbi(PORTB, PORTB0); // turn off left digit (tens)
		cbi(PORTB, PORTB1); // turn off right digit (units)

		screensaver_timeout--; // decrement screensaver timeout value
		if (screensaver_timeout == 0)
		{
			// shut down display function
			eeprom_write_byte(&eeprom_locked, locked);		   // save locked/unlocked status
			eeprom_write_word(&eeprom_dial.value, dial.value); // save current dial position
			if (combo_state != COMBO_MATCH_3)
				combo_state = COMBO_MATCH_0; // reset any combination attempts
			prog_state = PROG_0;			 // cancel any pending combination programming attempts

			return; // early exit from interrupt handler, leaving display blank; nothing left to do
		}

		digit ^= 0x01; // toggle between left & right digits

		delay_ms(DEBOUNCE_DELAY_MS);

		if (digit == DIGIT_LEFT)
		{
			// display left digit
			PORTA = LED_segment(dial.tens);
			sbi(PORTB, PORTB0); // turn on left digit (tens)
		}
		else
		{
			// display right digit
			PORTA = LED_segment(dial.units);
			sbi(PORTB, PORTB1); // turn on right digit (units)
		}
	}
}

// Timer1 overflow interrupt service routine
interrupt[TIM1_OVF] void timer1_ovf_isr(void)
{
	// Reinitialize Timer1 value
	TCNT1H = 0x5333 >> 8;
	TCNT1L = 0x5333 & 0xff;
	// Place your code here
}

void main(void)
{
	// Declare your local variables here

	// Input/Output Ports initialization
	// Port A initialization
	// Function: Bit7=In Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out
	DDRA = (0 << DDA7) | (1 << DDA6) | (1 << DDA5) | (1 << DDA4) | (1 << DDA3) | (1 << DDA2) | (1 << DDA1) | (1 << DDA0);
	// State: Bit7=T Bit6=1 Bit5=1 Bit4=1 Bit3=1 Bit2=1 Bit1=1 Bit0=1
	PORTA = (0 << PORTA7) | (1 << PORTA6) | (1 << PORTA5) | (1 << PORTA4) | (1 << PORTA3) | (1 << PORTA2) | (1 << PORTA1) | (1 << PORTA0);

	// Port B initialization
	// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=Out Bit0=Out
	DDRB = (0 << DDB7) | (0 << DDB6) | (0 << DDB5) | (0 << DDB4) | (0 << DDB3) | (0 << DDB2) | (1 << DDB1) | (1 << DDB0);
	// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=P Bit1=0 Bit0=0
	PORTB = (0 << PORTB7) | (0 << PORTB6) | (0 << PORTB5) | (0 << PORTB4) | (0 << PORTB3) | (1 << PORTB2) | (0 << PORTB1) | (0 << PORTB0);

	// Port C initialization
	// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
	DDRC = (0 << DDC7) | (0 << DDC6) | (0 << DDC5) | (0 << DDC4) | (0 << DDC3) | (0 << DDC2) | (0 << DDC1) | (0 << DDC0);
	// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
	PORTC = (0 << PORTC7) | (0 << PORTC6) | (0 << PORTC5) | (0 << PORTC4) | (0 << PORTC3) | (0 << PORTC2) | (0 << PORTC1) | (0 << PORTC0);

	// Port D initialization
	// Function: Bit7=In Bit6=Out Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
	DDRD = (0 << DDD7) | (1 << DDD6) | (0 << DDD5) | (0 << DDD4) | (0 << DDD3) | (0 << DDD2) | (0 << DDD1) | (0 << DDD0);
	// State: Bit7=T Bit6=1 Bit5=T Bit4=T Bit3=P Bit2=P Bit1=T Bit0=T
	PORTD = (0 << PORTD7) | (1 << PORTD6) | (0 << PORTD5) | (0 << PORTD4) | (1 << PORTD3) | (1 << PORTD2) | (0 << PORTD1) | (0 << PORTD0);

	// Timer/Counter 0 initialization
	// Clock source: System Clock
	// Clock value: 172.800 kHz
	// Mode: Normal top=0xFF
	// OC0 output: Disconnected
	// Timer Period: 1.0243 ms
	TCCR0 = (0 << WGM00) | (0 << COM01) | (0 << COM00) | (0 << WGM01) | (0 << CS02) | (1 << CS01) | (1 << CS00);
	TCNT0 = 0x4F;
	OCR0 = 0x00;

	// Timer/Counter 1 initialization
	// Clock source: System Clock
	// Clock value: 10.800 kHz
	// Mode: Normal top=0xFFFF
	// OC1A output: Disconnected
	// OC1B output: Disconnected
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer Period: 4.096 s
	// Timer1 Overflow Interrupt: On
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: Off
	// Compare B Match Interrupt: Off
	TCCR1A = (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0 << WGM11) | (0 << WGM10);
	TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12) | (1 << CS12) | (0 << CS11) | (1 << CS10);
	TCNT1H = 0x53;
	TCNT1L = 0x33;
	ICR1H = 0x00;
	ICR1L = 0x00;
	OCR1AH = 0x00;
	OCR1AL = 0x00;
	OCR1BH = 0x00;
	OCR1BL = 0x00;

	// Timer(s)/Counter(s) Interrupt(s) initialization
	TIMSK = (0 << OCIE2) | (0 << TOIE2) | (0 << TICIE1) | (0 << OCIE1A) | (0 << OCIE1B) | (1 << TOIE1) | (0 << OCIE0) | (1 << TOIE0);

	// External Interrupt(s) initialization
	// INT0: On
	// INT0 Mode: Falling Edge
	// INT1: Off
	// INT2: On
	// INT2 Mode: Falling Edge
	GICR |= (0 << INT1) | (1 << INT0) | (1 << INT2);
	MCUCR = (0 << ISC11) | (0 << ISC10) | (1 << ISC01) | (0 << ISC00);
	MCUCSR = (0 << ISC2);
	GIFR = (0 << INTF1) | (1 << INTF0) | (1 << INTF2);

// Global enable interrupts
#asm("sei")

	locked = eeprom_read_byte(&eeprom_locked); // saved locked status
	if (locked == STATUS_UNLOCKED)
	{
		unlock(); // unlocked
	}
	else if (locked == STATUS_LOCKED)
	{
		lock(); // locked
	}
	else
	{
		lock(); // unknown/unexpected state:  default is locked
	}

	if (locked == STATUS_LOCKED)
	{
		screensaver_timeout = SCREENSAVER_TIMEOUT_VALUE; // reset screensaver timeout value
		combo_state = COMBO_MATCH_0;					 // nothing happening
	}
	else
	{
		screensaver_timeout = 0;	 // if unlocked, current dial position is 3rd number of combination:  DO NOT REVEAL
		combo_state = COMBO_MATCH_3; // unlocked, waiting for motion to relock
	}

	dial.value = eeprom_read_word(&eeprom_dial.value); // saved dial position
	if ((dial.units >= 0) && (dial.units <= 9) && (dial.tens >= 0) && (dial.tens <= 9))
	{
		// saved dial position was valid; continue
	}
	else
	{
		dial.units = 0; // reset dial position
		dial.tens = 0;
	}

	combo[0].value = eeprom_read_word(&eeprom_combo[0].value); // saved combination 1/3
	combo[1].value = eeprom_read_word(&eeprom_combo[1].value); // saved combination 2/3
	combo[2].value = eeprom_read_word(&eeprom_combo[2].value); // saved combination 3/3

	if ((combo[0].units >= 0) && (combo[0].units <= 9) && (combo[0].tens >= 0) && (combo[0].tens <= 9) && (combo[1].units >= 0) && (combo[1].units <= 9) && (combo[1].tens >= 0) && (combo[1].tens <= 9) && (combo[2].units >= 0) && (combo[2].units <= 9) && (combo[2].tens >= 0) && (combo[2].tens <= 9))
	{
		// saved combination was valid; continue
	}
	else
	{
		combo[0].units = 3; // default combination 1/3
		combo[0].tens = 0;
		combo[1].units = 1; // default combination 2/3
		combo[1].tens = 0;
		combo[2].units = 4; // default combination 3/3
		combo[2].tens = 0;
	}

	sei(); // enable global interrupts

	sleep_enable();

	while (1)
	{
		// Place your code here

		delay_ms(DEBOUNCE_DELAY_MS);
		sleep_enter();
		// LED_blink(1);
	}
}
