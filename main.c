#include "main.h"
#include "LCD.h"

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
volatile unsigned char timer0_fract = 0;

volatile unsigned char step, step_adc, ugol1, ugol2, ugol3, ugol4, step_f ;
volatile unsigned long value1, value2;
volatile unsigned int U1, U2, CH1_count, CH2_count;


//static char *logo = "LOGO";
	const char str1[] PROGMEM = "Meíþ 0";
	const char str2[] PROGMEM = "Meíþ 1";
	const char str3[] PROGMEM = "Meíþ 2";
	const char str4[] PROGMEM = "Meíþ 3";
	const char str5[] PROGMEM = "Meíþ 4";
	const char str6[] PROGMEM = "Meíþ 5";
	
	const char* const menu_ptr[] PROGMEM = { str1, str2, str3, str4, str5, str6};
	const char* const str6_ptr PROGMEM = str6;
	

#define PA0_ON		PORTA |= (1<<PB0);
#define PA0_OFF		PORTA &=~(1<<PB0);

#define PA1_ON		PORTA |= (1<<PB1);
#define PA1_OFF		PORTA &=~(1<<PB1);

#define PA2_ON		PORTA |= (1<<PB2);
#define PA2_OFF		PORTA &=~(1<<PB2);

#define PA3_ON		PORTA |= (1<<PB3);
#define PA3_OFF		PORTA &=~(1<<PB3);

#define T2_8		TCCR2A		|= (1<<CS22)|(1<<CS20);
#define T2_64		TCCR2A		|= (1<<CS22)|(1<<CS21)|(1<<CS20);
#define T2_CLR		TCCR2A		&= 0b11111000;

#define ADC_0		ADMUX		&= 0b11100000;					// ADC0
#define ADC_1		ADMUX		|= (1<<MUX0);					// ADC1
#define ADC_2		ADMUX		|= (1<<MUX1);					// ADC2
#define ADC_3		ADMUX		|= (1<<MUX1)|(1<<MUX0);			// ADC3
#define ADC_start	ADCSRA		|= (1<<ADSC);

#define GY_ADR							  0b10100000
#define SLA_W					GY_ADR & 0b11111110
#define SLA_R					GY_ADR | 0b00000001
//#define ADDR					0x01


#define TWSR_MASK				0xF8



//*******************************************//

void I2C_INIT (void)
{
	TWBR		=72; //72;                // 12 - 400 ê?ö, 72 - 100 ê?ö
}


volatile unsigned char i2c_ADR, i2c_buf[14], i2c_data, count_data, R_W, READ=0, i2c_bysy, count_byte, index_buf;

ISR (TIMER0_OVF_vect)
{
	unsigned long m = timer0_millis;
	unsigned char f = timer0_fract;
	
	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}

unsigned long millis(void)
{
	unsigned long m;
	uint8_t oldSREG = SREG;
	
	cli();
	m = timer0_millis;
	SREG = oldSREG;

	return m;
}

unsigned long micros(void) {
	unsigned long m;
	uint8_t oldSREG = SREG, t;
	
	cli();
	m = timer0_overflow_count;

	t = TCNT0;
	if ((TIFR0 & _BV(TOV0)) && (t < 255))
	m++;

	SREG = oldSREG;
	
	//return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
	return ((m << 8) + t) << 2;
}

void delay_ms(unsigned long ms)
{
	uint16_t start = (uint16_t)micros();

	while (ms > 0) {
		if (((uint16_t)micros() - start) >= 1000) {
			ms--;
			start += 1000;
		}
	}
}

void delay_us(unsigned long us)
{
	unsigned long nCount = us;
		
	cli();
	for (; nCount!=0; nCount--)
	{
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
	}
	sei();
}

ISR (TWI_vect)
{
	cli();
	unsigned char i2c_status			 	 = TWSR & TWSR_MASK;
	
	switch (i2c_status)
	{
	
		case 0x08:
		case 0x10:
							if (READ == 0)
							{
								TWDR			 = SLA_W;//GY_ADR & R_W;
								TWCR		 	&= 0xDF;
								TWCR			|=	(1<<TWINT)|(1<<TWEN)|(1<<TWIE);
							}
							else
							{
								TWDR			 = SLA_R;//GY_ADR | R_W;
								TWCR		 	&= 0xDF;
								TWCR			|=	(1<<TWINT)|(1<<TWEN)|(1<<TWIE);
							}
		break;
	
		case 0x18:
							TWDR 			 = i2c_ADR;
							TWCR			|=	(1<<TWINT)|(1<<TWEN)|(1<<TWIE);
		break;

		case 0x28:			
							
							if (R_W == 0xFE)
							{
								if (count_data != 0)
								{
									
									TWDR			 = i2c_buf[index_buf];
									TWCR			|=	(1<<TWINT)|(1<<TWEN)|(1<<TWIE);
									index_buf++;
									count_data --;
								}
	
								else
								{
									index_buf		 = 0;
									i2c_bysy		 = 0;
									TWCR			|= (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
								}
							}
							else
							{
								TWCR			|= (1<<TWSTA)|(1<<TWINT)|(1<<TWEN)|(1<<TWIE);
								READ			 = 1;
							}
		break;

		case 0x00:
							TWCR			|= (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
		break;
	
		case 0x20:
					
							TWCR			|= (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
		break;

		case 0x30:
							TWCR			|= (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
		break;

		case 0x48:
							TWCR			|= (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
		break;

		case 0x40:
							if ((count_data - 1) > 0)
							{
								TWCR			|= 	(1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWEA);
							}

							else
							{
								TWCR			|= 	(1<<TWINT)|(1<<TWEN)|(1<<TWIE);
							}
		break;

		case 0x50:
							i2c_buf[index_buf] 		 = TWDR;
							index_buf++;
							count_data--;
							if ((count_data - 1) > 0)
							{
								TWCR			&= 0xBF;
								TWCR			|= 	(1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWEA);
							}
							
							else
							{
								TWCR			&= 0xBF;
								TWCR			|= 	(1<<TWINT)|(1<<TWEN)|(1<<TWIE);
							}
						

		break;

		case 0x58:
							i2c_buf[index_buf] 		 = TWDR;
							i2c_bysy		 = 0;
							count_data--;
							index_buf		 = 0;
							
							TWCR			&= 0xBF;
							TWCR			|= (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
							
		break;


	}

	sei();

}

void i2c_write (unsigned char ADR ,unsigned char count_byte)
{
	
	R_W				 = 0b11111110;
	count_data		 = count_byte;
	i2c_ADR			 = ADR;
	i2c_bysy		 =1;
	READ			 = 0;
	TWCR			|= (1<<TWSTA)|(1<<TWINT)|(1<<TWEN)|(1<<TWIE);
}



/*
void i2c_write (unsigned char ADR, unsigned char data )
{
	R_W				 = 0b11111110;
	count_data		 = 1;
	i2c_ADR			 = ADR;
	i2c_data		 = data;
	READ			 = 0;
	TWCR			|= (1<<TWSTA)|(1<<TWINT)|(1<<TWEN)|(1<<TWIE);			// ñòàðò
}
*/

void i2c_read (unsigned char ADR, unsigned char count_byte )
{
	R_W				 = 0b00000001;
	count_data		 = count_byte;
	i2c_ADR			 = ADR;
	i2c_bysy		 =1;
	READ			 = 0;
	TWCR			|= (1<<TWSTA)|(1<<TWINT)|(1<<TWEN)|(1<<TWIE);			// ñòàðò				

}



//*****************************************//

void INIT_T0 (void)
{
	TCCR0A		= (1<<CS01)|(1<<CS00);
	TIMSK0		= (1<<TOIE0);
}

void INIT_T1 (void)
{
	TCCR1A		= (1<<COM1A1)|(1<<COM1A0)|(1<<WGM11)|(1<<WGM10);
	TCCR1B		= (1<<WGM12)|(1<<CS10);
	OCR1A		= 50;	
}

void INIT_T2 (void)
{
	TCCR2A		|= (1<<WGM21);
	TIMSK2		|= (1<<OCIE2A);
	OCR2A		 = 0xFF;
}

void INIT_PORT (void)
{
	DDRC		= (1<<DB4)|(1<<DB5)|(1<<DB6)|(1<<DB7);
	PORTC		= 0x00;
	
	DDRA		= (1<<RS)|(1<<EN);
	PORTA		= 0x00;
	
	DDRB		= (1<<PB5);
}

void INIT_ADC (void)
{
	ADMUX		|= (1<<REFS0);
	ADCSRA		|= (1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}



ISR (TIMER2_COMP_vect)
{	
//********************************//
	
	if (step==0)
	{
		T2_CLR;
		T2_8;
		PA0_ON;
		OCR2A		= ugol1;
	}

	if (step==1)
	{
		PA0_OFF;
		OCR2A		= 255-ugol1;
	}

	if(step==2)
	{
		OCR2A		= 38;
	}

//********************************//
	
	if (step==3)
	{
		PA0_ON;
		OCR2A		= ugol2;
	}

	if (step==4)
	{
		PA0_OFF;
		OCR2A		= 255-ugol2;
	}

	if(step==5)
	{
		OCR2A		= 38;
	}

//********************************//
	
	if (step==6)
	{
		PA0_ON;
		OCR2A		= ugol3;
	}

	if (step==7)
	{
		PA0_OFF;
		OCR2A		= 255-ugol3;
	}

	if(step==8)
	{
		OCR2A		= 38;
	}

//********************************//
	
	if (step==9)
	{
		PA0_ON;
		OCR2A		= ugol4;
	}

	if (step==10)
	{
		PA0_OFF;
		OCR2A		= 255-ugol4;
	}

	if(step==11)
	{
		OCR2A		= 38;
	}

//********************************//
		
	if (step==11)
	{
		step		= 0;
		T2_CLR;
		T2_64;
		OCR2A		= 168;
	}
	else step++;

}



ISR (ADC_vect)
{
	
/*	
	if (step_adc==0)
	{
		ugol1 = 128 + ADC/8;
		ADC_1;
		PORTB		^= (1<<PB5);
	}

	if (step_adc==1)
	{
		ugol2 = 128 + ADC/8;
		ADC_0;
		PORTB		^= (1<<PB6);
	}

	if (step_adc==1) step_adc=0;
	else step_adc++;

	ADC_start;

*/



	cli();
	step_f=0;

	if (step_adc==0)
	{
		value1 += ADC;
		CH1_count++;
		
		if (CH1_count>20)
		{
			U1		= value1/CH1_count;
			ugol1 = 127 + U1/8;
			ADC_1;
			value1=0;
			CH1_count=0;
//			PORTB		^= (1<<PB5);
			step_f=1;
			ADC_1;
		}
				
	}
	

	if (step_adc==1)
	{
		value2 += ADC;
		CH2_count++;
		
		if (CH2_count>20)
		{
			U2	= value2/CH2_count;
			ugol2 = 127 + U2/8;
			ADC_0;	
			value2=0;
			CH2_count=0;
//			PORTB		^= (1<<PB6);
			step_f=1;
			ADC_0;
		}
			
	}

	
	if (step_adc==2) step_adc=0;
	else if (step_f)
	{
		step_adc++;
	}

	ADC_start;
	sei();
	

}



int main (void)
{
	uint32_t cur_time;	
	static uint32_t time = 0;
	uint8_t count = 0;
	uint32_t time_start=0, time_end = 0, time_loop;
	
	INIT_T0();
	
	INIT_T1();
//	INIT_T2();
	INIT_PORT();
//	INIT_ADC();
//	I2C_INIT();
	sei();
	lcd_init();
	ADC_0;
	
//	T2_8;
	
//	_delay_ms(1000);
	delay_ms(1000);
	
//	i2c_read (0x02, 14);


//i2c_buf [0] = 25;
/*
i2c_buf [1] = 2;
i2c_buf [2] = 3;
i2c_buf [3] = 4;
i2c_buf [4] = 5;
index_buf		 = 0;
*/
//i2c_write (15,1);
	lcd_set_xy(1,0);
	//lcd_out_PGM(str1);
	lcd_out_PGM_Items(menu_ptr, 0);
	lcd_set_xy(1,1);
	//lcd_out_PGM(str2);
	lcd_out_PGM_Items(&str6_ptr, 0);
	lcd_set_xy(0,0);
	lcd_send('>', DATA);	
	
	while(1)
	{
/*		if (i2c_bysy == 0)
		{
			PORTB = i2c_buf[13];
		}
*/
//			_delay_ms(10);
//		delay_ms(10);
// 		cli();
// 		RS_1();
// 		delay_us(4);		
// 		RS_0();
// 		sei();

//		i2c_read (0x02, 14);	
		
		ugol3=ugol1;
		ugol4=ugol1;
		
		
		cur_time = micros();
		if((int32_t)(cur_time - time) >= 0) {
			//time_end = millis();
			time_loop = cur_time - time_start;
			time_start = cur_time;
			
			time = cur_time + TIME_FRAME_US;
// 			lcd_clear();
// 			lcd_set_xy(1,0);			
// 			lcd_out_PGM_Items(menu_ptr, 0);
// 			lcd_set_xy(1,1);
// 			lcd_out_PGM_Items(menu_ptr, 3);
// 			lcd_set_xy(0,0);
// 			lcd_send('>', DATA);
			lcd_clear_xy(7,0,5);
			lcd_set_xy(7,0);
			lcd_out_var ((uint16_t) ((time_loop>>2)));
			
		}
	}
}

