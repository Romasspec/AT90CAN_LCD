#include "main.h"
#include "LCD.h"

void lcd_init(void) {
	
	delay_ms(40);
	EN_RS_0();			// E V
	DB_0();
	DB_PORT |= (1<<DB4)|(1<<DB5)|(0<<DB6)|(0<<DB7);
	EN_1();				// E ^

	delay_ms(5);
	EN_RS_0();			// E V
	DB_0();
	DB_PORT |= (1<<DB4)|(1<<DB5)|(0<<DB6)|(0<<DB7);
	EN_1();				// E ^

	delay_us(200);		//more 100us
	EN_RS_0();			// E V
	DB_0();
	DB_PORT |= (1<<DB4)|(1<<DB5)|(0<<DB6)|(0<<DB7);
	EN_1();				// E ^

	delay_us(200);
	EN_RS_0();			// E V
	DB_0();
	DB_PORT |= (0<<DB4)|(1<<DB5)|(0<<DB6)|(0<<DB7);
	EN_1();				// E ^

	delay_us(200);
	EN_RS_0();			// E V
	DB_0();
	DB_PORT |= (0<<DB4)|(1<<DB5)|(0<<DB6)|(0<<DB7);
	EN_1();				// E ^

	delay_us(200);
	EN_RS_0();			// E V
	DB_0();
	DB_PORT |= (0<<DB4)|(0<<DB5)|(0<<DB6)|(1<<DB7);
	EN_1();				// E ^

	delay_us(200);
	EN_RS_0();			// E V
	DB_0();
	DB_PORT |= (0<<DB4)|(0<<DB5)|(0<<DB6)|(0<<DB7);
	EN_1();				// E ^

	delay_us(200);
	EN_RS_0();			// E V
	DB_0();
	DB_PORT |= (1<<DB4)|(0<<DB5)|(0<<DB6)|(0<<DB7);
	EN_1();				// E ^

	delay_us(200);
	EN_RS_0();			// E V
	DB_0();
	DB_PORT |= (0<<DB4)|(0<<DB5)|(0<<DB6)|(0<<DB7);
	EN_1();				// E ^

	delay_us(200);
	EN_RS_0();			// E V
	DB_0();
	DB_PORT |= (1<<DB4)|(1<<DB5)|(1<<DB6)|(1<<DB7);
	EN_1();				// E ^

	delay_us(200);
	EN_RS_0();			// E V
	DB_0();
	RS_1();
	DB_PORT |= (0<<DB4)|(0<<DB5)|(0<<DB6)|(0<<DB7);
	EN_1();				// E ^
	
	delay_us(200);
	EN_RS_0();			// E V
	DB_0();
	RS_1();
	DB_PORT |= (0<<DB4)|(0<<DB5)|(0<<DB6)|(0<<DB7);
	EN_1();				// E ^
	
	delay_us(200);
	
	lcd_set_state(LCD_ENABLE,CURSOR_DISABLE,NO_BLINK);
	lcd_clear();
	lcd_send(0x06,COMMAND);	
}

void lcd_send(uint8_t byte, dat_or_comm dc)  {

	DB_0();
	EN_RS_0();
	if (dc)
	{
		RS_1();
	}
	
	DB_PORT |= (byte>>4) & 0x0F;
	EN_1();
	delay_ms(1);
	EN_0();
	delay_us(40);

	DB_0();
	DB_PORT |= byte & 0x0F;
	EN_1();
	delay_us(2);
	EN_0();
	delay_us(40);
}

void lcd_set_state(lcd_state state, cursor_state cur_state, cursor_mode cur_mode)  {
	if (state==LCD_DISABLE)  {
		lcd_send(0x08,COMMAND);
		} else {
		if (cur_state==CURSOR_DISABLE) {
			if (cur_mode==NO_BLINK)  {
				lcd_send(0x0C,COMMAND);
				} else {
				lcd_send(0x0D,COMMAND);
			}
			} else  {
			if (cur_mode==NO_BLINK)  {
				lcd_send(0x0E,COMMAND);
				} else {
				lcd_send(0x0F,COMMAND);
			}
		}
	}
}

void lcd_clear(void) {
	lcd_send(0x01,COMMAND);
	delay_ms(2);
}

void lcd_clear_xy(uint8_t x, uint8_t y, uint8_t lenght){
	lcd_set_xy(x,y);
	for(uint8_t i = 0; i<lenght; i++) {
		lcd_send(' ', DATA);
	}
}

void lcd_set_xy(uint8_t x, uint8_t y)  {

	lcd_send( ((((y&1)<<6)+(x&0x0F))|128),COMMAND);
}

void lcd_out (char *txt) {
	while(*txt) {
		lcd_send(*txt, DATA);
		txt++;
	}
}

void lcd_out_var (uint16_t value) {
	char buffer[21];
	sprintf(buffer, "%u", value);
	lcd_out(buffer);
}

void lcd_out_PGM (const char *txt_PGM) {
	//char* i = (char*) txt_PGM;
	char c;
	while((c=pgm_read_byte(txt_PGM++))) {
		c = convertCyr(c);
		lcd_send(c,DATA);
		//	i++;
	}
}

void lcd_out_PGM_Items (const char* const* menu, uint8_t num) {
	uint16_t ptr = pgm_read_word(menu + num);
	lcd_out_PGM((const char*) ptr);
}


uint8_t convertCyr(uint8_t ch){
	char rch = ch;
	switch (ch){
		case 0xC0: rch = 0x41; break;
		case 0xC1: rch = 0xA0; break;
		case 0xC2: rch = 0x42; break;
		case 0xC3: rch = 0xA1; break;
		case 0xC4: rch = 0xE0; break;
		case 0xC5: rch = 0x45; break;
		case 0xC6: rch = 0xA3; break;
		case 0xC7: rch = 0xA4; break;
		case 0xC8: rch = 0xA5; break;
		case 0xC9: rch = 0xA6; break;
		case 0xCA: rch = 0x4B; break;
		case 0xCB: rch = 0xA7; break;
		case 0xCC: rch = 0x4D; break;
		case 0xCD: rch = 0x48; break;
		case 0xCE: rch = 0x4F; break;
		case 0xCF: rch = 0xA8; break;

		case 0xD0: rch = 0x50; break;
		case 0xD1: rch = 0x43; break;
		case 0xD2: rch = 0x54; break;
		case 0xD3: rch = 0xA9; break;
		case 0xD4: rch = 0xAA; break;
		case 0xD5: rch = 0x58; break;
		case 0xD6: rch = 0xE1; break;
		case 0xD7: rch = 0xAB; break;
		case 0xD8: rch = 0xAC; break;
		case 0xD9: rch = 0xE2; break;
		case 0xDA: rch = 0xAD; break;
		case 0xDB: rch = 0xAE; break;
		case 0xDC: rch = 0x62; break;
		case 0xDD: rch = 0xAF; break;
		case 0xDE: rch = 0xB0; break;
		case 0xDF: rch = 0xB1; break;

		case 0xE0: rch = 0x61; break;
		case 0xE1: rch = 0xB2; break;
		case 0xE2: rch = 0xB3; break;
		case 0xE3: rch = 0xB4; break;
		case 0xE4: rch = 0xE3; break;
		case 0xE5: rch = 0x65; break;
		case 0xE6: rch = 0xB6; break;
		case 0xE7: rch = 0xB7; break;
		case 0xE8: rch = 0xB8; break;
		case 0xE9: rch = 0xB9; break;
		case 0xEA: rch = 0xBA; break;
		case 0xEB: rch = 0xBB; break;
		case 0xEC: rch = 0xBC; break;
		case 0xED: rch = 0xBD; break;
		case 0xEE: rch = 0x6F; break;
		case 0xEF: rch = 0xBE; break;

		case 0xF0: rch = 0x70; break;
		case 0xF1: rch = 0x63; break;
		case 0xF2: rch = 0xBF; break;
		case 0xF3: rch = 0x79; break;
		case 0xF4: rch = 0xE4; break;
		case 0xF5: rch = 0x78; break;
		case 0xF6: rch = 0xE5; break;
		case 0xF7: rch = 0xC0; break;
		case 0xF8: rch = 0xC1; break;
		case 0xF9: rch = 0xE6; break;
		case 0xFA: rch = 0xC2; break;
		case 0xFB: rch = 0xC3; break;
		case 0xFC: rch = 0xC4; break;
		case 0xFD: rch = 0xC5; break;
		case 0xFE: rch = 0xC6; break;
		case 0xFF: rch = 0xC7; break;

		case 0xA8: rch = 0xA2; break;
		case 0xB8: rch = 0xB5; break;
	}
	return rch;
}
