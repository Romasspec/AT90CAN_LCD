#ifndef LCD_H_
#define LCD_H_
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

//LCD
#define DB4			PC0
#define DB5			PC1
#define DB6			PC2
#define DB7			PC3
#define DB_PORT		PORTC
#define EN			PA3
#define EN_PORT		PORTA
#define RS			PA4
#define RS_PORT		PORTA

#define RS_1()		RS_PORT |= (1<<RS)
#define RS_0()		RS_PORT &=~(1<<RS)
#define EN_1()		RS_PORT |= (1<<EN)
#define EN_0()		RS_PORT &=~(1<<EN)

#define DB_MASK			((1<<DB4)|(1<<DB5)|(1<<DB6)|(1<<DB7))
#define DB_0()			(DB_PORT &=~DB_MASK)

#define EN_RS_MASK		((1<<EN)|(1<<RS))
#define EN_RS_0()		(EN_PORT &=~EN_RS_MASK)

typedef enum {
	COMMAND = 0,
	DATA = !COMMAND
} dat_or_comm;

typedef enum {
	LCD_DISABLE = 0,
	LCD_ENABLE = !LCD_DISABLE
} lcd_state;

typedef enum {
	CURSOR_DISABLE = 0,
	CURSOR_ENABLE = !CURSOR_DISABLE
} cursor_state;

typedef enum {
	NO_BLINK = 0,
	BLINK = !NO_BLINK
} cursor_mode;

void lcd_init(void);
void lcd_set_state(lcd_state state, cursor_state cur_state, cursor_mode cur_mode);
void lcd_clear(void);
void lcd_clear_xy(uint8_t x, uint8_t y, uint8_t lenght);
void lcd_send(uint8_t byte, dat_or_comm dc);
void lcd_set_xy(uint8_t x, uint8_t y);
void lcd_out (char *txt);
void lcd_out_PGM (const char *txt_PGM);
uint8_t convertCyr(uint8_t ch);
void lcd_out_PGM_Items (const char* const* menu, uint8_t num);
void lcd_out_var (uint16_t value);


#endif /* LCD_H_ */