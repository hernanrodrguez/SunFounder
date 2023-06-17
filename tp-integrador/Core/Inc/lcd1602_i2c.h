/*
 * lcd1602_i2c.h
 *
 *  Created on: Mar 29, 2023
 *      Author: carlassaraf
 */

#ifndef INC_LCD1602_I2C_H_
#define INC_LCD1602_I2C_H_

#ifdef STM32F103xB
#include "stm32f1xx_hal.h"
#elif defined(__LPC17XX__)
#include "chip.h"
#endif

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYLEFT 0x02

// flags for display and cursor control
#define LCD_BLINKON 0x01
#define LCD_CURSORON 0x02
#define LCD_DISPLAYON 0x04

// flags for display and cursor shift
#define LCD_MOVERIGHT 0x04
#define LCD_DISPLAYMOVE 0x08

// flags for function set
#define LCD_5x10DOTS 0x04
#define LCD_2LINE 0x08
#define LCD_8BITMODE 0x10

// flag for backlight control
#define LCD_BACKLIGHT 0x08

#define LCD_ENABLE_BIT 0x04

// By default these LCD display drivers are on bus address 0x3f
#define ADDR (0x27)

// Modes for lcd_send_byte
#define LCD_CHARACTER  1
#define LCD_COMMAND    0

#define MAX_LINES      2
#define MAX_CHARS      16

#ifdef STM32F103xB
#define i2c_transmit_byte(i2c, addr, val)	HAL_I2C_Master_Transmit(i2c, (addr << 1), &val, 1, 100)
// Prototipo para inicializar LCD
void lcd_init(I2C_HandleTypeDef *hi2c1);
#elif defined(__LPC17XX__)
#define i2c_transmit_byte(i2c, addr, val)	Chip_I2C_MasterSend(i2c, addr, &val, 1)
// Prototipo para inicializar LCD
void lcd_init(I2C_ID_T i2c_id);
#endif

// Prototipos de funciones
void i2c_write_byte(uint8_t val);
void lcd_toggle_enable(uint8_t val);
void lcd_send_byte(uint8_t val, int mode);
void lcd_clear(void);
void lcd_set_cursor(int line, int position);
void lcd_string(const char *s);

static void inline lcd_char(char val) {
    lcd_send_byte(val, LCD_CHARACTER);
}

#endif /* INC_LCD1602_I2C_H_ */
