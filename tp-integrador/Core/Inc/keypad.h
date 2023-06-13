/*
 * debounce.h
 *
 *  Created on: Jun 2, 2021
 *      Author: hernan
 */

#ifndef INC_KEYPAD_BALANZA_H_
#define INC_KEYPAD_BALANZA_H_

#include <stdint.h>

#define KEYPAD_TICKS	(4500)

void set_row(uint8_t row);
uint8_t read_cols(void);
uint8_t read_keypad(void);

#endif /* INC_DEBOUNCE_H_ */
