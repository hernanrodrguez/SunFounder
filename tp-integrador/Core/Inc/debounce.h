/*
 * debounce.h
 *
 *  Created on: Jun 2, 2021
 *      Author: hernan
 */

#ifndef INC_DEBOUNCE_H_
#define INC_DEBOUNCE_H_

#include <stdint.h>

#define DEBOUNCE_TICKS 	(500)


typedef enum {
	BTN_FREE = 0, BTN_HOLD
} btn_state;
typedef struct {
	uint16_t debounce_ticks;
	uint16_t debounce;
	uint8_t state;
	uint8_t pin_state;
	uint8_t pin_prev_state;
	uint8_t active_low;
} debounce_t;

void debounce_init(debounce_t *s, uint8_t active_low, uint16_t debounce);
void debounce_check(debounce_t *s, uint32_t state);
int debounce_edge(debounce_t *s);
btn_state debounce_btn_state(debounce_t *s);

#endif /* INC_DEBOUNCE_H_ */
