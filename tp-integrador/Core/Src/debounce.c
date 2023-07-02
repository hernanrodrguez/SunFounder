/*
 * debounce.c
 *
 *  Created on: Jun 2, 2021
 *      Author: hernan
 */

#include "debounce.h"

/**
 * \fn 		: void debounce_init(debounce_t *s, uint8_t active_low, uint16_t debounce)
 * \brief 	: Inicializa las columnas
 * \details : Se setea una struct para manejar cada columna
 * \author 	: Hernan
 * \date   	: 26/09/2021
 * \param 	: [in] debounce_t *s
 * \param 	: [in] uint8_t active_low
 * \param 	: [in] uint16_t debounce
 * \return 	: void
 * */
void debounce_init(debounce_t *s, uint8_t active_low, uint16_t debounce){
	s->debounce = debounce;
	s->debounce_ticks = 0;
	s->active_low = (active_low)?1:0;
	s->state = 0;
    s->pin_state = (uint8_t)BTN_FREE;
	s->pin_prev_state = (uint8_t)BTN_HOLD;
}

/**
 * \fn 		: void debounce_check(debounce_t *s, uint32_t state)
 * \brief 	: Chequea el estado del pulsador/columna
 * \details : Es la MdE que filtra el rebote en cada entrada
 * \author 	: Hernan
 * \date   	: 26/09/2021
 * \param 	: [in] debounce_t *s
 * \param 	: [in] uint32_t state
 * \return 	: void
 * */
void debounce_check(debounce_t *s, uint32_t state){
	int btn;
	btn = (state) ? 1 : 0;
	btn = btn ^ s->active_low;

	switch (s->state){
	case 0:
		if(btn){
			s->debounce_ticks = s->debounce;
			s->state = 1;
		}
		s->pin_state = BTN_FREE;
		s->pin_prev_state = BTN_FREE;
		break;
	case 1:
		if (!(--(s->debounce_ticks))){
			if(btn){
				s->pin_state = BTN_HOLD;
				s->pin_prev_state = BTN_FREE;
				s->state = 2;
			}else{
				s->state = 0;
			}
		}
		break;
	case 2:
		if(!btn){
			s->debounce_ticks = s->debounce;
			s->state = 3;
		}
		s->pin_state = BTN_HOLD;
		s->pin_prev_state = BTN_HOLD;
		break;
	case 3:
		if (!(--(s->debounce_ticks))){
			if(!btn){
				s->pin_state = BTN_FREE;
				s->pin_prev_state = BTN_HOLD;
				s->state = 0;
			}else{
				s->state = 2;
			}
		}
		break;
	}
}

/**
 * \fn 		: int debounce_edge(debounce_t *s)
 * \brief 	: Verifica el sentido del pulso
 * \details : Sirve para distinguir un pulsador apretado continuo
 * \author 	: Hernan
 * \date   	: 26/09/2021
 * \param 	: [in] debounce_t *s
 * \return 	: int :	1 == nuevo pulso, 0 == se mantuvo
 * */
int debounce_edge(debounce_t *s){
	return (s->pin_state == BTN_HOLD) && (s->pin_prev_state == BTN_FREE);
}

/**
 * \fn 		: btn_state debounce_btn_state(debounce_t *s)
 * \brief 	: Estado actual
 * \details : Devuelve el estado actual del boton
 * \author 	: Hernan
 * \date   	: 26/09/2021
 * \param 	: [in] debounce_t *s
 * \return 	: btn_state : 0 btn == free , 1 btn == hold
 * */
btn_state debounce_btn_state(debounce_t *s){
	return s->pin_state;
}
