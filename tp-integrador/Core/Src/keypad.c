/*
 * keypad_balanza.c
 *
 *  Created on: Oct 24, 2021
 *      Author: tobias
 */
#include "keypad.h"
#include "debounce.h"
#include "main.h"

debounce_t deb_col_1; 	//! Variable para inicializar
debounce_t deb_col_2;	//! Variable para inicializar
debounce_t deb_col_3;	//! Variable para inicializar
/**
 * \fn 		: void set_row(uint8_t row)
 * \brief 	: Fuerza '1' para leer el teclado matricial
 * \details : Barre las filas para leer el teclado
 * \author 	: Tobias Bavasso Piizzi
 * \date   	: 26/09/2021
 * \param 	: [in] uint8_t row
 * \return 	: void
 * */
void set_row(uint8_t row){
	switch(row){
	case 1:
		HAL_GPIO_WritePin(GPIOA, ROW_4_Pin|ROW_3_Pin|ROW_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, ROW_1_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOA, ROW_4_Pin|ROW_3_Pin|ROW_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, ROW_2_Pin, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOA, ROW_4_Pin|ROW_2_Pin|ROW_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, ROW_3_Pin, GPIO_PIN_RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOA, ROW_3_Pin|ROW_2_Pin|ROW_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, ROW_4_Pin, GPIO_PIN_RESET);
		break;
	default:
		HAL_GPIO_WritePin(GPIOA, ROW_4_Pin|ROW_3_Pin|ROW_2_Pin|ROW_1_Pin, GPIO_PIN_SET);
		break;
	}
}

/**
 * \fn 		: uint8_t read_cols(void)
 * \brief 	: Barre las columnas del keypad
 * \details : Hace el chequeo de cada reobte en columna
 * \author 	: Tobias Bavasso Piizzi
 * \date   	: 26/09/2021
 * \param 	: void
 * \return 	: uint8_t : 1 == COL1 , 2 == COL2 , 3 == COL3
 * */
uint8_t read_cols(){
	debounce_check(&deb_col_1, HAL_GPIO_ReadPin(GPIOA, COL_1_Pin));
	if(debounce_edge(&deb_col_1))
		return 1;
	debounce_check(&deb_col_2, HAL_GPIO_ReadPin(GPIOA, COL_2_Pin));
	if(debounce_edge(&deb_col_2))
		return 2;
	debounce_check(&deb_col_3, HAL_GPIO_ReadPin(GPIOA, COL_3_Pin));
	if(debounce_edge(&deb_col_3))
		return 3;
	return 0;
}

/**
 * \fn 		: uint8_t read_keypad()
 * \brief 	: Inicializa los parametros para promediar una muestra
 * \details : Trababaja con un buffer y setenado todo en '0'
 * \author 	: Tobias Bavasso Piizzi
 * \date   	: 26/09/2021
 * \param 	: void
 * \return 	: uint8_t :
 * */
uint8_t read_keypad(){
	static uint8_t current_row = 1;
	static uint32_t keypad_millis = KEYPAD_TICKS;
	uint8_t current_col = 0;

	current_col = read_cols();
	if(current_col){
		return (((current_row-1)*3) + current_col);
	}

	switch(current_row){
	case 1:
		if(!(--keypad_millis)){
			set_row(2);
			keypad_millis = KEYPAD_TICKS;
			current_row = 2;
		}
		break;
	case 2:
		if(!(--keypad_millis)){
			set_row(3);
			keypad_millis = KEYPAD_TICKS;
			current_row = 3;
		}
		break;
	case 3:
		if(!(--keypad_millis)){
			set_row(4);
			keypad_millis = KEYPAD_TICKS;
			current_row = 4;
		}
		break;
	case 4:
		if(!(--keypad_millis)){
			set_row(1);
			keypad_millis = KEYPAD_TICKS;
			current_row = 1;
		}
		break;
	default:
		set_row(1);
		keypad_millis = KEYPAD_TICKS;
		current_row = 1;
		break;
	}
	return 0;
}
