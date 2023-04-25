/*
 * motor.c
 *
 *  Created on: Apr 12, 2023
 *      Author: 12423
 */
#include "motor.h"
#include "main.h"


void move_forward(){
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 1);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, 0);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, 0);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, 1);
}


void move_backward(){
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 0);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, 1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, 1);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, 0);
}


void move_left(){
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 1);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, 0);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, 1);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, 0);
}


void move_right(){
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 0);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, 1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, 0);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, 1);
}


void move_stop(){
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 0);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, 0);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, 0);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, 0);
}


