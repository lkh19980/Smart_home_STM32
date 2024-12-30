/*
 * mylib.c
 *
 *  Created on: Nov 6, 2024
 *      Author: ilove
 */

#include "main.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdio.h"

extern UART_HandleTypeDef huart2;
extern

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 10);
	return ch;
}

int __io_getchar(void)
{
	char ch;
	while(HAL_UART_Receive(&huart2, &ch, 1, 10) != HAL_OK);
	HAL_UART_Transmit(&huart2, &ch, 1, 10);	// echo
	if(ch == '\r') HAL_UART_Transmit(&huart2, "\n", 1, 10);
	return ch;
}

void Wait()
{
	while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != 0);
}

void ProgramStart(char *name)
{
	printf("\033[2J\033[1;1H\n"); // [y;xH : move cur to (x,y) 2J: 화면 클리어
	printf("Program(%s) started... Blue button to start\r\n", name);
	Wait(); //while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != 0);
}
