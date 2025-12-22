/*
 * LIB_DEBUG.c
 *
 *  Created on: Dec 22, 2025
 *      Author: Vlady-Chuwi
 */

#include "LIB_DEBUG.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define UART	huart3
extern UART_HandleTypeDef UART;

#ifndef NDEBUG

static char bufferTxt[30];//buffer para enviar datos maximo 30

void DEBUG_Imprimir(char* texto)
{
	if (texto==NULL) return;
	uint16_t size=(uint16_t)strlen(texto);
	HAL_UART_Transmit(&UART, (uint8_t *)texto, size, HAL_MAX_DELAY);
}
void DEBUG_Encoders(int32_t encL, int32_t encR,float longitud)
{

	sprintf(bufferTxt," EncL=%ld ",encL);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," EncR=%ld " ,encR);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," LEncL=%0.2f ",encL*longitud);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," LEncR=%0.2f \r \n" ,encR*longitud);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
}
void DEBUG_Odometria(float angulo,float x,float y)
{
	sprintf(bufferTxt," angulo=%0.2f ",angulo);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," x=%0.2f ",x);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," y=%0.2f \r \n",y);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
}
void DEBUG_IMU()
{

}
void DEBUG_PID1()
{

}
void DEBUG_PID2()
{

}
void DEBUG_Menu()
{

}

void DEBUG_ADC(uint16_t A0,uint16_t A1,uint16_t A2,uint16_t A3,uint16_t A4)
{
	sprintf(bufferTxt," ADC0=%u ",A0);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," ADC1=%u ",A1);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," ADC2=%u ",A2);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," ADC3=%u ",A3);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	sprintf(bufferTxt," ADC4=%u \r \n",A4);
	HAL_UART_Transmit(&UART, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
}
#endif

