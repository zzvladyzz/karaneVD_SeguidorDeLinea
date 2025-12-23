/*
 * LIB_LED_INDA.c
 *
 *  Created on: Dec 13, 2025
 *      Author: Vlady-Chuwi
 */
#include "LIB_MENU.h"
#include "gpio.h"


//static Estado_LED_e 	Aviso=Aviso_espera;
static Menu_Parametros_e 	Menu=Menu_Inicial;
static bool BTN_SEL=false;
static Tiempo_espera_s Tiempo={0,true,0};


/**
 *	@brief 	Se ejecuta un tiempo de espera para los avisos
 */
void	Tiempo_Espera(void)
{

	uint32_t tiempo_actual=HAL_GetTick();
	switch (Tiempo.estado_interno) {
		case 0:
			Tiempo.tiempo_de_espera=tiempo_actual;
			Tiempo.esperando=true;
			Tiempo.estado_interno=1;
			break;
		case 1:
			if(tiempo_actual-Tiempo.tiempo_de_espera>300)
			{
				Tiempo.estado_interno=2;
			}
			break;
		case 2:
			Tiempo.estado_interno=0;
			Tiempo.esperando=false;
			break;
		default:
			break;
	}
}

/**
 * @brief 	Menu donde se vera en que opcion entrar
 * @param	MENU: Se debe inicializar el ENUM antes para poder mover entre menus
 * @param 	BTN: Es una valor de 1 a 3 que viene de los pulsadores ya filtrado
 * @note	Una vez terminada esta funcion verificar que el valor del adc o del pulsador quede
 * 			en cero para evitar los rebotes innecesarios
 */
void 	Menu_Navegacion(uint8_t BTN)
{

	if(BTN_SEL==false){
		switch (BTN) {
			case BTN_DERECHA:
				Menu=(Menu==Menu_Inicial)?Menu_Final:(Menu-1);
				Menu_LED(Estado_BTN_derecha);
				break;
			case BTN_IZQUIERDA:
				Menu=(Menu==Menu_Final)?Menu_Inicial:(Menu+1);
				Menu_LED(Estado_BTN_izquierda);
				break;
			case BTN_ACEPTAR:
				Menu=Menu;
				BTN_SEL=true;
				Menu_LED(Estado_BTN_ok);
				break;
			default:
				break;
		}
	}
	else{
		switch (BTN) {
			case BTN_ACEPTAR:
				BTN_SEL=false;
				Menu_LED(Estado_BTN_salir);
				break;
			default:
				break;
		}
	}


}
/**
 * @brief Menu para realizar la funcion seleccionada
 */
void	Menu_Ejecucion(void)
{
	if(BTN_SEL==true){
		switch (Menu) {
			case Menu_Inicial:
				Menu_Avisos(Aviso_espera);
				break;
			case Opcion_Iniciar_CodigoA:
				Menu_Avisos(Aviso_error_sensor);
				break;
			case Opcion_Calibracion_IMU:
				Menu_Avisos(Aviso_bateria_baja);
				break;
			case Opcion_Calibracion_Sensores:
				Menu_Avisos(Aviso_desconectado);
				break;
			case Opcion_Calibracion_X:
				Menu_Avisos(Aviso_fallo);
				break;
			case Opcion_Configuracion_PID_1:
				Menu_Avisos(Aviso_fallo_comunicacion);
				break;
			case Opcion_Configuracion_PID_2:
				Menu_Avisos(Aviso_ok);
				break;
			case Opcion_Monitoreo:
				Menu_Avisos(Aviso_procesando);
				break;
			case Opcion_Guardar:
				Menu_Avisos(Aviso_fallo_comunicacion);
				break;
			case Menu_Final:
				Menu_Avisos(Aviso_espera);
				break;

			default:
				break;
			}
	}
}

/**
 * @brief Aca se indicara mediante leds los errores o cualquier otra accion
 * @param Aviso: Se debera mandar una estructura para saber que aviso atender con los LED's
 * @note	Esta funcion necesita ser llamada mediante los retardos de esta misma lib para parpadear
 * 			los avisos
 */
void Menu_Avisos(Estado_LED_e Alarmas)
{
	Tiempo_Espera();
	if(Tiempo.esperando==false)
	{
		switch(Alarmas){
			case Aviso_fallo:
				HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
				HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
				HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
				HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
				HAL_GPIO_TogglePin(LED5_GPIO_Port, LED5_Pin);
				HAL_GPIO_TogglePin(LED6_GPIO_Port, LED6_Pin);
				break;
			case Aviso_bateria_baja:
				HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
				HAL_GPIO_TogglePin(LED6_GPIO_Port, LED6_Pin);
				break;
			case Aviso_ok:
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
				break;
			case Aviso_desconectado:
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
				break;
			case Aviso_conectado:
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
				HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
				HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
				HAL_GPIO_TogglePin(LED5_GPIO_Port, LED5_Pin);
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
				break;
			case Aviso_espera:	//Desde aca seria los avisos en binario 1
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
				break;
			case Aviso_procesando:
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
				break;
			case Aviso_error_sensor:
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
				HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
				break;
			case Aviso_fallo_comunicacion:
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin,	GPIO_PIN_RESET);
				HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
				break;
			default:
				break;
		}
	}
}
void	Menu_LED(Estado_LED_e LED)
{
	switch(LED){
				case Estado_BTN_derecha:
					HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
					HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
					break;
				case Estado_BTN_izquierda:
					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
					HAL_GPIO_TogglePin(LED6_GPIO_Port, LED6_Pin);
					break;
				case Estado_BTN_ok:
					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
					break;
				case Estado_BTN_salir:
					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
					break;
				default:
					break;

		}
}

