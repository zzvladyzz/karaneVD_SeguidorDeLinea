/*
 * LIB_LED_INDA.h
 *
 *  Created on: Dec 13, 2025
 *      Author: Vlady-Chuwi
 */

#ifndef INC_LIB_MENU_H_
#define INC_LIB_MENU_H_

#include <stdint.h>
#include <stdbool.h>

#define BTN_DERECHA		1
#define BTN_IZQUIERDA	2
#define BTN_ACEPTAR		3


/**
 *	@brief 	Estructura para definir retardos dentro de los menus si se necesita
 */
typedef struct{
	uint32_t tiempo_de_espera;
	bool esperando;
	uint8_t estado_interno;
}Tiempo_espera_s;

/**
 *	@brief 	Estructura para definir los menus y realizar ajustes o ejecutar el programa
 */
typedef enum{
	Menu_Inicial=0,
	Opcion_Configuracion_PID_1,
	Opcion_Configuracion_PID_2,
	Opcion_Calibracion_IMU,
	Opcion_Calibracion_Sensores,
	Opcion_Calibracion_X,
	Opcion_Monitoreo,
	Opcion_Iniciar_CodigoA,
	Opcion_Guardar,
	Menu_Final
}Menu_Parametros_e;

/**
 *	@brief 	Estructura para generar avisos segun la necesidad
 */
typedef enum{
	Estado_BTN_derecha,
	Estado_BTN_izquierda,
	Estado_BTN_ok,
	Estado_BTN_salir,
	Aviso_fallo,
	Aviso_espera,
	Aviso_conectado,
	Aviso_procesando,
	Aviso_ok,
	Aviso_desconectado,
	Aviso_error_sensor,
	Aviso_bateria_baja,
	Aviso_fallo_comunicacion,

}Estado_LED_e;

void	Tiempo_Espera(void);
void 	Menu_Navegacion(uint8_t BTN);
void	Menu_Ejecucion(void);
void	Menu_Avisos(Estado_LED_e Alarmas);
void	Menu_LED(Estado_LED_e LED);

#endif /* INC_LIB_MENU_H_ */
