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

#define BTN_ARRIBA		1
#define BTN_ABAJO		2
#define BTN_ACEPTAR		3

/*
 * PODEMOS USAR LOS TICK DEL ENCODER COMO AJUSTE PARA CIERTO PARAMETROS ASI EVITAMOS EL USO DE
 * LA BOTONERA
 */

/*
 *Estructura para definir los menus e indicadores
 */
typedef struct{
	uint32_t tiempo_espera;
	bool en_espera;
	uint8_t estado_interno;
}Tiempo_espera_s;
typedef enum{
	Pagina_Inicial=0,
	Opcion_Configuracion_PID_1,
	Opcion_Configuracion_PID_2,
	Opcion_Calibracion_IMU,
	Opcion_Calibracion_Sensores,
	Opcion_Calibracion_X,
	Opcion_Monitoreo,
	Opcion_Iniciar_CodigoA,
	Opcion_Guardar,
	Pagina_Final
}Menu_Parametros_e;
typedef enum{
	Estado_ok,
	Estado_fallo,
	Estado_advertencia,
	Aviso_conectado,
	Aviso_procesando,
	Aviso_ok,
	Aviso_listo,
	Aviso_desconectado,
	Aviso_error_sensor,
	Aviso_bateria_baja,
	Aviso_fallo_comunicacion,
}Avisos_Alarmas_e;
void	Tiempo_Espera();
void 	Menu_Navegacion(uint8_t BTN);
void	Menu_Ejecucion(void);
void	Menu_Avisos(Avisos_Alarmas_e Aviso_);

#endif /* INC_LIB_MENU_H_ */
