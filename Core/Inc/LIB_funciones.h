/*
 * LIB_funciones.h
 *
 *  Created on: Dec 22, 2025
 *      Author: Vlady-Chuwi
 */

#ifndef INC_LIB_FUNCIONES_H_
#define INC_LIB_FUNCIONES_H_
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
/*
 * Constantes fisicas del robot para odometria
 */
#define TICKS_POR_REVOLUCION  200.0 // Cuantos ticks da el encoder por vuelta completa
#define DIAMETRO_RUEDA_MM      	29.1571 // Diámetro de la rueda en mm y grosor 12,7
#define DISTANCIA_ENTRE_RUEDAS_MM 105.8 // Distancia entre los centros de las ruedas
#define LONGITUD_MUESTRA_MM (M_PI * DIAMETRO_RUEDA_MM / TICKS_POR_REVOLUCION)
/*
 * Variables de estado global del robot para odometria
 */
typedef struct{
	float robot_X;     // Posición X actual en mm
	float robot_Y;     // Posición Y actual en mm
	float robot_Angulo_rad; // Orientación/Rumbo actual en radianes
	float robot_Angulo_deg;
	volatile int32_t ticks_L ; // Contadores de los encoders (desde el callback EXTI)
	volatile int32_t ticks_R ;
}odometria_init_t;
/*
 * Variables para control del motor
 */
typedef struct
{
	uint16_t pwmR;
	uint16_t pwmL;
	bool  enable_PWM;
}motores_init_t;


void funcion_odometria(odometria_init_t* odometria);
void motores(motores_init_t* motores);
float Filtro_Kalman_odometria(float delta_theta_encoders,float gyro_rate_z,float dt);

#endif /* INC_LIB_FUNCIONES_H_ */
