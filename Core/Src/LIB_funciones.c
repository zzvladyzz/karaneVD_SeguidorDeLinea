/*
 * LIB_funciones.c
 *
 *  Created on: Dec 22, 2025
 *      Author: Vlady-Chuwi
 */
#include "LIB_funciones.h"
#include "tim.h"
#include "gpio.h"
#include <stdint.h>
#define Motor htim1

extern TIM_HandleTypeDef Motor;

/**
 * @brief Codigo para odometria implementarlo en un timer para la ejecucion
 * @param odometria se inicializa con los datos a usar
 */
 void funcion_odometria(odometria_init_t* odometria)
{
// 1. Obtener los ticks desde la última vez y resetear contadores para evitar overflow de datos
// Usamos variables intermedias para ser thread-safe (si las interrupciones EXTI pueden ocurrir ahora)
int32_t delta_ticks_L = odometria->ticks_L;
int32_t delta_ticks_R = odometria->ticks_R;
odometria->ticks_L = 0; // Resetear el contador
odometria->ticks_R = 0; // Resetear el contador

// 2. Convertir ticks a distancia lineal recorrida (en mm)
float distancia_L = delta_ticks_L * LONGITUD_MUESTRA_MM;
float distancia_R = delta_ticks_R * LONGITUD_MUESTRA_MM;

// Distancia promedio recorrida por el centro del robot
float delta_distancia = (distancia_L + distancia_R) / 2.0;

// 3. Calcular el cambio en la orientación angular (en radianes)
float delta_angulo = (distancia_R - distancia_L) / DISTANCIA_ENTRE_RUEDAS_MM;

// 4. Actualizar la posición global del robot (Integración)

// Usamos el ángulo actual + la mitad del cambio angular para mayor precisión (Runge-Kutta de 2do orden)
float angulo_promedio = odometria->robot_Angulo_rad + delta_angulo / 2.0;

// Calcular el desplazamiento en X e Y y sumarlo a la posición actual
odometria->robot_X += delta_distancia * cos(angulo_promedio);
odometria->robot_Y += delta_distancia * sin(angulo_promedio);

// 5. Actualizar el ángulo global del robot
odometria->robot_Angulo_rad += delta_angulo;

// Opcional: Mantener el ángulo entre -PI y PI (para evitar overflow float)
if (odometria->robot_Angulo_rad > M_PI) odometria->robot_Angulo_rad -= 2 * M_PI;
if (odometria->robot_Angulo_rad < -M_PI) odometria->robot_Angulo_rad += 2 * M_PI;
odometria->robot_Angulo_deg=(odometria->robot_Angulo_rad*180.0)/M_PI;
}

void motores(motores_init_t* motores)
{
	  __HAL_TIM_SetCompare(&Motor,TIM_CHANNEL_1,motores->pwmL);
	  __HAL_TIM_SetCompare(&Motor,TIM_CHANNEL_2,0);
	  __HAL_TIM_SetCompare(&Motor,TIM_CHANNEL_3,0);
	  __HAL_TIM_SetCompare(&Motor,TIM_CHANNEL_4,motores->pwmR);
	  HAL_GPIO_WritePin(EN_MOT_GPIO_Port, EN_MOT_Pin,motores->enable_PWM);
}
