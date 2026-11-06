#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"
#include "stdlib.h"
#include "math.h"
typedef struct {
	GPIO_TypeDef *dir_GPIO;				// ileri yön
	uint16_t dir1_pin_forward;
	uint16_t dir2_pin_back;
    TIM_HandleTypeDef *htim;    		// Timer handle pointer'ı
    uint32_t tim_channel;           	// Timer kanalı
    volatile float current_duty;      	// Mevcut duty cycle
    volatile float target_duty;      	// Hedef duty cycle
    uint32_t max_duty;          		// Maksimum duty sınırı
    volatile uint8_t is_running;        // Motor çalışıyor mu?
    volatile float ramp_val;
    volatile uint16_t duty_to_pwm;
    volatile uint8_t ramp_active;
    uint8_t zero_delay;
    uint16_t ramp_time_ms;
} Motor_t;

typedef struct {
	volatile float current_duty;
	volatile float target_duty;
    uint8_t is_running;
    uint8_t ramp_active;
    uint8_t percent;
} Motor_Status_t;

void Motor_Init(Motor_t *motor, TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *dir_GPIO, uint16_t dir1_pin_forward, uint16_t dir2_pin_back);
void Motor_SetDutyWithRamp(Motor_t *motor, float target_duty, uint16_t ramp_time_ms);
void Motor_UpdateRamp(Motor_t *motor);
void Motor_SetDuty(Motor_t *motor, float duty);
void Motor_Start(Motor_t *motor);
void Motor_Stop(Motor_t *motor);
void Motor_GetStatus(Motor_t *motor, Motor_Status_t *status);
void Motor_DutyConfig(Motor_t *motor,float duty);

#endif
