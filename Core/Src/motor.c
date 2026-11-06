#include "motor.h"
#include <math.h>

#define DEADZONE  0.005f
#define MAX_TIM_PWM 10499   // TIMER çıkışına uygulanabilcek maks PWM
#define RAMP_UPDATE_MS 10		// 10ms 100hz ile


void Motor_Init(Motor_t *motor, TIM_HandleTypeDef *htim, uint32_t tim_channel, GPIO_TypeDef *dir_GPIO, uint16_t dir1_pin_forward, uint16_t dir2_pin_back){
	motor->htim = htim;
	motor->tim_channel = tim_channel;
	motor->current_duty = 0;
	motor->is_running = 0;
	motor->ramp_val = 0;
	motor->target_duty = 0;
	motor->duty_to_pwm=0;
	motor->ramp_active = 0;
	motor->dir_GPIO = dir_GPIO;
	motor->dir1_pin_forward = dir1_pin_forward;
	motor->dir2_pin_back = dir2_pin_back;
	motor->zero_delay = 0;
	motor->ramp_time_ms = 0;

	HAL_GPIO_WritePin(dir_GPIO, dir1_pin_forward, GPIO_PIN_SET);
	HAL_GPIO_WritePin(dir_GPIO, dir2_pin_back, GPIO_PIN_SET);

}

//  DUTY LİMİT  DEĞERLERİ VE YÖN AYARLMASI YAPILIR
void Motor_DutyConfig(Motor_t *motor, float duty)
{
	if (duty > 1.0f) duty = 1.0f;
	if (duty < -1.0f) duty = -1.0f;

	if (fabsf(duty) < DEADZONE) duty = 0.0f;

	if (duty>0.0f)
	{
		// İLERİ YÖN DIR1 HIGH, DIR2 LOW
		HAL_GPIO_WritePin(motor->dir_GPIO, motor->dir1_pin_forward, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->dir_GPIO, motor->dir2_pin_back, GPIO_PIN_SET);
	}
	else if (duty<0.0f)
	{
		// GERİ YÖN DIR1 LOW, DIR2 HIGH
		HAL_GPIO_WritePin(motor->dir_GPIO, motor->dir1_pin_forward, GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor->dir_GPIO, motor->dir2_pin_back, GPIO_PIN_RESET);
	}

	else
	{	// MOTOR STOP KONUMUNA GETİRİLİR
		HAL_GPIO_WritePin(motor->dir_GPIO, motor->dir1_pin_forward, GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor->dir_GPIO, motor->dir2_pin_back, GPIO_PIN_SET);
	}

}

// 	RAMPASIZ ANLIK DEĞİŞİM İÇİN
void Motor_SetDuty(Motor_t *motor, float duty){

	Motor_DutyConfig(motor, duty);

	motor->current_duty = duty;
	motor->target_duty = duty;
	motor->ramp_active = 0;

	motor->duty_to_pwm = (uint16_t)(fabs(duty)*MAX_TIM_PWM);
	__HAL_TIM_SET_COMPARE(motor->htim, motor->tim_channel, motor->duty_to_pwm);
}

// Rampa ile duty ayarla

void Motor_SetDutyWithRamp(Motor_t *motor, float target_duty, uint16_t ramp_time_ms) {
	if (motor->target_duty == target_duty) {
	        return;

	}

	if (target_duty > 1.0f) target_duty = 1.0f;
	if (target_duty < -1.0f) target_duty = -1.0f;
	if (fabsf(target_duty) < DEADZONE) target_duty = 0.0f;
    motor->target_duty = target_duty;
    // Rampa adımını hesapla
    if (ramp_time_ms >= RAMP_UPDATE_MS ) {
        // Toplam adım sayısı
        uint16_t total_steps = ramp_time_ms / RAMP_UPDATE_MS;

        if (total_steps > 0) {

            float difference = motor->target_duty - motor->current_duty;
            motor->ramp_val = fabsf(difference) / total_steps; // KÜSURATLI OLANLAR YUKARI YUVARLAR DAHA SONRA HEDEFE GELİNCE FAZLA ADIMLAR KESİLİR

            if (motor->ramp_val < 0.0001f) motor->ramp_val = 0.0001f;
            // ÇOK KÜÇÜK DEĞİŞİKLİKLER OLDUĞUNDA SİSTEM GÖRMEZ O YÜZDEN 1 ARTTIRIYORUZ
            // DAHA SONRA KONTROL KISIMDA FAZLASI KESİLİYOR
            motor->ramp_active = 1;

        } else {
        	// RAMPA ZAMANI YOKSA ya da == 0'sa DİREKT ÇALIŞTIR
            Motor_SetDuty(motor, target_duty);
        }
    } else {
      //  RAMPA ZAMANI YOKSA ya da == 0'sa DİREKT ÇALIŞTIR
        Motor_SetDuty(motor, target_duty);
    }
}

// TİMER İNTERRUPT İÇİNDE HER 10MS DE BİR PWM ADIMLARINI ARTIRACAK
// örn: 1sn girildi 1/10ms = 100 adım, 100 adımda istenen duty oranına getirecek
void Motor_UpdateRamp(Motor_t *motor) {
    // Motor ramp modunda değilse veya hedefe ulaştıysa çık
	if (!motor->ramp_active)
	        return;

	if (motor->zero_delay > 0)
	{
		motor->zero_delay--;
	    __HAL_TIM_SET_COMPARE(motor->htim, motor->tim_channel, 0);
	    Motor_DutyConfig(motor, 0);
	    if (motor->zero_delay == 0) {
	        motor->current_duty = 0.0f;
	    }
	    return;
    }
    // --------- YÖN DEĞİŞİMİ ALGILAMA ---------
	if ((motor->current_duty * motor->target_duty) <0 )
	{
		 __HAL_TIM_SET_COMPARE(motor->htim, motor->tim_channel, 0);
		 Motor_DutyConfig(motor, 0);
		 motor->current_duty=0;
		 motor->zero_delay = 2;
		 return;
	}

    else
    {
    	float difference = motor->target_duty - motor->current_duty;
        /* --------- NORMAL RAMPA --------- */
        if (fabsf(difference) > motor->ramp_val)
        {
        	if (difference > 0) {
        	    motor->current_duty += motor->ramp_val;
        	} else {
        	    motor->current_duty -= motor->ramp_val;
        	}
        }
        else
        {	// hedefe yakalştı, son adım doğrudan hedefe
            motor->current_duty = motor->target_duty;

        }
    }
    motor->duty_to_pwm = (uint16_t)fabsf(motor->current_duty * MAX_TIM_PWM);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->tim_channel, motor->duty_to_pwm);

    Motor_DutyConfig(motor, motor->current_duty);

    if(fabsf(motor->current_duty - motor->target_duty)< 0.001f){
    	motor->current_duty = motor->target_duty;
    	motor->ramp_val = 0;
    	motor->ramp_active =0;
    }

}

void Motor_Start(Motor_t *motor){
	HAL_TIM_PWM_Start(motor->htim, motor->tim_channel); // TIM aktif et, fakat duty 0'la
	Motor_SetDuty(motor, 0);
	motor->is_running = 1;
}

void Motor_Stop(Motor_t *motor){
	motor->ramp_active=0;
	Motor_SetDuty(motor,0);
	HAL_TIM_PWM_Stop(motor->htim, motor->tim_channel);
	motor->is_running = 0;

}

void Motor_GetStatus(Motor_t *motor, Motor_Status_t *status) {
    status->current_duty = motor->current_duty;
    status->target_duty = motor->target_duty;
    status->is_running = motor->is_running;
    status->ramp_active = (motor->ramp_val > 0);
    status->percent = (motor->current_duty * 100);
}

