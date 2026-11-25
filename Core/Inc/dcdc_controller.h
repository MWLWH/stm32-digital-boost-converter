/*
 * EYS.h
 *
 *  Created on: Nov 17, 2025
 *      Author: Melih
 */

#ifndef INC_DCDC_CONTROLLER_H_
#define INC_DCDC_CONTROLLER_H_

#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"

// PWM switching frequency: 100 kHz
#define MAX_D           350         // Max duty cycle under load
#define MIN_D           3           // Min duty cycle at no-load

#define V_OUTR1         250000      // Output voltage divider upper resistor (ohm)
#define V_OUTR2         9910        // Output voltage divider lower resistor (ohm)

#define V_INR1          169000      // Input voltage divider upper resistor (ohm)
#define V_INR2          9760        // Input voltage divider lower resistor (ohm)

#define RSHUNT          0.005f      // Current sense shunt resistor (5 mOhm)
#define A_CAL_M         1.011818f   // Current calibration slope factor (calibrated at 17A test)
#define A_CAL_N         0.038500f   // Current calibration offset compensation (A)
#define ADC_A_OFFSET    6
#define INAGAIN         20          // Current sense amplifier gain (V/V)
#define VREF            48
#define V_MAX           59          // Over-voltage protection threshold (V)
#define A_MAX           20          // Over-current protection threshold (A)
#define T_S             0.0001f     // Control loop sampling period (100 us / 10 kHz)

#define Vrefint         1.21
#define Vref_calb       *VREFINT_CAL_ADDR

#define V_ALPHA         0.1f        // EMA filter coefficient for voltage measurements
#define A_ALPHA         0.5f        // EMA filter coefficient for current measurement
#define V_REF_ALPHA     0.4f        // EMA filter coefficient for voltage reference
#define A_REF_ALPHA     0.5f        // EMA filter coefficient for current reference
#define MEDIAN_SIZE     5           // Moving median filter window size

#define V_OUT_CAL_FACTOR 0.99953f   // Output voltage fine calibration multiplier
#define V_IN_CAL_FACTOR  0.99542f   // Input voltage fine calibration multiplier
#define A_CAL_FACTOR     1.0f       // Current fine calibration multiplier

#define MAX_D_STEP      5           // Maximum duty-cycle step per control cycle (slew-rate limit)

typedef union
{
	uint16_t V_OUT;

	struct
	{
		uint8_t lsb : 8;
		uint8_t msb : 8;
	};
}EYS_TYPEDEF_CAN_M_V_OUT;

typedef union
{
	uint16_t V_IN;

	struct
	{
		uint8_t lsb : 8;
		uint8_t msb : 8;
	};
}EYS_TYPEDEF_CAN_M_V_IN;

typedef union
{
	uint16_t C_OUT;

	struct
	{
		uint8_t lsb : 8;
		uint8_t msb : 8;
	};
}EYS_TYPEDEF_CAN_M_C_OUT;

typedef union
{
	uint16_t C_REF;

	struct
	{
		uint8_t lsb : 8;
		uint8_t msb : 8;
	};
}EYS_TYPEDEF_CAN_M_C_REF;

typedef struct {
	uint8_t message[8];
	EYS_TYPEDEF_CAN_M_V_OUT v_out;
	EYS_TYPEDEF_CAN_M_V_IN  v_in;
	EYS_TYPEDEF_CAN_M_C_OUT c_out;
	EYS_TYPEDEF_CAN_M_C_REF c_ref;
	uint8_t received_message[2];
	uint8_t ecu_current_ref;
	uint8_t h2_ratio;
	uint32_t cnt;
}EYS_TYPEDEF_CAN_MESSAGE;

typedef struct
{
	uint16_t ADC_BUF[5];
	float VRADC;
	float V_OUT;
	float V_CAL_OUT;
	float V_CAL_IN;
	float A_CAL;
	float V_IN;
	float A_IN;
	float V_REF;
	float I_REF;
	float A;
	float WATT;
	float V_Ki;
	float V_Kp;
	float V_NO_LOAD_Ki;
	float V_NO_LOAD_Kp;
	float V_NO_LOAD_Integral;
	float I_Kp;
	float I_Ki;
	float V_E;                  // Voltage control error
	float I_E;                  // Current control error
	float V_Integral;
	float I_Integral;
	float PI_OUT;
	float PI_A_OUT;
	float EMA_V_OUT;
	float EMA_V_IN;
	float EMA_A;
	float EMA_V_REF;
	float EMA_A_REF;
	float undervoltage_error;
	float Median_Filter_Buffer[5];
	float Median_Filter_A_OUT;
	int ecu_flag;
	int D;
	EYS_TYPEDEF_CAN_MESSAGE CAN;
	CAN_TxHeaderTypeDef TxHeader;
	CAN_RxHeaderTypeDef RxHeader;
	CAN_FilterTypeDef canfilterconfig;
	uint32_t TxMailbox;

	uint8_t tim_flag;
	uint8_t soft_start_control;
	uint8_t bit;
	uint16_t cnt;
}EYS_TYPEDEF;

typedef enum
{
	IDLE,           // Standby state with PWM disabled (used for safe startup and debugging)
	NO_LOAD,        // No-load operating mode (output current below quiescent threshold)
	WITH_LOAD,      // Loaded operating mode (active dual-loop CC/CV regulation)
	Err             // Latched fault state triggered when voltage/current exceeds safe limits (duty = 0)
}EYS_MODE_TYPEDEF;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim8;

extern EYS_TYPEDEF eys;
extern EYS_MODE_TYPEDEF Mode;

void INIT(EYS_TYPEDEF *eys);
void EMA_Filter(EYS_TYPEDEF *eys);
void PI(EYS_TYPEDEF *eys);
void PI_NO_LOAD(EYS_TYPEDEF *eys);
void PI_A(EYS_TYPEDEF *eys);
void Data(EYS_TYPEDEF *eys);
void PWM_Stop(EYS_TYPEDEF *eys);
void PWM_Change(EYS_TYPEDEF *eys);
void EYS_Soft_Start(EYS_TYPEDEF *eys);
float Apply_Median_Filter(EYS_TYPEDEF *eys);
void can_config();
void can_data();
int  Slew_Rate_Limit(EYS_TYPEDEF *eys, float target_D);

#endif /* INC_DCDC_CONTROLLER_H_ */