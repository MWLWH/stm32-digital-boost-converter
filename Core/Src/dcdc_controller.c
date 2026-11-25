/*
 * EYS.c
 *
 *  Created on: Nov 17, 2025
 *      Author: Melih
 */

#include "dcdc_controller.h"

void INIT(EYS_TYPEDEF *eys)
{
	eys->V_Kp = 0.28883f;           // Voltage loop proportional gain (under load)
	eys->V_Ki = 95.8197f;           // Voltage loop integral gain (under load)
	eys->V_NO_LOAD_Kp = 5.0f;       // Voltage loop proportional gain (no-load)
	eys->V_NO_LOAD_Ki = 100.0f;     // Voltage loop integral gain (no-load)
	eys->V_REF = 52.5f;             // Target output voltage setpoint (V)
	eys->I_Kp = 5.f;                // Current loop proportional gain
	eys->I_Ki = 95.81970f;          // Current loop integral gain
	eys->soft_start_control = 1;
	eys->bit = 1;
	Mode = NO_LOAD;
	eys->ecu_flag = 0;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)eys->ADC_BUF, 5);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
}

void PI(EYS_TYPEDEF *eys)
{
    float integral_temp = eys->V_Integral + (eys->V_E * T_S);
    eys->PI_OUT = (eys->V_Kp * eys->V_E) + (eys->V_Ki * integral_temp);

    if(eys->PI_OUT > MAX_D)
    {
        eys->PI_OUT = MAX_D;
        if(eys->V_E < 0) // Anti-windup clamping
        {
            eys->V_Integral = integral_temp;
        }
    }
    else if(eys->PI_OUT < MIN_D)
    {
        eys->PI_OUT = MIN_D;
        if(eys->V_E > 0) // Anti-windup clamping
        {
            eys->V_Integral = integral_temp;
        }
    }
    else
    {
        eys->V_Integral = integral_temp;
    }
}

void PI_NO_LOAD(EYS_TYPEDEF *eys)
{
	float integral_temp = eys->V_NO_LOAD_Integral + (eys->V_E * T_S);
	eys->PI_OUT = (eys->V_NO_LOAD_Kp * eys->V_E) + (eys->V_NO_LOAD_Ki * integral_temp);

	if(eys->PI_OUT > MAX_D)
	{
		eys->PI_OUT = MAX_D;
		eys->D = MAX_D;
		if(eys->V_E < 0)
		{
			eys->V_NO_LOAD_Integral = integral_temp;
		}
	}
	else if(eys->PI_OUT < MIN_D)
	{
		if(eys->V_E > 0)
		{
			eys->V_NO_LOAD_Integral = integral_temp;
		}
		eys->D = (eys->PI_OUT < 0) ? 0 : MIN_D;
	}
	else
	{
		eys->V_NO_LOAD_Integral = integral_temp;
		eys->D = (int)eys->PI_OUT;
	}
}

void PI_A(EYS_TYPEDEF *eys)
{
    float integral_temp = eys->I_Integral + (eys->I_E * T_S);
    eys->PI_A_OUT = (eys->I_Kp * eys->I_E) + (eys->I_Ki * integral_temp);

    // Clamp only the current loop output candidate; do not modify eys->D directly
    if(eys->PI_A_OUT > MAX_D)
    {
        eys->PI_A_OUT = MAX_D;
        if(eys->I_E < 0) // Anti-windup clamping
        {
            eys->I_Integral = integral_temp;
        }
    }
    else if(eys->PI_A_OUT < MIN_D)
    {
        eys->PI_A_OUT = MIN_D;
        if(eys->I_E > 0) // Anti-windup clamping
        {
            eys->I_Integral = integral_temp;
        }
    }
    else
    {
        eys->I_Integral = integral_temp;
    }
}

void Data(EYS_TYPEDEF *eys)
{
	eys->VRADC = (3.3f * Vref_calb) / eys->ADC_BUF[4];
	eys->V_OUT = (((eys->ADC_BUF[0] * eys->VRADC) / 4095.0f) * (V_OUTR1 + V_OUTR2)) / (V_OUTR2);
	eys->V_CAL_OUT = eys->V_OUT * V_OUT_CAL_FACTOR;

	eys->V_IN = (((eys->ADC_BUF[1] * eys->VRADC) / 4095.0f) * (V_INR1 + V_INR2)) / (V_INR2);
	eys->V_CAL_IN = eys->V_IN * V_IN_CAL_FACTOR;

	float raw_current = ((((eys->ADC_BUF[3]) * eys->VRADC) / INAGAIN) / 4095.0f) / (RSHUNT);
	eys->A = (raw_current * A_CAL_M) + A_CAL_N;

	if(!eys->ecu_flag)
	{
		eys->I_REF = (eys->ADC_BUF[2] * eys->VRADC / 4095.0f) * 12.27f;
	}

	if(((eys->ADC_BUF[2] * eys->VRADC / 4095.0f) * 12.27f) < 2 && eys->ecu_flag)
	{
		eys->I_REF = eys->CAN.ecu_current_ref;
	}
	else
	{
		eys->ecu_flag = 0;
	}

	eys->CAN.cnt++;
	if (eys->CAN.cnt > 50000)
	{
		eys->CAN.cnt = 0;
		eys->ecu_flag = 0;
	}

	eys->Median_Filter_A_OUT = Apply_Median_Filter(eys);
	EMA_Filter(eys);

	eys->V_E = eys->V_REF - eys->EMA_V_OUT;
	eys->I_E = eys->I_REF - eys->EMA_A;

	eys->WATT = eys->EMA_V_OUT * eys->EMA_A;
	eys->A_IN = (eys->EMA_V_IN > 0.1f) ? (eys->WATT / eys->EMA_V_IN) : 0.0f;
}

void PWM_Stop(EYS_TYPEDEF *eys)
{
	eys->D = 0;
	TIM8->CCR2 = eys->D;
}

void EYS_Soft_Start(EYS_TYPEDEF *eys)
{
	if(eys->soft_start_control)
	{
		if(eys->D < MIN_D)
		{
			eys->D++;
			TIM8->CCR2 = eys->D;
		}
		else
		{
			eys->soft_start_control = 0;
		}
	}
	if(!eys->soft_start_control)
	{
		TIM8->CCR2 = eys->D;
	}
}

void PWM_Change(EYS_TYPEDEF *eys)
{
	TIM8->CCR2 = eys->D;
}

void EMA_Filter(EYS_TYPEDEF *eys)
{
	if(eys->bit)
	{
		eys->EMA_V_OUT = eys->V_CAL_OUT;
		eys->EMA_V_IN  = eys->V_CAL_IN;
		eys->bit = 0;
	}
	else
	{
		eys->EMA_V_OUT = eys->EMA_V_OUT * (1.0f - V_ALPHA) + eys->V_CAL_OUT * V_ALPHA;

		if(eys->V_IN < 0.1f)
		{
			eys->EMA_V_IN = eys->V_CAL_IN;
		}
		else
		{
			eys->EMA_V_IN = eys->EMA_V_IN * (1.0f - V_ALPHA) + eys->V_CAL_IN * V_ALPHA;
		}
	}

	eys->EMA_A_REF = eys->EMA_A_REF * (1.0f - A_REF_ALPHA) + eys->I_REF * A_REF_ALPHA;

	if(eys->Median_Filter_A_OUT < 0.1f)
	{
	    eys->EMA_A = eys->Median_Filter_A_OUT;
	}
	else
	{
	    eys->EMA_A = eys->EMA_A * (1.0f - A_ALPHA) + eys->Median_Filter_A_OUT * A_ALPHA;
	}
}

float Apply_Median_Filter(EYS_TYPEDEF *eys)
{
    for(int i = 0; i < MEDIAN_SIZE - 1; i++)
    {
        eys->Median_Filter_Buffer[i] = eys->Median_Filter_Buffer[i + 1];
    }
    eys->Median_Filter_Buffer[MEDIAN_SIZE - 1] = eys->A;

    float sorted_copy[MEDIAN_SIZE];
    for(int i = 0; i < MEDIAN_SIZE; i++)
    {
        sorted_copy[i] = eys->Median_Filter_Buffer[i];
    }

    for(int i = 0; i < MEDIAN_SIZE - 1; i++)
    {
        for(int j = 0; j < MEDIAN_SIZE - i - 1; j++)
        {
            if(sorted_copy[j] > sorted_copy[j + 1])
            {
                float temp = sorted_copy[j];
                sorted_copy[j] = sorted_copy[j + 1];
                sorted_copy[j + 1] = temp;
            }
        }
    }

    return sorted_copy[MEDIAN_SIZE / 2];
}

void can_config()
{
	eys.TxHeader.DLC = 8;
	eys.TxHeader.IDE = CAN_ID_EXT;
	eys.TxHeader.RTR = CAN_RTR_DATA;
	eys.TxHeader.ExtId = 0x00000011;

	eys.canfilterconfig.FilterActivation     = CAN_FILTER_ENABLE;
	eys.canfilterconfig.FilterBank           = 0;
	eys.canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	eys.canfilterconfig.FilterMode           = CAN_FILTERMODE_IDLIST;
	eys.canfilterconfig.FilterScale          = CAN_FILTERSCALE_32BIT;

	// Accepted Extended CAN ID Bank 1 (VCU current reference command)
	eys.canfilterconfig.FilterIdHigh         = 0x0181;
	eys.canfilterconfig.FilterIdLow          = 0x900C;

	// Accepted Extended CAN ID Bank 2 (duplicate ID to fill 32-bit ID list slot)
	eys.canfilterconfig.FilterMaskIdHigh     = 0x0181;
	eys.canfilterconfig.FilterMaskIdLow      = 0x900C;

	eys.canfilterconfig.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &eys.canfilterconfig);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan1);
}

void can_data()
{
	eys.CAN.c_out.C_OUT = eys.A * 1000;
	eys.CAN.v_in.V_IN   = eys.EMA_V_IN * 1000;
	eys.CAN.v_out.V_OUT = eys.EMA_V_OUT * 1000;
	eys.CAN.c_ref.C_REF = eys.EMA_A_REF * 1000;

	eys.CAN.message[0] = eys.CAN.v_in.msb;
	eys.CAN.message[1] = eys.CAN.v_in.lsb;
	eys.CAN.message[2] = eys.CAN.v_out.msb;
	eys.CAN.message[3] = eys.CAN.v_out.lsb;
	eys.CAN.message[4] = eys.CAN.c_out.msb;
	eys.CAN.message[5] = eys.CAN.c_out.lsb;
	eys.CAN.message[6] = eys.CAN.c_ref.msb;
	eys.CAN.message[7] = eys.CAN.c_ref.lsb;

	HAL_CAN_AddTxMessage(&hcan1, &eys.TxHeader, eys.CAN.message, &eys.TxMailbox);
}

int Slew_Rate_Limit(EYS_TYPEDEF *eys, float target_D)
{
	int t_D = (int)target_D;

	if(t_D > MAX_D) t_D = MAX_D;
	if(t_D < MIN_D) t_D = MIN_D;

	if((t_D - eys->D) > MAX_D_STEP)
	{
		eys->D += MAX_D_STEP;
	}
	else if((eys->D - t_D) > MAX_D_STEP)
	{
		eys->D -= MAX_D_STEP;
	}
	else
	{
		eys->D = t_D;
	}

	return eys->D;
}