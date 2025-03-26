/**
 * @file alg_power_limit.h
 * @author lez 
 * @brief ??????
 * @version 1.1
 * @date 2024-07-1 0.1 24????
 *
 * @copyright ZLLC 2024
 *
 */

#ifndef ALG_POWER_LIMIT_OLD_H
#define ALG_POWER_LIMIT_OLD_H

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "arm_math.h"
#include "dvc_djimotor.h"
#include "config.h"

/* Exported macros -----------------------------------------------------------*/

#define RAD_TO_RPM              9.5493f
#define CMD_CURRENT_TO_TORQUE   ((20.f/16384.f)*0.3f)   //???3508????????cmd?????
#define REDUATION               (3591.f/187.f)         //???
 
class Class_DJI_Motor_C620;

class Class_Power_Limit
{
    public:

    inline void Set_Power_Limit(float __total_power_limit);
    inline void Set_Chassis_Buffer(float __buffer);
    inline void Set_Supercap_Enegry(float __energy);
    inline void Set_Supercap_Voltage(float __voltage);
    inline void Set_Supercap_Print_Flag(uint8_t __flag);

    float Get_Torque_Current(uint8_t num);

    void Set_Motor(Class_DJI_Motor_C620 (&Motor)[4]);
    //?????????????????
    void Output(Class_DJI_Motor_C620 (&Motor)[4]);
    float Calculate_Limit_K(float omega[],float torque[],float power_limit,uint8_t motor_nums);

    void TIM_Adjust_PeriodElapsedCallback(Class_DJI_Motor_C620 (&Motor)[4]);
    
    protected:

    float Limit_K = 1.0f;
    float Chassis_Buffer;
    float Buffer_K = 1.5;
    float Buffer_power_limit = 45.f;
    float Buffer_power;
    const float Min_Buffer = 30.0f; 
    const float Protected_Buffer = 30.0f;

    //???? rad?rpm??
	float Toque_Coefficient = 1.99688994e-6f * (3591/187) / 13.93f;  // (20/16384)*(0.3)*(187/3591)/9.55

    float current_to_torqure = (20.f/16384.f)*(0.3f);
    float rpm_to_omega = (PI/30.f)/13.92f;

    //??????
	float k1 = 1.3;		// k1 
	float k2 = 0.015;		// k2 
	float Alpha = 0.0f;
    float Tansfer_Coefficient = 9.55f;  //???? w*t/Tansfer_Coefficient

    //????
    float Supercap_Energy;
    //????
    float Supercap_Voltage;
    //??????
    uint8_t Supercap_Print_Flag = 0;

    //????
    float equation_a;
    float equation_b;
    float equation_c;

    //?????????
    float Input_Torque[4];  
    //?????????
    float Output_Torque[4];  
    //???????
    float Torque_Now[4];  
    //??????????? rad/s
    float Omega[4];	 
    //???????
    float Total_Power_Limit;  
    //???????
    float Total_Predict_Power = 0;  
    //????
    float Predict_Power[4]; 
    //??????
	float Power_Scale;  
    //?????????
	float Scaled_Give_Power[4];  
};

/**
 * @brief ???????
 *
 */
void Class_Power_Limit::Set_Power_Limit(float __total_power_limit)
{
    Total_Power_Limit = __total_power_limit;
}

/**
 * @brief ????????????
 *
 */
void Class_Power_Limit::Set_Chassis_Buffer(float __buffer)
{
    Chassis_Buffer = __buffer;
}

/**
 * @brief ????????????
 *
 */
void Class_Power_Limit::Set_Supercap_Enegry(float __energy)
{
   Supercap_Energy = __energy;
}

/**
 * @brief ??????????
 *
 */
void Class_Power_Limit::Set_Supercap_Voltage(float __voltage)
{
    Supercap_Voltage = __voltage;
}

/**
 * @brief ??????????
 *
 */
void Class_Power_Limit::Set_Supercap_Print_Flag(uint8_t __flag)
{
    Supercap_Print_Flag = __flag;
}

/* Exported types ------------------------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/