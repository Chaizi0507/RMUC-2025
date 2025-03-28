/**
 * @file crt_gimbal.cpp
 * @author cjw
 * @brief 云台
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_gimbal.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{
    // imu初始化
    Boardc_BMI.Init();

    // yaw轴电机
    Motor_Yaw_A.PID_Angle.Init(30.f, 5.0f, 0.0f, 0.0f, Motor_Yaw_A.Get_Output_Max(), Motor_Yaw_A.Get_Output_Max());
    Motor_Yaw_A.PID_Omega.Init(40.0f, 15.0f, 0.0f, 0.0f, 6000, Motor_Yaw_A.Get_Output_Max(), 50, 10);
    Motor_Yaw_A.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Yaw_A.Get_Output_Max(), Motor_Yaw_A.Get_Output_Max());
    Motor_Yaw_A.Init(&hfdcan2, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE, 2048);

    Motor_Yaw_B.PID_Angle.Init(20.f, 2.0f, 0.0f, 0.0f, Motor_Yaw_B.Get_Output_Max(), Motor_Yaw_B.Get_Output_Max());
    Motor_Yaw_B.PID_Omega.Init(40.0f, 5.0f, 0.0f, 0.0f, 6000, Motor_Yaw_B.Get_Output_Max(), 50, 10);
    Motor_Yaw_B.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Yaw_B.Get_Output_Max(), Motor_Yaw_B.Get_Output_Max());
    Motor_Yaw_B.Init(&hfdcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE, 2048);

    Motor_Main_Yaw.PID_Angle.Init(0.18f, 0.0f, 0.0f, 0.0f, 3, 15);
    Motor_Main_Yaw.PID_Omega.Init(1000.0f, 5.0f, 0.0f, 0.0f, 400.0f, 2048.0f);
    Motor_Main_Yaw.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Main_Yaw.Get_Output_Max(), Motor_Main_Yaw.Get_Output_Max());
    Motor_Main_Yaw.Init(&hfdcan3, LK_Motor_ID_0x141, LK_Motor_Control_Method_ANGLE, 2048);
    
    // pitch轴电机
    Motor_Pitch_A.PID_Angle.Init(25.f, 2.0f, 0.0f, 0.0f, Motor_Pitch_B.Get_Output_Max(), Motor_Pitch_B.Get_Output_Max());
    Motor_Pitch_A.PID_Omega.Init(60.0f, 10.0f, 0.0f, 0.0f, 6000, Motor_Pitch_A.Get_Output_Max());
    Motor_Pitch_A.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Pitch_A.Get_Output_Max(), Motor_Pitch_A.Get_Output_Max());
    Motor_Pitch_A.Init(&hfdcan2, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_ANGLE, 3413);

    Motor_Pitch_B.PID_Angle.Init(30.f, 7.0f, 0.0f, 0.0f, Motor_Pitch_B.Get_Output_Max(), Motor_Pitch_B.Get_Output_Max());
    Motor_Pitch_B.PID_Omega.Init(60.0f, 10.0f, 0.0f, 0.0f, 6000, Motor_Pitch_B.Get_Output_Max());
    Motor_Pitch_B.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Pitch_B.Get_Output_Max(), Motor_Pitch_B.Get_Output_Max());
    Motor_Pitch_B.Init(&hfdcan1, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_ANGLE, 3413);

    Motor_Main_Yaw.Set_Zero_Position(212.308533f);
    Motor_Yaw_A.Set_Zero_Position(294.9169f);
    Motor_Yaw_B.Set_Zero_Position(88.9013f);
    Motor_Pitch_A.Set_Zero_Position(166.157227f);
    Motor_Pitch_B.Set_Zero_Position(262.749023f);
}


/**
 * @brief 输出到电机
 *
 */
float temp_ang_a,temp_ang_b;
float pre_omega_a,pre_omega_b,pre_angle_main = 0.f;
float last_angle_a,last_angle_b;
float pre_pitch_a = 0.f,pre_pitch_b = 0.f;
float pre_yaw_b = 0.f,pre_yaw_a = 0.f;
float last_pitch_a,last_pitch_b,last_yaw_a,last_yaw_b,last_yaw_main;
void Class_Gimbal::Output()
{
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        // 云台失能
        Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_TORQUE);
        Motor_Pitch_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Pitch_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);

        Motor_Yaw_A.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw_A.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Yaw_A.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Yaw_B.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw_B.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Yaw_B.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Main_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Main_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Main_Yaw.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Pitch_A.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch_A.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch_A.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Pitch_B.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch_B.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch_B.PID_Torque.Set_Integral_Error(0.0f);

        Motor_Yaw_A.Set_Target_Torque(0.0f);
        Motor_Yaw_B.Set_Target_Torque(0.0f);
        Motor_Main_Yaw.Set_Target_Torque(0.0f);
        Motor_Pitch_A.Set_Target_Torque(0.0f);
        Motor_Pitch_B.Set_Target_Torque(0.0f);

        Motor_Main_Yaw.Set_Out(0.0f);
        Motor_Yaw_A.Set_Out(0.0f);
        Motor_Yaw_B.Set_Out(0.0f);
        Motor_Pitch_A.Set_Out(0.0f);
        Motor_Pitch_B.Set_Out(0.0f);

        A_Cruise_Flag = 0;
        B_Cruise_Flag = 0;

    }
    else // 非失能模式
    {
        if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        {
            //控制方式
            Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
            Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Pitch_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Pitch_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

            // 限制角度
            Math_Constrain(&Target_Pitch_Angle_A, Min_Pitch_Angle, Max_Pitch_Angle);
            Math_Constrain(&Target_Pitch_Angle_B, Min_Pitch_Angle, Max_Pitch_Angle);
            Math_Constrain(&Target_Yaw_Angle_A, Min_Yaw_Angle_A, Max_Yaw_Angle_A);
            Math_Constrain(&Target_Yaw_Angle_B, Min_Yaw_Angle_B, Max_Yaw_Angle_B);

            // 设置目标角度
            Motor_Yaw_A.Set_Target_Angle(0.0f);
            Motor_Yaw_B.Set_Target_Angle(0.0f);
            Motor_Main_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch_A.Set_Target_Angle(Target_Pitch_Angle_A);
            Motor_Pitch_B.Set_Target_Angle(Target_Pitch_Angle_A);

            // 标志位
            A_Cruise_Flag = 0;
            B_Cruise_Flag = 0;
            pre_angle_main = Boardc_BMI.Get_Angle_Yaw();
        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
        {
            // if(MiniPC->Get_Main_Yaw_Status() == Main_Yaw_Cruise)
            // {
                // A云台控制逻辑
                if (MiniPC->Get_Auto_aim_Status_A() == Auto_aim_Status_DISABLE)
                {
        
                    Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                    Motor_Pitch_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                    if(A_Cruise_Flag == 0)//巡航初启动
                    {
                        Motor_Yaw_A.Set_Target_Omega_Angle(CRUISE_SPEED_YAW);
                        Motor_Pitch_A.Set_Target_Omega_Angle(CRUISE_SPEED_PITCH);
                        A_Cruise_Flag = 1;
                        if(pre_omega_a == CRUISE_SPEED_YAW * 1.50f)
                        {
                            Motor_Yaw_A.Set_Target_Omega_Angle(CRUISE_SPEED_YAW);
                        }
                        else if(pre_omega_a == -CRUISE_SPEED_YAW * 1.50f)
                        {
                            Motor_Yaw_A.Set_Target_Omega_Angle(-CRUISE_SPEED_YAW);
                        }
                    }               

                    if (Get_True_Angle_Yaw_A() < -90.f && Get_True_Angle_Yaw_A() > -170.f)
                        Motor_Yaw_A.Set_Target_Omega_Angle(-CRUISE_SPEED_YAW);
                    else if (Get_True_Angle_Yaw_A() <= -10.f && Get_True_Angle_Yaw_A() > -90.0f)
                        Motor_Yaw_A.Set_Target_Omega_Angle(CRUISE_SPEED_YAW);

                    if (Get_True_Angle_Pitch_A() >= 15.0f)
                        Motor_Pitch_A.Set_Target_Omega_Angle(-CRUISE_SPEED_PITCH);
                    else if (Get_True_Angle_Pitch_A() <= 0.0f)
                        Motor_Pitch_A.Set_Target_Omega_Angle(CRUISE_SPEED_PITCH);

                    // 云台控制方式
                    if(MiniPC->Get_Rx_Pitch_Angle_A() != 0.0f || MiniPC->Get_Rx_Yaw_Angle_A() != 0.0f)//00正常巡航
                    {
                        A_Invert_Flag = 1;//不用了(有bug)，先置为0，启用置1
                        temp_ang_a = MiniPC->Get_Rx_Yaw_Angle_A();
                    }
                    else
                    {
                        A_Invert_Flag = 0;
                    }
                    if(A_Invert_Flag == 1)
                    {
                        Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                        Motor_Pitch_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

                        Motor_Pitch_A.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle_A());//提前预瞄

                        if(temp_ang_a >= last_angle_a || temp_ang_a <= -160.f)//预测位在逆时针方向
                        {                    
                            Motor_Yaw_A.Set_Target_Omega_Angle(CRUISE_SPEED_YAW * 1.50f);
                        }
                        else
                        {
                            Motor_Yaw_A.Set_Target_Omega_Angle(- CRUISE_SPEED_YAW * 1.50f);//预测位在顺时针方向
                        }								
                        A_Cruise_Flag = 0;                  
                    }
                    last_angle_a = Get_True_Angle_Yaw_A();//保留上一帧数据      
                    pre_omega_a = Motor_Yaw_A.Get_Target_Omega_Angle();       

                    pre_yaw_a = Get_True_Angle_Yaw_A();
                    pre_pitch_a = Get_True_Angle_Pitch_A();
                }
                else if (MiniPC->Get_Auto_aim_Status_A() == Auto_aim_Status_ENABLE)
                {
                    Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                    Motor_Pitch_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

                    Motor_Yaw_A.Set_Target_Angle(MiniPC->Get_Rx_Yaw_Angle_A());
                    Motor_Pitch_A.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle_A());

                    if(MiniPC->Get_Rx_Pitch_Angle_A() == 0.0f && MiniPC->Get_Rx_Yaw_Angle_A() == 0.0f)
                    {
                        Motor_Yaw_A.Set_Target_Angle(pre_yaw_a);
                        Motor_Pitch_A.Set_Target_Angle(pre_pitch_a);
                    }

                    A_Cruise_Flag = 0;
                    A_Invert_Flag = 0;
                }
                // B云台控制逻辑
                if (MiniPC->Get_Auto_aim_Status_B() == Auto_aim_Status_DISABLE)
                {
                    // 云台控制方式
                    Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                    Motor_Pitch_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                    if(B_Cruise_Flag == 0)//巡航初启动
                    {
                        Motor_Yaw_B.Set_Target_Omega_Angle(-CRUISE_SPEED_YAW);
                        Motor_Pitch_B.Set_Target_Omega_Angle(CRUISE_SPEED_PITCH);
                        B_Cruise_Flag = 1;
                        if(pre_omega_b == CRUISE_SPEED_YAW * 1.50f)
                        {
                            Motor_Yaw_B.Set_Target_Omega_Angle(CRUISE_SPEED_YAW);
                        }
                        else if(pre_omega_b == -CRUISE_SPEED_YAW * 1.50f)
                        {
                            Motor_Yaw_B.Set_Target_Omega_Angle(-CRUISE_SPEED_YAW);
                        }
                    }

                    if (Get_True_Angle_Yaw_B() > 90.f && Get_True_Angle_Yaw_B() < 172.f)
                    {
                        Motor_Yaw_B.Set_Target_Omega_Angle(CRUISE_SPEED_YAW);
                    }
                    
                    else if (Get_True_Angle_Yaw_B() >= 10.f && Get_True_Angle_Yaw_B() <= 90.0f)
                    {
                        Motor_Yaw_B.Set_Target_Omega_Angle(-CRUISE_SPEED_YAW);
                    }

                    if (Get_True_Angle_Pitch_B() >= 15.0f)
                        Motor_Pitch_B.Set_Target_Omega_Angle(-CRUISE_SPEED_PITCH);
                    else if (Get_True_Angle_Pitch_B() <= 0.0f)
                        Motor_Pitch_B.Set_Target_Omega_Angle(CRUISE_SPEED_PITCH);

                    if(MiniPC->Get_Rx_Pitch_Angle_B() != 0.f || MiniPC->Get_Rx_Yaw_Angle_B() != 0.f)//00巡航
                    {
                        B_Invert_Flag = 1;//不用了(有bug)，先置为0，启用置1
                        temp_ang_b = MiniPC->Get_Rx_Yaw_Angle_B();//预测位
                    }
                    else
                    {
                         B_Invert_Flag = 0;
                    }
                    if(B_Invert_Flag == 1)
                    {
                        Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                        Motor_Pitch_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

                        Motor_Pitch_B.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle_B());//提前预瞄
                                        
                        if(temp_ang_b <= last_angle_b || temp_ang_b > 160.f)//预测位在顺时针方向     
                        {
                            Motor_Yaw_B.Set_Target_Omega_Angle(-CRUISE_SPEED_YAW * 1.50f);
                        }
                        else//预测位在逆时针方向
                        {
                            Motor_Yaw_B.Set_Target_Omega_Angle(CRUISE_SPEED_YAW * 1.50f);
                        }
                        B_Cruise_Flag = 0;
                    }
                    last_angle_b = Get_True_Angle_Yaw_B();
                    pre_omega_b = Motor_Yaw_B.Get_Target_Omega_Angle();

                    pre_yaw_b = Get_True_Angle_Yaw_B();
                    pre_pitch_b = Get_True_Angle_Pitch_B();
                }
                else if (MiniPC->Get_Auto_aim_Status_B() == Auto_aim_Status_ENABLE)//右头自瞄开启
                {
                    Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                    Motor_Pitch_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

                    Motor_Yaw_B.Set_Target_Angle(MiniPC->Get_Rx_Yaw_Angle_B());
                    Motor_Pitch_B.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle_B());

                    if(MiniPC->Get_Rx_Pitch_Angle_B() == 0.f && MiniPC->Get_Rx_Yaw_Angle_B() == 0.f)//
                    {
                        Motor_Yaw_B.Set_Target_Omega_Angle(pre_yaw_b);
                        Motor_Pitch_B.Set_Target_Omega_Angle(pre_pitch_b);
                    }

                    B_Cruise_Flag = 0;
                    B_Invert_Flag = 0;
                }
                //大yaw不动
                // Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
                // Motor_Main_Yaw.Set_Target_Angle(pre_angle_main);
               if(MiniPC->Get_Gimbal_Angular_Velocity_Yaw_Main() != 0.f)
               {
                   Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_OMEGA);
                   Motor_Main_Yaw.Set_Target_Omega_Angle(float(MiniPC->Get_Gimbal_Angular_Velocity_Yaw_Main()/100.f));
                   pre_angle_main = Boardc_BMI.Get_Angle_Yaw();
               }
               else
               {
                   Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
                   Motor_Main_Yaw.Set_Target_Angle(pre_angle_main);
               }
            // }
            // else if(MiniPC->Get_Main_Yaw_Status() == Main_Yaw_Working)
            // {
                    // //左云台
                    // Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                    // Motor_Pitch_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

                    // Motor_Yaw_A.Set_Target_Angle(MiniPC->Get_Rx_Yaw_Angle_A());
                    // Motor_Pitch_A.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle_A());
                    
                    // //右云台
                    // Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                    // Motor_Pitch_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

                    // Motor_Yaw_B.Set_Target_Angle(MiniPC->Get_Rx_Yaw_Angle_B());
                    // Motor_Pitch_B.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle_B());

                    // //大yaw
                    // Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
                    // Motor_Main_Yaw.Set_Target_Angle(Boardc_BMI.Get_Angle_Yaw() + MiniPC->Get_Rx_Angle_Yaw_Main());
            // }
            last_yaw_a = Get_True_Angle_Yaw_A();
            last_yaw_b = Get_True_Angle_Yaw_B();
            last_pitch_a = Get_True_Angle_Pitch_A();
            last_pitch_b = Get_True_Angle_Pitch_B();
            last_yaw_main = Get_Target_Yaw_Angle();
        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() == MiniPC_Status_DISABLE))
        {

            Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
            Motor_Pitch_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Pitch_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

            Motor_Main_Yaw.Set_Target_Angle(last_yaw_main);
            Motor_Pitch_A.Set_Target_Angle(last_pitch_a);
            Motor_Pitch_B.Set_Target_Angle(last_pitch_b);
            Motor_Yaw_A.Set_Target_Angle(last_yaw_a);
            Motor_Yaw_B.Set_Target_Angle(last_yaw_b);
        }
    }
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    //控制模式
    Output();

    //坐标系转换
    Yaw_Angle_Transform_A();
    Yaw_Angle_Transform_B();
    Pitch_Angle_Transform_A();
    Pitch_Angle_Transform_B();
    Yaw_Angle_Transform_Main();

    MiniPC_Update(); // 上位机数据更新    
    if(Get_Gimbal_Control_Type() != Gimbal_Control_Type_DISABLE)
    {
        Limit_Update(); //电控PID限位
    }
    //PID输出
    Motor_Yaw_A.TIM_PID_PeriodElapsedCallback();
    Motor_Yaw_B.TIM_PID_PeriodElapsedCallback();
    Motor_Main_Yaw.TIM_Process_PeriodElapsedCallback();
    Motor_Pitch_A.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch_B.TIM_PID_PeriodElapsedCallback();
    //输出方向更新
    PID_Update();
}

void Class_Gimbal::Yaw_Angle_Transform_Main()
{
    // LK电机正方向未设置 不一定有效 6020有效
    float temp_yaw;
    if (Motor_Main_Yaw.Get_Now_Angle() > Motor_Main_Yaw.Get_Zero_Position())
    {
        temp_yaw = (-(Motor_Main_Yaw.Get_Now_Angle() - Motor_Main_Yaw.Get_Zero_Position())); // 电机数据转现实坐标系
        if (temp_yaw < -180.0f)
            temp_yaw += 360.0f;
    }
    else if (Motor_Main_Yaw.Get_Now_Angle() < Motor_Main_Yaw.Get_Zero_Position())
    {
        temp_yaw = (Motor_Main_Yaw.Get_Zero_Position() - Motor_Main_Yaw.Get_Now_Angle());
        if (temp_yaw > 180.0f)
            temp_yaw -= 360.0f;
    }
    Set_True_Angle_Yaw_Main(-temp_yaw);
    Control_Update_Main();
}

void Class_Gimbal::Yaw_Angle_Transform_A()
{
    float temp_yaw;
    if (Motor_Yaw_A.Get_Now_Angle() > Motor_Yaw_A.Get_Zero_Position())
    {
        temp_yaw = (-(Motor_Yaw_A.Get_Now_Angle() - Motor_Yaw_A.Get_Zero_Position())); // 电机数据转现实坐标系
        if (temp_yaw < -180.0f)
            temp_yaw += 360.0f;
    }
    else if (Motor_Yaw_A.Get_Now_Angle() < Motor_Yaw_A.Get_Zero_Position())
    {
        temp_yaw = (Motor_Yaw_A.Get_Zero_Position() - Motor_Yaw_A.Get_Now_Angle());
        if (temp_yaw > 180.0f)
            temp_yaw -= 360.0f;
    }
    Set_True_Angle_Yaw_A(-temp_yaw);

    temp_yaw = IMU_Data_A.Yaw - (IMU_Data_A.Yaw - Get_True_Angle_Yaw_A());
    if(temp_yaw > 180.f)
        temp_yaw -= 360.f;
    if(temp_yaw < -180.f)
        temp_yaw += 360.f;
    Set_True_Angle_Yaw_A(temp_yaw);

    // 边缘解算
    int invert_flag_a = 0;
    float temp_error = 0, temp_min = 0, pre_angle = 0, pre_omega = 0;
    //
    temp_error = Get_Target_Yaw_Angle_A() - Get_True_Angle_Yaw_A() - invert_flag_a * 180;
    while (temp_error > 360.0f)
        temp_error -= 360.0f;
    while (temp_error < 0.0f)
        temp_error += 360.0f;
    if (fabs(temp_error) < (360.0f - fabs(temp_error)))
        temp_min = fabs(temp_error);
    else
        temp_min = 360.0f - fabs(temp_error);
    if (temp_min > 180.0f)
    {
        invert_flag_a = !invert_flag_a;
        // 重新计算误差
        temp_error = Get_Target_Yaw_Angle_A() - Get_True_Angle_Yaw_A() - invert_flag_a * 180.0f;
    }

    if (temp_error > 180.0f)
        temp_error -= 360.0f;
    else if (temp_error < -180.0f)
        temp_error += 360.0f;

    Set_Target_Yaw_Angle_A(Get_True_Angle_Yaw_A() + temp_error);

    Motor_Yaw_A.Set_Transform_Angle(Get_True_Angle_Yaw_A());
    Motor_Yaw_A.Set_Transform_Omega(IMU_Data_A.Omega_Z);
}

void Class_Gimbal::Yaw_Angle_Transform_B()
{
    float temp_yaw;
    // 如果电机Yaw_B的当前角度大于零位置
    if (Motor_Yaw_B.Get_Now_Angle() > Motor_Yaw_B.Get_Zero_Position())//电机零位校准
    {
        temp_yaw = (-(Motor_Yaw_B.Get_Now_Angle() - Motor_Yaw_B.Get_Zero_Position())); // 电机数据转现实坐标系
        if (temp_yaw < -180.0f)
            temp_yaw += 360.0f;
    }
    else if (Motor_Yaw_B.Get_Now_Angle() < Motor_Yaw_B.Get_Zero_Position())
    {
        temp_yaw = (Motor_Yaw_B.Get_Zero_Position() - Motor_Yaw_B.Get_Now_Angle());
        if (temp_yaw > 180.0f)
            temp_yaw -= 360.0f;
    }
    Set_True_Angle_Yaw_B(-temp_yaw);
    temp_yaw = IMU_Data_B.Yaw - (IMU_Data_B.Yaw - Get_True_Angle_Yaw_B());
    if(temp_yaw > 180.f)
        temp_yaw -= 360.f;
    if(temp_yaw < -180.f)
        temp_yaw += 360.f;

    Set_True_Angle_Yaw_B(temp_yaw);

    Motor_Yaw_B.Set_Transform_Angle(Get_True_Angle_Yaw_B());
    Motor_Yaw_B.Set_Transform_Omega(IMU_Data_B.Omega_Z);

    // 边缘解算
    int invert_flag_b = 0;
    float temp_error = 0, temp_min = 0, pre_angle = 0, pre_omega = 0;

    temp_error = Get_Target_Yaw_Angle_B() - Get_True_Angle_Yaw_B() - invert_flag_b * 180;
    while (temp_error > 360.0f)
        temp_error -= 360.0f;
    while (temp_error < 0.0f)
        temp_error += 360.0f;
    if (fabs(temp_error) < (360.0f - fabs(temp_error)))
        temp_min = fabs(temp_error);
    else
        temp_min = 360.0f - fabs(temp_error);
    if (temp_min > 180.0f)
    {
        invert_flag_b = !invert_flag_b;
        // 重新计算误差
        temp_error = Get_Target_Yaw_Angle_B() - Get_True_Angle_Yaw_B() - invert_flag_b * 180.0f;
    }

    if (temp_error > 180.0f)
        temp_error -= 360.0f;
    else if (temp_error < -180.0f)
        temp_error += 360.0f;

    Set_Target_Yaw_Angle_B(Get_True_Angle_Yaw_B() + temp_error);
}

void Class_Gimbal::Pitch_Angle_Transform_A()
{
    float temp_pitch;
    // 如果电机Pitch_A的当前角度大于零位置
    if (Motor_Pitch_A.Get_Now_Angle() > Motor_Pitch_A.Get_Zero_Position())
    {
        temp_pitch = (-(Motor_Pitch_A.Get_Now_Angle() - Motor_Pitch_A.Get_Zero_Position())); // 电机数据转现实坐标系
        if (temp_pitch < -180.0f)
            temp_pitch += 360.0f;
    }
    else if (Motor_Pitch_A.Get_Now_Angle() < Motor_Pitch_A.Get_Zero_Position())
    {
        temp_pitch = (Motor_Pitch_A.Get_Zero_Position() - Motor_Pitch_A.Get_Now_Angle());
        if (temp_pitch > 180.0f)
            temp_pitch -= 360.0f;
    }
    Set_True_Angle_Pitch_A(IMU_Data_A.Pitch);

    Motor_Pitch_A.Set_Transform_Angle(Get_True_Angle_Pitch_A());
    Motor_Pitch_A.Set_Transform_Omega(IMU_Data_A.Omega_Y);
    Motor_Pitch_A.Set_Transform_Torque(Motor_Pitch_A.Get_Now_Torque());
}

void Class_Gimbal::Pitch_Angle_Transform_B()
{
    float temp_pitch;
    if (Motor_Pitch_B.Get_Now_Angle() > Motor_Pitch_B.Get_Zero_Position())
    {
        temp_pitch = (-(Motor_Pitch_B.Get_Now_Angle() - Motor_Pitch_B.Get_Zero_Position())); // 电机数据转现实坐标系
        if (temp_pitch < -180.0f)
            temp_pitch += 360.0f;
    }
    else if (Motor_Pitch_B.Get_Now_Angle() < Motor_Pitch_B.Get_Zero_Position())
    {
        temp_pitch = (Motor_Pitch_B.Get_Zero_Position() - Motor_Pitch_B.Get_Now_Angle());
        if (temp_pitch > 180.0f)
            temp_pitch -= 360.0f;
    }
    Set_True_Angle_Pitch_B(IMU_Data_B.Pitch);
    Motor_Pitch_B.Set_Transform_Angle(Get_True_Angle_Pitch_B());
    Motor_Pitch_B.Set_Transform_Omega(IMU_Data_B.Omega_Y);
    Motor_Pitch_B.Set_Transform_Torque(-Motor_Pitch_B.Get_Now_Torque());
}
int invert_flag_main = 0;
void Class_Gimbal::Control_Update_Main()
{
    
    float temp_error = 0, temp_min = 0, pre_angle = 0, pre_omega = 0;

    temp_error = Get_Target_Yaw_Angle() - Boardc_BMI.Get_Angle_Yaw() - invert_flag_main * 180;
    while (temp_error > 360.0f)
        temp_error -= 360.0f;
    while (temp_error < 0.0f)
        temp_error += 360.0f;
    if (fabs(temp_error) < (360.0f - fabs(temp_error)))
        temp_min = fabs(temp_error);
    else
        temp_min = 360.0f - fabs(temp_error);
    if (temp_min > 150.0f)
    {
        invert_flag_main = !invert_flag_main;
        // 重新计算误差
        temp_error = Get_Target_Yaw_Angle() - Boardc_BMI.Get_Angle_Yaw() - invert_flag_main * 180.0f;
    }

    if (temp_error > 180.0f)
        temp_error -= 360.0f;
    else if (temp_error < -180.0f)
        temp_error += 360.0f;

    Set_Target_Yaw_Angle(Boardc_BMI.Get_Angle_Yaw() + temp_error);
    Motor_Main_Yaw.Set_Transform_Angle(Boardc_BMI.Get_Angle_Yaw());
    Motor_Main_Yaw.Set_Transform_Omega(Boardc_BMI.Get_Gyro_Yaw());

    pre_angle = Boardc_BMI.Get_Angle_Yaw();
    pre_omega = Boardc_BMI.Get_Gyro_Yaw();
}

void Class_Gimbal::Yaw_Angle_Limit(Enum_Motor_Yaw_Type Motor_Yaw_Type)//角度限制（暂时停用）
{
    volatile int Motor_Type = Motor_Yaw_Type;
    switch (Motor_Type)
    {
    case (Yaw_A):
    {
        if (MiniPC->Get_Rx_Yaw_Angle_A() < -20.f && MiniPC->Get_Rx_Yaw_Angle_A() > -160.f)
            MiniPC->Set_Auto_Limit_Status_A(Auto_Limit_Status_DISABLE);
        else
            MiniPC->Set_Auto_Limit_Status_A(Auto_Limit_Status_ENABLE);
        break;
    }
    case (Yaw_B):
    {
        if (MiniPC->Get_Rx_Yaw_Angle_B() > 20.f && MiniPC->Get_Rx_Yaw_Angle_B() < 160.f)
            MiniPC->Set_Auto_Limit_Status_B(Auto_Limit_Status_DISABLE);
        else
            MiniPC->Set_Auto_Limit_Status_B(Auto_Limit_Status_ENABLE);
        break;
    }
    }
}

void Class_Gimbal::MiniPC_Update()//上位机数据更新
{
    MiniPC->Set_Gimbal_Now_Pitch_Angle_A(Get_True_Angle_Pitch_A());
    MiniPC->Set_Gimbal_Now_Pitch_Angle_B(Get_True_Angle_Pitch_B());
    MiniPC->Set_Gimbal_Now_Yaw_Angle(Get_True_Angle_Yaw_Main());
    MiniPC->Set_Gimbal_Now_Yaw_Angle_A(Get_True_Angle_Yaw_A());
    MiniPC->Set_Gimbal_Now_Yaw_Angle_B(Get_True_Angle_Yaw_B());
}
float Gravity_Compensate = 0.f;
void Class_Gimbal::PID_Update()
{
    Motor_Pitch_A.Set_Out(-Motor_Pitch_A.Get_Out() - Gravity_Compensate);//PID输出值方向校准（硬件层面问题）

}
int A_limit_flag = 0,B_limit_flag = 0;
float temp_pre_omega_a, temp_pre_omega_b;
void Class_Gimbal::Limit_Update()
{
    if(Get_True_Angle_Yaw_A() > -90.f && Get_True_Angle_Yaw_A() < -25.f)
    {
        Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Yaw_A.Set_Target_Omega_Angle(120.f);
        A_limit_flag = 1;
        temp_pre_omega_a = Motor_Yaw_A.Get_Target_Omega_Angle();
    }
    else if(Get_True_Angle_Yaw_A() > -155.f && Get_True_Angle_Yaw_A() < -90.f)
    {
        Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Yaw_A.Set_Target_Omega_Angle(-120.f);
        A_limit_flag = 1;
        temp_pre_omega_a = Motor_Yaw_A.Get_Target_Omega_Angle();
    }
    if(A_limit_flag == 1)
    {
        if(Get_True_Angle_Yaw_A() > -5.f && Get_True_Angle_Yaw_A() < 5.f)
        {
            Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Yaw_A.Set_Target_Angle(0.f);
            A_limit_flag = 0;
        }
        else
        {
            Motor_Yaw_A.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Yaw_A.Set_Target_Omega_Angle(temp_pre_omega_a);
        }
    }

    if(Get_True_Angle_Yaw_B() < 90.f && Get_True_Angle_Yaw_B() > 25.f)
    {
        Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Yaw_B.Set_Target_Omega_Angle(-120.f);
        B_limit_flag = 1;
        temp_pre_omega_b = Motor_Yaw_B.Get_Target_Omega_Angle();
    }
    else if(Get_True_Angle_Yaw_B() < 155.f && Get_True_Angle_Yaw_B() > 90.f)
    {
        Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Yaw_B.Set_Target_Omega_Angle(120.f);
        B_limit_flag = 1;
        temp_pre_omega_b = Motor_Yaw_B.Get_Target_Omega_Angle();
    }
    if(B_limit_flag == 1)
    {
        if(Get_True_Angle_Yaw_B() > -5.f && Get_True_Angle_Yaw_B() < 5.f)
        {
            Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Yaw_B.Set_Target_Angle(0.f);
            B_limit_flag = 0;
        }
        else
        {
            Motor_Yaw_B.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Yaw_B.Set_Target_Omega_Angle(temp_pre_omega_b);
        }
    }
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
