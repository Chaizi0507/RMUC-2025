#include "crt_observe.h"

float vaEstimateKF_F[4] = {1.0f, 0.003f, 
                           0.0f, 1.0f};	   // ״̬ת�ƾ��󣬿�������Ϊ0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // �������Э�����ʼֵ

float vaEstimateKF_Q[4] = {1.0f, 0.0f, 
                           0.0f, 1.0f};    // Q�����ʼֵ

float vaEstimateKF_R[4] = {50.0f, 0.0f, 
                            0.0f,  50.0f}; 	
														
float vaEstimateKF_K[4];
													 
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};

float  RAMP_float(float final, float now, float ramp);
uint32_t OBSERVE_TIME=3;//����������3ms	

void Class_observe::xvEstimateKF_Init(void)
{
    Kalman_Filter_Init(&vaEstimateKF, 2, 0, 2);
	
	memcpy(vaEstimateKF.F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(vaEstimateKF.P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(vaEstimateKF.Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(vaEstimateKF.R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(vaEstimateKF.H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}

void Class_observe::xvEstimateKF_Update( float acc,float vel)
{   	
	 memcpy(vaEstimateKF.Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
   memcpy(vaEstimateKF.R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
	
    //�������˲�������ֵ����
    vaEstimateKF.MeasuredVector[0] =	vel;//�����ٶ�
    vaEstimateKF.MeasuredVector[1] = acc;//�������ٶ�
    		
    //�������˲������º���
    Kalman_Filter_Update(&vaEstimateKF);

    // ��ȡ����ֵ
    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = vaEstimateKF.FilteredValue[i];
    }
}																 
																 


void 	Class_observe::TIM_Calculate_PeriodElapsedCallback(uint32_t OBSERVE_TIME) 
{  
		// wr= Wheel_Motor[1]->motor.Get_Now_Omega()+Boardc_BMI->Get_Gyro_Roll()+Right_Leg->d_alpha;//�ұ�������ת����Դ�ؽ��ٶȣ����ﶨ�����˳ʱ��Ϊ��
		// vrb=wr*0.09f+Right_Leg->L0*Right_Leg->d_theta*arm_cos_f32(Right_Leg->theta)+Right_Leg->d_L0*arm_sin_f32(Right_Leg->theta);//����bϵ���ٶ�
		
		// wl= Wheel_Motor[0]->motor.Get_Now_Omega()+Boardc_BMI->Get_Gyro_Roll()+Left_Leg->d_alpha;//���������ת����Դ�ؽ��ٶȣ����ﶨ�����˳ʱ��Ϊ��
		// vlb=wl*0.09f+Left_Leg->L0*Left_Leg->d_theta*arm_cos_f32(Left_Leg->theta)+Left_Leg->d_L0*arm_sin_f32(Left_Leg->theta);//����bϵ���ٶ�
		
		// aver_v=(vrb+vlb)/2.0f;//ȡƽ��
    	// xvEstimateKF_Update(-Boardc_BMI->Get_Motion_Accel_X_N(),aver_v);
		
		// //ԭ����ת�Ĺ�����v_filter��x_filterӦ�ö���Ϊ0
		// v_filter=vel_acc[0];//�õ��������˲�����ٶ�
		// x_filter=x_filter+v_filter*((float)OBSERVE_TIME/1000.0f);

	//�����ֱ���������ٶȣ������ںϵĻ���������
	v_filter=( Wheel_Motor[1]->motor.Get_Now_Omega()+Wheel_Motor[1]->motor.Get_Now_Omega())*(0.09f)/2.0f;
//	v_filter=0.f;
	x_filter=x_filter+v_filter*((float)OBSERVE_TIME/1000.0f);
	}


float  RAMP_float( float  final, float  now, float  ramp )
{
	  float  buffer = 0;
		
	  buffer = final - now;
	
		if (buffer > 0)
		{
				if (buffer > ramp)
				{  
						now += ramp;
				}   
				else
				{
						now += buffer;
				}
		}
		else
		{
				if (buffer < -ramp)
				{
						now += -ramp;
				}
				else
				{
						now += buffer;
				}
		}
		
		return now;
}