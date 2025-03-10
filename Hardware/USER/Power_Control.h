#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "PID.h"
#include "Measure.h"
#include "math.h"

int Power_on_Self_Test();
void  PID_Control(void);
void Input_undervoltage_protection(void);
void Mode_Judgment(void);
void HRTIM_DISABLE();
void HRTIM_ENABLE();
void Update_PWM(float ratio);
void Recovery_detect();
double map_input_to_output(double input_value) ;
#define MAX_DUTY 1.2    //DUTY=V_CAP/V_VIN
#define MIN_DUTY 0.1
#define MAX_P_DCDC 120  //���
#define MIN_P_DCDC -300 //�ŵ�

#define HRTIMMaster_Period 8000
#define FALASE 0
#define TRUE 1
#define V_SET 24.3f
#define BUCK_RIGHT_DUTY   (0.97 * HRTIMMaster_Period+50)    	  //0.9*5760+50=5234                     Buckģʽ�£����Ź̶�ռ�ձ�90%
#define BUCK_LEFT_MIN_DUTY		(0.10 * HRTIMMaster_Period+50)  //0.1*5760+50=626                        Buckģʽ�£�������Сռ�ձ�10%
#define BUCK_LEFT_MAX_DUTY   (0.97 * HRTIMMaster_Period+50) //0.9*5760+50=5234  			                 Buckģʽ�£��������ռ�ձ�85%

#define	BUCK_BOOST_LEFT_DUTY  (0.97 * HRTIMMaster_Period+50)   			 //0.9*5760+50=5234                 BUCK-BOOSTģʽ�£����Ź̶�ռ�ձ�90%����̫��Ų��˵�
#define BUCK_BOOST_RIGHT_MIN_DUTY  (0.70 * HRTIMMaster_Period+50)    //0.7*5760+50=4082                 BUCK-BOOSTģʽ�£�������Сռ�ձ�50%����̫��䲻�˵�
#define BUCK_BOOST_RIGHT_MAX_DUTY  (0.97 * HRTIMMaster_Period+50)    //0.9*5760+50=5234                 BUCK-BOOSTģʽ�£��������ռ�ձ�90%
#define MIN_REG_VALUE   25                     //HRTIM reg mini value

#define MIN_UVP_VAL    20.0f//20VǷѹ����
#define MIN_UVP_VAL_RE 21.0f//21VǷѹ�����ָ�
#define MAX_UVP_VAL 28.0f//28V��ѹ����
#define VIN_I_MAX 6.0f//10A��������
#define CAP_I_MAX 10.0f

extern float ratio_test;
 extern int PWM_CONTROL;
enum LOOP_MODE
{
    LOOP_MODE_CV = 1,                                       //��ѹģʽ
    LOOP_MODE_CC,                                            //����ģʽ
    DISCHANGE
};

enum CAP_MODE
{
    Normal=1,                                    //BUCKģ̬
		Standby,                                     //���ϻ����
    LISTEN

};


enum ERROR_CODE
{
  NO=0,
  VOLT,
 CURRENT,
  LOSE      
 
};
typedef struct {
  pid_type_def powerin_loop, currout_loop, voltout_loop;  //pid�ṹ��
  float P_set;                                        //�趨����������
	float I_Set;																						//����������
  float dcdc_power;                                       //DC-DC�趨����
  float I_Charge_limited;                                        //DC-DC�趨����
  float dcdc_max_curr;                                    //dcdc������
  float cap_v_max;                                        //��������ѹ
  float cap_max_curr;                                     //����������
  float Vloop_ratio;                                      //��ѹ�����
  float Cloop_ratio;                                      //���������
  float Volt_ratio;                                       //PID�������������ѹ��ֵ (Vout / Vin)
  float Real_ratio_forward;                                         
  float Real_ratio;                                        //ʵ��ADC����ֵ
	enum	CAP_MODE Cap_Mode;																//����״̬
	enum	LOOP_MODE Loop_Mode;															//PID��״̬
  enum ERROR_CODE error_code;                             //������
  float test_ratio;
  float DutyA;
  float DutyB;
	int buck_left_feedforward_duty;																			//BUCK����ǰ��ռ�ձ�
	int buck_boost_right_feedforward_duty;																			//BUCK-BOOST����ǰ��ռ�ձ�	
	int left_feedforward_duty;                              
  int right_feedforward_duty;
	uint8_t BBModeChange;   																//����ģʽ�л���־λ
	uint8_t flag;
	int left_duty;
  int right_duty;
	float I_DCDC_IN_MAX;																				//H���������������
	float I_CAP_IN_MAX;																					//H���Ҳ�����������
	float I_DCDC_OUT_MAX;																				//H��������������
	float I_CAP_OUT_MAX;																				//H���Ҳ����������
  uint8_t power_limit;                                        //��ع������ƿ���  
  float recovery_decrease_ratio;
} control_struct_t;

extern control_struct_t control;
#define M_MIN(a, b) ((a) > (b) ? (a) : (b)) 
#define M_MAX(a, b) ((a) < (b) ? (a) : (b))
#ifdef __cplusplus
}
#endif

#endif
