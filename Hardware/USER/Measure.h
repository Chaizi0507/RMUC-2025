#ifndef __MEASURE_H
#define __MEASURE_H
#include "Power_Control.h"

#ifdef __cplusplus
extern "C"
{
#endif
#define V_Magnification	16						//�����ѹ����
#define I_Magnification 20                       //�������оƬ�Ŵ���
#define VIN_Sense_resistor 0.004			   //���������ֵ����λΪŷ��
#define MOTOR_Sense_resistor 0.004
#define CAP_Sense_resistor 0.004
#define Measure_count 12
void ADC_init(void);
void ADC_Measure(void);
void ADC_VREF(void);
void ratio_init(void);
extern uint16_t LED_Cnt;

typedef struct {
    int measured_value;
    float measured_array[Measure_count];
    int bias;
} STRUCT_REPLACE;

typedef struct
{	
	STRUCT_REPLACE replace;
	float median_value;
	double real_valu1;      //��ʵֵ1 �ⲿ�Ǳ���
	double get_volt1;       //����ֵ1  ��Ƭ���˲�����
	double real_valu2;      //��ʵֵ2
	double get_volt2;       //����ֵ2
	
	double real_valu3;      //��ʵֵ1 �ⲿ�Ǳ���
	double get_volt3;       //����ֵ1  ��Ƭ���˲�����
	double real_valu4;      //��ʵֵ2
	double get_volt4;       //����ֵ2
	
	float offset;          //Ư��ϵ��
	float ratio;           //����ϵ��
	
	int16_t measured_value; //ԭʼADCֵ
	int16_t bias;							//ԭʼƫ����
	float filter_out;						//�������鴦����ֵ
	float Solved_value;						//��������ĵ�ѹ�͵���
	int16_t measured_array[20];
	float Solved_filter_out;
	
} ELEC_INFO_STRUCT;

extern ELEC_INFO_STRUCT ADC_I_IN;
extern ELEC_INFO_STRUCT ADC_VIN;
extern ELEC_INFO_STRUCT ADC_I_MOTOR;
extern ELEC_INFO_STRUCT ADC_I_CAP;
extern ELEC_INFO_STRUCT ADC_V_CAP;
extern uint16_t AD_Buf_1[3];
extern uint16_t AD_Buf_2[3];


float ringbuf_cal(int16_t *adc_ch , int16_t wide);
void ratio_init(void);
void ADC_Linear_calibration_init(ELEC_INFO_STRUCT *p, double real1, double get1, 
																											double real2, double get2);
float ADCvalue_to_ELEC(ELEC_INFO_STRUCT *p,uint16_t cn_kalman);//ADCֵ��ɵ�ѹ����
void ADC_Measure(void);
void ADC_init(void);
void sumBuffer();
extern uint8_t board_number;



typedef struct 
	{
		
		float P_In;	
		float P_Motor;
		float P_Cap;
		float V_DCDC;
		float I_DCDC;
		float P_DCDC;
		float efficiency;
		float surplus_energy;
		uint16_t V_REFINT_CAL;
		float p_beyondl;			//������ֵ
	} measure_struct_t;
extern measure_struct_t measure; 

typedef struct 
	{
	
		uint16_t Internally_read_values;	
		uint32_t Externally_read_values_total;
		float Average_value;
		uint16_t Externally_read_values;
		
	} ADC_V_REF_struct_t;
extern ADC_V_REF_struct_t ADC_V_REF; 


typedef struct
{
	float  Input;
	float  Output;
	float  Fc;
	float  Fs;
	float  Ka;
	float  Kb;
}LOWPASS_FILTER_STRUCT;//һ�׵�ͨ�˲�

#define PI2   6.28318530F // 2��
void low_filter_init(LOWPASS_FILTER_STRUCT *p, float sample_f, float cutoff_f);
void low_filter_calc(LOWPASS_FILTER_STRUCT *p);	//һ�׵�ͨ�˲�����
void replace_extremes_with_average(int16_t *array, int size); 
#ifdef __cplusplus
}
#endif

#endif
