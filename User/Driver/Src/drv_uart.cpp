/**
 * @file drv_uart.cpp
 * @author lez by yssickjgd
 * @brief UART通信初始化与配置流程
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 * 
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_uart.h"
#include "string.h"
#include "dvc_dwt.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_UART_Manage_Object UART1_Manage_Object = {0};
Struct_UART_Manage_Object UART2_Manage_Object = {0};
Struct_UART_Manage_Object UART3_Manage_Object = {0};
Struct_UART_Manage_Object UART4_Manage_Object = {0};
Struct_UART_Manage_Object UART5_Manage_Object = {0};
Struct_UART_Manage_Object UART6_Manage_Object = {0};
Struct_UART_Manage_Object UART7_Manage_Object = {0};
Struct_UART_Manage_Object UART10_Manage_Object = {0};

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

static void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *huart, uint32_t *SrcAddress, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength){


 huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

 huart->RxEventType = HAL_UART_RXEVENT_TC;

 huart->RxXferSize    = DataLength;

 SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);

 __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); 

 HAL_DMAEx_MultiBufferStart(&hdma_uart5_rx,(uint32_t)SrcAddress,(uint32_t)DstAddress,(uint32_t)SecondMemAddress,DataLength);
	

}
uint8_t Power_receive_data[32];
float temp_power = 0;
/**
 * @brief 初始化UART
 *
 * @param huart UART编号
 * @param Callback_Function 处理回调函数
 */
void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back Callback_Function, uint16_t Rx_Buffer_Length)
{
    if (huart->Instance == USART1)
    {
        UART1_Manage_Object.UART_Handler = huart;
        UART1_Manage_Object.Callback_Function = Callback_Function;
        UART1_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
       // HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);

        HAL_UART_Receive_IT(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART5)
    {
        UART5_Manage_Object.UART_Handler = huart;
        UART5_Manage_Object.Callback_Function = Callback_Function;
        UART5_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length*2);
    }
    else if (huart->Instance == UART7)
    {
        UART7_Manage_Object.UART_Handler = huart;
        UART7_Manage_Object.Callback_Function = Callback_Function;
        UART7_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.Rx_Buffer, UART7_Manage_Object.Rx_Buffer_Length);
				//__HAL_DMA_DISABLE_IT(&hdma_usart7_rx, DMA_IT_HT);
    }
    else if (huart->Instance == USART10)
    {
        UART10_Manage_Object.UART_Handler = huart;
        UART10_Manage_Object.Callback_Function = Callback_Function;
        UART10_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        //HAL_UARTEx_ReceiveToIdle_DMA(huart, UART10_Manage_Object.Rx_Buffer, UART10_Manage_Object.Rx_Buffer_Length);
				//__HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, Power_receive_data, 32);
    }
    
}

/**
 * @brief 发送数据帧
 *
 * @param huart UART编号
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length)
{
    return (HAL_UART_Transmit_DMA(huart, Data, Length));
}

/**
 * @brief UART的TIM定时器中断发送回调函数
 *
 */
void TIM_UART_PeriodElapsedCallback()
{
    // UART1超电通讯
    UART_Send_Data(&huart1, UART1_Manage_Object.Tx_Buffer, 10);
}


/**
 * @brief HAL库UART接收DMA空闲中断
 *
 * @param huart UART编号
 * @param Size 长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{    
    //停止DMA接收 保护处理过程
//    HAL_UART_DMAStop(huart);
    
    //选择回调函数
    if (huart->Instance == USART1)
    {
        UART1_Manage_Object.Rx_Length = Size;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);
        if( UART1_Manage_Object.Rx_Length<=UART1_Manage_Object.Rx_Buffer_Length)
            UART1_Manage_Object.Callback_Function(UART1_Manage_Object.Rx_Buffer, Size);
        else
        memset( UART1_Manage_Object.Rx_Buffer, 0, UART1_Manage_Object.Rx_Buffer_Length);

    }
    else if (huart->Instance == UART5)
    {
        UART5_Manage_Object.Rx_Length = Size;              
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length*2);
        if( UART5_Manage_Object.Rx_Length<=UART5_Manage_Object.Rx_Buffer_Length)
            UART5_Manage_Object.Callback_Function(UART5_Manage_Object.Rx_Buffer, Size);
        else
        memset( UART5_Manage_Object.Rx_Buffer, 0, UART5_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART7)
    {
        UART7_Manage_Object.Rx_Length = Size;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.Rx_Buffer, UART7_Manage_Object.Rx_Buffer_Length*2);
        if( UART7_Manage_Object.Rx_Length<=UART7_Manage_Object.Rx_Buffer_Length)
            UART7_Manage_Object.Callback_Function(UART7_Manage_Object.Rx_Buffer, Size);
        else
        memset( UART7_Manage_Object.Rx_Buffer, 0, UART7_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART10)
    {
        // UART10_Manage_Object.Rx_Length = Size;
        
        // HAL_UARTEx_ReceiveToIdle_DMA(huart, UART10_Manage_Object.Rx_Buffer, UART10_Manage_Object.Rx_Buffer_Length);
		// 		//__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
        // if( UART10_Manage_Object.Rx_Length<=UART10_Manage_Object.Rx_Buffer_Length)
        //     UART10_Manage_Object.Callback_Function(UART10_Manage_Object.Rx_Buffer, Size);
        // else
        // memset( UART10_Manage_Object.Rx_Buffer, 0, UART10_Manage_Object.Rx_Buffer_Length);

        HAL_UARTEx_ReceiveToIdle_DMA(huart, Power_receive_data, 32);
        for(int i=0;i<32;i++)
        {
            if((Power_receive_data[i] == 0xB6 && Power_receive_data[i+7] == 0xB7) || (Power_receive_data[i] == 0xB6 && Power_receive_data[i+4] == 0)){
                memcpy(&temp_power, &Power_receive_data[i+1], sizeof(float));
                break;
            }
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        UART1_Manage_Object.Rx_Length = 8;
        if( UART1_Manage_Object.Rx_Length<=UART1_Manage_Object.Rx_Buffer_Length)
            UART1_Manage_Object.Callback_Function(UART1_Manage_Object.Rx_Buffer, 8);
        else
        memset( UART1_Manage_Object.Rx_Buffer, 0, UART1_Manage_Object.Rx_Buffer_Length);
        HAL_UART_Receive_IT(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);//再开启接收中断
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
