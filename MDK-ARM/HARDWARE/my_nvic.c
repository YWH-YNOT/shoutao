#ifndef my_nvic_H_
#define my_nvic_H_
#include "headfile.h"

//变量定义
uint8_t uart6_data = 0;
uint8_t uart2_data = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{	
	if(huart == &huart6)
	{
		HAL_UART_Receive_IT(&huart6,&uart6_data,1);	
		HAL_UART_Transmit(&huart6, &uart6_data, sizeof(uart6_data), 0xffff);
	}
	if (huart == &huart2) 
	{
    get_JY901S(uart2_data);
    HAL_UART_Receive_IT(&huart2, &uart2_data, 1);
  }
	
}

/* 全局计数器和标志变量 */
uint32_t task1ms_cnt = 0;    // 1ms任务计数器
uint32_t task1s_cnt = 0;     // 1s任务计数器


uint32_t task200ms_cnt = 0;    // 1ms任务计数器
/* 任务执行标志（可被主循环检测） */
volatile bool task1ms_flag = false;

volatile bool task200ms_flag = false;

volatile bool task1s_flag = false;

int t1_flag = 0;
volatile int move_flag = false;
volatile bool direction_lock_flag = true; // 初始为true，表示可以检测方向

volatile bool trans_flag = true; // 初始为true，表示可以检测方向
/* 定时器中断回调函数（1ms触发一次） */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
    if (htim->Instance == TIM6) 
		{
				
        /* 1ms任务（每个中断周期执行） */
        task1ms_flag = true;			
	
        if (++task200ms_cnt >= 200) 
				{
						trans_flag = 1;					
            task200ms_cnt = 0;
            task200ms_flag = true;
					
        }
        /* 1s任务（每1000ms执行一次） */
        if (++task1s_cnt >= 1000) 
				{
						direction_lock_flag = true; // 1秒后重置标志位，允许再次检测方向
            task1s_cnt = 0;
            task1s_flag = true;
					
        }
				
    }
}

void key_read(void)
{
	// 确认一下是否为KEY1按下
  if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == 0)								
	{			
		//KEY1
		
		while(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == 0)
		{
			if(trans_flag == 1 && mode1_flag ==1)																							//定时发送
			{
				current_cmd = CMD_ANJIAN_SHIJUE;
				HAL_UART_Transmit_DMA(&huart6, &current_cmd, sizeof(current_cmd)); 
				HAL_Delay(10);
			}
			else if(trans_flag == 1 && mode2_flag ==1)
			{
				current_cmd = CMD_KAIGUAN_SHIJUE;
				HAL_UART_Transmit_DMA(&huart6, &current_cmd, sizeof(current_cmd)); 
				HAL_Delay(10);
			}
			else if(trans_flag == 1 && mode3_flag ==1)
			{
				current_cmd = CMD_BACKWARD;
				HAL_UART_Transmit_DMA(&huart6, &current_cmd, sizeof(current_cmd)); 
				HAL_Delay(10);
			}
			trans_flag = 0;
		}
		current_cmd = CMD_NONE;
  }
	// 确认一下是否为KEY2按下
  if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == 0)
	{
		//KEY1		
		while(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == 0)
		{
			if(trans_flag == 1 && mode1_flag ==1)																							//定时发送
			{
				current_cmd = CMD_ANJIAN_DONGZUO;
				HAL_UART_Transmit_DMA(&huart6, &current_cmd, sizeof(current_cmd)); 
				HAL_Delay(10);
			}
			else if(trans_flag == 1 && mode2_flag ==1)
			{
				current_cmd = CMD_KAIGUAN_DONGZUO;
				HAL_UART_Transmit_DMA(&huart6, &current_cmd, sizeof(current_cmd)); 
				HAL_Delay(10);
			}
			else if(trans_flag == 1 && mode3_flag ==1)
			{
				current_cmd = CMD_FORWARD;																										//向前
				HAL_UART_Transmit_DMA(&huart6, &current_cmd, sizeof(current_cmd)); 
				HAL_Delay(10);
			}
			trans_flag = 0;
		}		
		current_cmd = CMD_NONE;
  }
	
}




#endif 

