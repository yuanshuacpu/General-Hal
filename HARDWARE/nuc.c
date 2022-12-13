#include "nuc.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

// NUC通信数组
uint8_t NUC_rx_buff[2][NUC_RX_BUFF_SIZE];

// dma底层配置函数，主要负责接收地址的设置
void NUC_Addr_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	// 使能DMA串口接收
	SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

	// enalbe idle interrupt
	// 使能空闲中断
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

	// 失效DMA
	__HAL_DMA_DISABLE(&hdma_usart1_rx);
	while (hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart1_rx);
	}

	hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART6->DR);
	// 内存缓冲区1
	hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
	// 内存缓冲区2
	hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
	// 数据长度
	hdma_usart1_rx.Instance->NDTR = dma_buf_num;
	// 使能双缓冲区
	SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

	// 使能DMA
	__HAL_DMA_ENABLE(&hdma_usart1_rx);
}

static u8 bit32TObit8(int index_need, int bit32)
{
	union
	{
		int f;
		u8 byte[4];
	} u32val;
	u32val.f = bit32;
	return u32val.byte[index_need];
}
static char shortTou8(char bit, short data)
{
	union
	{
		short i;
		char byte[2];
	} u16val;
	u16val.i = data;
	return u16val.byte[bit];
}

/**
 *@Function:		NUC_Init()
 *@Description:	NUC初始化
 *@Param:       形参
 *@Return:	  	返回值
 */
void NUC_Init()
{

	NUC_Addr_Init(NUC_rx_buff[0], NUC_rx_buff[1], NUC_RX_BUFF_SIZE);

	// 使能发送DMA
	__HAL_DMA_SET_COUNTER(huart1.hdmatx, NUC_TX_BUFF_SIZE);
	__HAL_DMA_ENABLE(&hdma_usart1_tx);
	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);
}

int Pitch_Angle_NUC = 0;
int Yaw_Angle_NUC = 0;

float Yaw_Speed_NUC, Pitch_Speed_NUC;
float Chassis_Speed_Real[4];
/**
 *@Function:		NUC_Send_Data()
 *@Description:	NUC数据发送
 *@Param:       形参
 *@Return:	  	返回值
 */
void NUC_Send_Data()
{
	// 角度计算
	Yaw_Angle_NUC = (float)(CAN_Gimbal[0].Current_MechAngle - Gimbal_Init_Processing_Data.Origin_Init_Yaw_Angle) / 8192.0f * 180.0f * 10;
	// Pitch轴的角度就是真实的机械角处理之后发送过去.8192对应180度，单位：度*10
	Pitch_Angle_NUC = (float)(CAN_Gimbal[1].Current_MechAngle - Gimbal_Init_Processing_Data.Origin_Init_Pitch_Angle) / 8192.0f * 180.0f * 10;

	// 单位：度/s*10
	Yaw_Speed_NUC = ((float)CAN_Gimbal[0].Current_Speed * 360.0f) / (60.0f) * 10.0f;
	// 单位：度/s*10
	Pitch_Speed_NUC = ((float)CAN_Gimbal[1].Current_Speed * 360.0f) / (60.0f) * 10.0f;

	Chassis_Speed_Real[0] = (235.62f / 19.0f / 60.0f * (float)CAN_Chassis[0].Current_Speed); // 单位mm/s

	NUC_tx_buff[0] = 0x7a;
	NUC_tx_buff[2] = bit32TObit8(0, (short)Yaw_Angle_NUC);
	NUC_tx_buff[3] = bit32TObit8(1, (short)Yaw_Angle_NUC);
	NUC_tx_buff[4] = bit32TObit8(2, (short)Yaw_Angle_NUC);
	NUC_tx_buff[5] = bit32TObit8(3, (short)Yaw_Angle_NUC);
	NUC_tx_buff[6] = shortTou8(0, (short)Pitch_Speed_NUC); // 以下速度数据视觉都没有使用
	NUC_tx_buff[7] = shortTou8(1, (short)Pitch_Speed_NUC);
	NUC_tx_buff[8] = shortTou8(0, (short)Yaw_Speed_NUC);
	NUC_tx_buff[9] = shortTou8(1, (short)Yaw_Speed_NUC);

	NUC_tx_buff[12] = (0) | (Referee_Inf.Robot_Color << 1) | (0 << 2) | (0 << 3); // 己方颜色从上主板中发送过来的，机器人是红方时候是0，蓝方的时候就是1。

	NUC_tx_buff[14] = shortTou8(0, (short)Chassis_Speed_Real[0]); // 速度，单位mm/s
	NUC_tx_buff[15] = shortTou8(1, (short)Chassis_Speed_Real[0]);

	NUC_tx_buff[18] = shortTou8(0, (short)Pitch_Angle_NUC);
	NUC_tx_buff[19] = shortTou8(1, (short)Pitch_Angle_NUC);

	NUC_tx_buff[22] = 0x7b;

	HAL_UART_Transmit_DMA(&huart1, NUC_tx_buff, NUC_TX_BUFF_SIZE);
}
/**
 *@Function:		NUC_Data_Decode(uint8_t *buff)
 *@Description:	NUC接收数据解析
 *@Param:       形参
 *@Return:	  	返回值
 */
int NUC_Signal = 500;
void NUC_Data_Decode(uint8_t *buff)
{
	if ((buff[0] + buff[3] + buff[5]) % 255 == buff[9])
	{
		NUC_Data.NUC_Shoot_Allow_Flag = (buff[0] == 0x1f);

		NUC_Data.Yaw_Angle = (short)(buff[2] << 8 | buff[1]);
		NUC_Data.Pitch_Angle = (short)(buff[4] << 8 | buff[3]);

		NUC_Data.Armor_Type = buff[6]; // 装甲类型 0未识别到 1小装甲 2大装甲

		// 0为手动，1自动
		NUC_Data.Auto_Shoot = buff[5];

		if (NUC_Data.Armor_Type == 0)
		{
			NUC_Data.Yaw_Angle = 0;
			NUC_Data.Pitch_Angle = 0;
		}

		NUC_Signal = 500;
	}
	else
	{
		NUC_Data.NUC_Shoot_Allow_Flag = 0;
		NUC_Data.Armor_Type = 0;
		NUC_Data.Yaw_Angle = 0;
		NUC_Data.Pitch_Angle = 0;
	}
}

// 串口中断
void USAR_UART1_IDLECallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		volatile uint16_t this_time_rx_len = 0;

		if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			/* Current memory buffer used is Memory 0 */

			// 失效DMA
			__HAL_DMA_DISABLE(&hdma_usart1_rx);

			// get receive data length, length = set_data_length - remain_length
			// 获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = NUC_RX_BUFF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

			// reset set_data_lenght
			// 重新设定数据长度
			__HAL_DMA_SET_COUNTER(huart1.hdmarx, NUC_RX_BUFF_SIZE);

			// set memory buffer 1
			// 设定缓冲区1
			hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

			// enable DMA
			// 使能DMA
			__HAL_DMA_ENABLE(&hdma_usart1_rx);

			if (this_time_rx_len == 14) // 校验接收
			{
				// 处理数据
				NUC_Data_Decode(NUC_rx_buff[0]);
			}
		}
		else
		{
			/* Current memory buffer used is Memory 1 */
			// disable DMA
			// 失效DMA
			__HAL_DMA_DISABLE(&hdma_usart1_rx);

			// get receive data length, length = set_data_length - remain_length
			// 获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
			// reset set_data_lenght
			// 重新设定数据长度
			__HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);

			// set memory buffer 0
			// 设定缓冲区0
			DMA2_Stream2->CR &= ~(DMA_SxCR_CT);

			// enable DMA
			// 使能DMA
			__HAL_DMA_ENABLE(&hdma_usart1_rx);

			if (this_time_rx_len == 14) // 校验接收
			{
				// 处理数据
				NUC_Data_Decode(NUC_rx_buff[1]);
			}
		}
	}
}
