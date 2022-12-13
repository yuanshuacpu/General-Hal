#include "referee.h"


extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

//串口6接收数据缓冲
uint8_t Usart_Referee_Rx_Buf[2][USART_RX_BUF_LENGHT];
uint8_t Usart_Referee_Tx_Buf[MAX_SIZE];



//dma底层配置函数，主要负责接收地址的设置
void Referee_Addr_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
		//使能DMA串口接收
  	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
	
	  //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
		
	  //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //数据长度
    hdma_usart6_rx.Instance->NDTR = dma_buf_num;
    //使能双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
	
}

void Referee_Init(void)
{
			Referee_Addr_Init(Usart_Referee_Rx_Buf[0], Usart_Referee_Rx_Buf[1], 1024);
			
			//使能发送DMA
			__HAL_DMA_SET_COUNTER(huart6.hdmatx, MAX_SIZE);
			__HAL_DMA_ENABLE(&hdma_usart6_tx);
			__HAL_UART_CLEAR_FLAG(&huart6,UART_FLAG_TC);
}




//串口中断
void USAR_UART6_IDLECallback(UART_HandleTypeDef *huart)
{
		if(huart == &huart6)
		{		
				volatile uint16_t this_time_rx_len = 0;
				volatile uint16_t index = 0; //当前的数据序号
				uint16_t data_length = 0; //帧数据长度
				
        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
 
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
						
						//reset set_data_lenght
            //重新设定数据长度
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
						
						for(;index<this_time_rx_len;)
						{
							if(Usart_Referee_Rx_Buf[0][index] != 0xA5)
							{;}
							else
							{
									if(Verify_CRC8_Check_Sum(&Usart_Referee_Rx_Buf[0][index],5) == 1)
									{
										data_length = Usart_Referee_Rx_Buf[0][index+2]<<8 | Usart_Referee_Rx_Buf[0][index+1] + 9;
										if(index+data_length > this_time_rx_len)
										{ break; }
										if(Verify_CRC16_Check_Sum(&Usart_Referee_Rx_Buf[0][index],data_length))	//CRC16校验（CRC8不必再校验）
										{
											//裁判系统数据解析
											RefereeInfo_Decode(&Usart_Referee_Rx_Buf[0][index]);	
										}			
						      
									index += data_length-1;//因为for后面有index++
									}
							}
							index++;
						}

        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
						//reset set_data_lenght
            //重新设定数据长度
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);

            //set memory buffer 0
            //设定缓冲区0
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);
						
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

						for(;index<this_time_rx_len;)
						{
							if(Usart_Referee_Rx_Buf[1][index] != 0xA5)
							{;}
							else
							{
									if(Verify_CRC8_Check_Sum(&Usart_Referee_Rx_Buf[1][index],5) == 1)
									{
										data_length = Usart_Referee_Rx_Buf[1][index+2]<<8 | Usart_Referee_Rx_Buf[1][index+1] + 9;
										if(index+data_length > this_time_rx_len)
										{ break; }
										if(Verify_CRC16_Check_Sum(&Usart_Referee_Rx_Buf[1][index],data_length))	//CRC16校验（CRC8不必再校验）
										{
											//裁判系统数据解析
											RefereeInfo_Decode(&Usart_Referee_Rx_Buf[1][index]);	
										}			
						      
									index += data_length-1;//因为for后面有index++
									}
							}
							index++;
						}

        }
    }
}



/**
  * @brief     pack data to bottom device
  * @param[in] sof：framehearder
  * @param[in] cmd_id:  command id of data
  * @param[in] *p_data: pointer to the data to be sent
  * @param[in] len:     the data length
  */
void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
	static uint8_t seq = 0;
	unsigned char i=i;
	uint8_t tx_buff[MAX_SIZE];
	
  uint16_t frame_length = frameheader_len + cmd_len + len + crc_len;   //数据帧长度	

	memset(tx_buff,0,frame_length);  //存储数据的数组清零
	
	/*****帧头打包*****/
	tx_buff[0] = sof;//数据帧起始字节
	memcpy(&tx_buff[1],(uint8_t*)&len, sizeof(len));//数据帧中data的长度
	tx_buff[3] = seq;//包序号
	Append_CRC8_Check_Sum(tx_buff,frameheader_len);  //帧头校验CRC8

	/*****命令码打包*****/
	memcpy(&tx_buff[frameheader_len],(uint8_t*)&cmd_id, cmd_len);
	
	/*****数据打包*****/
	memcpy(&tx_buff[frameheader_len+cmd_len], p_data, len);
	Append_CRC16_Check_Sum(tx_buff,frame_length);  //一帧数据校验CRC16

	if (seq == 0xff) seq=0;
  else seq++;

	
	/*****数据上传*****/


	  
		while (__HAL_UART_GET_FLAG(&huart6,UART_IT_TC) == RESET) ; //等待之前的字符发送完成
		
		HAL_UART_Transmit_DMA(&huart6,tx_buff,frame_length);
		
	
//	memcpy(Usart_Referee_Tx_Buf,tx_buff,MAX_SIZE);
//	DMA_Cmd(DMA_Stream_Referee_TX, ENABLE);
//	while(DMA_GetFlagStatus(DMA_Stream_Referee_TX,DMA_FLAG_TCIF6)==RESET);
//	DMA_ClearFlag(DMA_Stream_Referee_TX,DMA_FLAG_TCIF6);
//	DMA_SetCurrDataCounter(DMA_Stream_Referee_TX, MAX_SIZE);	
}
