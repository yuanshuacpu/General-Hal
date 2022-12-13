#include "referee.h"


extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

//����6�������ݻ���
uint8_t Usart_Referee_Rx_Buf[2][USART_RX_BUF_LENGHT];
uint8_t Usart_Referee_Tx_Buf[MAX_SIZE];



//dma�ײ����ú�������Ҫ������յ�ַ������
void Referee_Addr_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
		//ʹ��DMA���ڽ���
  	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
	
	  //enalbe idle interrupt
    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
		
	  //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //�ڴ滺����1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //�ڴ滺����2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //���ݳ���
    hdma_usart6_rx.Instance->NDTR = dma_buf_num;
    //ʹ��˫������
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
	
}

void Referee_Init(void)
{
			Referee_Addr_Init(Usart_Referee_Rx_Buf[0], Usart_Referee_Rx_Buf[1], 1024);
			
			//ʹ�ܷ���DMA
			__HAL_DMA_SET_COUNTER(huart6.hdmatx, MAX_SIZE);
			__HAL_DMA_ENABLE(&hdma_usart6_tx);
			__HAL_UART_CLEAR_FLAG(&huart6,UART_FLAG_TC);
}




//�����ж�
void USAR_UART6_IDLECallback(UART_HandleTypeDef *huart)
{
		if(huart == &huart6)
		{		
				volatile uint16_t this_time_rx_len = 0;
				volatile uint16_t index = 0; //��ǰ���������
				uint16_t data_length = 0; //֡���ݳ���
				
        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
 
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
						
						//reset set_data_lenght
            //�����趨���ݳ���
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);

            //set memory buffer 1
            //�趨������1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //ʹ��DMA
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
										if(Verify_CRC16_Check_Sum(&Usart_Referee_Rx_Buf[0][index],data_length))	//CRC16У�飨CRC8������У�飩
										{
											//����ϵͳ���ݽ���
											RefereeInfo_Decode(&Usart_Referee_Rx_Buf[0][index]);	
										}			
						      
									index += data_length-1;//��Ϊfor������index++
									}
							}
							index++;
						}

        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
						//reset set_data_lenght
            //�����趨���ݳ���
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);

            //set memory buffer 0
            //�趨������0
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);
						
            //enable DMA
            //ʹ��DMA
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
										if(Verify_CRC16_Check_Sum(&Usart_Referee_Rx_Buf[1][index],data_length))	//CRC16У�飨CRC8������У�飩
										{
											//����ϵͳ���ݽ���
											RefereeInfo_Decode(&Usart_Referee_Rx_Buf[1][index]);	
										}			
						      
									index += data_length-1;//��Ϊfor������index++
									}
							}
							index++;
						}

        }
    }
}



/**
  * @brief     pack data to bottom device
  * @param[in] sof��framehearder
  * @param[in] cmd_id:  command id of data
  * @param[in] *p_data: pointer to the data to be sent
  * @param[in] len:     the data length
  */
void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
	static uint8_t seq = 0;
	unsigned char i=i;
	uint8_t tx_buff[MAX_SIZE];
	
  uint16_t frame_length = frameheader_len + cmd_len + len + crc_len;   //����֡����	

	memset(tx_buff,0,frame_length);  //�洢���ݵ���������
	
	/*****֡ͷ���*****/
	tx_buff[0] = sof;//����֡��ʼ�ֽ�
	memcpy(&tx_buff[1],(uint8_t*)&len, sizeof(len));//����֡��data�ĳ���
	tx_buff[3] = seq;//�����
	Append_CRC8_Check_Sum(tx_buff,frameheader_len);  //֡ͷУ��CRC8

	/*****��������*****/
	memcpy(&tx_buff[frameheader_len],(uint8_t*)&cmd_id, cmd_len);
	
	/*****���ݴ��*****/
	memcpy(&tx_buff[frameheader_len+cmd_len], p_data, len);
	Append_CRC16_Check_Sum(tx_buff,frame_length);  //һ֡����У��CRC16

	if (seq == 0xff) seq=0;
  else seq++;

	
	/*****�����ϴ�*****/


	  
		while (__HAL_UART_GET_FLAG(&huart6,UART_IT_TC) == RESET) ; //�ȴ�֮ǰ���ַ��������
		
		HAL_UART_Transmit_DMA(&huart6,tx_buff,frame_length);
		
	
//	memcpy(Usart_Referee_Tx_Buf,tx_buff,MAX_SIZE);
//	DMA_Cmd(DMA_Stream_Referee_TX, ENABLE);
//	while(DMA_GetFlagStatus(DMA_Stream_Referee_TX,DMA_FLAG_TCIF6)==RESET);
//	DMA_ClearFlag(DMA_Stream_Referee_TX,DMA_FLAG_TCIF6);
//	DMA_SetCurrDataCounter(DMA_Stream_Referee_TX, MAX_SIZE);	
}
