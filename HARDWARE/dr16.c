#include "dr16.h"

#define DBUS_RX_BUFF_SIZE 36 //һ֡ң�������ݵĳ���18�ֽ�
//ң�������ݽ��ջ�������
static uint8_t DBUS_rx_buff[2][DBUS_RX_BUFF_SIZE];
//static uint8_t DBUS_rx_buff[DBUS_RX_BUFF_SIZE];


extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;


void DR16_DataDecode(volatile const uint8_t *dbus_rx_buff)
{
	//ң�����ź������
	DBUS.DR16_Signal = 100;
    
    DBUS.RC.ch0          = ((( dbus_rx_buff[0] | (dbus_rx_buff [1]<<8) ) & 0x07FF)-1024);                     
	DBUS.RC.ch1          = ((( dbus_rx_buff[1]>>3 | dbus_rx_buff[2]<<5 ) & 0x07FF)-1024);                      
	DBUS.RC.ch2          = ((( dbus_rx_buff[2]>>6 | dbus_rx_buff[3]<<2 | dbus_rx_buff[4]<<10 ) & 0x07FF)-1024); 
	DBUS.RC.ch3          = (( dbus_rx_buff[4]>>1 | dbus_rx_buff[5]<<7 ) & 0x07FF)-1024;                        
	DBUS.RC.ch4          = ((( dbus_rx_buff[16] | (dbus_rx_buff [17]<<8) ) & 0x07FF)-1024);                         

	DBUS.RC.Switch_Left  = (( dbus_rx_buff[5]>>4 ) & 0x00C )>>2;                                             
	DBUS.RC.Switch_Right = ( dbus_rx_buff[5]>>4 ) & 0x003;                                                   
	
	DBUS.PC.X    = dbus_rx_buff[6] | (dbus_rx_buff[7] << 8);                                            
	
	DBUS.PC.Y       = dbus_rx_buff[8] | (dbus_rx_buff[9] << 8);                                            

	DBUS.PC.Press_Left   = dbus_rx_buff[12];                                                                   
	DBUS.PC.Press_Right  = dbus_rx_buff[13];                                                                   
	DBUS.PC.Keyboard     = dbus_rx_buff[14] | dbus_rx_buff[15] << 8;                                           
	
}

void DR16_Init()
{
//	 __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
//	__HAL_DMA_DISABLE(&hdma_usart3_rx);
//		HAL_DMAEx_MultiBufferStart_IT(&hdma_usart3_rx,USART3->DR,(uint32_t)&DBUS_rx_buff[0][0],(uint32_t)&DBUS_rx_buff[1][0],DBUS_RX_BUFF_SIZE);
//	 __HAL_DMA_ENABLE(&hdma_usart3_rx);

//ʹ��DMA���ڽ���
  	SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
	
	  //enalbe idle interrupt
    //ʹ�ܿ����ж�
   __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
		
	  //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //�ڴ滺����1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(DBUS_rx_buff[0]);
    //�ڴ滺����2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(DBUS_rx_buff[1]);
    //���ݳ���
    hdma_usart3_rx.Instance->NDTR = DBUS_RX_BUFF_SIZE;
    //ʹ��˫������
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

//  
//	HAL_DMAEx_MultiBufferStart(&hdma_usart3_rx,(uint32_t)&USART3->DR,(uint32_t)DBUS_rx_buff[0],(uint32_t)DBUS_rx_buff[1],DBUS_RX_BUFF_SIZE);
	  //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
	
}
void USAR_UART3_IDLECallback(UART_HandleTypeDef *huart)
{
		
	if(huart == &huart3)
		{		
				volatile uint16_t this_time_rx_len = 0;
        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
 
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = DBUS_RX_BUFF_SIZE - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = DBUS_RX_BUFF_SIZE;

            //set memory buffer 1
            //�趨������1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == 18)
            {
                DR16_DataDecode(DBUS_rx_buff[0]);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = DBUS_RX_BUFF_SIZE - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = DBUS_RX_BUFF_SIZE;

            //set memory buffer 0
            //�趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == 18)
            {
                //����ң��������
                DR16_DataDecode(DBUS_rx_buff[1]);
            }
        }
    }
	
}
