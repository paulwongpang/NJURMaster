#include "main.h"

/*******串口6变量*********************/
u8 Rx_6_Buf[256];	
u8 Tx6Buffer[256];
u8 Tx6Counter=0;
u8 count6=0; 
u8 Tx6DMABuffer[256]={0};
u8 RX6DMABuffer[2][BSP_USART6_DMA_RX_BUF_LEN];
/***********************************/

/**
  * @brief 串口6初始化 
  * @param BaudRate
  * @retval None
  * @details DMA发送，RXNE接收中断
  *          BaudRate 115200
  *          TX  DMA2_Stream6_Ch5  PG14
  *          RX  DMA2_Stream1_Ch5  PG9
  */
void Usart6_Init(u32 br_num)
{
  
  USART_InitTypeDef USART_InitStructure;
  USART_ClockInitTypeDef USART_ClockInitStruct;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);

  //配置中断优先级
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //设置引脚复用器映射
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6);
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);

  //GPIO端口模式设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOG, &GPIO_InitStructure);

  //串口参数初始化
  USART_InitStructure.USART_BaudRate = br_num;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART6, &USART_InitStructure);

  USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;
  USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;
  USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;
  USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable;
  USART_ClockInit(USART6, &USART_ClockInitStruct);

  //配置 Usart6_TX的DMA2_Stream2_Ch5
  DMA_DeInit(DMA2_Stream6);  //将DMA通道6寄存器重设为缺省值
  while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE){}  //等待DMA可配置
  DMA_InitStructure.DMA_Channel = DMA_Channel_5;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART6->DR;//DMA外设地址，这里为串口接收发送数据的地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Tx6DMABuffer;//存放DMA传输数据的存储器地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = 0;  //初始化数据传输量
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream6, &DMA_InitStructure);//初始化DMA Stream

  //配置 Usart6_RX的DMA2_Stream1_Ch5
  DMA_DeInit(DMA2_Stream1);
  while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE){}
  DMA_InitStructure.DMA_Channel = DMA_Channel_5;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART6->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&RX6DMABuffer[0][0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到储存器
  DMA_InitStructure.DMA_BufferSize =sizeof(BSP_USART6_DMA_RX_BUF_LEN)/2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);//初始化DMA Stream

  DMA_DoubleBufferModeConfig(DMA2_Stream1, (uint32_t)&RX6DMABuffer[1][0], DMA_Memory_0);   //first used memory configuration
  DMA_DoubleBufferModeCmd(DMA2_Stream1, ENABLE);
  DMA_Cmd(DMA2_Stream1, ENABLE);

  USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);  //RXNE 读数据寄存器非空
  USART_Cmd(USART6, ENABLE); 
  USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);    //使能USART6的DMA发送
  USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);

}

/**
  * @brief 串口6中断
  * @param None
  * @retval None
  * @details RXNE中断，由此进入Usart6通讯协议解析
  */
void USART6_IRQHandler(void)
{
  static uint32_t this_time_rx6_len = 0;
  if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
	{
    //clear the idle pending flag
    //USART_ClearITPendingBit(USART6,USART_IT_IDLE);清除IDLE中断标志位不能这样完成
	USART6->DR;
	USART6->SR;
  //Target is Memory0
    if(DMA_GetCurrentMemoryTarget(DMA2_Stream1) == 0)
    {
      DMA_Cmd(DMA2_Stream1,DISABLE);
      this_time_rx6_len = BSP_USART6_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream1);
      DMA2_Stream1->NDTR = (uint16_t)BSP_USART6_DMA_RX_BUF_LEN;                             //relocate the dma memory pointer to the beginning position
      DMA2_Stream1->CR |= (uint32_t)(DMA_SxCR_CT);                                          //we select the memory1 to receive data
      DMA_Cmd(DMA2_Stream1,ENABLE);
      if(this_time_rx6_len <= RS_FRAME_LENGTH)       //
      {
		      Usart6_DataPrepare(RX6DMABuffer[0]);
      }
    }
    //Target is Memory1
    else
    {
      DMA_Cmd(DMA2_Stream1,DISABLE);
      this_time_rx6_len = BSP_USART6_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream1);  //get the length of data that DMA has transferred
      DMA2_Stream1->NDTR = (uint16_t)BSP_USART6_DMA_RX_BUF_LEN;                             //relocate the dma memory pointer to the beginning position
      DMA2_Stream1->CR &= ~(uint32_t)(DMA_SxCR_CT);                                         //we select the memory1 to receive data
      DMA_Cmd(DMA2_Stream1,ENABLE);
      if(this_time_rx6_len <= RS_FRAME_LENGTH)       //
      {  
		      Usart6_DataPrepare(RX6DMABuffer[1]);
      }
    }
  }
}


/**
  * @brief 串口6的DMA发送函数，发送一组数据
  * @param DataToSend 要发送数据的数组的指针
  * @param data_num 要发送的数据的个数
  * @retval None
  */
void Usart6_Send(unsigned char *DataToSend ,u8 data_num)
{
  	u8 i;
	static uint16_t num_usart6=0;
	static u8 len_usart6=0;
	
	DMA_Cmd(DMA2_Stream6, DISABLE);
	DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6);//清除DMA2_Steam6传输完成标志
	num_usart6 = DMA_GetCurrDataCounter(DMA2_Stream6);
	for(i=0;i<data_num;i++)
	{
		Tx6Buffer[count6++] = *(DataToSend+i);
	}
	for (i=0;i<(u8)num_usart6;i++)
	{ 
		Tx6DMABuffer[i]=Tx6Buffer[((u8)(len_usart6-num_usart6+i))];
	}
	for (;i<(u8)(num_usart6+data_num);i++)
	{
		Tx6DMABuffer[i]=*(DataToSend+i-num_usart6);
	}
	len_usart6=count6;
	while (DMA_GetCmdStatus(DMA2_Stream6) != DISABLE){}	//确保DMA可以被设置  
	DMA2_Stream6->NDTR = (uint16_t)(num_usart6+data_num);          //数据传输量  
	DMA_Cmd(DMA2_Stream6, ENABLE);
}


