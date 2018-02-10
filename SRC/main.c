/* CMSIS 5.1.1
 * DFM 2.2.0
 * Пример работы с драйвером LCD экрана LC75832 в режиме 1/2-duty
 * Исходный контроллер STM32F103RCT6
 * Отладочная плата Open103R
 * (https://www.waveshare.com/wiki/Open103R)
 *
 * Спасибо https://github.com/firatsoygul/PT6523
 * и http://www.avislab.com/blog
 * 
 */

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f10x_spi.h"              // Keil::Device:StdPeriph Drivers:SPI
#include "stm32f10x_dma.h"              // Keil::Device:StdPeriph Drivers:DMA

#define CLK_Pin    GPIO_Pin_13            // CLK = PB[13]
#define CLK_Speed  GPIO_Speed_50MHz
#define CLK_Mode   GPIO_Mode_AF_PP
#define CLK_Port   GPIOB
#define CLK_Bus    RCC_APB2Periph_GPIOB

#define DO_Pin     GPIO_Pin_15            // MOSI (DO) = PB[15]
#define DO_Speed   GPIO_Speed_50MHz
#define DO_Mode    GPIO_Mode_AF_PP
#define DO_Port    GPIOB
#define DO_Bus     RCC_APB2Periph_GPIOB

#define CE_Pin     GPIO_Pin_12            // CE = PB[12]
#define CE_Speed   GPIO_Speed_50MHz
#define CE_Mode    GPIO_Mode_Out_PP
#define CE_Port    GPIOB
#define CE_Bus     RCC_APB2Periph_GPIOB

#define INH_Pin    GPIO_Pin_10            // INH = PC[10]
#define INH_Speed  GPIO_Speed_50MHz
#define INH_Mode   GPIO_Mode_Out_PP
#define INH_Port   GPIOC
#define INH_Bus    RCC_APB2Periph_GPIOC

#define CMD_INPUT 0x45

//__align(4) uint8_t static_mode_data[9];
__align(4) uint8_t duty_mode_data[9*2];

uint32_t Seconds = 1;
uint8_t invers = 0;

void SetSysClockTo72(void)
{
    ErrorStatus HSEStartUpStatus;
 
    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
    /* Системный RESET RCC (делать не обязательно, но полезно на этапе отладки) */
    RCC_DeInit();
 
    /* Включаем HSE (внешний кварц) */
    RCC_HSEConfig( RCC_HSE_ON);
 
    /* Ждем пока HSE будет готов */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
 
    /* Если с HSE все в порядке */
    if (HSEStartUpStatus == SUCCESS)
    {
    /* Следующие две команды касаются исключительно работы с FLASH.
    Если вы не собираетесь использовать в своей программе функций работы с Flash,
    FLASH_PrefetchBufferCmd( ) та FLASH_SetLatency( ) можно закомментировать */
 
        /* Включаем Prefetch Buffer */
        //FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);
 
        /* FLASH Latency.
    Рекомендовано устанавливать:
        FLASH_Latency_0 - 0 < SYSCLK? 24 MHz
        FLASH_Latency_1 - 24 MHz < SYSCLK ? 48 MHz
        FLASH_Latency_2 - 48 MHz < SYSCLK ? 72 MHz */
        //FLASH_SetLatency( FLASH_Latency_2);
 
        /* HCLK = SYSCLK */ /* Смотри на схеме AHB Prescaler. Частота не делится (RCC_SYSCLK_Div1) */
        RCC_HCLKConfig( RCC_SYSCLK_Div1);
 
        /* PCLK2 = HCLK */ /* Смотри на схеме APB2 Prescaler. Частота не делится (RCC_HCLK_Div1)  */
        RCC_PCLK2Config( RCC_HCLK_Div1);
 
        /* PCLK1 = HCLK/2 */ /* Смотри на схеме APB1 Prescaler. Частота делится на 2 (RCC_HCLK_Div2)
        потому что на выходе APB1 должно быть не более 36МГц (смотри схему) */
        RCC_PCLK1Config( RCC_HCLK_Div2);
 
        /* PLLCLK = 8MHz * 9 = 72 MHz */
        /* Указываем PLL от куда брать частоту (RCC_PLLSource_HSE_Div1) и на сколько ее умножать (RCC_PLLMul_9) */
        /* PLL может брать частоту с кварца как есть (RCC_PLLSource_HSE_Div1) или поделенную на 2 (RCC_PLLSource_HSE_Div2). Смотри схему */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
 
        /* Включаем PLL */
        RCC_PLLCmd( ENABLE);
 
        /* Ждем пока PLL будет готов */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }
 
        /* Переключаем системное тактирование на PLL */
        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);
 
        /* Ждем пока переключиться */
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    { /* Проблемы с HSE. Тут можно написать свой код, если надо что-то делать когда микроконтроллер не смог перейти на работу с внешним кварцом */
 
        /* Пока тут заглушка - вечный цикл*/
        while (1)
        {
        }
    }
}

void PeriphConfig(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	
  RCC_APB2PeriphClockCmd(CLK_Bus, ENABLE);
  GPIO_InitStructure.GPIO_Pin = CLK_Pin;
  GPIO_InitStructure.GPIO_Mode = CLK_Mode;
  GPIO_InitStructure.GPIO_Speed = CLK_Speed;
  GPIO_Init(CLK_Port, &GPIO_InitStructure);	
	
  GPIO_InitStructure.GPIO_Pin = DO_Pin;
  GPIO_InitStructure.GPIO_Mode = DO_Mode;
  GPIO_InitStructure.GPIO_Speed = DO_Speed;
  GPIO_Init(DO_Port, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = CE_Pin;
  GPIO_InitStructure.GPIO_Mode = CE_Mode;
  GPIO_InitStructure.GPIO_Speed = CE_Speed;
  GPIO_Init(CE_Port, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(INH_Bus, ENABLE);
  GPIO_InitStructure.GPIO_Pin = INH_Pin;
  GPIO_InitStructure.GPIO_Mode = INH_Mode;
  GPIO_InitStructure.GPIO_Speed = INH_Speed;
  GPIO_Init(INH_Port, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	//SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  //SPI_InitStructure.SPI_CRCPolynomial = 7;
  
	SPI_Init(SPI2, &SPI_InitStructure);
  SPI_Cmd(SPI2, ENABLE);

}

static void DMA_Configuration(void)
{

  DMA_InitTypeDef DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(SPI2->DR);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)duty_mode_data;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 9;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);
}

static void sendByte(uint8_t byte)
{
  SPI_I2S_SendData(SPI2,byte);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET){};
}

//void LCD_Driver_send(void) 
//{
//  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET){};

//  //GPIO_ResetBits(CE_Port,CE_Pin);
//  sendByte(CMD_INPUT);
//  GPIO_SetBits(CE_Port,CE_Pin);

//	static_mode_data[6]&= 0xFC;
//	static_mode_data[7] = 0x00;
//  static_mode_data[8] = 0x01;

//	sendByte(static_mode_data[0]);
//	sendByte(static_mode_data[1]);
//	sendByte(static_mode_data[2]);
//	sendByte(static_mode_data[3]);
//	sendByte(static_mode_data[4]);
//	sendByte(static_mode_data[5]);
//	sendByte(static_mode_data[6]);
//	sendByte(static_mode_data[7]);
//	sendByte(static_mode_data[8]);	
//		
//	GPIO_ResetBits(CE_Port,CE_Pin);
//		
////  DMA_Cmd(DMA1_Channel5,DISABLE);
////  DMA_SetCurrDataCounter(DMA1_Channel5,9);
////  DMA_Cmd(DMA1_Channel5,ENABLE);
////  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
//}

void LCD_Driver_send_duty(void) 
{
	
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET){};

  //GPIO_ResetBits(CE_Port,CE_Pin);
  sendByte(CMD_INPUT);
  GPIO_SetBits(CE_Port,CE_Pin);

	duty_mode_data[6]&= 0xFC;
	duty_mode_data[7] = 0x00;
  duty_mode_data[8] = 0x80;
		
	//dt =1
	//000010000000	
		
	sendByte(duty_mode_data[0]);
	sendByte(duty_mode_data[1]);
	sendByte(duty_mode_data[2]);
	sendByte(duty_mode_data[3]);
	sendByte(duty_mode_data[4]);
	sendByte(duty_mode_data[5]);
	sendByte(duty_mode_data[6]);
	sendByte(duty_mode_data[7]);
	sendByte(duty_mode_data[8]);	
		
	GPIO_ResetBits(CE_Port,CE_Pin);
		
	sendByte(CMD_INPUT);
  GPIO_SetBits(CE_Port,CE_Pin);

	duty_mode_data[15]&= 0xFC;
	duty_mode_data[16] = 0x00;
  duty_mode_data[17] = 0x01;

	sendByte(duty_mode_data[9]);
	sendByte(duty_mode_data[10]);
	sendByte(duty_mode_data[11]);
	sendByte(duty_mode_data[12]);
	sendByte(duty_mode_data[13]);
	sendByte(duty_mode_data[14]);
	sendByte(duty_mode_data[15]);
	sendByte(duty_mode_data[16]);
	sendByte(duty_mode_data[17]);	
		
	GPIO_ResetBits(CE_Port,CE_Pin);
		
//  DMA_Cmd(DMA1_Channel5,DISABLE);
//  DMA_SetCurrDataCounter(DMA1_Channel5,9);
//  DMA_Cmd(DMA1_Channel5,ENABLE);
//  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
}

void TimerConfig(void)
{
// TIMER4
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
 
    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIMER_InitStructure.TIM_Prescaler = 36000;
    TIMER_InitStructure.TIM_Period = 100-1; // секунда
    TIM_TimeBaseInit(TIM4, &TIMER_InitStructure);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
 
    /* NVIC Configuration */
    /* Enable the TIM4_IRQn Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


int main(void)
{	

	uint8_t onebit = 0x80;
	uint8_t numbute = 0x00;
	
	SetSysClockTo72();
	
	PeriphConfig();
	//DMA_Configuration();
	TimerConfig();
	
	GPIO_SetBits(INH_Port,INH_Pin);
	GPIO_ResetBits(CE_Port,CE_Pin);
	
	
	
	while(1)
	{
		if(Seconds == 0)
		{
			
//			if (onebit == 0x00)
//			{
//				onebit = 0x80;
//				numbute++;
//				duty_mode_data[numbute-1] = 0x00;
//			}
//			else
//				onebit >>=  1;
//			
//			if (numbute == 7)
//				numbute = numbute + 2;
//			
//			if (numbute == 16)
//				numbute = 0;
//			
//			duty_mode_data[numbute] = onebit;
			
			
			
			if (onebit == 0xFF)
			{
				onebit = 0x80;
				numbute++;
			}
			else
			{
				onebit >>=  1;	
				onebit |= (1 << 7);
			}

			if (numbute == 16)
			{
				while(numbute)
				{
					duty_mode_data[numbute-1] = 0x00;
					numbute--;
				}
			}
			
			
			duty_mode_data[numbute] = onebit;
			

			LCD_Driver_send_duty();
			
			Seconds = 5;

		}
	}
}

//void DMA1_Channel5_IRQHandler(void) 
//{

//  if(DMA_GetITStatus(DMA1_IT_TC5) == SET) {

//    DMA_ClearITPendingBit(DMA1_IT_TC5);

//    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET){};

//    GPIO_ResetBits(CE_Port,CE_Pin);
//		
//		GPIOC->ODR ^= GPIO_Pin_9;
//  }
//}

void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		// Обязательно сбрасываем флаг
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		
		GPIOC->ODR ^= GPIO_Pin_9;
		
		if(Seconds)
			Seconds--;
	}
}
