#include "stm32f0xx_conf.h"
#include <stdio.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
GPIO_InitTypeDef        GPIO_InitStructure;
uint16_t TimerPeriod = 0;
uint16_t Channel3Pulse = 0;
__IO uint16_t  ADC1ConvertedValue = 0, ADC1ConvertedVoltage = 0;
/* Private function prototypes -----------------------------------------------*/
void TIM_Config(void);
void USART_Configuration(unsigned int BaudRate);
void SendUSART(USART_TypeDef* USARTx,uint8_t ch);
void ADC1_Config(void);
void led_on(void);
void led_off(void);
/* Private functions ---------------------------------------------------------*/
/*
 * _write
 */
int _write(int file, char *ptr, int len)
{
	for (int i = 0; i < len; ++i)
	{
		SendUSART(USART1, *(ptr++));
	}
    return len;
}

void SysTick_Handler(void) {
	static int i = 0;
	static uint16_t data[20]= {0};
	static uint16_t avrg_data = 0;

	/* Get ADC1 converted data */
	ADC1ConvertedValue = ADC_GetConversionValue(ADC1);

	/* Compute the voltage */
	data[i++] = ADC1ConvertedValue;
	if(i > 10)
	{
		for (int j = 0; j < 10; ++j)
		{
			avrg_data+= data[j];
		}
		avrg_data = avrg_data /10;
		if(avrg_data > 2047)
			GPIOA->ODR ^= GPIO_Pin_1;
		i = 0;
	}
	printf("ADC =  %d\r\n", ADC1ConvertedValue);
}

int main(void)
{

#if 1
  /* GPIOC Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* Configure PC8 and PC9 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif


	SysTick_Config(SystemCoreClock/100);
	USART_Configuration(115200);
	ADC1_Config();
#if 0
	TIM_Config();

	/* Compute the value to be set in ARR regiter to generate signal frequency at 17.57 Khz */
	TimerPeriod = (SystemCoreClock / 17570 ) - 1;
	/* Compute CCR1 value to generate a duty cycle at 50% for channel 1 */
	Channel3Pulse = (uint16_t) (((uint32_t) 100 * (TimerPeriod - 1)) / 100);
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);

	/* Time Base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* Channel 1, 2, 3 and 4 Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	/* TIM3 counter enable */
	TIM_Cmd(TIM3, ENABLE);

	/* TIM3 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM3, ENABLE);

	GPIOA->ODR |= GPIO_Pin_1;
#endif

	while(1)
	{

	}

}


/**
  * @brief  Configure the TIM3 Pins.
  * @param  None
  * @retval None
  */
void TIM_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA Clocks enable */
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOC, ENABLE);
  
  /* GPIOA Configuration: Channel 1, 2, 3 and 4 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_0);
}

 /*****************************************khai bao USART********************/  
void USART_Configuration(unsigned int BaudRate)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_1); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_1); 

	USART_Cmd(USART1, ENABLE);  
}

void SendUSART(USART_TypeDef* USARTx,uint8_t ch)
{
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
	USART_SendData(USARTx, (uint8_t) ch);
}

/**
  * @brief  ADC1 channel11 configuration
  * @param  None
  * @retval None
  */
void ADC1_Config(void)
{
  ADC_InitTypeDef          ADC_InitStructure;
  GPIO_InitTypeDef         GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef        TIM_OCInitStructure; 
  
  /* GPIOC Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  
  /* ADC1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
  /* TIM3 Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  
  /* Configure ADC Channel11 as analog input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* TIM3 Configuration *******************************************************/
  TIM_DeInit(TIM3);
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  TIM_OCStructInit(&TIM_OCInitStructure);
    
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 0xFF;
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  /* TIM3 TRGO selection */
  TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
  
  /* ADC1 Configuration *******************************************************/
  /* ADCs DeInit */  
  ADC_DeInit(ADC1);
  
  /* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits*/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;    
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
  ADC_Init(ADC1, &ADC_InitStructure); 
  
  /* Convert the ADC1 Channel 11 with 239.5 Cycles as sampling time */ 
  ADC_ChannelConfig(ADC1, ADC_Channel_0 , ADC_SampleTime_28_5Cycles);   

  /* ADC Calibration */
  ADC_GetCalibrationFactor(ADC1);

  /* Enable the auto delay feature */    
  ADC_WaitModeCmd(ADC1, ENABLE); 
  
  /* Enable the Auto power off mode */
  ADC_AutoPowerOffCmd(ADC1, ENABLE); 
  
  /* Enable ADCperipheral[PerIdx] */
  ADC_Cmd(ADC1, ENABLE);     
  
  /* Wait the ADCEN falg */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN)); 
  
  /* TIM2 enable counter */
  TIM_Cmd(TIM3, ENABLE);
  
  /* ADC1 regular Software Start Conv */ 
  ADC_StartOfConversion(ADC1);
}

void led_on(void)
{
	GPIOA->ODR &= ~GPIO_Pin_1;
}

void led_off(void)
{
	GPIOA->ODR |= GPIO_Pin_1;
}