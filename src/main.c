#include "stm32f0xx_conf.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
GPIO_InitTypeDef        GPIO_InitStructure;
uint16_t TimerPeriod = 0;
uint16_t Channel3Pulse = 0;
/* Private function prototypes -----------------------------------------------*/
void TIM_Config(void);
void USART_Configuration(unsigned int BaudRate);
void SendUSART(USART_TypeDef* USARTx,uint8_t ch);
/* Private functions ---------------------------------------------------------*/

void SysTick_Handler(void) {
  static uint16_t tick = 0;
  static uint16_t i =0;

	i = (i+1) & 127;
	// // GPIOC->ODR ^= GPIO_Pin_8;
	TIM3->CCR3 = (uint16_t) (((uint32_t) i * (TimerPeriod - 1)) / 127);
  switch (tick++) {
  	case 50:
  		tick = 0;
  		// i++;
  		// if(i >100 )
  		// 	i = 0;
  		// // // GPIOC->ODR ^= GPIO_Pin_8;
  		// TIM3->CCR3 = (uint16_t) (((uint32_t) i * (TimerPeriod - 1)) / 100);
  		break;
  }
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
	// TIM_Config();

	/* Compute the value to be set in ARR regiter to generate signal frequency at 17.57 Khz */
	TimerPeriod = (SystemCoreClock / 17570 ) - 1;
	/* Compute CCR1 value to generate a duty cycle at 50% for channel 1 */
	Channel3Pulse = (uint16_t) (((uint32_t) 100 * (TimerPeriod - 1)) / 100);

	// SysTick_Config(SystemCoreClock/100);
	USART_Configuration(115200);

	// /* TIM3 clock enable */
	// RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);

	// /* Time Base configuration */
	// TIM_TimeBaseStructure.TIM_Prescaler = 0;
	// TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	// TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
	// TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	// TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	// TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	// /* Channel 1, 2, 3 and 4 Configuration in PWM mode */
	// TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	// TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	// TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	// TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	// TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	// TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	// TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	// TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
	// TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	// /* TIM3 counter enable */
	// TIM_Cmd(TIM3, ENABLE);

	// /* TIM3 Main Output Enable */
	// TIM_CtrlPWMOutputs(TIM3, ENABLE);

	// GPIOA->ODR |= GPIO_Pin_1;
	while(1)
	{
		SendUSART(USART1,'A');
		SendUSART(USART1,'B');
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