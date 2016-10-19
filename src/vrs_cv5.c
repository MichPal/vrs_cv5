/*
 * vrs_cv5.c
 *
 *  Created on: 18. 10. 2016
 *      Author: TOMAS
 */
#include "vrs_cv5.h"

uint16_t ADCvalue=0;
uint8_t pom=0;

void adc_init(void)
{
	ADC_InitTypeDef ADC_InitStructure;

	/* Enable the HSI oscillator */
	RCC_HSICmd(ENABLE);

	/* Check that HSI oscillator is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

	/* Enable ADC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* Initialize ADC structure */
	ADC_StructInit(&ADC_InitStructure);

	/* ADC1 configuration */
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADCx regular channel8 configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_384Cycles);
	ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);
	//ADC_ITConfig(ADC1,ADC_IT_OVR,ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn; //zoznam prerušení nájdete v
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the ADC */
	ADC_Cmd(ADC1, ENABLE);

	/* Wait until the ADC1 is ready */
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET);
	ADC_SoftwareStartConv(ADC1);
	//while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) asm ("nop");

}

void gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // USART


	/* Configure ADCx Channel 0 as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure USART pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
}

void usart_init()
{
	USART_InitTypeDef USART_InitStructure;

//	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = 9800;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);	//USART na prerusenie

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //zoznam prerušení nájdete v
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART2,ENABLE);	// povolenie USART

}

void SendChar(char ch)
{
	USART_SendData(USART2,(char) ch);
	while(!USART_GetFlagStatus(USART2,USART_FLAG_TC));
}

void SendString(char* stringa)
{
	uint8_t i=0;
	while(stringa[i]!=0)
	{
		SendChar(stringa[i]);
		i++;
	}
}

void ADC1_IRQHandler(void)
{
	if(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC))
	{
		ADCvalue=ADC_GetConversionValue(ADC1);
	}
	//if(ADC_GetFlagStatus(ADC1,ADC_SR_OVR))	ADC_ClearFlag(ADC1,ADC_FLAG_OVR);
}

void USART2_IRQHandler()
{
	if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE))
	{
		//USART_ClearITPendingBit(USART2,USART_IT_RXNE);

		if (USART_ReceiveData(USART2)=='m')
		{
			if (pom==0) pom = 1;
			else pom = 0;
		}

	}

}
