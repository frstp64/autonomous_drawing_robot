
#include "antenna.h"
#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "LED.h"
#include "string.h"

int16_t antennaValuesTable[BYTES_PER_FRAME * FRAME_NUMBER / 2];
uint8_t isDataReady = 0;

DMA_InitTypeDef myDMAStruct;

void DMA2_Stream0_IRQHandler()
{
	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);
	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TEIF0);
	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_DMEIF0);
	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_FEIF0);

	isDataReady = 1;
    updateLED(3, 0);
    initAntenna();

	myDMAStruct.DMA_BufferSize = BYTES_PER_FRAME * FRAME_NUMBER/2;
	myDMAStruct.DMA_Channel = DMA_Channel_0;
	myDMAStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	myDMAStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	myDMAStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	myDMAStruct.DMA_Memory0BaseAddr = (uint32_t) antennaValuesTable;

//    DMA_Cmd(DMA2_Stream0, DISABLE);
//    DMA_DeInit(DMA2_Stream0);
//    DMA_Init(DMA2_Stream0, &myDMAStruct);
//    DMA_Cmd(DMA2_Stream0, ENABLE);

	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);
	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TEIF0);
	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_DMEIF0);
	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_FEIF0);

	ADC_ClearFlag(ADC1, ADC_FLAG_AWD);
	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
	ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
	ADC_ClearFlag(ADC1, ADC_FLAG_JSTRT);
	ADC_ClearFlag(ADC1, ADC_FLAG_STRT);
	ADC_ClearFlag(ADC1, ADC_FLAG_OVR);
	ADC_ClearITPendingBit(ADC2, ADC_IT_EOC);
	ADC_ClearITPendingBit(ADC2, ADC_IT_AWD);
	ADC_ClearITPendingBit(ADC2, ADC_IT_JEOC);
	ADC_ClearITPendingBit(ADC2, ADC_IT_OVR);


}

void initAntenna() {
	ADC_DeInit();
	// The pin for digital acquisition of the signal
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructAntennaSignal;
    GPIO_InitStructAntennaSignal.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructAntennaSignal.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructAntennaSignal.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructAntennaSignal.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructAntennaSignal.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructAntennaSignal);

    // The power detection pin
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructAntennaPower;
    GPIO_InitStructAntennaPower.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructAntennaPower.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructAntennaPower.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructAntennaPower.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructAntennaPower.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructAntennaPower);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    ADC_DeInit();

    ADC_InitTypeDef  ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    ADC_StructInit(&ADC_InitStructure);
    ADC_CommonStructInit(&ADC_CommonInitStructure);

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; //?
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_10b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // SHOULD BE AT DISABLE, ENABLE IS FASTEST
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;

    /* Now do the setup */
    ADC_CommonInit(&ADC_CommonInitStructure);
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
    ADC_Init(ADC2, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1,
    	ADC_SampleTime_3Cycles);

    ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1,
    	ADC_SampleTime_3Cycles);


    // Timer initialization for the dma acd set
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* Time base configuration */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 84* (SAMPLE_PERIOD_MICROSECONDS) - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    /* TIM2 TRGO selection */
    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
    // ADC_ExternalTrigConv_T2_TRGO
    /* TIM2 enable counter */
    TIM_Cmd(TIM3, ENABLE);

    // DMA initialization
    NVIC_InitTypeDef monInitStructNVIC;
    DMA_InitTypeDef myDMAStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	DMA_StructInit(&myDMAStruct);
    DMA_Cmd(DMA2_Stream0, DISABLE);
    DMA_DeInit(DMA2_Stream0);
	myDMAStruct.DMA_BufferSize = BYTES_PER_FRAME * FRAME_NUMBER/2;
	myDMAStruct.DMA_Channel = DMA_Channel_0;
	myDMAStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	myDMAStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	myDMAStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	myDMAStruct.DMA_Memory0BaseAddr = (uint32_t) antennaValuesTable;
	myDMAStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	myDMAStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    myDMAStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    myDMAStruct.DMA_Mode = DMA_Mode_Normal;
    myDMAStruct.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);//SPI3->DR;
    myDMAStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    myDMAStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    myDMAStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    myDMAStruct.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_Cmd(DMA2_Stream0, DISABLE);
    DMA_DeInit(DMA2_Stream0);
    DMA_Init(DMA2_Stream0, &myDMAStruct);
    DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA2_Stream0, ENABLE);



    monInitStructNVIC.NVIC_IRQChannel= DMA2_Stream0_IRQn;
    monInitStructNVIC.NVIC_IRQChannelCmd = ENABLE;
    monInitStructNVIC.NVIC_IRQChannelPreemptionPriority = 0;
    monInitStructNVIC.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&monInitStructNVIC);

    ADC_DMACmd(ADC1, DISABLE);


	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_84Cycles);
    ADC_Cmd(ADC1, ENABLE);
    ADC_Cmd(ADC2, ENABLE);

}

uint16_t getPowerValue() {
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_84Cycles);
	// Start the conversion
	ADC_SoftwareStartConv(ADC2);
	// Wait until conversion completion
	while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET);
	// Get the conversion value
	return ADC_GetConversionValue(ADC2);
}

void startAntennaDataCapture() {

	isDataReady = 0;
	//DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
    //updateLED(3, 1);

    ADC_DMACmd(ADC1, ENABLE);
    DMA_Cmd(DMA2_Stream0, ENABLE);

}

uint8_t isAntennaDataReady() {
	return isDataReady;
}

void getAntennaTableData(int frameNumber, uint8_t *tablePtr) {
	memcpy(tablePtr, &(antennaValuesTable[frameNumber*BYTES_PER_FRAME/2]), BYTES_PER_FRAME);
}
