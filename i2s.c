/*
 * i2s.c
 *
 *  Created on: May 30, 2024
 *      Author: hussamaldean
 */


#include "i2s.h"
#include "stm32f4xx.h"

void I2S3_Pins_Init(void)
{
	#define I2S_AF 0x06

	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN|RCC_AHB1ENR_GPIOAEN;

	GPIOA->MODER|=GPIO_MODER_MODE4_1;
	GPIOA->MODER&=~GPIO_MODER_MODE4_0;

	GPIOA->AFR[0]|=(I2S_AF<<GPIO_AFRL_AFSEL4_Pos);

	GPIOC->MODER|=GPIO_MODER_MODE7_1|GPIO_MODER_MODE10_1|GPIO_MODER_MODE12_1;

	GPIOC->MODER&=~(GPIO_MODER_MODE7_0|GPIO_MODER_MODE10_0|GPIO_MODER_MODE12_0);

	GPIOC->AFR[0]|=(I2S_AF<<GPIO_AFRL_AFSEL7_Pos);

	GPIOC->AFR[1]|=(I2S_AF<<GPIO_AFRH_AFSEL10_Pos)|(I2S_AF<<GPIO_AFRH_AFSEL12_Pos);


}


void I2S3_Init(void)
{
	/*Enable clock access to SPI3/I2S3*/

	RCC->APB1ENR|=RCC_APB1ENR_SPI3EN;

	/*Set SPI into I2S Mode*/
	SPI3->I2SCFGR|=SPI_I2SCFGR_I2SMOD;

	/*Set the mode to master transmit*/
	SPI3->I2SCFGR|=SPI_I2SCFGR_I2SCFG_1;

	/*Set the prescaler and ODD values*/
	SPI3->I2SPR=(3<<SPI_I2SPR_I2SDIV_Pos)|(1<<SPI_I2SPR_ODD_Pos);

	/*Enable Master Clock output*/
	SPI3->I2SPR|=SPI_I2SPR_MCKOE;


	/*Enable SPI_TX_DMA*/
	SPI3->CR2|=SPI_CR2_TXDMAEN;

	/*Enable SPI3 interrupt in NVIC*/
	NVIC_EnableIRQ(SPI3_IRQn);



	/*DMA Configuration*/

	/*Enable Clock Access to DMA1*/
	RCC->AHB1ENR|=RCC_AHB1ENR_DMA1EN;


	/*Configure DMA with the following parameters
	 * 1.Channel is 0.
	 * Memory increment mode.
	 * Direction Memory to Peripheral.
	 * Enable Transfer Complete Interrupt.
	 * Enable Circular Mode
	 *
	 * */
	DMA1_Stream7->CR|=(0x00<<DMA_SxCR_CHSEL_Pos)|DMA_SxCR_MINC|DMA_SxCR_DIR_0|DMA_SxCR_TCIE|DMA_SxCR_CIRC;




	/*Enable DMA1_Steam7 Interrupt in nVIC*/
	NVIC_EnableIRQ(DMA1_Stream7_IRQn);


	/*Enable I2S*/
	SPI3->I2SCFGR|=SPI_I2SCFGR_I2SE;


}





void I2S_SendData(uint16_t *data, uint16_t len)
{
	uint32_t i=0;

	while(i<len)
	{
		/*Wait until TXE is set*/
		while(!(SPI3->SR & (SPI_SR_TXE))){}

		/*Write the data to the data register*/
		SPI3->DR = data[i];
		i++;
	}
	/*Wait until TXE is set*/
	while(!(SPI3->SR & (SPI_SR_TXE))){}

	/*Wait for BUSY flag to reset*/
	while((SPI3->SR & (SPI_SR_BSY))){}

	/*Clear OVR flag*/
	(void)SPI3->DR;
	(void)SPI3->SR;
}

void I2S_SendData_DMA(uint16_t *data, uint16_t len)
{

	/*Set Peripheral address to SPI3->DR*/
	DMA1_Stream7->PAR=(uint32_t)&SPI3->DR;

	/*Set the memory address*/
	DMA1_Stream7->M0AR=(uint32_t)data;

	/*Set the length*/
	DMA1_Stream7->NDTR=len;

	/*Launch the DMA*/
	DMA1_Stream7->CR|=DMA_SxCR_EN;




}


void DMA1_Stream7_IRQHandler(void)
{
	/*Check the Intrrupt source of DMA1_Stream7*/
	if((DMA1->HISR & DMA_HISR_TCIF7) ==DMA_HISR_TCIF7 )
	{

		/*Enable SPI Tx buffer empty interrupt*/
		SPI3->CR2|=SPI_CR2_TXEIE;

		/*Clear Pending flag*/
		DMA1->HIFCR=DMA_HIFCR_CTCIF7;
	}
}

__WEAK void SPI3_DMA_TX_Completed(void)
{

}


