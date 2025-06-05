/*
 * i2c.c
 *
 *  Created on: May 30, 2024
 *      Author: hussamaldean
 */


#include "i2c.h"
#include "stdio.h"
uint8_t inited=0;

void i2c_init(void)
{
	#define I2C1_AF 0x04

	if(inited==0)
	{
		RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN; //enable gpiob clock
		RCC->APB1ENR|=RCC_APB1ENR_I2C1EN; //enable i2c1 clock

		//set PB6 and PB9 to alternative function
		GPIOB->MODER|=GPIO_MODER_MODE6_1|GPIO_MODER_MODE9_1;
		GPIOB->MODER&=~(GPIO_MODER_MODE6_0|GPIO_MODER_MODE9_0);

		GPIOB->AFR[0]|=(I2C1_AF<<GPIO_AFRL_AFSEL6_Pos);

		GPIOB->AFR[1]|=(I2C1_AF<<GPIO_AFRH_AFSEL9_Pos);

		GPIOB->OTYPER|=GPIO_OTYPER_OT6|GPIO_OTYPER_OT9; //set pb8 and pb9 as open drain

		I2C1->CR1=I2C_CR1_SWRST;

		I2C1->CR1&=~I2C_CR1_SWRST;

		I2C1->CR2|=(42U<<I2C_CR2_FREQ_Pos);

		I2C1->CCR=210;

		I2C1->TRISE=43; //output max rise

		I2C1->CR1|=I2C_CR1_PE;

		inited=1;
	}
}
char i2c_readByte(char saddr,char maddr, char *data)
{


	while(I2C1->SR2&I2C_SR2_BUSY){;}
	I2C1->CR1|=I2C_CR1_START;
	while(!(I2C1->SR1&I2C_SR1_SB)){;}
	I2C1->DR=saddr<<1;
	while(!(I2C1->SR1&I2C_SR1_ADDR)){;}
	(void)I2C1->SR2;
	while(!(I2C1->SR1&I2C_SR1_TXE)){;}
	I2C1->DR=maddr;
	while(!(I2C1->SR1&I2C_SR1_TXE)){;}
	I2C1->CR1|=I2C_CR1_START;
	while(!(I2C1->SR1&I2C_SR1_SB)){;}
	I2C1->DR=saddr<<1|1;
	while(!(I2C1->SR1&I2C_SR1_ADDR)){;}
	I2C1->CR1&=~I2C_CR1_ACK;
	(void)I2C1->SR2;
	I2C1->CR1|=I2C_CR1_STOP;
	while(!(I2C1->SR1&I2C_SR1_RXNE)){;}
	*data=I2C1->DR;
	return 0;
}

void i2c_writeByte(char saddr,char maddr,char data)
{



	while(I2C1->SR2&I2C_SR2_BUSY){;}          /*wait until bus not busy*/
	I2C1->CR1|=I2C_CR1_START;                 /*generate start*/
	while(!(I2C1->SR1&I2C_SR1_SB)){;}         /*wait until start bit is set*/
	I2C1->DR = saddr<< 1;                 	 /* Send slave address*/
	while(!(I2C1->SR1&I2C_SR1_ADDR)){;}      /*wait until address flag is set*/
	(void)I2C1->SR2; 											 /*clear SR2 by reading it */
	while(!(I2C1->SR1&I2C_SR1_TXE)){;}       /*Wait until Data register empty*/
	I2C1->DR = maddr;                        /* send memory address*/
	while(!(I2C1->SR1&I2C_SR1_TXE)){;}       /*wait until data register empty*/
	I2C1->DR = data;
	while (!(I2C1->SR1 & I2C_SR1_BTF));      /*wait until transfer finished*/
	I2C1->CR1 |=I2C_CR1_STOP;								 /*Generate Stop*/

}

void i2c_WriteMulti(char saddr,char maddr,char *buffer, uint8_t length)
{

	while (I2C1->SR2 & I2C_SR2_BUSY);           //wait until bus not busy
	I2C1->CR1 |= I2C_CR1_START;                   //generate start
	while (!(I2C1->SR1 & I2C_SR1_SB)){;}					//wait until start is generated
	I2C1->DR = saddr<< 1;                 	 			// Send slave address
	while (!(I2C1->SR1 & I2C_SR1_ADDR)){;}        //wait until address flag is set
	(void) I2C1->SR2; 						      //Clear SR2
	while (!(I2C1->SR1 & I2C_SR1_TXE));           //Wait until Data register empty
	I2C1->DR = maddr;                      				// send memory address
	while (!(I2C1->SR1 & I2C_SR1_TXE));           //wait until data register empty
	//sending the data
	for (uint8_t i=0;i<length;i++)
	 {
	 I2C1->DR=buffer[i]; 													//filling buffer with command or data
		while (!(I2C1->SR1 & I2C_SR1_BTF));
	 }

	I2C1->CR1 |= I2C_CR1_STOP;										//wait until transfer finished
}


void i2c_ReadMulti(char saddr,char maddr, int n, char* data)
{

	while (I2C1->SR2 & I2C_SR2_BUSY){;}
	I2C1->CR1|=I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB)){;}
	I2C1->DR=saddr<<1;
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){;}
	(void)I2C1->SR2;
	while(!(I2C1->SR1&I2C_SR1_TXE)){;}
	I2C1->DR = maddr;
	while(!(I2C1->SR1&I2C_SR1_TXE)){;}
	I2C1->CR1|=I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB)){;}
	I2C1->DR=saddr<<1|1;
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){;}
	(void)I2C1->SR2;
	I2C1->CR1|=I2C_CR1_ACK;
	while(n>0U)
		{
		if(n==1U)
				{
				I2C1->CR1&=~I2C_CR1_ACK;
					I2C1->CR1|=I2C_CR1_STOP;
					while(!(I2C1->SR1&I2C_SR1_RXNE)){;}
					*data++=I2C1->DR;
						break;
				}
			else
					{

					while(!(I2C1->SR1&I2C_SR1_RXNE)){;}
						(*data++)=I2C1->DR;
							n--;
				}
}
}

void i2c_bus_scan(void)
{
	int a=0; //address variable

	for (uint8_t i=0;i<128;i++) //go through all 127 address
	{
		I2C1->CR1 |= I2C_CR1_START; //generate start
		while(!(I2C1->SR1 & I2C_SR1_SB)); // wait to start to be generated
		I2C1->DR=(i<<1|0); // transmit the address
		while(!(I2C1->SR1)|!(I2C1->SR2)){}; //clear status register
		I2C1->CR1 |= I2C_CR1_STOP; //generate stop condition
		delay(1);//minium wait time is 40 uS, but for sure, leave it 100 uS
		a=(I2C1->SR1&I2C_SR1_ADDR); //read the status register address set
		if (a==2)//if the address is valid
		{

			printf("Found I2C device at address 0x%X (hexadecimal), or %d (decimal)\r\n",i,i);

		}
	}
}
