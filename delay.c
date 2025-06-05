/*
 * time_base.c
 *
 *  Created on: Dec 28, 2023
 *      Author: hussamaldean
 */

#include "delay.h"
#include "stm32f4xx.h"


#define	CTRL_ENABLE					(1U<<0) /*Enable SysTick Timer*/
#define CTRL_CLKSRC					(1U<<2) /*Clock source selection*/
#define CTRL_COUNTFLAG				(1U<<16) /*Count flag bit*/
#define CTRL_TICKINT				(1U<<1) /*Interrupt enable bit*/


volatile uint64_t mil; /*volatile variable to hold the ms counter*/

void delay_init(uint32_t freq)
{

	/*Set period to be 1ms*/
	SysTick->LOAD  = (freq/1000) - 1;

	/*Clear systick current value register */
	SysTick->VAL = 0;

	/*Enable systick and select internal clk src*/
	SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC ;

	/*Enable systick interrupt*/
	SysTick->CTRL  |= CTRL_TICKINT;

}



uint64_t millis()
	{

	__disable_irq(); /*Disable global interrupt*/

	uint64_t ml=mil; /*Get the current millis values and store in ml*/

	__enable_irq(); /*Enable global interrupt*/

	return ml;		/*Return the stored value*/
	}


/*Spin lock the CPU to delay*/
void delay(uint32_t time)
{

	uint64_t start=millis();
	while((millis() - start) < (time+1));
}

/*Interrupt handler of SysTick*/
//void SysTick_Handler(void)
//{
//	/*Increment the counter with every interrupt*/
//	mil++;
//}
