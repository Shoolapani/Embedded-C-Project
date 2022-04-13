/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Oct 7, 2021
 *      Author: Maa DURGA
 */
#include "stm32f407xx_gpio_driver.h"
#include <stdio.h>
/************************************************************
* @FunctionName 		:	GPIO_PeriClockControl

* @brief				:	enable the clock of GPIOx peripheral

* @param[in]			:   GPIO_REG_DEF_T *pGPIOx

* @param[in]			:   uint8_t EnorDi

* @return				:	None

* @Note				:

************************************************************/


void GPIO_PeriControl_Clock(GPIO_REG_DEF_T *pGPIOx, uint8_t EnorDi)
{
    if(EnorDi==ENABLE)
    {
//    	Enable the Clock
    	if(pGPIOx==GPIOA)
    	{
    		GPIOA_PCLK_EN();
    	}
    	else if (pGPIOx == GPIOB)
    		{
    			GPIOB_PCLK_EN();
    	}
    	else if (pGPIOx == GPIOC) {
    			GPIOC_PCLK_EN();
  		}
    	else if (pGPIOx == GPIOD)
    		{
    			GPIOD_PCLK_EN();
    		}
    	else if (pGPIOx == GPIOE)
    		{
    			GPIOE_PCLK_EN();
    		}
    	else if (pGPIOx == GPIOF)
    		{
    		GPIOF_PCLK_EN();
    		}
    	else if (pGPIOx == GPIOI){
    	    GPIOI_PCLK_EN();
    		}

    }
    else
    {
//    	Disable the clock

    	if(pGPIOx == GPIOA){
    						GPIOA_PCLK_DIS();
    					}
    	else if (pGPIOx == GPIOB)
    					{
    						GPIOB_PCLK_DIS();
    					}
    	else if (pGPIOx == GPIOC)
    					{
    						GPIOC_PCLK_DIS();
    					}
    	else if (pGPIOx == GPIOD)
    					{
    						GPIOD_PCLK_DIS();
    					}
    	else if (pGPIOx == GPIOE)
    					{
    						GPIOE_PCLK_DIS();
    					}
    	else if (pGPIOx == GPIOF)
    					{
    						GPIOF_PCLK_DIS();
    					}
    	else if (pGPIOx == GPIOG)
    					{
    						GPIOG_PCLK_DIS();
    					}
    	else if (pGPIOx == GPIOH)
    					{
    						GPIOH_PCLK_DIS();
    					}
    	else if (pGPIOx == GPIOI)
    					{
    						GPIOI_PCLK_DIS();
    					}
    }

}


/************************************************************
* @FunctionName 		: GPIO_Init

* @brief				:	Intialiazation of GPIOx peripheral

* @param[in]			:

* @param[in]			:

* @return				:	None

* @Note				:

************************************************************/

void GPIO_Init(GPIO_Handle_t *P_GPIO_Handle)
{
	uint32_t temp=0;//temp register.

// 	1.Enable the clock Peripheral

	GPIO_PeriControl_Clock(P_GPIO_Handle->pGPIOx,ENABLE);

//	2.Configure the mode of the GPIO Pin

	if(P_GPIO_Handle->GPIO_PinConfig.GPIO_PinMode <=GPIO_MODE_ANALOG){
	temp=P_GPIO_Handle->GPIO_PinConfig.GPIO_PinMode << (2* P_GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

	P_GPIO_Handle->pGPIOx->MODER &= ~(3<< (2* P_GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));//For Good Practice
	P_GPIO_Handle->pGPIOx->MODER |=temp;
	temp=0;
	}
	else
	{
			if (P_GPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
				//1. Configure the FTSR
				EXTI->EXTI_FTSR |=(1<<P_GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			}else if(P_GPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
//			clear the corresponding RTSR bit as well
				EXTI->EXTI_RTSR &=~(1<<P_GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
//			1. Configure the RTSR
				EXTI->EXTI_RTSR |=(1<<P_GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

			}else if(P_GPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
				//			clear the corresponding RTSR bit as well
				EXTI->EXTI_RTSR &=~(1<<P_GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

				//1. Configure both the FTSR and RTSR
				EXTI->EXTI_FTSR |=(1<<P_GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
				EXTI->EXTI_RTSR &=~(1<<P_GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

			}

			//2. Configure the GPIO port selection in SYSCFG_EXTICR
			uint8_t temp1=P_GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber/4;
			uint8_t temp2=P_GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber %4;
			uint8_t portcode=GPIO_BASEADDR_TO_CODE(P_GPIO_Handle->pGPIOx);
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[temp1]=(portcode <<(4*temp2));


			//3. Enable the the EXTI interrupt using EXTI_IMR register
				EXTI->EXTI_IMR |=(1<<P_GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);


	}
// 	3.Configure the speed;

	temp= (P_GPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed << ((2* P_GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)));

	P_GPIO_Handle->pGPIOx->OSPEEDR &= ~(3<< (2* P_GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));//For Good Practice
	P_GPIO_Handle->pGPIOx->OSPEEDR |=temp;
	temp=0;

//  4. Configure the PuPD Settings.

	temp= (P_GPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdControl << ((2*P_GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)));

	P_GPIO_Handle->pGPIOx->PUPDR  &= ~(3<< (2* P_GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));//For Good Practice
	P_GPIO_Handle->pGPIOx->PUPDR |=temp;
	temp=0;

//  5. Configure the optype.

	temp= (P_GPIO_Handle->GPIO_PinConfig.GPIO_PinOpType << ((P_GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)));

	P_GPIO_Handle->pGPIOx->OSPEEDR &= ~(3<< (P_GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));//For Good Practice
	P_GPIO_Handle->pGPIOx->OSPEEDR |=temp;
	temp=0;

//	6. Configure the ALT function mode.

	if(P_GPIO_Handle->GPIO_PinConfig.GPIO_PinAlltFunMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1,temp2;
		temp1=P_GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber /8;
		temp2=P_GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber %8;
		P_GPIO_Handle->pGPIOx->AF[temp1] &=~(15<<4*temp2);
		P_GPIO_Handle->pGPIOx->AF[temp1] |=(P_GPIO_Handle->GPIO_PinConfig.GPIO_PinAlltFunMode << (4 *temp2));

	}


}


/************************************************************
* @FunctionName 		: GPIO_DeInit

* @brief				:	DEintialiazation of GPIOx peripheral

* @param[in]			: 	GPIO_REG_DEF_T *pGPIOx

* @param[in]			:

* @return				:	None

* @Note				:

************************************************************/

void GPIO_DeInit(GPIO_REG_DEF_T *pGPIOx)
{
	if(pGPIOx == GPIOA)
		{
			GPIOA_REG_RESET();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_REG_RESET();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_REG_RESET();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_REG_RESET();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_REG_RESET();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_REG_RESET();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_REG_RESET();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_REG_RESET();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_REG_RESET();
		}


}

/************************************************************
* @FunctionName 		: GPIO_ReadFromInputPin


* @brief				:	Read Data from the output

* @param[in]			:	GPIO_REG_DEF_T *P_GPIOX

* @param[in]			:	uint8_t PinNumber

* @return				:	uint8_t

* @Note				:

************************************************************/


uint8_t GPIO_ReadFromInputPin(GPIO_REG_DEF_T *P_GPIOX,uint8_t PinNumber)
{
	uint8_t value;
	value=(uint8_t)((P_GPIOX->IDR >> PinNumber)&(0x001));//Typecast uint32_t to uint8_t

	return value;
}

/************************************************************
* @FunctionName 		: GPIO_ReadFromInputPort



* @brief				: Read from the Input Port

* @param[in]			: GPIO_REG_DEF_T *P_GPIOX

* @param[in]			:

* @return				:	uint16_t

* @Note				:

************************************************************/

uint16_t GPIO_ReadFromInputPort(GPIO_REG_DEF_T *P_GPIOX)
{
	uint16_t value;
	value=(uint16_t)P_GPIOX->IDR;
	return value;
}

/************************************************************
* @FunctionName 		: GPIO_WriteToOutputPin



* @brief				: write to the Output the pin

* @param[in]			: GPIO_REG_DEF_T *P_GPIOX

* @param[in]			: uint8_t PinNumber

* @param[in]			: uint8_t value

* @return				: NONE

* @Note				:

************************************************************/


void GPIO_WriteToOutputPin(GPIO_REG_DEF_T *P_GPIOX,uint8_t PinNumber,uint8_t value)
{
    if(value == GPIO_PIN_SET)
    {
    	P_GPIOX->ODR |= (1<<PinNumber);
    }
    else{
    	P_GPIOX->ODR &= ~(1<<PinNumber);
    }
}

/************************************************************
* @FunctionName 		: GPIO_WriteToOutputPort



* @brief				: write to the Output the Port

* @param[in]			: GPIO_REG_DEF_T *P_GPIOX

* @param[in]			: uint8_t value

* @return				: NONE

* @Note					:

************************************************************/

void GPIO_WriteToOutputPort(GPIO_REG_DEF_T *P_GPIOX,uint16_t value)
{
	P_GPIOX->ODR=value;
	return;
}

/************************************************************
* @FunctionName 		: GPIO_ToggleOutputPin



* @brief				: write to the Output of the Port

* @param[in]			: GPIO_REG_DEF_T *P_GPIOX

* @param[in]			: uint8_t PinNumber

* @return				: NONE

* @Note					:

************************************************************/

// XOR Operation
void GPIO_ToggleOutputPin(GPIO_REG_DEF_T *P_GPIOX,uint8_t PinNumber)
{
	P_GPIOX->ODR ^=(1<<PinNumber);
	return;
}


/************************************************************
 * @FunctionName 		:	GPIO_IRQInterruptConfig
 * @brief				:	Used for configuring the interrupt
 *
 * @param[in]			:	uint8_t IRQNumber
 * @param[in]			:	uint8_t EnorDi
 *
 * @return				:	None
 *
 * @Note				:
 ************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi==ENABLE){
		if(IRQNumber <=31){
//			Program ISER0 register
			*NVIC_ISER0 |=(1<< IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber <64){
//			Program ISER1 register
			*NVIC_ISER1 |=(1<< (IRQNumber % 32));
		}else if(IRQNumber >=64 && IRQNumber < 96 ){
//			Program ISER1 register
			*NVIC_ISER2 |=(1<< (IRQNumber % 64));
		}

}
	else{
		if(IRQNumber <=31){
//			Program ICER0 register
			*NVIC_ICER0 |=(1<< IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber <64){
//			Program ICER1 register
			*NVIC_ICER0 |=(1<< (IRQNumber % 32));
		}else if(IRQNumber >=64 && IRQNumber < 96 ){
//			Program ICER1 register
			*NVIC_ICER0 |=(1<< (IRQNumber % 64));
		}
	}

}

/************************************************************
 * @FunctionName 		:	GPIO_IRQPriorityConfig
 * @brief				:	Used for configuring the interrupt Priority
 *
 * @param[in]			:	uint8_t IRQNumber
 * @param[in]			:	uint32_t IRQPriority
 *
 * @return				:	None
 *
 * @Note				:
 ************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
//	1.First Lets Find out the IPR register
	uint8_t iprx= IRQNumber/4;
	uint8_t iprx_section= IRQNumber %4;

	uint8_t shift_amount= (8 *iprx_section) + (8-NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx)|=(IRQPriority << shift_amount);

}

/************************************************************
 * @FunctionName 		:	GPIO_IRQHanding
 * @brief				:	Used for configuring the interrupt Priority
 *
 * @param[in]			:	uint8_t PinNumber
 * @param[in]			:
 *
 * @return				:	None
 *
 * @Note				:
 ************************************************************/
void GPIO_IRQHanding(uint8_t PinNumber)
{
	//clear the exti PR register corresponding to the pin number
		if(EXTI->EXTI_PR & (1 << PinNumber)){
			//clear the pending register
			EXTI->EXTI_PR |= (1 << PinNumber);
		}

}



