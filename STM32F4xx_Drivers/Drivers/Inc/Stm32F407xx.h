/*
 * Stm32F407xx.h
 * MICROCONTROLLER SPECIFIC
 *  Created on: Oct 6, 2021
 *      Author: Mahadev
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include <stdint.h>

#define __vo volatile

/******************************** Processor Specific Details ****************************************************************/
/*
 * Arm cortex Mx Processor implemented
 */

#define NO_PR_BITS_IMPLEMENTED	4
/*
 * IRQ(Interrupt Request Number)
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

/*
 * Arm Cortex Mx  Processor NVIC ISERx Register address.
 */

#define NVIC_ISER0			((__vo uint32_t *)0xE000E100)// add 32 bit or 4byte
#define NVIC_ISER1			((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2			((__vo uint32_t *)0xE000E108)

/*
 *
 */

#define NVIC_PR_BASE_ADDR	((__vo uint32_t *)0xE000E280)

/*
 * Arm Cortex Mx  Processor NVIC ICERx Register address.
 */

#define NVIC_ICER0			((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2			((__vo uint32_t*)0xE000E188)

/*
 * Base Addresses of the Memories.
 *
 */

#define FLASH_BASE_ADDRESS 	0x08000000U
#define SRAM1_BASE_ADDRESS 	0x20000000U
#define SRAM2_BASE_ADDRESS 	0x2001C000U

/*
 *BASE ADDRESSES OF VARIOUS BUS PERIPHERALS.
 *
 */

#define  PERIPH_BASE_ADDRESS 	0x40000000U
#define  APB1_BASE_ADDRESS		0x40000000U
#define  APB2_BASE_ADDRESS		APB1_BASE_ADDRESS
#define  AHB1_BASE_ADDRESS		0x40020000U
#define  AHB2_BASE_ADDRESS		0x50000000U


/*
 * BASE ADDRESSES OF VARIOUS GPIO PERIPHERALS WHICH ARE CONNECTED TO AHB1 BUS.
 */

#define GPIOA_BASE_ADDRESS 		(AHB1_BASE_ADDRESS + 0x0000)
#define GPIOB_BASE_ADDRESS 		(AHB1_BASE_ADDRESS + 0x0400)
#define GPIOC_BASE_ADDRESS 		(AHB1_BASE_ADDRESS + 0x0800)
#define GPIOD_BASE_ADDRESS 		(AHB1_BASE_ADDRESS + 0x0C00)
#define GPIOE_BASE_ADDRESS 		(AHB1_BASE_ADDRESS + 0x1000)
#define GPIOF_BASE_ADDRESS 		(AHB1_BASE_ADDRESS + 0x1400)
#define GPIOG_BASE_ADDRESS 		(AHB1_BASE_ADDRESS + 0x1800)
#define GPIOH_BASE_ADDRESS 		(AHB1_BASE_ADDRESS + 0x1C00)
#define GPIOI_BASE_ADDRESS 		(AHB1_BASE_ADDRESS + 0x2000)


/*
 ************************************************************************************************************************************
 *
 *				Peripheral Register  definition Structures
 *
 *				GPIO peripheral Register Definition
 *************************************************************************************************************************************
 */


typedef struct{
	__vo uint32_t MODER;//GPIO port mode register (GPIOx_MODER),Address offset: 0x00
	__vo uint32_t OTYPER;//GPIO port mode register (GPIOx_MODER),Address offset: 0x00
	__vo uint32_t OSPEEDR;//GPIO port mode register (GPIOx_MODER),Address offset: 0x00
	__vo uint32_t PUPDR;//GPIO port mode register (GPIOx_MODER),Address offset: 0x00
	__vo uint32_t IDR;//GPIO port mode register (GPIOx_MODER),Address offset: 0x00
	__vo uint32_t ODR;//GPIO port mode register (GPIOx_MODER),Address offset: 0x00
	__vo uint32_t BSRR;//GPIO port mode register (GPIOx_MODER),Address offset: 0x00
	__vo uint32_t LCKR;//GPIO port mode register (GPIOx_MODER),Address offset: 0x00
	__vo uint32_t AF[2];//GPIO port mode register (GPIOx_MODER),Address offset: 0x00


}GPIO_REG_DEF_T;


typedef struct{
			__vo uint32_t RCC_CR;			/* RCC clock control register					Address offset: 0x00 */
			__vo uint32_t RCC_PLLCFGR;		/* RCC PLL configuration register				Address offset: 0x04 */
			__vo uint32_t RCC_CFGR;			/* RCC clock configuration register				Address offset: 0x08 */
			__vo uint32_t RCC_CIR;			/* RCC clock interrupt register					Address offset: 0x0C */
			__vo uint32_t RCC_AHB1RSTR;		/* RCC AHB1 peripheral reset register			Address offset: 0x10 */
			__vo uint32_t RCC_AHB2RSTR;		/* RCC AHB2 peripheral reset register			Address offset: 0x14 */
			__vo uint32_t RCC_AHB3RSTR;		/* RCC AHB3 peripheral reset register			Address offset: 0x18 */
			uint32_t	  RESERVED0;		/* Reserved, 0x1C */
			__vo uint32_t RCC_APB1RSTR;		/* RCC APB1 peripheral reset register			Address offset: 0x20 */
			__vo uint32_t RCC_APB2RSTR;		/* RCC APB2 peripheral reset register			Address offset: 0x24 */
			uint32_t      RESERVED1[2];		/* Reserved, 0x28 - 0x2C */
			__vo uint32_t RCC_AHB1ENR;		/* RCC AHB1 peripheral clock register			Address offset: 0x30 */
			__vo uint32_t RCC_AHB2ENR;		/* RCC AHB2 peripheral clock enable register	Address offset: 0x34 */
			__vo uint32_t RCC_AHB3ENR;		/* RCC AHB3 peripheral clock enable register	Address offset: 0x38 */
			uint32_t      RESERVED2;		/* Reserved, 0x3C*/
			__vo uint32_t RCC_APB1ENR;		/* RCC APB1 peripheral clock enable register	Address offset: 0x40 */
			__vo uint32_t RCC_APB2ENR;		/* RCC APB2 peripheral clock enable register	Address offset: 0x44 */
			uint32_t      RESERVED3[2];		/* Reserved, 0x48 - 0x4C */
			__vo uint32_t RCC_AHB1LPENR;	/* RCC AHB1 peripheral clock enable in low power mode register		Address offset: 0x50 */
			__vo uint32_t RCC_AHB2LPENR;	/* RCC AHB2 peripheral clock enable in low power mode register		Address offset: 0x54 */
			__vo uint32_t RCC_AHB3LPENR;	/* RCC AHB3 peripheral clock enable in low power mode register		Address offset: 0x58 */
			uint32_t      RESERVED4;		/* Reserved, 0x5C	*/
			__vo uint32_t RCC_APB1LPENR;	/* RCC APB1 peripheral clock enable in low power mode register		Address offset: 0x60 */
			__vo uint32_t RCC_APB2LPENR;	/* RCC APB2 peripheral clock enabled in low power mode register		Address offset: 0x64 */
			uint32_t      RESERVED5[2];		/* Reserved, 0x68 - 0x6C */
			__vo uint32_t RCC_BDCR;			/* RCC Backup domain control register			Address offset: 0x70 */
			__vo uint32_t RCC_CSR;			/* RCC clock control & status register			Address offset: 0x74 */
			uint32_t      RESERVED6[2];		/* Reserved, 0x78 - 0x7C */
			__vo uint32_t RCC_SSCGR;		/* RCC spread spectrum clock generation register	Address offset: 0x80 */
			__vo uint32_t RCC_PLLI2SCFGR;	/* RCC PLLI2S configuration register			Address offset: 0x84 */
			__vo uint32_t RCC_PLLSAICFGR;	/* RCC PLL configuration register				Address offset: 0x88 */
			__vo uint32_t RCC_DCKCFGR;		/* RCC Dedicated Clock Configuration Register   Address offset: 0x8C */


}RCC_REG_DEF_T;

/*
 * Base Address of RCC Peripheral
 */

#define RCC_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0x3800)

/*
 * For interrupt EXTI
 */

#define  EXTI_BASE_ADDRESS	  0x40013C00U
#define  SYSCFG_BASE_ADDRESS  0x40013800U

/*
 * Peripheral Register Definition structure for EXTI
 */
typedef struct{
	__vo uint32_t EXTI_IMR ;//Interrupt mask register
	__vo uint32_t EXTI_EMR ;//Event mask register
	__vo uint32_t EXTI_RTSR	;//Rising trigger selection register
	__vo uint32_t EXTI_FTSR;//Falling trigger selection register
	__vo uint32_t EXTI_SWIER;//Software interrupt event register
	__vo uint32_t EXTI_PR;//Pending register Address

}EXTI_Reg_Def_t;



/*
 * EXTI register Pointer;
 */
#define EXTI  	((EXTI_Reg_Def_t *)EXTI_BASE_ADDRESS)

/*
 * Peripheral Register Definition structure for SYSCFG
 */

typedef struct
{
	__vo uint32_t MEMRMP;       /*!< Give a short description,                    Address offset: 0x00      */
	__vo uint32_t PMC;          /*!< TODO,     									  Address offset: 0x04      */
	__vo uint32_t EXTICR[4];    /*!< TODO , 									  Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2];  /*!< TODO          							  Reserved, 0x18-0x1C    	*/
	__vo uint32_t CMPCR;        /*!< TODO         								  Address offset: 0x20      */
	uint32_t      RESERVED2[2];  /*!<                                             Reserved, 0x24-0x28 	    */
	__vo uint32_t CFGR;         /*!< TODO                                         Address offset: 0x2C   	*/
} SYSCFG_RegDef_t;

#define SYSCFG 		((SYSCFG_RegDef_t *)SYSCFG_BASE_ADDRESS)


/*
 * GPIO PERIPHERAL DEFINITION (Peripheral base address type casted to GPIO_REG_DEF_T)
 */

#define GPIOA   ((GPIO_REG_DEF_T *)GPIOA_BASE_ADDRESS)
#define GPIOB   ((GPIO_REG_DEF_T *)GPIOB_BASE_ADDRESS)
#define GPIOC   ((GPIO_REG_DEF_T *)GPIOC_BASE_ADDRESS)
#define GPIOD   ((GPIO_REG_DEF_T *)GPIOD_BASE_ADDRESS)
#define GPIOE   ((GPIO_REG_DEF_T *)GPIOE_BASE_ADDRESS)
#define GPIOF   ((GPIO_REG_DEF_T *)GPIOF_BASE_ADDRESS)
#define GPIOG   ((GPIO_REG_DEF_T *)GPIOG_BASE_ADDRESS)
#define GPIOH   ((GPIO_REG_DEF_T *)GPIOH_BASE_ADDRESS)
#define GPIOI   ((GPIO_REG_DEF_T *)GPIOI_BASE_ADDRESS)

#define RCC ((RCC_REG_DEF_T *)RCC_BASE_ADDRESS)

/*
 * Clock Enable MACROS Of GPIOX Peripheral
 */

#define GPIOA_PCLK_EN()		(RCC->RCC_AHB1ENR |=(1<<0))
#define GPIOB_PCLK_EN()		(RCC->RCC_AHB1ENR |=(1<<1))
#define GPIOC_PCLK_EN()		(RCC->RCC_AHB1ENR |=(1<<2))
#define GPIOD_PCLK_EN()		(RCC->RCC_AHB1ENR |=(1<<3))
#define GPIOE_PCLK_EN()		(RCC->RCC_AHB1ENR |=(1<<4))
#define GPIOF_PCLK_EN()		(RCC->RCC_AHB1ENR |=(1<<5))
#define GPIOG_PCLK_EN()		(RCC->RCC_AHB1ENR |=(1<<6))
#define GPIOH_PCLK_EN()		(RCC->RCC_AHB1ENR |=(1<<7))
#define GPIOI_PCLK_EN()		(RCC->RCC_AHB1ENR |=(1<<8))

/*
 * Clock Enable MACROS Of GPIOX Peripheral
 */

#define SYSCFG_PCLK_EN()	(RCC->RCC_APB2ENR |=(1<<14))

// For Interrupt APB2 SYSCF_EN
#define APB2_SYS_En()    (RCC->RCC_APB2ENR |=(1<<14));

/*
 * Clock Disable MACROS Of GPIOX Peripheral
 */

#define GPIOA_PCLK_DIS()		(RCC->RCC_AHB1ENR &=~(1<<0))
#define GPIOB_PCLK_DIS()		(RCC->RCC_AHB1ENR &=~(1<<1))
#define GPIOC_PCLK_DIS()		(RCC->RCC_AHB1ENR &=~(1<<2))
#define GPIOD_PCLK_DIS()		(RCC->RCC_AHB1ENR &=~(1<<3))
#define GPIOE_PCLK_DIS()		(RCC->RCC_AHB1ENR &=~(1<<4))
#define GPIOF_PCLK_DIS()		(RCC->RCC_AHB1ENR &=~(1<<5))
#define GPIOG_PCLK_DIS()		(RCC->RCC_AHB1ENR &=~(1<<6))
#define GPIOH_PCLK_DIS()		(RCC->RCC_AHB1ENR &=~(1<<7))
#define GPIOI_PCLK_DIS()		(RCC->RCC_AHB1ENR &=~(1<<8))

/*
 * Macros to reset GPIOX peripheral (do.. while.. condition zero loop)
 */

#define GPIOA_REG_RESET()		 do{(RCC->RCC_AHB1RSTR |= (1<<0)); (RCC->RCC_AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()		 do{(RCC->RCC_AHB1RSTR |= (1<<1)); (RCC->RCC_AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()		 do{(RCC->RCC_AHB1RSTR |= (1<<2)); (RCC->RCC_AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()        do{(RCC->RCC_AHB1RSTR |= (1<<3));  (RCC->RCC_AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()        do{(RCC->RCC_AHB1RSTR |= (1<<4));  (RCC->RCC_AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET()        do{(RCC->RCC_AHB1RSTR |= (1<<5));  (RCC->RCC_AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET()        do{(RCC->RCC_AHB1RSTR |= (1<<6));  (RCC->RCC_AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET()        do{(RCC->RCC_AHB1RSTR |= (1<<7));  (RCC->RCC_AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET()        do{(RCC->RCC_AHB1RSTR |= (1<<8));  (RCC->RCC_AHB1RSTR &= ~(1<<8));}while(0)

/*
 *  Returns The Port Code For GPIOX base Address. This MACRO returns Code between (0 to 7)
 */

#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7: \
								        (x == GPIOI)?8:0)




/*
 * Generic Macros
 */

#define ENABLE  	  1
#define DISABLE 	  0

#define SET			   1
#define RESET		   0

#define GPIO_PIN_SET   SET
#define GPIO_PIN_RESET RESET



#endif /* INC_STM32F407XX_H_ */
