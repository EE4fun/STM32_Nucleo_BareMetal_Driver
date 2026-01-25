#ifndef INC_STM32401XX_H_
#define INC_STM32401XX_H_
#include <stdint.h>
#include <stdio.h>

#define __vo   volatile


/********************** START: Processor Specific Details ***********************************
 * ARN Cortex Mx Processor NVIC ISERx register Addresses
 * Doc: ARM  DUI553 Table 4-2
 */
/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10C )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)
/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)  /* ARM doc dui0553, p219 */

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/*
 * base addresses of FLASH and SRAM  main flash memory from RM0368
 */
#define FLASH_BASEADDR  0x08000000U  // FLASH MEMORY BASE ADDRESS
#define SRAM1_BASEADDR  0x30000000U
#define SRAM2_BASEADDR  0x30004000U
#define ROM_BASEADDR	0x1FF00000U  //SYSTEM MEMORY
#define SRAM            SRAM1_BASEADDR


/*
 *  AHBx and APBx Bus Peripheral base addresses
 *
 */
#define PERIPH_BASE       0x40000000U            /* peripheral base address from RM0368 */
#define APB1PERIPH_BASE   PERIPH_BASE
#define APB2PERIPH_BASE   0x40010000U
#define AHB1PERIPH_BASE   0x40020000U
#define AHB2PERIPH_BASE   0x50000000U


/*
 * BASE ADDRESS OF PERIPHERAL HANGING ON THE AHB4 BUS
 */
#define GPIOA_BASEADDR   (AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASEADDR   (AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASEADDR   (AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASEADDR   (AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASEADDR   (AHB1PERIPH_BASE + 0x1000U)
#define GPIOH_BASEADDR   (AHB1PERIPH_BASE + 0x1C00U)

#define RCC_BASEADDR     (AHB1PERIPH_BASE + 0x3800U)

/*
 * Base addresses of peripherals which are hanging o APB1 bus
 * TODO: Complete for all other peripherals
 */


#define I2C1_BASEADDR   (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR   (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR   (APB1PERIPH_BASE + 0x5C00)
#define SPI2_BASEADDR   (APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASEADDR   (APB1PERIPH_BASE + 0x3C00U)

#define USART2_BASEADDR (APB1PERIPH_BASE + 0x4400U)


/*
 * Base addresses of peripherals which are hanging o APB2 bus
 * TODO: Complete for all other peripherals
 */
#define EXTI_BASEADDR     (APB2PERIPH_BASE + 0x3C00U)
#define SYSCFG_BASEADDR   (APB2PERIPH_BASE + 0x3800U)

#define SPI1_BASEADDR     (APB2PERIPH_BASE + 0x3000U)
#define SPI4_BASEADDR     (APB2PERIPH_BASE + 0x3400U)

#define USART1_BASEADDR   (APB2PERIPH_BASE + 0x1000U)
#define USART6_BASEADDR   (APB2PERIPH_BASE + 0x1400U)


/*
 *  Note: Registers of a peripheral are specific to MCU
 *
 */

typedef struct
{
	__vo uint32_t	MODER;                   /* GPIO port mode register                    */
	__vo uint32_t	OTYPER;                   /* GPIO port mode register                    */
	__vo uint32_t	OSPEEDR;                   /* GPIO port mode register                    */
	__vo uint32_t	PUPDR;                   /* GPIO port mode register                    */
	__vo uint32_t	IDR;                   /* GPIO port mode register                    */
	__vo uint32_t	ODR;                   /* GPIO port mode register                    */
	__vo uint32_t	BSRR;                   /* GPIO port mode register                    */
	__vo uint32_t	LCKR;                   /* GPIO port mode register                    */
	__vo uint32_t	AFR[2];                   /* GPIO port alternate function low register.                    */
}GPIO_RegDef_t;

/*
 *  Note: Registers of a peripheral are specific to MCU : SPI
 *
 */

typedef struct
{
	    __vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
		__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
		__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x08 */
		__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x0C */
		__vo uint32_t CRCPR;      /*!< TODO,     										Address offset: 0x10 */
		__vo uint32_t RXCRCR;     /*!< TODO,     										Address offset: 0x14 */
		__vo uint32_t TXCRCR;     /*!< TODO,     										Address offset: 0x18 */
		__vo uint32_t I2SCFGR;    /*!< TODO,     										Address offset: 0x1C */
		__vo uint32_t I2SPR;      /*!< TODO,     										Address offset: 0x20 */
}SPI_RegDef_t;


typedef struct
{
	__vo uint32_t	CR;                   /* RCC port   register                    */
	__vo uint32_t	PLLCFGR;                   /* GPIO port mode register                    */
	__vo uint32_t	CFGR;                   /* GPIO port mode register                    */
	__vo uint32_t	CIR;                   /* RCC port   register                    */
	__vo uint32_t	AHB1RSTR;                   /* RCC port   register                    */
	__vo uint32_t	AHB2RSTR;                   /* RCC port   register                    */
	     uint32_t	RESERVED0;                   /* RCC port   register                    */
	     uint32_t	RESERVED1;                   /* RCC port   register                    */
	__vo uint32_t	APB1RSTR;                   /* RCC port   register                    */
	__vo uint32_t	APB2RSTR;                   /* RCC port   register                    */
	     uint32_t	RESERVED2;                   /* RCC port   register                    */
	     uint32_t	RESERVED3;                   /* RCC port   register                    */
	__vo uint32_t	AHB1ENR;                   /* RCC port   register                    */
	__vo uint32_t	AHB2ENR;                   /* RCC port   register                    */
	     uint32_t	RESERVED4;                   /* RCC port   register                    */
	     uint32_t	RESERVED5;                   /* RCC port   register                    */
	__vo uint32_t	APB1ENR;                   /* RCC port   register                    */
	__vo uint32_t	APB2ENR;                   /* RCC port   register                    */
	     uint32_t	RESERVED6;                   /* RCC port   register                    */
	     uint32_t	RESERVED7;                   /* RCC port   register                    */
	__vo uint32_t	AHB1LPENR;                   /* RCC port   register                    */
	__vo uint32_t	AHB2LPENR;                   /* RCC port   register                    */
	__vo uint32_t	RESERVED8;                   /* RCC port   register                    */
	     uint32_t	RESERVED9;                   /* RCC port   register                    */
	__vo uint32_t	APB1LPENR;                   /* RCC port   register                    */
	__vo uint32_t	APB2LPENR;                   /* RCC port   register                    */
	     uint32_t	RESERVED10;                   /* RCC port   register                    */
	     uint32_t	RESERVED11;                   /* RCC port   register                    */
	__vo uint32_t	BDCR;                   /* RCC port   register                    */
	__vo uint32_t	CSR;                   /* RCC port   register                    */
	     uint32_t	RESERVED12;                   /* RCC port   register                    */
	     uint32_t	RESERVED13;                   /* RCC port   register                    */
	__vo uint32_t	SSCGR;                   /* RCC port   register                    */
	__vo uint32_t	PLLI2SCFGR;                   /* RCC port   register                    */
	__vo uint32_t	DCKCFGR;                   /* RCC Dedicated Clocks configuration register,                 Address offset: 0x8C                  */
} RCC_RefDef_t;

/*
 * Interrupt definition : RM0368, 10.3
 */
typedef struct
{
	__vo uint32_t IMR;   /* EXTI port   register */
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;

/*
 * Peripheral definition for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;   /* SYSCFG register 0x00 */
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;

/*
 * peripheral definition
 */
#define  GPIOA        ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define  GPIOB        ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define  GPIOC        ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define  GPIOD        ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define  GPIOE        ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define  GPIOH        ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define  RCC          ((RCC_RefDef_t*)RCC_BASEADDR)
#define  EXTI         ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define  SYSCFG       ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define  SPI1         ((SPI_RegDef_t*)SPI1_BASEADDR)
#define  SPI2         ((SPI_RegDef_t*)SPI2_BASEADDR)
#define  SPI3         ((SPI_RegDef_t*)SPI4_BASEADDR)
#define  SPI4         ((SPI_RegDef_t*)SPI4_BASEADDR)
/*
 * Clock enable Macros for GPIOx peripherals
 *
 */
#define GPIOA_PCLK_EN()           ( RCC->AHB1ENR  |=(1<<0) )
#define GPIOB_PCLK_EN()           ( RCC->AHB1ENR  |=(1<<1) )
#define GPIOC_PCLK_EN()           ( RCC->AHB1ENR  |=(1<<2) )
#define GPIOD_PCLK_EN()           ( RCC->AHB1ENR  |=(1<<3) )
#define GPIOE_PCLK_EN()           ( RCC->AHB1ENR  |=(1<<4) )
#define GPIOH_PCLK_EN()           ( RCC->AHB1ENR  |=(1<<7) )

/*
 * Clock enable Macros for SPIx peripherals
 *
 */
#define SPI1_PCLK_EN()            ( RCC->APB2ENR  |=(1<<12) )
#define SPI2_PCLK_EN()            ( RCC->APB1ENR  |=(1<<14) )
#define SPI3_PCLK_EN()            ( RCC->APB1ENR  |=(1<<15) )
#define SPI4_PCLK_EN()            ( RCC->APB2ENR  |=(1<<13) )



/*
 * Clock enable Macros for  SYSCFG peripherals
 * dm00096844.pdf:
 */
#define SYSCFG_PCLK_EN()           ( RCC->APB2ENR  |=(1<<14) )
/*
 * Clock disable Macros for GPIOx peripherals
 * dm00096844.pdf: 6.3.5
 */
#define GPIOA_REG_RESET()    do{ ( RCC->AHB1RSTR |= (1<<0) ); ( RCC->AHB1RSTR &= ~(1<<0) ); }while (0)
#define GPIOB_REG_RESET()    do{ ( RCC->AHB1RSTR |= (1<<1) ); ( RCC->AHB1RSTR &= ~(1<<1) ); }while (0)
#define GPIOC_REG_RESET()    do{ ( RCC->AHB1RSTR |= (1<<2) ); ( RCC->AHB1RSTR &= ~(1<<2) ); }while (0)
#define GPIOD_REG_RESET()    do{ ( RCC->AHB1RSTR |= (1<<3) ); ( RCC->AHB1RSTR &= ~(1<<3) ); }while (0)
#define GPIOE_REG_RESET()    do{ ( RCC->AHB1RSTR |= (1<<4) ); ( RCC->AHB1RSTR &= ~(1<<4) ); }while (0)
#define GPIOH_REG_RESET()    do{ ( RCC->AHB1RSTR |= (1<<7) ); ( RCC->AHB1RSTR &= ~(1<<7) ); }while (0)

#define GPIO_BASEADDR_TO_CODE(x)  ( (x == GPIOA) ? 0 : \
		                            (x == GPIOB) ? 1 : \
		                           	(x == GPIOC) ? 2 : \
		                     		(x == GPIOD) ? 3 : \
		                     		(x == GPIOE) ? 4 : \
		                     		(x == GPIOH) ? 7 : 0 )\


/*
 * Clock disable Macros for SPIx peripherals
 * dm00096844.pdf: 6.3.5  Table 22
 */
#define SPI1_REG_RESET()    do{ ( RCC->APB1RSTR |= (1<<12) ); ( RCC->APB1RSTR &= ~(1<<12) ); }while (0)
#define SPI2_REG_RESET()    do{ ( RCC->APB1RSTR |= (1<<14) ); ( RCC->APB1RSTR &= ~(1<<14) ); }while (0)
#define SPI3_REG_RESET()    do{ ( RCC->APB1RSTR |= (1<<15) ); ( RCC->APB1RSTR &= ~(1<<15) ); }while (0)
#define SPI4_REG_RESET()    do{ ( RCC->APB1RSTR |= (1<<13) ); ( RCC->APB1RSTR &= ~(1<<13) ); }while (0)

#define SPI_BASEADDR_TO_CODE(x)  ( (x == SPI1) ? 0 : \
		                            (x == SPI2) ? 1 : \
		                           	(x == SPI3) ? 2 : \
		                     		(x == SPI4) ? 3 : 0 )\

/*
 * I2C peripheral clock
 */

//some generic macros

#define  ENABLE            1
#define  DISABLE           0
#define  SET               ENABLE
#define  RESET             DISABLE
#define  GPIO_PIN_SET      SET
#define  GPIO_PIN_RESET    RESET
#define  FLAG_RESET        RESET
#define  FLAG_SET          SET


/*
 * @ IRQ(Interrupt Request Number of STM32F401x MCU
 *
 * Ref: dm00096844 Table 38
 */
#define IRQ_NO_EXTI0      6
#define IRQ_NO_EXTI1      7
#define IRQ_NO_EXTI2      8
#define IRQ_NO_EXTI3      9
#define IRQ_NO_EXTI4      10
#define IRQ_NO_EXTI9_5    23
#define IRQ_NO_EXTI15_10  40

/*
 * @ IRQ Priority STM32F401x MCU
 *
 * Ref: dm00096844
 */
#define NVIC_IRQ_PRI0     0
#define NVIC_IRQ_PRI1     1
#define NVIC_IRQ_PRI2     2
#define NVIC_IRQ_PRI3     3
#define NVIC_IRQ_PRI4     4
#define NVIC_IRQ_PRI5     5
#define NVIC_IRQ_PRI6     6
#define NVIC_IRQ_PRI7     7
#define NVIC_IRQ_PRI8     8
#define NVIC_IRQ_PRI9     9
#define NVIC_IRQ_PRI10    10
#define NVIC_IRQ_PRI11    11
#define NVIC_IRQ_PRI12    12
#define NVIC_IRQ_PRI13    13
#define NVIC_IRQ_PRI14    14
#define NVIC_IRQ_PRI15    15

/*
 * @ Bit position definitions of SPI peripheral for STM32F401x MCU
 *
 * Ref: dm00096844 20.5.1
 */
#define SPI_CP1_CPHA		0
#define SPI_CP1_CPOL		1
#define SPI_CP1_MSTR		2
#define SPI_CP1_BR			3
#define SPI_CP1_SSM			9
#define SPI_CP1_RXONLY		10
#define SPI_CP1_DFF			11
#define SPI_CP1_BIDIOE		14
#define SPI_CP1_BIDIMODE	15

/*
 * @ Bit position definitions of SPI peripheral for STM32F401x MCU
 *
 * Ref: dm00096844 20.5.1
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIR		7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRC_NEXT	12
#define SPI_CR1_CRC_EN		13
#define SPI_CR1_BIDI_OE		14
#define SPI_CR1_BIDI_M		15
#endif /*INC_STM32401XX_H_*/
