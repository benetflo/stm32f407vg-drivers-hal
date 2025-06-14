#ifndef INC_STM32F407G_DISC1_H_
#define INC_STM32F407G_DISC1_H_

#include <stdint.h>

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR 				0x08000000U   	//
#define SRAM1_BASEADDR 				0x20000000U   	//
#define SRAM2_BASEADDR        0x2001C000U   	//
#define ROM_BASEADDR          0x1FFF0000U   	//system memory
#define SRAM 						      SRAM1_BASEADDR	//SRAM1 used as standard

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE					  0x40000000U
#define APB1PERIPH_BASE				PERIPH_BASE		// base address of APB1 BUS
#define APB2PERIPH_BASE				0x40010000U		// base address of APB2 BUS
#define AHB1PERIPH_BASE				0x40020000U     // base address of AHB1 BUS
#define AHB2PERIPH_BASE				0x50000000U		// base address of AHB2 BUS

/*
 * Base addresses of peripherals which are on the AHB1 bus
 */

#define GPIOA_BASEADDR				0x40020000U		// GPIO PORT A
#define GPIOB_BASEADDR				0x40020400U		// GPIO PORT B
#define GPIOC_BASEADDR				0x40020800U		// GPIO PORT C
#define GPIOD_BASEADDR				0x40020C00U		// GPIO PORT D
#define GPIOE_BASEADDR				0x40021000U 	// GPIO PORT E
#define GPIOF_BASEADDR				0x40021400U		// GPIO PORT F
#define GPIOG_BASEADDR				0x40021800U		// GPIO PORT G
#define GPIOH_BASEADDR				0x40021C00U		// GPIO PORT H
#define GPIOI_BASEADDR				0x40022000U		// GPIO PORT I

#define RCC_BASEADDR				  0x40023800U

/*
 * Base addresses of peripherals which are on the APB1 bus
 */

#define I2C1_BASEADDR				  0x40005400U
#define I2C2_BASEADDR				  0x40005800U
#define I2C3_BASEADDR				  0x40005C00U

#define SPI2_BASEADDR				  0x40003800U
#define SPI3_BASEADDR				  0x40003C00U

#define USART2_BASEADDR				0x40004400U
#define USART3_BASEADDR				0x40004800U

#define UART4_BASEADDR				0x40004C00U
#define UART5_BASEADDR				0x40005000U


/*
 * Base addresses of peripherals which are on the APB2 bus
 */

#define SPI1_BASEADDR				  0x40013000U

#define USART1_BASEADDR				0x40011000U
#define USART6_BASEADDR				0x40011400U

#define EXTI_BASEADDR				  0x40013C00U
#define SYSCFG_BASEADDR				0x40013800U

/*
 * TODO: COMMENT
 */

typedef struct
{
	volatile uint32_t MODER;				// GPIO port mode register
	volatile uint32_t OTYPER;				// GPIO port output type register
	volatile uint32_t OSPEEDR;				// GPIO port output speed register
	volatile uint32_t PUPDR;				// GPIO port pull-up/pull-down register
	volatile uint32_t IDR;					// GPIO port input data register
	volatile uint32_t ODR;					// GPIO port output data register
	volatile uint32_t BSRRL;
	volatile uint32_t BSRRH;
	volatile uint32_t LCKR;					// GPIO port configuration lock register
	volatile uint32_t AFR[2];				/*
									   	   	   GPIO alternate function low register  [0]
									   	   	   GPIO alternate function high register [1]
											*/
} GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR;					// RCC clock control register
	volatile uint32_t PLLCFGR;				// RCC PLL configuration register
	volatile uint32_t CFGR;					// RCC clock configuration register
	volatile uint32_t CIR;					// RCC clock interrupt register
	volatile uint32_t AHB1RSTR;				// RCC AHB1 peripheral reset register
	volatile uint32_t AHB2RSTR;				// RCC AHB2 peripheral reset register
	volatile uint32_t AHB3RSTR;				// RCC AHB3 peripheral reset register
	uint32_t		  RESERVED0;
	volatile uint32_t APB1RSTR;				// RCC APB1 peripheral reset register
	volatile uint32_t APB2RSTR;				// RCC APB2 peripheral reset register
	uint32_t	      RESERVED1[2];
	volatile uint32_t AHB1ENR;				// RCC AHB1 peripheral clock register
	volatile uint32_t AHB2ENR;				// RCC AHB2 peripheral clock enable register
	volatile uint32_t AHB3ENR;				// RCC AHB3 peripheral clock enable register
	uint32_t		  RESERVED2;
	volatile uint32_t APB1ENR;				// RCC APB1 peripheral clock enable register
	volatile uint32_t APB2ENR;				// RCC APB2 peripheral clock enable register
	uint32_t		  RESERVED3[2];
	volatile uint32_t AHB1LPENR;			// RCC AHB1 peripheral clock enable in low power mode register
	volatile uint32_t AHB2LPENR;			// RCC AHB2 peripheral clock enable in low power mode register
	volatile uint32_t AHB3LPENR;			// RCC AHB3 peripheral clock enable in low power mode register
	uint32_t		  RESERVED4;
	volatile uint32_t APB1LPENR;			// RCC APB1 peripheral clock enable in low power mode register
	volatile uint32_t APB2LPENR;			// RCC APB2 peripheral clock enabled in low power mode register
	uint32_t		  RESERVED5[2];
	volatile uint32_t BDCR;					// RCC Backup domain control register
	volatile uint32_t CSR;					// RCC clock control & status register
	uint32_t		  RESERVED6[2];
	volatile uint32_t SSCGR;				// RCC spread spectrum clock generation register
	volatile uint32_t PLLI2SCFGR;			// RCC PLLI2S configuration register
	volatile uint32_t PLLSAICFGR;			// RCC PLL configuration register
	volatile uint32_t DCKCFGR;				// RCC Dedicated Clock Configuration Register


} RCC_RegDef_t;




/*
 * Peripheral definitions
 */

#define GPIOA ((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *) GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *) GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *) GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *) GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *) GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *) GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *) GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t *) GPIOI_BASEADDR)

#define RCC   ((RCC_RegDef_t *) RCC_BASEADDR)

/*
 * Clock enable macros for GPIOx, I2Cx, SPIx, USARTx, SYSCFG peripherals
 */

#define GPIOA_PCLK_EN()      ( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()		   ( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()		   ( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()		   ( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()		   ( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()		   ( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()		   ( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()		   ( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()		   ( RCC->AHB1ENR |= (1 << 8) )

#define I2C1_PCLK_EN()		   ( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()		   ( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()		   ( RCC->APB1ENR |= (1 << 23) )

#define SPI1_PCLK_EN()		   ( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()		   ( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()		   ( RCC->APB1ENR |= (1 << 15) )

#define USART1_PCLK_EN()	   ( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN()	   ( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()	   ( RCC->APB1ENR |= (1 << 18) )
#define USART6_PCLK_EN()	   ( RCC->APB2ENR |= (1 << 5) )

#define SYSCFG_PCLK_EN()     ( RCC->APB2ENR |= (1 << 14) )

/*
 * Clock disable macros for GPIOx, I2Cx, SPIx, USARTx, SYSCFG peripherals
 */

#define GPIOA_PCLK_DI()      ( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()		   ( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()		   ( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()		   ( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()		   ( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()		   ( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()		   ( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()		   ( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()		   ( RCC->AHB1ENR &= ~(1 << 8) )

#define I2C1_PCLK_DI()		   ( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()		   ( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()		   ( RCC->APB1ENR &= ~(1 << 23) )

#define SPI1_PCLK_DI()		   ( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()		   ( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()		   ( RCC->APB1ENR &= ~(1 << 15) )

#define USART1_PCLK_DI()	   ( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCLK_DI()	   ( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI()	   ( RCC->APB1ENR &= ~(1 << 18) )
#define USART6_PCLK_DI()	   ( RCC->APB2ENR &= ~(1 << 5) )

#define SYSCFG_PCLK_DI()     ( RCC->APB2ENR &= ~(1 << 14) )

#endif
