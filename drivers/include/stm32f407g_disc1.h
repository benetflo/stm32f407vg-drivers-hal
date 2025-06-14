#ifndef STM32F407G_DISC1_H
#define STM32F407G_DISC1_H

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR 				0x08000000U   	//
#define SRAM1_BASEADDR 				0x20000000U   	//
#define SRAM2_BASEADDR              		0x2001C000U   	//
#define ROM_BASEADDR                		0x1FFF0000U   	//system memory
#define SRAM 					SRAM1_BASEADDR	//SRAM1 used as standard

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE				0x40000000U
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

/*
 * Base addresses of peripherals which are on the APB1 bus
 */

#define I2C1_BASEADDR				0x40005400U
#define I2C2_BASEADDR				0x40005800U
#define I2C3_BASEADDR				0x40005C00U

#define SPI2_BASEADDR				0x40003800U
#define SPI3_BASEADDR				0x40003C00U

#define USART2_BASEADDR				0x40004400U
#define USART3_BASEADDR				0x40004800U

#define UART4_BASEADDR				0x40004C00U
#define UART5_BASEADDR				0x40005000U


/*
 * Base addresses of peripherals which are on the APB2 bus
 */

#define SPI1_BASEADDR				0x40013000U

#define USART1_BASEADDR				0x40011000U
#define USART6_BASEADDR				0x40011400U

#define EXTI_BASEADDR				0x40013C00U
#define SYSCFG_BASEADDR				0x40013800U

#endif
