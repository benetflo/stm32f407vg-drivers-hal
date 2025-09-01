#include "stm32f407g-disc1_gpio_driver.h"

/*****************************************************************
 * @fn
 *
 */

void gpio_peri_clk_control(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	}else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}
	}
}


void gpio_init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp = 0;

	// configure mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		//Non interrupt modes
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear bit
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;

	}else{
		//interrupt modes
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR bit
			EXTI->RTSR &= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding FTSR bit
			EXTI->FTSR &= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);
		// Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;

	//2. configure speed

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear bit
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//3. configure pull up pull down settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear bit
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear bit
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;
	//5. configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_AF){
		//configure the alternate function registers
		uint8_t afreg_low_or_high, afreg_bit_pos;

		afreg_low_or_high = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8);
		afreg_bit_pos = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);
		pGPIOHandle->pGPIOx->AFR[afreg_low_or_high] &= ~(0xF << (4 * afreg_bit_pos));
		pGPIOHandle->pGPIOx->AFR[afreg_low_or_high] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * afreg_bit_pos);
	}
}

void gpio_deinit(GPIO_RegDef_t *pGPIOx){
	// reset all registers of given port (RCC_AHB1RSTR)
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}
}


uint8_t gpio_read_pin(GPIO_RegDef_t *pGPIOx, uint8_t pin_num){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> pin_num) & 0x00000001 ); // shift position to LSB of IDR REG
	return value;
}

uint16_t gpio_read_port(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

void gpio_write_pin(GPIO_RegDef_t *pGPIOx, uint8_t pin_num, uint8_t value){
	if(value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << pin_num);
	}else{
		pGPIOx->ODR &= ~(1 << pin_num);
	}
}

void gpio_write_port(GPIO_RegDef_t *pGPIOx, uint16_t value){
	pGPIOx->ODR = value;
}

void gpio_toggle_pin(GPIO_RegDef_t *pGPIOx, uint8_t pin_num){
	pGPIOx->ODR ^= ( 1 << pin_num);
}


void gpio_irq_config(uint8_t irq_num, uint8_t EnOrDi){
	// PROCESSOR SPECIFIC
	if(EnOrDi == ENABLE){
		if(irq_num <= 31){
			// ISER0 register
			*NVIC_ISER0 |= (1 << irq_num);
		}else if(irq_num > 31 && irq_num < 64){
			// ISER1 register
			*NVIC_ISER1 |= (1 << irq_num % 32);
		}else if(irq_num >= 64 && irq_num < 96){
			// ISER2 register
			*NVIC_ISER3 |= (1 << irq_num % 64);
		}
	}else{
		if(irq_num <= 31){
			// ICER0 register
			*NVIC_ICER0 |= (1 << irq_num);
		}else if(irq_num > 31 && irq_num < 64){
			// ICER1 register
			*NVIC_ICER1 |= (1 << irq_num % 32);
		}else if(irq_num >= 64 && irq_num < 96){
			// ICER2 register
			*NVIC_ICER3 |= (1 << irq_num % 64);
		}
	}
}

void gpio_irq_prio_config(uint8_t irq_num, uint32_t irq_prio){
	//1. find out the ipr register
	uint8_t iprx = irq_num / 4;
	uint8_t iprx_section = irq_num % 4;

	// FIRST 4 BITS ARE NOT IMPLEMENTED SO NEED TO SHIFT
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (irq_prio << shift_amount);
}

// called in application code for ISR function. enter specific pin
void gpio_irq_handling(uint8_t pin_num){
	//clear the EXTI PR REGISTER corresponding to the pin number
	if(EXTI->PR & (1 << pin_num)){
		// clear
		EXTI->PR |= (1 << pin_num);
	}
}
