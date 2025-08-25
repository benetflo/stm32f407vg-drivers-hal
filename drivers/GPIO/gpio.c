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


void gpio_irq_config(uint8_t irq_num, uint8_t irq_priority, uint8_t EnOrDi){

}

void gpio_irq_handling(uint8_t pin_num){

}
