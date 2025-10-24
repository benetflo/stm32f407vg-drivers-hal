#include "hw_gpio.h"

// ARRAY TO STORE IRQ CALLBACK FUNCTIONS FROM APPLICATION
static GPIO_IRQ_Callback_t callbacks[16] = {NULL};

// FUNCTION TO SET IRQ CALLBACK FUNCTION
GPIO_Status_t GPIO_IRQSetCallback(uint8_t GPIO_NUM, GPIO_IRQ_Callback_t callback)
{
	#ifdef DEBUG
		if(GPIO_NUM > 15)
		{
			return GPIO_ERR_INVALID_PIN;
		}
	#endif

	callbacks[GPIO_NUM] = callback;
	return GPIO_OK;
}

void EXTI0_IRQHandler(void)
{
	gpio_irq_handling(0);
    if(callbacks[0] != NULL)
    {
        callbacks[0]();
    }
}

void EXTI1_IRQHandler(void)
{
	gpio_irq_handling(1);
    if(callbacks[1] != NULL)
    {
        callbacks[1]();
    }
}

void EXTI2_IRQHandler(void)
{
	gpio_irq_handling(2);
    if(callbacks[2] != NULL)
    {
        callbacks[2]();
    }
}

void EXTI3_IRQHandler(void)
{
	gpio_irq_handling(3);
    if(callbacks[3] != NULL)
    {
        callbacks[3]();
    }
}

void EXTI4_IRQHandler(void)
{
	gpio_irq_handling(4);
    if(callbacks[4] != NULL)
    {
        callbacks[4]();
    }
}

void EXTI9_5_IRQHandler(void)
{
    for(uint8_t i = 5; i <= 9; i++)
    {
        if(EXTI->PR & (1 << i))  // Pending bit satt?
        {
            gpio_irq_handling(i);  // Cleara interrupt

            if(callbacks[i] != NULL)
            {
                callbacks[i]();
            }
        }
    }
}

void EXTI15_10_IRQHandler(void)
{
    for(uint8_t i = 10; i <= 15; i++)
    {
        if(EXTI->PR & (1 << i))  // Pending bit satt?
        {
            gpio_irq_handling(i);  // Cleara interrupt

            if(callbacks[i] != NULL)
            {
                callbacks[i]();
            }
        }
    }
}


/*
 * GPIO CONFIG FUNCTIONS
 */

GPIO_Status_t GPIO_Init(GPIO_RegDef_t* GPIO_PORT, const uint8_t GPIO_NUM, const uint8_t GPIO_MODE, const uint8_t GPIO_SPEED, const uint8_t GPIO_PUPD, const uint8_t GPIO_OUTPUT_TYPE)
{

	// ADD DEBUG FOR ALL PARAMETERS

	#ifdef DEBUG
		if(GPIO_PORT != PA &&
		   GPIO_PORT != PB &&
		   GPIO_PORT != PC &&
		   GPIO_PORT != PD &&
		   GPIO_PORT != PE)
		{
			// PORT DOES NOT EXIST
			return GPIO_ERR_INVALID_PORT;
		}else if(GPIO_NUM > 15)
		{
			// GPIO NUM DOES NOT EXIST
			return GPIO_ERR_INVALID_PIN;
		}else if(GPIO_MODE != GPIO_IN &&
				 GPIO_MODE != GPIO_OUT &&
				 GPIO_MODE != GPIO_AF &&
				 GPIO_MODE != GPIO_ANALOG)
		{
			// GPIO VALUE DOES NOT EXIST
			return GPIO_ERR_INVALID_MODE;
		}
	#endif

	GPIO_Handle_t gpio;
    gpio.pGPIOx = GPIO_PORT;
    gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_NUM;
    gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE;
    gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED;
    gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE;
    gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD;

    gpio_init(&gpio);
    return GPIO_OK;
}

GPIO_Status_t GPIO_IRQInit(GPIO_RegDef_t* GPIO_PORT, const uint8_t GPIO_NUM, const uint8_t GPIO_MODE, const uint8_t GPIO_SPEED, const uint8_t GPIO_PUPD, const uint8_t GPIO_OUTPUT_TYPE, const uint8_t GPIO_IRQ_PRIORITY)
{

	// ADD DEBUG FOR ALL PARAMETERS

	#ifdef DEBUG
		if(GPIO_PORT != PA &&
		   GPIO_PORT != PB &&
		   GPIO_PORT != PC &&
		   GPIO_PORT != PD &&
		   GPIO_PORT != PE)
		{
			// PORT DOES NOT EXIST
			return GPIO_ERR_INVALID_PORT;
		}else if(GPIO_NUM > 15)
		{
			// GPIO NUM DOES NOT EXIST
			return GPIO_ERR_INVALID_PIN;
		}else if(GPIO_MODE != GPIO_MODE_IT_FT &&
				 GPIO_MODE != GPIO_MODE_IT_RT &&
				 GPIO_MODE != GPIO_MODE_IT_RFT)
		{
			// GPIO VALUE DOES NOT EXIST
			return GPIO_ERR_INVALID_MODE;
		}else if((GPIO_IRQ_PRIORITY < NVIC_IRQ_PRIO0) || (GPIO_IRQ_PRIORITY > NVIC_IRQ_PRIO15))
		{
			return GPIO_ERR_INVALID_IRQ_PRIORITY;
		}
	#endif

	GPIO_Handle_t gpio;
    gpio.pGPIOx = GPIO_PORT;
    gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_NUM;
    gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE;
    gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED;
    gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE;
    gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD;

    gpio_init(&gpio);


    uint8_t IRQNum;

    if(GPIO_NUM == 0)
    {
    	IRQNum = IRQ_NO_EXTI0;
    }else if(GPIO_NUM == 1)
    {
    	IRQNum = IRQ_NO_EXTI1;
    }else if(GPIO_NUM == 2)
    {
    	IRQNum = IRQ_NO_EXTI2;
    }else if(GPIO_NUM == 3)
    {
    	IRQNum = IRQ_NO_EXTI3;
    }else if(GPIO_NUM == 4)
    {
    	IRQNum = IRQ_NO_EXTI4;
    }else if(GPIO_NUM >= 5 && GPIO_NUM <= 9)
    {
    	IRQNum = IRQ_NO_EXTI9_5;
    }else if(GPIO_NUM >= 10 && GPIO_NUM <= 15)
    {
    	IRQNum = IRQ_NO_EXTI15_10;
    }else{
    	return GPIO_ERR_INVALID_PIN;
    }

	gpio_irq_prio_config(IRQNum, GPIO_IRQ_PRIORITY);
	gpio_irq_config(IRQNum, ENABLE);

    return GPIO_OK;
}

GPIO_Status_t GPIO_DeInit(GPIO_RegDef_t* GPIO_PORT)
{

	#ifdef DEBUG
		if(GPIO_PORT != PA &&
		   GPIO_PORT != PB &&
		   GPIO_PORT != PC &&
		   GPIO_PORT != PD &&
		   GPIO_PORT != PE)
		{
			// PORT DOES NOT EXIST
			return GPIO_ERR_INVALID_PORT;
		}
	#endif

    gpio_deinit(GPIO_PORT);
    return GPIO_OK;
}

/*************************************************************************************************************************************
 * GPIO Write functions
 *************************************************************************************************************************************/

GPIO_Status_t GPIO_WritePin(GPIO_RegDef_t *GPIO_PORT, const uint8_t GPIO_NUM, const uint8_t GPIO_VALUE)
{
	#ifdef DEBUG
		if(GPIO_PORT != PA &&
		   GPIO_PORT != PB &&
		   GPIO_PORT != PC &&
		   GPIO_PORT != PD &&
		   GPIO_PORT != PE)
		{
			// PORT DOES NOT EXIST
			return GPIO_ERR_INVALID_PORT;
		}else if(GPIO_NUM > 15)
		{
			// GPIO NUM DOES NOT EXIST
			return GPIO_ERR_INVALID_PIN;
		}else if(GPIO_VALUE != GPIO_HIGH && GPIO_VALUE != GPIO_LOW)
		{
			// GPIO VALUE DOES NOT EXIST
			return GPIO_ERR_INVALID_VALUE;
		}
	#endif

	gpio_write_pin(GPIO_PORT, GPIO_NUM, GPIO_VALUE);
	return GPIO_OK;
}

GPIO_Status_t GPIO_WritePort(GPIO_RegDef_t *GPIO_PORT, const uint8_t GPIO_VALUE)
{
	#ifdef DEBUG
		if(GPIO_PORT != PA &&
		   GPIO_PORT != PB &&
		   GPIO_PORT != PC &&
		   GPIO_PORT != PD &&
		   GPIO_PORT != PE)
		{
			// PORT DOES NOT EXIST
			return GPIO_ERR_INVALID_PORT;
		}else if(GPIO_VALUE != GPIO_HIGH && GPIO_VALUE != GPIO_LOW)
		{
			// GPIO VALUE DOES NOT EXIST
			return GPIO_ERR_INVALID_VALUE;
		}
	#endif

	gpio_write_port(GPIO_PORT, GPIO_VALUE);
	return GPIO_OK;
}

GPIO_Status_t GPIO_TogglePin(GPIO_RegDef_t *GPIO_PORT, const uint8_t GPIO_NUM)
{
	#ifdef DEBUG
		if(GPIO_PORT != PA &&
		   GPIO_PORT != PB &&
		   GPIO_PORT != PC &&
		   GPIO_PORT != PD &&
		   GPIO_PORT != PE)
		{
			// PORT DOES NOT EXIST
			return GPIO_ERR_INVALID_PORT;
		}else if(GPIO_NUM > 15)
		{
			// GPIO NUM DOES NOT EXIST
			return GPIO_ERR_INVALID_PIN;
		}
	#endif

	gpio_toggle_pin(GPIO_PORT, GPIO_NUM);
	return GPIO_OK;
}


/*************************************************************************************************************************************
 * GPIO Read functions
 *************************************************************************************************************************************/

uint8_t GPIO_ReadPin(GPIO_RegDef_t *GPIO_PORT, const uint8_t GPIO_NUM)
{
	#ifdef DEBUG
		if(GPIO_PORT != PA &&
		   GPIO_PORT != PB &&
		   GPIO_PORT != PC &&
		   GPIO_PORT != PD &&
		   GPIO_PORT != PE)
		{
			// PORT DOES NOT EXIST
			return GPIO_ERR_INVALID_PORT;
		}else if(GPIO_NUM > 15)
		{
			// GPIO NUM DOES NOT EXIST
			return GPIO_ERR_INVALID_PIN;
		}
	#endif

	uint8_t byte_read = gpio_read_pin(GPIO_PORT, GPIO_NUM);

	return byte_read;
}

uint16_t GPIO_ReadPort(GPIO_RegDef_t *GPIO_PORT)
{
	#ifdef DEBUG
		if(GPIO_PORT != PA &&
		   GPIO_PORT != PB &&
		   GPIO_PORT != PC &&
		   GPIO_PORT != PD &&
		   GPIO_PORT != PE)
		{
			// PORT DOES NOT EXIST
			return GPIO_ERR_INVALID_PORT;
		}
	#endif

	uint16_t bytes_read = gpio_read_port(GPIO_PORT);

	return bytes_read;
}
