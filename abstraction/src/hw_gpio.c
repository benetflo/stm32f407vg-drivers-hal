#include "hw_gpio.h"
#include <string.h>

// ARRAY TO STORE IRQ CALLBACK FUNCTIONS FROM APPLICATION
// Each index represents the corresponding EXTI line :)
static GPIO_IRQ_Callback_t callbacks[16] = {NULL};

// FUNCTION TO SET IRQ CALLBACK FUNCTION
GPIO_Status_t GPIO_IRQSetCallback(uint8_t GPIO_PIN, GPIO_IRQ_Callback_t callback_func)
{
	#ifdef DEBUG
		if(GPIO_PIN > 15)
		{
			return GPIO_ERR_INVALID_PIN;
		}
	#endif

	callbacks[GPIO_PIN] = callback_func;
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

/**
 * @brief Initialize a GPIO pin in standard mode
 * @param GPIO_PORT Pointer to GPIO port register: PA,PB,PC,PD,PE
 * @param GPIO_PIN Pin number to GPIO: 0-15
 * @param GPIO_MODE Mode of GPIO: GPIO_IN, GPIO_OUT, GPIO_AF, GPIO_ANALOG
 * @param GPIO_SPEED Speed of GPIO: GPIO_SPEED_LOW, GPIO_SPEED_MEDIUM, GPIO_SPEED_FAST, GPIO_SPEED_HIGH
 * @param GPIO_PUPD GPIO pull up, pull down config: GPIO_PUPD_DISABLED, GPIO_PULL_UP, GPIO_PULL_DOWN
 * @param GPIO_OUTPUT_TYPE Output type of GPIO: GPIO_OPEN_DRAIN, GPIO_PUSH_PULL, GPIO_OP_TYPE_DISABLED
 * @param GPIO_AF_NUM Alternate function number if GPIO is in AF mode: GPIO_AF_NONE, GPIO_AFx (x = (0->15))
 */
GPIO_Status_t GPIO_Init(GPIO_RegDef_t* GPIO_PORT, const uint8_t GPIO_PIN, const uint8_t GPIO_MODE, const uint8_t GPIO_SPEED, const uint8_t GPIO_PUPD, const uint8_t GPIO_OUTPUT_TYPE, uint8_t GPIO_AF_NUM)
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
		else if(GPIO_PIN > 15)
		{
			// GPIO PIN DOES NOT EXIST
			return GPIO_ERR_INVALID_PIN;
		}
		else if(GPIO_MODE != GPIO_IN &&
				 GPIO_MODE != GPIO_OUT &&
				 GPIO_MODE != GPIO_AF &&
				 GPIO_MODE != GPIO_ANALOG)
		{
			// GPIO VALUE DOES NOT EXIST
			return GPIO_ERR_INVALID_MODE;
		}
		else if(GPIO_SPEED != GPIO_SPEED_LOW &&
				GPIO_SPEED != GPIO_SPEED_MEDIUM &&
				GPIO_SPEED != GPIO_SPEED_FAST &&
				GPIO_SPEED != GPIO_SPEED_HIGH)
		{
			return GPIO_ERR_INVALID_SPEED;
		}
		else if(GPIO_PUPD != GPIO_PUPD_DISABLED &&
				GPIO_PUPD != GPIO_PULL_UP &&
				GPIO_PUPD != GPIO_PULL_DOWN)
		{
			return GPIO_ERR_INVALID_PUPD;
		}
		else if(GPIO_OUTPUT_TYPE != GPIO_OPEN_DRAIN &&
				GPIO_OUTPUT_TYPE != GPIO_PUSH_PULL &&
				GPIO_OUTPUT_TYPE != GPIO_OP_TYPE_DISABLED)
		{
			return GPIO_ERR_INVALID_OUTPUT_TYPE;
		}
		else if(GPIO_AF_NUM > GPIO_AF15 && GPIO_AF_NUM != GPIO_AF_NONE)
		{
			return GPIO_ERR_INVALID_AF_NUM;
		}
	#endif

	GPIO_Handle_t gpio;
	memset(&gpio,0,sizeof(gpio));

	gpio.pGPIOx = GPIO_PORT;
	gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN;
	gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE;
	gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED;
	gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD;
	gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE;

	// GPIO AF MODE
	if(GPIO_MODE == GPIO_MODE_AF)
	{
		if(GPIO_AF_NUM == GPIO_AF_NONE)
		{
			return GPIO_ERR_INVALID_GPIO_CONFIG;    // GPIO_AF_NUM REQUIRED
		}

		gpio.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF_NUM;
	}

    gpio_init(&gpio);
    return GPIO_OK;
}

/**
 * @brief Initialize a GPIO pin in interrupt mode
 * @param GPIO_PORT Pointer to GPIO port register: PA,PB,PC,PD,PE
 * @param GPIO_PIN Pin number to GPIO: 0-15
 * @param IRQ_MODE IRQ mode of GPIO: IRQ_MODE_FALLING, IRQ_MODE_RISING, IRQ_MODE_FALLRISE
 * @param GPIO_SPEED Speed of GPIO: GPIO_SPEED_LOW, GPIO_SPEED_MEDIUM, GPIO_SPEED_FAST, GPIO_SPEED_HIGH
 * @param GPIO_PUPD GPIO pull up, pull down config: GPIO_PUPD_DISABLED, GPIO_PULL_UP, GPIO_PULL_DOWN
 * @param GPIO_IRQ_PRIORITY IRQ priority for current pin: 0-15
 * @param CALLBACK_FUNC application function that will run when IRQ is triggered
 */
GPIO_Status_t GPIO_IRQInit(GPIO_RegDef_t* GPIO_PORT, const uint8_t GPIO_PIN, const uint8_t IRQ_MODE, const uint8_t GPIO_SPEED, const uint8_t GPIO_PUPD, const uint8_t GPIO_IRQ_PRIORITY, GPIO_IRQ_Callback_t CALLBACK_FUNC)
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
		else if(GPIO_PIN > 15)
		{
			// GPIO PIN DOES NOT EXIST
			return GPIO_ERR_INVALID_PIN;
		}
		else if(IRQ_MODE != IRQ_MODE_FALLING &&
				IRQ_MODE != IRQ_MODE_RISING &&
				IRQ_MODE != IRQ_MODE_FALLRISE)
		{
			// GPIO VALUE DOES NOT EXIST
			return GPIO_ERR_INVALID_IRQMODE;
		}
		else if(GPIO_SPEED != GPIO_SPEED_LOW &&
				GPIO_SPEED != GPIO_SPEED_MEDIUM &&
				GPIO_SPEED != GPIO_SPEED_FAST &&
				GPIO_SPEED != GPIO_SPEED_HIGH)
		{
			return GPIO_ERR_INVALID_SPEED;
		}
		else if(GPIO_PUPD != GPIO_PUPD_DISABLED &&
				GPIO_PUPD != GPIO_PULL_UP &&
				GPIO_PUPD != GPIO_PULL_DOWN)
		{
			return GPIO_ERR_INVALID_PUPD;
		}
		else if(GPIO_IRQ_PRIORITY > 15)
		{
			return GPIO_ERR_INVALID_IRQ_PRIORITY;
		}
	#endif

	GPIO_Handle_t gpio;
	memset(&gpio,0,sizeof(gpio));

    gpio.pGPIOx = GPIO_PORT;
    gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN;
    gpio.GPIO_PinConfig.GPIO_PinMode = IRQ_MODE;
    gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED;
    gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD;

    gpio_init(&gpio);

    uint8_t IRQNum;

    if(GPIO_PIN == 0)
    {
    	IRQNum = IRQ_NO_EXTI0;
    }
    else if(GPIO_PIN == 1)
    {
    	IRQNum = IRQ_NO_EXTI1;
    }
    else if(GPIO_PIN == 2)
    {
    	IRQNum = IRQ_NO_EXTI2;
    }
    else if(GPIO_PIN == 3)
    {
    	IRQNum = IRQ_NO_EXTI3;
    }
    else if(GPIO_PIN == 4)
    {
    	IRQNum = IRQ_NO_EXTI4;
    }
    else if(GPIO_PIN >= 5 && GPIO_PIN <= 9)
    {
    	IRQNum = IRQ_NO_EXTI9_5;
    }
    else if(GPIO_PIN >= 10 && GPIO_PIN <= 15)
    {
    	IRQNum = IRQ_NO_EXTI15_10;
    }
    else
    {
    	return GPIO_ERR_INVALID_PIN;
    }

	gpio_irq_prio_config(IRQNum, GPIO_IRQ_PRIORITY);
	gpio_irq_config(IRQNum, ENABLE);
	GPIO_IRQSetCallback(GPIO_PIN, CALLBACK_FUNC);	// store application callback function to be called by the corresponding EXTI that matches GPIO_PIN

    return GPIO_OK;
}

/**
 * @brief De-initialize a GPIO port
 * @param GPIO_PORT Pointer to GPIO port register: PA,PB,PC,PD,PE
 */
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

/**
 * @brief Write to a GPIO pin
 * @param GPIO_PORT Pointer to GPIO port register: PA,PB,PC,PD,PE
 * @param GPIO_PIN Pin number to GPIO: 0-15
 * @param VALUE write value to pin: GPIO_HIGH, GPIO_LOW
 */
GPIO_Status_t GPIO_WritePin(GPIO_RegDef_t *GPIO_PORT, const uint8_t GPIO_PIN, const uint8_t VALUE)
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
		else if(GPIO_PIN > 15)
		{
			// GPIO PIN DOES NOT EXIST
			return GPIO_ERR_INVALID_PIN;
		}
		else if(VALUE != GPIO_HIGH && VALUE != GPIO_LOW)
		{
			// GPIO VALUE DOES NOT EXIST
			return GPIO_ERR_INVALID_VALUE;
		}
	#endif

	gpio_write_pin(GPIO_PORT, GPIO_PIN, VALUE);
	return GPIO_OK;
}

/**
 * @brief Write to a GPIO port
 * @param GPIO_PORT Pointer to GPIO port register: PA,PB,PC,PD,PE
 * @param VALUE 16-bit value to write to port (each bit represents one pin)
 */
GPIO_Status_t GPIO_WritePort(GPIO_RegDef_t *GPIO_PORT, const uint16_t VALUE)
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

	gpio_write_port(GPIO_PORT, VALUE);
	return GPIO_OK;
}

/**
 * @brief Toggle GPIO pin
 * @param GPIO_PORT Pointer to GPIO port register: PA,PB,PC,PD,PE
 * @param GPIO_PIN Pin number to GPIO: 0-15
 */
GPIO_Status_t GPIO_TogglePin(GPIO_RegDef_t *GPIO_PORT, const uint8_t GPIO_PIN)
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
		else if(GPIO_PIN > 15)
		{
			// GPIO NUM DOES NOT EXIST
			return GPIO_ERR_INVALID_PIN;
		}
	#endif

	gpio_toggle_pin(GPIO_PORT, GPIO_PIN);
	return GPIO_OK;
}

/**
 * @brief Read GPIO pin
 * @param GPIO_PORT Pointer to GPIO port register: PA,PB,PC,PD,PE
 * @param GPIO_PIN Pin number to GPIO: 0-15
 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *GPIO_PORT, const uint8_t GPIO_PIN)
{
	#ifdef DEBUG
		if(GPIO_PORT != PA &&
		   GPIO_PORT != PB &&
		   GPIO_PORT != PC &&
		   GPIO_PORT != PD &&
		   GPIO_PORT != PE)
		{
			// PORT DOES NOT EXIST
			return 0xFF; // GPIO_ERR_INVALID_PIN
		}
		else if(GPIO_PIN > 15)
		{
			// GPIO NUM DOES NOT EXIST
			return 0xFF; //GPIO_ERR_INVALID_PIN
		}
	#endif

	uint8_t byte_read = gpio_read_pin(GPIO_PORT, GPIO_PIN);

	return byte_read;
}

/**
 * @brief Read GPIO port
 * @param GPIO_PORT Pointer to GPIO port register: PA,PB,PC,PD,PE
 */
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
			return 0xFFFF;//GPIO_ERR_INVALID_PORT
		}
	#endif

	uint16_t bytes_read = gpio_read_port(GPIO_PORT);

	return bytes_read;
}


