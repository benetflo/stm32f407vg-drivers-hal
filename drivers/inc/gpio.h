#ifndef INC_STM32F407G_DISC1_GPIO_DRIVER_H_
#define INC_STM32F407G_DISC1_GPIO_DRIVER_H_

#include "stm32f407g-disc1.h"

typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct
{
	//pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx; // Holds base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;


#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15


#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_AF 2
#define GPIO_MODE_ANALOG 3

// Interrupt modes
#define GPIO_MODE_IT_FT 4    // Falling edge detection
#define GPIO_MODE_IT_RT 5	 // Rising edge detection
#define GPIO_MODE_IT_RFT 6   // Rising aswell as falling edge detection


#define GPIO_OUT_TYPE_PP 0
#define GPIO_OUT_TYPE_OD 1


#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_FAST 2
#define GPIO_SPEED_HIGH 3

#define GPIO_NO_PUPD 0
#define GPIO_PIN_PU 1
#define GPIO_PIN_PD 2



void gpio_peri_clk_control(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

void gpio_init(GPIO_Handle_t *pGPIOHandle);
void gpio_deinit(GPIO_RegDef_t *pGPIOx);

uint8_t gpio_read_pin(GPIO_RegDef_t *pGPIOx, uint8_t pin_num);
uint16_t gpio_read_port(GPIO_RegDef_t *pGPIOx);
void gpio_write_pin(GPIO_RegDef_t *pGPIOx, uint8_t pin_num, uint8_t value);
void gpio_write_port(GPIO_RegDef_t *pGPIOx, uint16_t value);
void gpio_toggle_pin(GPIO_RegDef_t *pGPIOx, uint8_t pin_num);

void gpio_irq_config(uint8_t irq_num, uint8_t EnOrDi);
void gpio_irq_prio_config(uint8_t irq_num, uint32_t irq_prio);
void gpio_irq_handling(uint8_t pin_num);


#endif /* INC_STM32F407G_DISC1_GPIO_DRIVER_H_ */
