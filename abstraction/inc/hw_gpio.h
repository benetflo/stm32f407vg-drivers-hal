#ifndef INC_HW_GPIO_H_
#define INC_HW_GPIO_H_

#include "stm32f407g-disc1.h"

// GPIO mode macros
#define GPIO_IN					GPIO_MODE_IN
#define GPIO_OUT				GPIO_MODE_OUT
#define	GPIO_AF					GPIO_MODE_AF
#define GPIO_ANALOG				GPIO_MODE_ANALOG

// IRQ mode macros
#define IRQ_MODE_FALLING   GPIO_MODE_IT_FT    // Falling edge
#define IRQ_MODE_RISING    GPIO_MODE_IT_RT	  // Rising edge
#define IRQ_MODE_FALLRISE  GPIO_MODE_IT_RFT   // Rising + falling edge

// GPIO value macros
#define GPIO_HIGH				1
#define GPIO_LOW				0

// GPIO port macros
#define PA 						GPIOA
#define PB 						GPIOB
#define PC 						GPIOC
#define PD						GPIOD
#define PE						GPIOE

// GPIO speed macros
#define GPIO_SPEED_LOW 			0
#define GPIO_SPEED_MEDIUM 		1
#define GPIO_SPEED_FAST 		2
#define GPIO_SPEED_HIGH 		3

// GPIO AF pin mode macros
#define GPIO_AF_NONE            255
#define GPIO_AF0                0
#define GPIO_AF1                1
#define GPIO_AF2                2
#define GPIO_AF3                3
#define GPIO_AF4                4
#define GPIO_AF5                5
#define GPIO_AF6                6
#define GPIO_AF7                7
#define GPIO_AF8                8
#define GPIO_AF9                9
#define GPIO_AF10               10
#define GPIO_AF11               11
#define GPIO_AF12               12
#define GPIO_AF13               13
#define GPIO_AF14               14
#define GPIO_AF15               15

// GPIO pull up/down macros
#define GPIO_PUPD_DISABLED		GPIO_NO_PUPD
#define GPIO_PULL_UP			GPIO_PIN_PU
#define GPIO_PULL_DOWN			GPIO_PIN_PD

// GPIO output mode macros
#define GPIO_OPEN_DRAIN			GPIO_OUT_TYPE_OD
#define GPIO_PUSH_PULL			GPIO_OUT_TYPE_PP
#define GPIO_OP_TYPE_DISABLED	GPIO_NO_OP_TYPE

// EXTI (External Interrupt) IRQ handlers for GPIO pins
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

// GPIO error codes
typedef enum
{
	GPIO_OK = 0,
	GPIO_ERR_INVALID_PORT,
	GPIO_ERR_INVALID_PIN,
	GPIO_ERR_INVALID_MODE,
	GPIO_ERR_INVALID_VALUE,
	GPIO_ERR_INVALID_SPEED,
	GPIO_ERR_INVALID_OUTPUT_TYPE,
	GPIO_ERR_INVALID_PUPD,
	GPIO_ERR_INVALID_IRQ_PRIORITY,
	GPIO_ERR_INVALID_AF_NUM,
	GPIO_ERR_INVALID_GPIO_CONFIG,
	GPIO_ERR_INVALID_IRQMODE
} GPIO_Status_t;

// Type definition for GPIO interrupt callback functions
typedef void (*GPIO_IRQ_Callback_t)(void);

// GPIO function definitions
GPIO_Status_t GPIO_IRQSetCallback(uint8_t GPIO_NUM, GPIO_IRQ_Callback_t callback);

GPIO_Status_t GPIO_Init(GPIO_RegDef_t* GPIO_PORT, const uint8_t GPIO_NUM, const uint8_t GPIO_MODE, const uint8_t GPIO_SPEED, const uint8_t GPIO_PUPD, const uint8_t GPIO_OUTPUT_TYPE, uint8_t GPIO_AF_NUM);
GPIO_Status_t GPIO_IRQInit(GPIO_RegDef_t* GPIO_PORT, const uint8_t GPIO_PIN, const uint8_t IRQ_MODE, const uint8_t GPIO_SPEED, const uint8_t GPIO_PUPD, const uint8_t GPIO_IRQ_PRIORITY, GPIO_IRQ_Callback_t callback_func);

GPIO_Status_t GPIO_DeInit(GPIO_RegDef_t *GPIO_PORT);

GPIO_Status_t GPIO_WritePin(GPIO_RegDef_t *GPIO_PORT, const uint8_t GPIO_NUM,uint8_t VALUE);
GPIO_Status_t GPIO_WritePort(GPIO_RegDef_t *GPIO_PORT, const uint16_t GPIO_VALUE);

GPIO_Status_t GPIO_TogglePin(GPIO_RegDef_t *GPIO_PORT, const uint8_t GPIO_NUM);

uint8_t GPIO_ReadPin(GPIO_RegDef_t *GPIO_PORT, const uint8_t GPIO_NUM);
uint16_t GPIO_ReadPort(GPIO_RegDef_t *GPIO_PORT);

#endif
