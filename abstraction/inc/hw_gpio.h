#ifndef INC_HW_GPIO_H_
#define INC_HW_GPIO_H_

#include "stm32f407g-disc1.h"



// GPIO MODES
#define GPIO_IN					GPIO_MODE_IN
#define GPIO_OUT				GPIO_MODE_OUT
#define	GPIO_AF					GPIO_MODE_AF
#define GPIO_ANALOG				GPIO_MODE_ANALOG


// GPIO VALUES
#define GPIO_HIGH				1
#define GPIO_LOW				0

// GPIO PORT NAMES
#define PA 						GPIOA
#define PB 						GPIOB
#define PC 						GPIOC
#define PD						GPIOD
#define PE						GPIOE

// GPIO SPEEDS
#define GPIO_SPEED_LOW 			0
#define GPIO_SPEED_MEDIUM 		1
#define GPIO_SPEED_FAST 		2
#define GPIO_SPEED_HIGH 		3


#define GPIO_PUPD_DISABLED		GPIO_NO_PUPD
#define GPIO_PULL_UP			GPIO_PIN_PU
#define GPIO_PULL_DOWN			GPIO_PIN_PD

#define GPIO_OPEN_DRAIN			GPIO_OUT_TYPE_OD
#define GPIO_PUSH_PULL			GPIO_OUT_TYPE_PP
#define GPIO_OP_TYPE_DISABLED	GPIO_NO_OP_TYPE


// IRQ MODES DEFINES TAKEN FROM MCU SPECIFIC HEADER FILE

//#define GPIO_MODE_IT_FT 4    // Falling edge detection
//#define GPIO_MODE_IT_RT 5	 // Rising edge detection
//#define GPIO_MODE_IT_RFT 6   // Rising aswell as falling edge detection


void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);



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
	GPIO_ERR_INVALID_IRQ_PRIORITY
} GPIO_Status_t;

typedef void (*GPIO_IRQ_Callback_t)(void);
GPIO_Status_t GPIO_IRQSetCallback(uint8_t GPIO_NUM, GPIO_IRQ_Callback_t callback);

GPIO_Status_t GPIO_Init(GPIO_RegDef_t* GPIO_PORT, const uint8_t GPIO_NUM, const uint8_t GPIO_MODE, const uint8_t GPIO_SPEED, const uint8_t GPIO_PUPD, const uint8_t GPIO_OUTPUT_TYPE);
GPIO_Status_t GPIO_IRQInit(GPIO_RegDef_t* GPIO_PORT, const uint8_t GPIO_NUM, const uint8_t GPIO_MODE, const uint8_t GPIO_SPEED, const uint8_t GPIO_PUPD, const uint8_t GPIO_OUTPUT_TYPE, const uint8_t GPIO_IRQ_PRIORITY);
GPIO_Status_t GPIO_DeInit(GPIO_RegDef_t *GPIO_PORT);

GPIO_Status_t GPIO_WritePin(GPIO_RegDef_t *GPIO_PORT, const uint8_t GPIO_NUM,uint8_t VALUE);
GPIO_Status_t GPIO_WritePort(GPIO_RegDef_t *GPIO_PORT, const uint8_t GPIO_VALUE);
GPIO_Status_t GPIO_TogglePin(GPIO_RegDef_t *GPIO_PORT, const uint8_t GPIO_NUM);

uint8_t GPIO_ReadPin(GPIO_RegDef_t *GPIO_PORT, const uint8_t GPIO_NUM);
uint16_t GPIO_ReadPort(GPIO_RegDef_t *GPIO_PORT);

#endif /* INC_HW_GPIO_H_ */

