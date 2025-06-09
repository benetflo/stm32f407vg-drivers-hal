#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#define RCC_BASE_ADDR 0x40023800UL

#define RCC_APB1_ENR_OFFSET 0x40UL
#define RCC_APB1_ENR_ADDR (RCC_BASE_ADDR + RCC_APB1_ENR_OFFSET)

#define I2C_CR1_REG_OFFSET 0x00UL

#define I2C1_BASE_ADDR 0x40005400UL
#define I2C2_BASE_ADDR 0x40005800UL
#define I2C3_BASE_ADDR 0x40005C00UL

#define I2C1_CR1_REG_ADDR (I2C1_BASE_ADDR + I2C_CR1_REG_OFFSET)
#define I2C2_CR1_REG_ADDR (I2C2_BASE_ADDR + I2C_CR1_REG_OFFSET)
#define I2C3_CR1_REG_ADDR (I2C3_BASE_ADDR + I2C_CR1_REG_OFFSET)

void init_i2c1(void);
void init_i2c2(void);
void init_i2c3(void);
void init_i2c(uint8_t i2c_num);

#endif
