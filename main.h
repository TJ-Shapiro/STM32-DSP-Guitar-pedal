#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx_hal.h"

// Function prototypes
uint16_t readPotentiometer(void);
void SSD1306_DrawProgressBar(I2C_HandleTypeDef *hi2c, uint8_t x, uint8_t y,
                           uint8_t width, uint8_t height, uint8_t progress);

#endif
