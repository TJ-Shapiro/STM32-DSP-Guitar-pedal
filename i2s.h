/*
 * i2s.h
 *
 *  Created on: May 30, 2024
 *      Author: hussamaldean
 */

#ifndef I2S_H_
#define I2S_H_

#include "stdint.h"

void I2S3_Pins_Init(void);
void I2S3_Init(void);
void I2S_SendData(uint16_t *data, uint16_t len);
void I2S_SendData_DMA(uint16_t *data, uint16_t len);

#endif /* I2S_H_ */
