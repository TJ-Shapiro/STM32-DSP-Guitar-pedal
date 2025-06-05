#include "ssd1306.h"
#include <string.h>

// Screen buffer
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// Write command to SSD1306
static void SSD1306_WriteCommand(I2C_HandleTypeDef *hi2c, uint8_t command) {
    uint8_t buf[2] = {SSD1306_CMD, command};
    HAL_I2C_Master_Transmit(hi2c, SSD1306_ADDR, buf, 2, HAL_MAX_DELAY);
}

// Initialize the display
void SSD1306_Init(I2C_HandleTypeDef *hi2c) {
    // Wait for the screen to boot
    HAL_Delay(100);

    // Init sequence
    SSD1306_WriteCommand(hi2c, 0xAE);   // Display off
    SSD1306_WriteCommand(hi2c, 0x20);   // Set Memory Addressing Mode
    SSD1306_WriteCommand(hi2c, 0x10);   // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
    SSD1306_WriteCommand(hi2c, 0xB0);   // Set Page Start Address for Page Addressing Mode,0-7
    SSD1306_WriteCommand(hi2c, 0xC8);   // Set COM Output Scan Direction
    SSD1306_WriteCommand(hi2c, 0x00);   // Set low column address
    SSD1306_WriteCommand(hi2c, 0x10);   // Set high column address
    SSD1306_WriteCommand(hi2c, 0x40);   // Set start line address
    SSD1306_WriteCommand(hi2c, 0x81);   // set contrast control register
    SSD1306_WriteCommand(hi2c, 0xFF);
    SSD1306_WriteCommand(hi2c, 0xA1);   // Set segment re-map 0 to 127
    SSD1306_WriteCommand(hi2c, 0xA6);   // Set normal display
    SSD1306_WriteCommand(hi2c, 0xA8);   // Set multiplex ratio(1 to 64)
    SSD1306_WriteCommand(hi2c, 0x3F);
    SSD1306_WriteCommand(hi2c, 0xA4);   // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    SSD1306_WriteCommand(hi2c, 0xD3);   // Set display offset
    SSD1306_WriteCommand(hi2c, 0x00);   // No offset
    SSD1306_WriteCommand(hi2c, 0xD5);   // Set display clock divide ratio/oscillator frequency
    SSD1306_WriteCommand(hi2c, 0xF0);   // Set divide ratio
    SSD1306_WriteCommand(hi2c, 0xD9);   // Set pre-charge period
    SSD1306_WriteCommand(hi2c, 0x22);
    SSD1306_WriteCommand(hi2c, 0xDA);   // Set com pins hardware configuration
    SSD1306_WriteCommand(hi2c, 0x12);
    SSD1306_WriteCommand(hi2c, 0xDB);   // Set vcomh
    SSD1306_WriteCommand(hi2c, 0x20);   // 0x20,0.77xVcc
    SSD1306_WriteCommand(hi2c, 0x8D);   // Set DC-DC enable
    SSD1306_WriteCommand(hi2c, 0x14);
    SSD1306_WriteCommand(hi2c, 0xAF);   // Turn on SSD1306 panel

    // Clear screen
    SSD1306_Clear(hi2c);
}

// Clear the display
void SSD1306_Clear(I2C_HandleTypeDef *hi2c) {
    memset(SSD1306_Buffer, 0, sizeof(SSD1306_Buffer));
    SSD1306_UpdateScreen(hi2c);
}

// Update the display
void SSD1306_UpdateScreen(I2C_HandleTypeDef *hi2c) {
    uint8_t i;
    for(i = 0; i < 8; i++) {
        SSD1306_WriteCommand(hi2c, 0xB0 + i);
        SSD1306_WriteCommand(hi2c, 0x00);
        SSD1306_WriteCommand(hi2c, 0x10);

        // Write data
        HAL_I2C_Mem_Write(hi2c, SSD1306_ADDR, SSD1306_DATA, 1,
                         &SSD1306_Buffer[SSD1306_WIDTH * i],
                         SSD1306_WIDTH, HAL_MAX_DELAY);
    }
}

// Draw a pixel
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color) {
    if(x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;

    if(color) {
        SSD1306_Buffer[x + (y/8)*SSD1306_WIDTH] |= 1 << (y%8);
    } else {
        SSD1306_Buffer[x + (y/8)*SSD1306_WIDTH] &= ~(1 << (y%8));
    }
}

// Write a string (simple implementation)
void SSD1306_WriteString(I2C_HandleTypeDef *hi2c, uint8_t x, uint8_t y, char* str) {
    uint8_t currentX = x;
    uint8_t currentY = y;

    while(*str) {
        if(*str >= 32 && *str <= 126) {  // Printable ASCII range
            const uint8_t *charPtr = &Font5x7[(*str - 32) * 5];

            for(uint8_t col = 0; col < 5; col++) {
                uint8_t line = charPtr[col];
                for(uint8_t row = 0; row < 8; row++) {
                    if(line & (1 << row)) {
                        SSD1306_DrawPixel(currentX + col, currentY + row, 1);
                    }
                }
            }
            currentX += 6;  // 5 pixels + 1 space
        }
        str++;
    }
    SSD1306_UpdateScreen(hi2c);
}

void SSD1306_DrawProgressBar(I2C_HandleTypeDef *hi2c, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t progress)
{
    // Clear the area where the progress bar will be
    for(uint8_t w = 0; w < width; w++) {
        for(uint8_t h = 0; h < height; h++) {
            SSD1306_DrawPixel(x + w, y + h, 0);  // Clear pixel
        }
    }

    // Draw outline
    for(uint8_t w = 0; w < width; w++) {
        SSD1306_DrawPixel(x + w, y, 1);
        SSD1306_DrawPixel(x + w, y + height - 1, 1);
    }
    for(uint8_t h = 0; h < height; h++) {
        SSD1306_DrawPixel(x, y + h, 1);
        SSD1306_DrawPixel(x + width - 1, y + h, 1);
    }

    // Fill progress
    uint8_t fill_width = (width - 2) * progress / 100;
    for(uint8_t w = 0; w < fill_width; w++) {
        for(uint8_t h = 0; h < height - 2; h++) {
            SSD1306_DrawPixel(x + 1 + w, y + 1 + h, 1);
        }
    }

    SSD1306_UpdateScreen(hi2c);
}


