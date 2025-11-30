/**
  ******************************************************************************
  * @file    ssd1306_ll.c
  * @author  hrnkrc
  * @brief   Implementation of the SSD1306 OLED Display Driver using STM32 LL API.
  * @date    November 2025
  ******************************************************************************
  */

#include "ssd1306_ll.h"
#include <string.h> // Required for memset

/* ========================================================================== */
/* PRIVATE VARIABLES                                                          */
/* ========================================================================== */

/** @brief Display Frame Buffer (1 bit per pixel) */
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

/** * @brief Standard 5x7 ASCII Font Table (Lite Version)
 * Contains basic symbols, numbers, and uppercase letters to save flash.
 */
const uint8_t Font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
    // Other symbols omitted for memory optimization
};

/* ========================================================================== */
/* PRIVATE FUNCTION PROTOTYPES                                                */
/* ========================================================================== */

/**
 * @brief  Sends data via I2C using LL (Low Layer) drivers.
 * @param  reg   Register to write to (0x00 for Command, 0x40 for Data).
 * @param  data  Pointer to data buffer.
 * @param  count Number of bytes to send.
 */
static void I2C_Write(uint8_t reg, uint8_t *data, uint16_t count)
{
    /* 1. Initiate I2C Transfer */
    LL_I2C_HandleTransfer(SSD1306_I2C_PORT, SSD1306_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT, count + 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

    /* 2. Send Register/Control Byte */
    while(!LL_I2C_IsActiveFlag_TXIS(SSD1306_I2C_PORT));
    LL_I2C_TransmitData8(SSD1306_I2C_PORT, reg);

    /* 3. Send Data Payload */
    for(uint16_t i=0; i<count; i++) {
        while(!LL_I2C_IsActiveFlag_TXIS(SSD1306_I2C_PORT));
        LL_I2C_TransmitData8(SSD1306_I2C_PORT, data[i]);
    }

    /* 4. Wait for Stop Condition */
    while(!LL_I2C_IsActiveFlag_STOP(SSD1306_I2C_PORT));
    LL_I2C_ClearFlag_STOP(SSD1306_I2C_PORT);
}

/**
 * @brief  Sends a single command byte to the display.
 * @param  cmd Command byte to send.
 */
static void WriteCmd(uint8_t cmd) {
    I2C_Write(0x00, &cmd, 1);
}

/* ========================================================================== */
/* PUBLIC FUNCTIONS                                                           */
/* ========================================================================== */

void SSD1306_Init(void)
{
    LL_mDelay(100);

    // SSD1306 Initialization Sequence
    WriteCmd(0xAE); // Display Off
    WriteCmd(0x20); WriteCmd(0x00); // Memory Addressing Mode: Horizontal
    WriteCmd(0xB0); // Page Start Address
    WriteCmd(0xC8); // COM Output Scan Direction
    WriteCmd(0x00); // Set Low Column Address
    WriteCmd(0x10); // Set High Column Address
    WriteCmd(0x40); // Set Start Line
    WriteCmd(0x81); WriteCmd(0xFF); // Contrast Control
    WriteCmd(0xA1); // Segment Re-map
    WriteCmd(0xA6); // Normal Display
    WriteCmd(0xA8); WriteCmd(SSD1306_HEIGHT - 1); // Multiplex Ratio
    WriteCmd(0xD3); WriteCmd(0x00); // Display Offset
    WriteCmd(0xD5); WriteCmd(0xF0); // Display Clock Divide Ratio
    WriteCmd(0xD9); WriteCmd(0x22); // Pre-charge Period
    WriteCmd(0xDA); WriteCmd(0x02); // COM Pins Hardware Configuration
    WriteCmd(0xDB); WriteCmd(0x20); // VCOMH Deselect Level
    WriteCmd(0x8D); WriteCmd(0x14); // Charge Pump Setting
    WriteCmd(0xAF); // Display On

    SSD1306_Clear();
    SSD1306_UpdateScreen();
}

void SSD1306_Clear(void)
{
    memset(SSD1306_Buffer, 0, sizeof(SSD1306_Buffer));
}

void SSD1306_UpdateScreen(void)
{
    // Write buffer to display RAM page by page
    for (uint8_t i = 0; i < SSD1306_HEIGHT / 8; i++) {
        WriteCmd(0xB0 + i); // Set Page Start Address
        WriteCmd(0x00);     // Set Low Column
        WriteCmd(0x10);     // Set High Column

        // 0x40 indicates Data Stream
        I2C_Write(0x40, &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH);
    }
}

void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;

    if (color)
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y % 8));
    else
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
}

void SSD1306_DrawChar(uint8_t x, uint8_t y, char c)
{
    // Check if character is within valid font range (ASCII 32-126 is typical)
    // Our lite font covers limited range, so we fallback to Space for unknown chars.
    if (c < 32 || c > 90) c = 32;

    for (uint8_t i = 0; i < 5; i++) { // Font width is 5
        uint8_t line = Font5x7[c - 32][i];
        for (uint8_t j = 0; j < 8; j++) {
            if (line & (1 << j)) SSD1306_DrawPixel(x + i, y + j, 1);
        }
    }
}

void SSD1306_WriteString(uint8_t x, uint8_t y, char *str)
{
    while (*str) {
        SSD1306_DrawChar(x, y, *str);
        x += 6; // Move cursor: Font width (5) + Spacing (1)
        if(x > SSD1306_WIDTH) break; // Clip if out of bounds
        str++;
    }
}

void SSD1306_DrawProgressBar(uint8_t percent)
{
    if (percent > 100) percent = 100;

    // Calculate filled width based on screen width minus margins
    uint8_t width = (percent * (SSD1306_WIDTH - 4)) / 100;

    // Draw Frame (Top and Bottom Lines)
    // Location: y=20 to y=31 (Bottom part of 32px screen)
    for(uint8_t i = 0; i < SSD1306_WIDTH; i++) {
        SSD1306_DrawPixel(i, 20, 1); // Top Border
        SSD1306_DrawPixel(i, 31, 1); // Bottom Border
    }
    // Draw Frame (Left and Right Lines)
    for(uint8_t i = 20; i <= 31; i++) {
        SSD1306_DrawPixel(0, i, 1);   // Left Border
        SSD1306_DrawPixel(127, i, 1); // Right Border
    }

    // Fill the Bar
    for(uint8_t x = 2; x < 2 + width; x++) {
        for(uint8_t y = 22; y < 30; y++) {
            SSD1306_DrawPixel(x, y, 1);
        }
    }
}
