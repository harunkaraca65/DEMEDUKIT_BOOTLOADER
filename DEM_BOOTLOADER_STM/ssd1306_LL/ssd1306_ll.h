/**
  ******************************************************************************
  * @file    ssd1306_ll.h
  * @author  hrnkrc
  * @brief   Header file for the SSD1306 OLED Display Driver (Low Layer).
  * Supports 128x32 and 128x64 resolutions via I2C.
  * @date    November 2025
  ******************************************************************************
  */

#ifndef SSD1306_LL_H
#define SSD1306_LL_H

#include "main.h"

/* ========================================================================== */
/* HARDWARE CONFIGURATION                                                     */
/* ========================================================================== */

/** @brief I2C Peripheral instance used for the display */
#define SSD1306_I2C_PORT        I2C1

/** @brief I2C Address of the SSD1306 (7-bit address << 1) */
#define SSD1306_I2C_ADDR        (0x3C << 1) // 0x78

/** @brief Display Width in pixels */
#define SSD1306_WIDTH           128

/** @brief Display Height in pixels (Change to 64 if needed) */
#define SSD1306_HEIGHT          32

/* ========================================================================== */
/* PUBLIC FUNCTION PROTOTYPES                                                 */
/* ========================================================================== */

/**
 * @brief  Initializes the SSD1306 display controller.
 * Sends the initialization command sequence via I2C.
 */
void SSD1306_Init(void);

/**
 * @brief  Clears the display buffer (fills with 0).
 * Call UpdateScreen() to apply changes.
 */
void SSD1306_Clear(void);

/**
 * @brief  Sends the content of the RAM buffer to the OLED display memory.
 * This function must be called to make drawing operations visible.
 */
void SSD1306_UpdateScreen(void);

/**
 * @brief  Sets a single pixel in the buffer.
 * @param  x X coordinate (0 to WIDTH-1)
 * @param  y Y coordinate (0 to HEIGHT-1)
 * @param  color 1 for On (White), 0 for Off (Black)
 */
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);

/**
 * @brief  Draws a single ASCII character to the buffer.
 * @param  x X coordinate
 * @param  y Y coordinate
 * @param  c The character to draw (A-Z, 0-9 supported in lite font)
 */
void SSD1306_DrawChar(uint8_t x, uint8_t y, char c);

/**
 * @brief  Draws a string to the buffer.
 * @param  x X coordinate
 * @param  y Y coordinate
 * @param  str Pointer to the null-terminated string
 */
void SSD1306_WriteString(uint8_t x, uint8_t y, char *str);

/**
 * @brief  Draws a graphical progress bar at the bottom of the screen.
 * @param  percent Completion percentage (0-100)
 */
void SSD1306_DrawProgressBar(uint8_t percent);

#endif /* SSD1306_LL_H */
