/*
 * lcd_i2c.h
 *
 *  Created on: Apr 18, 2025
 *      Author: benbr
 */

#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "stm32f4xx_hal.h"



// LCD Functions (using your exact implementations)
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_clear(void);
void lcd_put_cur(int row, int col);
void lcd_init(void);
void lcd_send_string(char *str);

// Display Functions (matching your case structure)
void Display_Screen1(float h, float f);
void Display_Screen2(float Hmax, float Hmin);
void Display_Screen3(float Tmax, float Tmin);
void Display_Screen4(int8_t StepperState);
void Display_Screen5(uint32_t tReset);
void Display_Screen6(float tLBon100);

#endif
