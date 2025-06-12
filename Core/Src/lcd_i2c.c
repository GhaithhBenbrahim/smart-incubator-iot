/*
 * lcd_i2c.c
 *
 *  Created on: Apr 18, 2025
 *      Author: benbr
 */

#include "lcd_i2c.h"
#include <stdio.h>
extern I2C_HandleTypeDef hi2c1;

#define SLAVE_ADDRESS_LCD (0x27 << 1)

void lcd_send_cmd(char cmd) {
  char data_u, data_l;
  uint8_t data_t[4];
  data_u = (cmd&0xf0);
  data_l = ((cmd<<4)&0xf0);
  data_t[0] = data_u|0x0C;
  data_t[1] = data_u|0x08;
  data_t[2] = data_l|0x0C;
  data_t[3] = data_l|0x08;
  HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data(char data) {
  char data_u, data_l;
  uint8_t data_t[4];
  data_u = (data&0xf0);
  data_l = ((data<<4)&0xf0);
  data_t[0] = data_u|0x0D;
  data_t[1] = data_u|0x09;
  data_t[2] = data_l|0x0D;
  data_t[3] = data_l|0x09;
  HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t *)data_t, 4, 100);
}

void lcd_clear(void) {
  lcd_send_cmd(0x80);
  for(int i=0; i<70; i++) lcd_send_data(' ');
}

void lcd_put_cur(int row, int col) {
  switch(row) {
    case 0: col |= 0x80; break;
    case 1: col |= 0xC0; break;
  }
  lcd_send_cmd(col);
}

void lcd_init(void) {
  HAL_Delay(50);
  lcd_send_cmd(0x30);
  HAL_Delay(5);
  lcd_send_cmd(0x30);
  HAL_Delay(1);
  lcd_send_cmd(0x30);
  HAL_Delay(10);
  lcd_send_cmd(0x20);
  HAL_Delay(10);
  lcd_send_cmd(0x28);
  HAL_Delay(1);
  lcd_send_cmd(0x08);
  HAL_Delay(1);
  lcd_send_cmd(0x01);
  HAL_Delay(1);
  HAL_Delay(1);
  lcd_send_cmd(0x06);
  HAL_Delay(1);
  lcd_send_cmd(0x0C);
}

void lcd_send_string(char *str) {
  while(*str) lcd_send_data(*str++);
}

/* New display functions matching your cases */
void Display_Screen1(float h, float f) {
  char buffer[16];
  lcd_clear();
  lcd_put_cur(0, 0);
  sprintf(buffer, "Hum: %.1f %%", h);
  lcd_send_string(buffer);
  lcd_put_cur(1, 0);
  sprintf(buffer, "Temp: %.1f %cC", f, 223);
  lcd_send_string(buffer);
}

void Display_Screen2(float Hmax, float Hmin) {
  char buffer[16];
  //lcd_clear();
  lcd_put_cur(0, 0);
  sprintf(buffer, "Hmax: %.1f %%", Hmax);
  lcd_send_string(buffer);
  lcd_put_cur(1, 0);
  sprintf(buffer, "Hmin: %.1f %%", Hmin);
  lcd_send_string(buffer);
}

void Display_Screen3(float Tmax, float Tmin) {
  char buffer[16];
  //lcd_clear();
  lcd_put_cur(0, 0);
  sprintf(buffer, "Tmax: %.1f %cC", Tmax ,223);
  lcd_send_string(buffer);
  lcd_put_cur(1, 0);
  sprintf(buffer, "Tmin: %.1f %cC", Tmin ,223);
  lcd_send_string(buffer);
}

void Display_Screen4(int8_t StepperState) {
  lcd_clear();
  lcd_put_cur(0, 0);
  lcd_send_string("Stepper State:");
  lcd_put_cur(1, 0);

  if(StepperState == 1 ) {
    lcd_send_string("Opening");
  } else if(StepperState == -1  ) {
    lcd_send_string("Closing");
  } else {
    lcd_send_string("Idle");
  }
}

void Display_Screen5(uint32_t tReset) {
  uint32_t seconds = (HAL_GetTick() - tReset)/1000;
  uint32_t days = seconds/86400;
  seconds %= 86400;
  uint32_t hours = seconds/3600;
  seconds %= 3600;
  uint32_t minutes = seconds/60;
  seconds %= 60;

  char buffer[17];
  lcd_clear();
  lcd_put_cur(0, 6);
  lcd_send_string("TIME");
  lcd_put_cur(1, 0);
  sprintf(buffer, "%02lu:%02lu:%02lu:%02lu", days, hours, minutes, seconds);
  lcd_send_string(buffer);
}

/*
void Display_Screen6(float tLBon100) {
  char buffer[16];
  lcd_clear();
  lcd_put_cur(0, 0);
  sprintf(buffer, "LB ON: %.1f %%", tLBon100);
  lcd_send_string(buffer);
}*/

