/*
 * LiquidCrystal_I2C.c
 *
 * Author: TRAN NGUYEN HIEN
 * Major: Electronic And Communication Engineering
 */

#include "LiquidCrystal_I2C.h"
#include <stdio.h>
LiquidCrystal_I2C_Def lcd;

void LCDI2C_init(I2C_HandleTypeDef *hi2c, uint8_t lcd_Addr, uint8_t lcd_cols, uint8_t lcd_rows) {
  lcd.hi2c = hi2c;
  lcd.Addr = lcd_Addr << 1;
  lcd.cols = lcd_cols;
  lcd.rows = lcd_rows;
  lcd.backlightval = LCD_BACKLIGHT;

  HAL_Delay(50);

  LCDI2C_expanderWrite(lcd.backlightval);
  HAL_Delay(1000);

  LCDI2C_write4bits(0x03 << 4);
  HAL_Delay(5);
  LCDI2C_write4bits(0x03 << 4);
  HAL_Delay(5);
  LCDI2C_write4bits(0x03 << 4);
  HAL_Delay(1);
  LCDI2C_write4bits(0x02 << 4);

  lcd.displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
  if (lcd.rows > 1) {
    lcd.displayfunction |= LCD_2LINE;
  }

  LCDI2C_command(LCD_FUNCTIONSET | lcd.displayfunction);

  lcd.displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  LCDI2C_display();

  lcd.displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  LCDI2C_command(LCD_ENTRYMODESET | lcd.displaymode);

  LCDI2C_clear();
}

void LCDI2C_begin(uint8_t cols, uint8_t rows) {
  lcd.cols = cols;
  lcd.rows = rows;
}

void LCDI2C_clear() {
  LCDI2C_command(LCD_CLEARDISPLAY);
  HAL_Delay(2);
}

void LCDI2C_home() {
  LCDI2C_command(LCD_RETURNHOME);
  HAL_Delay(2);
}

void LCDI2C_noDisplay() {
  lcd.displaycontrol &= ~LCD_DISPLAYON;
  LCDI2C_command(LCD_DISPLAYCONTROL | lcd.displaycontrol);
}

void LCDI2C_display() {
  lcd.displaycontrol |= LCD_DISPLAYON;
  LCDI2C_command(LCD_DISPLAYCONTROL | lcd.displaycontrol);
}

void LCDI2C_noBlink() {
  lcd.displaycontrol &= ~LCD_BLINKON;
  LCDI2C_command(LCD_DISPLAYCONTROL | lcd.displaycontrol);
}

void LCDI2C_blink() {
  lcd.displaycontrol |= LCD_BLINKON;
  LCDI2C_command(LCD_DISPLAYCONTROL | lcd.displaycontrol);
}

void LCDI2C_noCursor() {
  lcd.displaycontrol &= ~LCD_CURSORON;
  LCDI2C_command(LCD_DISPLAYCONTROL | lcd.displaycontrol);
}

void LCDI2C_cursor() {
  lcd.displaycontrol |= LCD_CURSORON;
  LCDI2C_command(LCD_DISPLAYCONTROL | lcd.displaycontrol);
}

void LCDI2C_scrollDisplayLeft() {
  LCDI2C_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void LCDI2C_scrollDisplayRight() {
  LCDI2C_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void LCDI2C_leftToRight() {
  lcd.displaymode |= LCD_ENTRYLEFT;
  LCDI2C_command(LCD_ENTRYMODESET | lcd.displaymode);
}

void LCDI2C_rightToLeft() {
  lcd.displaymode &= ~LCD_ENTRYLEFT;
  LCDI2C_command(LCD_ENTRYMODESET | lcd.displaymode);
}

void LCDI2C_autoscroll() {
  lcd.displaymode |= LCD_ENTRYSHIFTINCREMENT;
  LCDI2C_command(LCD_ENTRYMODESET | lcd.displaymode);
}

void LCDI2C_noAutoscroll() {
  lcd.displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  LCDI2C_command(LCD_ENTRYMODESET | lcd.displaymode);
}

void LCDI2C_backlight() {
  lcd.backlightval = LCD_BACKLIGHT;
  LCDI2C_expanderWrite(0);
}

void LCDI2C_noBacklight() {
  lcd.backlightval = LCD_NOBACKLIGHT;
  LCDI2C_expanderWrite(0);
}

void LCDI2C_setCursor(uint8_t col, uint8_t row) {
  const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
  if (row >= lcd.rows) row = lcd.rows - 1;
  LCDI2C_command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void LCDI2C_createChar(uint8_t location, uint8_t charmap[]) {
  location &= 0x7;
  LCDI2C_command(LCD_SETCGRAMADDR | (location << 3));
  for (int i = 0; i < 8; i++) {
    LCDI2C_write(charmap[i]);
  }
}

void LCDI2C_command(uint8_t value) {
  LCDI2C_send(value, 0);
}

void LCDI2C_write(uint8_t value) {
  LCDI2C_send(value, Rs);
}

void LCDI2C_write_String(char* str) {
  while (*str) {
    LCDI2C_write(*str++);
  }
}

void LCDI2C_write_Int(int value) {
  char buffer[12];
  sprintf(buffer, "%d", value);
  LCDI2C_write_String(buffer);
}

void LCDI2C_write_Float(float value, uint8_t precision) {
  char buffer[20];
  char format[10];
  sprintf(format, "%%.%df", precision);
  sprintf(buffer, format, value);
  LCDI2C_write_String(buffer);
}

void LCDI2C_send(uint8_t value, uint8_t mode) {
  uint8_t highnib = value & 0xF0;
  uint8_t lownib = (value << 4) & 0xF0;
  LCDI2C_write4bits(highnib | mode);
  LCDI2C_write4bits(lownib | mode);
}

void LCDI2C_write4bits(uint8_t value) {
  LCDI2C_expanderWrite(value);
  LCDI2C_pulseEnable(value);
}

void LCDI2C_expanderWrite(uint8_t data) {
  uint8_t d[1] = {data | lcd.backlightval};
  HAL_I2C_Master_Transmit(lcd.hi2c, lcd.Addr, d, 1, 10);
}

void LCDI2C_pulseEnable(uint8_t data) {
  LCDI2C_expanderWrite(data | En);
  HAL_Delay(1);
  LCDI2C_expanderWrite(data & ~En);
  HAL_Delay(1);
}

void LCDI2C_blink_on() { LCDI2C_blink(); }
void LCDI2C_blink_off() { LCDI2C_noBlink(); }
void LCDI2C_cursor_on() { LCDI2C_cursor(); }
void LCDI2C_cursor_off() { LCDI2C_noCursor(); }
void LCDI2C_setBacklight(uint8_t new_val) {
  if (new_val) LCDI2C_backlight();
  else LCDI2C_noBacklight();
}
void LCDI2C_load_custom_character(uint8_t char_num, uint8_t *rows) {
  LCDI2C_createChar(char_num, rows);
}
void LCDI2C_printstr(const char str[]) {
  LCDI2C_write_String((char*)str);
}
