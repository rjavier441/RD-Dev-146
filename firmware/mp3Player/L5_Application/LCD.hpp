#ifndef _LCD_H
#define _LCD_H

#include "lpc17xx.h"
#include <string>

#define DEFAULT_BRIGHTNESS 8
#define DEFAULT_CONTRAST 40

void initLCD(void);
void initLCD(int rate);
void clearLCD(void);
void offLCD(void);
void onLCD(void);
void putChar(char c);
void putString(char *s);
void setCursor(int line, int col);
void cursorLeft(void);
void cursorRight(void);
void cursorBlinkOn(void);
void cursorBlinkOff(void);
void backspaceLCD(void);
void contrast(int c);
void brightness(int b);
void displayLeft(void);
void displayRight(void);
void changeBAUDrate(int r);
void displayFirmware(void);
void displayBAUDrate(void);
void displayI2C(void);
void turnOffDisplay(void);
void turnONDisplay(void);
void scroll(char *s);
void manuelFirstScroll(std::string s);
void manuelSecondScroll(std::string s);
void manuelBothScroll(std::string s1, std::string s2);
// void firstLineScroll(char *s);
// void secondLineScroll(char *s);

#endif
