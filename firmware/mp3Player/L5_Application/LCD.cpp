#include "tasks.hpp"
#include "uart3.hpp"
#include "utilities.h" // vTaskDelay()
#include <LCD.hpp>
#include <cstring>
#include <stdio.h> // printf() / scanf()
#include <string>

void initLCD(void) {
  Uart3::getInstance().init(9600);
  vTaskDelay(100);
}
void initLCD(int rate) { Uart3::getInstance().init(rate); }
void putChar(char c) { Uart3::getInstance().putChar(c); }
void putString(char *s) {
  while (*s != '\0') {
    Uart3::getInstance().putChar(*s);
    s++;
  }
}

void offLCD(void) {
  Uart3::getInstance().putChar(0xFE);
  Uart3::getInstance().putChar(0x42);
}
void onLCD(void) {
  Uart3::getInstance().putChar(0xFE);
  Uart3::getInstance().putChar(0x41);
}

void setCursor(int line, int col) {
  int position = 0x00;
  if (col > 16 || col < 1 || line < 1 || line > 2) {
    putString("Wrong cursor setup.");
    return;
  }
  if (line == 1)
    position = 0x00 + col - 1;
  if (line == 2)
    position = 0x40 + col - 1;
  Uart3::getInstance().putChar(0xFE);
  Uart3::getInstance().putChar(0x45);
  Uart3::getInstance().putChar(position);
}

void cursorLeft(void) {
  Uart3::getInstance().putChar(0xFE);
  Uart3::getInstance().putChar(0x49);
}
void cursorRight(void) {
  Uart3::getInstance().putChar(0xFE);
  Uart3::getInstance().putChar(0x4A);
}
void cursorBlinkOn(void) {
  Uart3::getInstance().putChar(0xFE);
  Uart3::getInstance().putChar(0x4B);
}
void cursorBlinkOff(void) {
  Uart3::getInstance().putChar(0xFE);
  Uart3::getInstance().putChar(0x4C);
}
void backspaceLCD(void) {
  Uart3::getInstance().putChar(0xFE);
  Uart3::getInstance().putChar(0x4E);
}
void clearLCD(void) {
  Uart3::getInstance().putChar(0xFE);
  Uart3::getInstance().putChar(0x51);
  setCursor(1, 1);
  vTaskDelay(2);
}
void contrast(int c) {
  if (c < 1 || c > 50) {
    putString("Contrast: 1-50");
    return;
  }
  Uart3::getInstance().putChar(0xFE);
  Uart3::getInstance().putChar(0x52);
  Uart3::getInstance().putChar(c);
  vTaskDelay(1);
}
void brightness(int b) {
  if (b < 1 || b > 8) {
    putString("Brightness: 1-8.");
    return;
  }
  Uart3::getInstance().putChar(0xFE);
  Uart3::getInstance().putChar(0x53);
  Uart3::getInstance().putChar(b);
}

void displayLeft(void) {
  Uart3::getInstance().putChar(0xFE);
  Uart3::getInstance().putChar(0x55);
}
void displayRight(void) {
  Uart3::getInstance().putChar(0xFE);
  Uart3::getInstance().putChar(0x56);
}

void changeBAUDrate(int r) {
  int rate = 4;
  if (r < 1 || r > 8) {
    putString("BAUD rate: 1-8.");
    return;
  }
  Uart3::getInstance().putChar(0xFE);
  Uart3::getInstance().putChar(0x61);
  Uart3::getInstance().putChar(r);
  vTaskDelay(5);
}
void displayFirmware(void) {
  Uart3::getInstance().putChar(0xFE);
  Uart3::getInstance().putChar(0x70);
  vTaskDelay(5);
}
void displayBAUDrate(void) {
  Uart3::getInstance().putChar(0xFE);
  Uart3::getInstance().putChar(0x71);
  vTaskDelay(10);
}
void displayI2C(void) {
  Uart3::getInstance().putChar(0xFE);
  Uart3::getInstance().putChar(0x72);
  vTaskDelay(5);
}

void turnOffDisplay(void) {
  brightness(1);
  offLCD();
}
void turnONDisplay(void) {
  brightness(DEFAULT_BRIGHTNESS);
  onLCD();
}

void scroll(char *s) {
  int counter = 16;
  //  char *left = s;
  //  char *right = s;
  while (*s != '\0' && counter--) {
    Uart3::getInstance().putChar(*s);
    s++;
  }
  if (*s != '\0') {
    vTaskDelay(1000);
    while (*s != '\0') {
      displayLeft();
      Uart3::getInstance().putChar(*s);
      vTaskDelay(800);
      s++;
    }
  }
}

void manuelFirstScroll(std::string s) {
  char *c = new char[17]; // or
  if (s.length() > 16) {
    for (int i = 0; i < (s.length() - 16 + 1); i++) {
      setCursor(1, 1);
      std::string sub = s.substr(i, 16);
      std::strcpy(c, sub.c_str());
      putString(c);
      vTaskDelay(500);
      // delete[] c;
    }
  } else {
    setCursor(1, 1);
    std::strcpy(c, s.c_str());
    putString(c);
  }
}
void manuelSecondScroll(std::string s) {
  char *c = new char[17]; // or
  if (s.length() > 16) {
    for (int i = 0; i < (s.length() - 16 + 1); i++) {
      setCursor(2, 1);
      std::string sub = s.substr(i, 16);
      std::strcpy(c, sub.c_str());
      putString(c);
      vTaskDelay(500);
      // delete[] c;
    }
  } else {
    setCursor(2, 1);
    std::strcpy(c, s.c_str());
    putString(c);
  }
}
void manuelSecondScroll(std::string s1, std::string s2) {
  char *c;
  std::string sub1 = s1.substr(0, 16);
  c = new char[sub1.length() + 1];
  std::strcpy(c, sub1.c_str());
  setCursor(1, 1);
  putString(c);

  std::string sub2 = s2.substr(0, 16);
  c = new char[sub2.length() + 1];
  std::strcpy(c, sub2.c_str());
  setCursor(2, 1);
  putString(c);

  // if (s.length() > 16) {
  //   for (int i = 0; i < (s.length() - 16 + 1); i++) {
  //     setCursor(2, 1);
  //     std::strcpy(c, sub.c_str());
  //     putString(c);
  //     vTaskDelay(800);
  // delete[] c;
  //}
  //}
}
