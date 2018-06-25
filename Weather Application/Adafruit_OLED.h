#ifndef _ADAFRUIT_OLED_H
#define _ADAFRUIT_OLED_H

void writeCommand(unsigned char c);
void writeData(unsigned char c);
void Adafruit_Init(void);
void goTo(int x, int y);
unsigned int Color565(unsigned char r, unsigned char g, unsigned char b);
void fillScreen(unsigned int fillcolor);
void fillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int fillcolor);
void drawFastVLine(int x, int y, int h, unsigned int color);
void drawFastHLine(int x, int y, int w, unsigned int color);
void drawPixel(int x, int y, unsigned int color);
void invert(char v);



#endif
