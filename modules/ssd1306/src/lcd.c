#include "lcd.h"
#include "ssd1306.h"

void LCD_Init(void) {
    SSD1306_Init();
}

void LCD_Clear(u16 Color) {
    SSD1306_Fill(Color == BLACK ? SSD1306_COLOR_BLACK : SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();
}

/*
  Function description: display the image
  Entry data: x1, y1:  start coordinates
              x2, y2:  end coordinates
              *image:  pointer to image buffer
  Return value: None
  Note: image buffere contains 16-bit pixel colors
        and its size must be (x2-x1+1) * (y2-y1+1) * 2
  */
void LCD_ShowPicture(u16 x1, u16 y1, u16 x2, u16 y2, u8 * image) {
    u16 color;
    int i = 0;
    for (u16 y = y1; y <= y2; y++) {
        for (u16 x = x1; x <= x2; x++) {
            color = (image[i] << 8) | image[i + 1]; // Combine two bytes into one 16-bit color
            SSD1306_DrawPixel(x, y, color == WHITE ? SSD1306_COLOR_WHITE : SSD1306_COLOR_BLACK);
            i += 2;
        }
    }
    SSD1306_UpdateScreen();
}

void LCD_ShowChar(u16 x, u16 y, char num, u8 mode, u16 color) {
    SSD1306_GotoXY(x, y);
    SSD1306_Putc(num, &Font_7x10, color == BLACK ? SSD1306_COLOR_BLACK : SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();
}

void LCD_ShowString(u16 x, u16 y, const char * p, u16 color) {
    SSD1306_GotoXY(x, y);
    SSD1306_Puts(p, &Font_7x10, color == BLACK ? SSD1306_COLOR_BLACK : SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();
}
