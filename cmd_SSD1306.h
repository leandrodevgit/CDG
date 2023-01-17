#ifndef _cmd_SSD1306_H
#define _cmd_SSD1306_H

//comandos para o SSD1306__________________________________________________________________
#define SSD1306_LCDWIDTH 128
#define SSD1306_LCDHEIGHT 64
#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6  	// Set Normal/Inverse Display
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF
#define SSD1306_SETDISPLAYOFFSET 0xD3   // Set Display Offset
#define SSD1306_SETCOMPINS 0xDA
#define SSD1306_SETVCOMDETECT 0xDB     	// This command adjusts the VCOMH regulator output.
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5 // Set Display Clock Divide Ratio/ Oscillator Frequency
#define SSD1306_SETPRECHARGE 0xD9
#define SSD1306_SETMULTIPLEX 0xA8 		// Set Multiplex Ratio
#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10
#define SSD1306_SETSTARTLINE 0x40 		// Set Display Start Line
#define SSD1306_MEMORYMODE 0x20   		// define como a memoria do ssd ser� acessada
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR  0x22
#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8
#define SSD1306_SEGREMAP 0xA1
#define SSD1306_CHARGEPUMP 0x8D
#define SSD1306_EXTERNALVCC 0x01
#define SSD1306_SWITCHCAPVCC 0x02

#endif