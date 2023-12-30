/*

  Works with Nano and Mini
  I haven't tried it with Uno

*/

#include <Arduino.h>

#include "SimpleILI9341.h"

#include <avr/pgmspace.h>

const unsigned short  alert[32 * 32 + 2] PROGMEM = {
  32, // width
  32, // height
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0840, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 0, 32 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x1080, 0xAC66, 0xEDE8, 0xFE69, 0xC4C6, 0x2901, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 1, 64 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xBCC6, 0xFE68, 0xFE68, 0xFE6A, 0xFE68, 0xEDE8, 0x18A1, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 2, 96 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x8344, 0xFE48, 0xFE8C, 0xFFDD, 0xFFFF, 0xFEF0, 0xFE48, 0xB466, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 3, 128 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x1880, 0xEDC7, 0xFE48, 0xFF99, 0xFFBC, 0xFF9B, 0xFFBD, 0xFE6A, 0xFE48, 0x5A23, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 4, 160 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9BE5, 0xFE28, 0xFED0, 0xFFBC, 0xFF7A, 0xFF9A, 0xFF9B, 0xFF35, 0xFE28, 0xBCA6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 5, 192 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3962, 0xFE28, 0xFE28, 0xFF9A, 0xFF79, 0xFF9A, 0xFF9B, 0xFF9A, 0xFFBD, 0xFE6B, 0xFE28, 0x72E3, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 6, 224 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xB465, 0xFE28, 0xFEF2, 0xFF7A, 0xFF79, 0xFF7A, 0xFF9A, 0xFF7A, 0xFF7A, 0xFF78, 0xFE28, 0xDD67, 0x0860, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 7, 256 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x5A22, 0xFE07, 0xFE29, 0xFF9B, 0xFF37, 0xFF58, 0xFF79, 0xFF79, 0xFF79, 0xFF58, 0xFF9B, 0xFEAE, 0xFE07, 0x93A4, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 8, 288 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xC4A5, 0xFE07, 0xFF15, 0xFF37, 0xFF36, 0xAD11, 0x2965, 0x2965, 0xCDF4, 0xFF37, 0xFF37, 0xFF79, 0xFE07, 0xFE07, 0x2901, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 9, 320 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x7B03, 0xFDE7, 0xFE4B, 0xFF79, 0xFEF4, 0xFF15, 0xB552, 0x2945, 0x2945, 0xDE55, 0xFF16, 0xFF15, 0xFF58, 0xFED1, 0xFDE7, 0xAC25, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 10, 352 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0840, 0xDD26, 0xFDE7, 0xFF57, 0xFED3, 0xFED2, 0xFEF4, 0xBD93, 0x2124, 0x2124, 0xDE75, 0xFF14, 0xFED3, 0xFED3, 0xFF7A, 0xFE08, 0xFDE7, 0x49A2, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 11, 384 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9BA4, 0xFDC6, 0xFE6E, 0xFF36, 0xFE90, 0xFEB1, 0xFED3, 0xC592, 0x2124, 0x2124, 0xE675, 0xFED3, 0xFEB2, 0xFEB1, 0xFEF3, 0xFEF3, 0xFDC6, 0xBC45, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 12, 416 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3141, 0xF5C6, 0xF5C7, 0xFF58, 0xFE90, 0xFE6F, 0xFE8F, 0xFEB1, 0xCDB2, 0x2104, 0x2104, 0xF6B4, 0xFEB1, 0xFE90, 0xFE8F, 0xFE90, 0xFF58, 0xFE0A, 0xF5C6, 0x72A3, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 13, 448 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xABE4, 0xF5A6, 0xFEB1, 0xFED3, 0xFE4E, 0xFE6E, 0xFE6F, 0xFE90, 0xD5F2, 0x18E3, 0x18E3, 0xFED4, 0xFE90, 0xFE6F, 0xFE6F, 0xFE6E, 0xFE91, 0xFF36, 0xF5A6, 0xCCA5, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 14, 480 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x5202, 0xF5A6, 0xF5C7, 0xFF58, 0xFE4D, 0xFE4D, 0xFE4D, 0xFE4E, 0xFE6F, 0xDE11, 0x18C3, 0x18C3, 0xFED3, 0xFE6F, 0xFE6E, 0xFE4E, 0xFE4D, 0xFE4D, 0xFF16, 0xFE2C, 0xF5A6, 0x9363, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 15, 512 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xBC44, 0xF585, 0xFED3, 0xFE6F, 0xFE2C, 0xFE2C, 0xFE2D, 0xFE4D, 0xFE4E, 0xE630, 0x10A2, 0x2104, 0xFED1, 0xFE4E, 0xFE4D, 0xFE4D, 0xFE2D, 0xFE2C, 0xFE4D, 0xFF37, 0xF586, 0xF585, 0x28E1, 0x0000, 0x0000, 0x0000, 0x0000, // row 16, 544 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x7282, 0xF565, 0xF5EA, 0xFF16, 0xFE0B, 0xFE0B, 0xFE0B, 0xFE2C, 0xFE2C, 0xFE4D, 0xF670, 0x1082, 0x2924, 0xFEB0, 0xFE2D, 0xFE2C, 0xFE2C, 0xFE2C, 0xFE0B, 0xFE0B, 0xFEB2, 0xFE6F, 0xF565, 0xA383, 0x0000, 0x0000, 0x0000, 0x0000, // row 17, 576 pixels
  0x0000, 0x0000, 0x0000, 0x0840, 0xD4C4, 0xF565, 0xFEF5, 0xFE0C, 0xFDE9, 0xFDEA, 0xFE0A, 0xFE0B, 0xFE0B, 0xFE2C, 0xFE8F, 0x0861, 0x2964, 0xFE8F, 0xFE2C, 0xFE0B, 0xFE0B, 0xFE0B, 0xFE0A, 0xFDEA, 0xFE0B, 0xFF37, 0xF586, 0xF565, 0x4181, 0x0000, 0x0000, 0x0000, // row 18, 608 pixels
  0x0000, 0x0000, 0x0000, 0x9343, 0xF545, 0xF60C, 0xFED3, 0xFDC8, 0xFDC8, 0xFDC9, 0xFDE9, 0xFDEA, 0xFDEA, 0xFE0B, 0xFE8E, 0x0861, 0x3184, 0xFE6D, 0xFE0B, 0xFE0A, 0xFDEA, 0xFDEA, 0xFDE9, 0xFDC9, 0xFDC9, 0xFE4E, 0xFEB2, 0xF545, 0xB3E3, 0x0000, 0x0000, 0x0000, // row 19, 640 pixels
  0x0000, 0x0000, 0x28E0, 0xF544, 0xF545, 0xFF17, 0xFDC8, 0xFDA7, 0xFDA7, 0xFDC8, 0xFDC8, 0xFDC9, 0xFDC9, 0xFDE9, 0xFE6C, 0x10A2, 0x39C4, 0xFE4C, 0xFDEA, 0xFDE9, 0xFDC9, 0xFDC9, 0xFDC8, 0xFDC8, 0xFDA7, 0xFDA8, 0xFF16, 0xF588, 0xF544, 0x6222, 0x0000, 0x0000, // row 20, 672 pixels
  0x0000, 0x0000, 0xA383, 0xF524, 0xF64E, 0xFE4E, 0xFD86, 0xFD86, 0xFD87, 0xFDA7, 0xFDA7, 0xFDA8, 0xFDC8, 0xFDC8, 0xFE2A, 0xA469, 0xB4EA, 0xFE2A, 0xFDC9, 0xFDC8, 0xFDC8, 0xFDA8, 0xFDA7, 0xFDA7, 0xFD87, 0xFD86, 0xFDEA, 0xFED3, 0xF524, 0xC443, 0x0000, 0x0000, // row 21, 704 pixels
  0x0000, 0x51C1, 0xF504, 0xF546, 0xFF16, 0xF565, 0xFD65, 0xFD65, 0xFD86, 0xFD86, 0xFD86, 0xFDA7, 0xFDA7, 0xFDA7, 0xFDE8, 0xFE6A, 0xFE4A, 0xFDE8, 0xFDA7, 0xFDA7, 0xFDA7, 0xFDA7, 0xFD86, 0xFD86, 0xFD86, 0xFD65, 0xFD65, 0xFEB2, 0xF5CA, 0xF504, 0x8AE2, 0x0000, // row 22, 736 pixels
  0x0000, 0xB3A2, 0xED03, 0xFE92, 0xFDC9, 0xF543, 0xF544, 0xFD44, 0xFD65, 0xFD65, 0xFD65, 0xFD86, 0xFD86, 0xFD86, 0xFDA7, 0xFDC7, 0xFDC7, 0xFDA7, 0xFD86, 0xFD86, 0xFD86, 0xFD86, 0xFD65, 0xFD65, 0xFD65, 0xFD44, 0xF544, 0xFD86, 0xFEF5, 0xED03, 0xE4C3, 0x1880, // row 23, 768 pixels
  0x7241, 0xECE3, 0xF567, 0xFED3, 0xF523, 0xF523, 0xF523, 0xF543, 0xF544, 0xF544, 0xFD65, 0xFD65, 0xFD65, 0xFD65, 0xD4E6, 0x39C5, 0x39A5, 0xD4E6, 0xFD86, 0xFD65, 0xFD65, 0xFD65, 0xFD65, 0xF544, 0xF544, 0xF543, 0xF523, 0xF523, 0xFE2E, 0xF5EC, 0xECE3, 0x9B42, // row 24, 800 pixels
  0xD443, 0xECE3, 0xFED4, 0xF565, 0xF502, 0xF502, 0xF522, 0xF523, 0xF523, 0xF543, 0xF544, 0xF544, 0xF544, 0xFD65, 0x8B64, 0x18C3, 0x18C3, 0x8344, 0xFD85, 0xFD44, 0xF544, 0xF544, 0xF544, 0xF543, 0xF523, 0xF523, 0xF522, 0xF502, 0xF523, 0xFEF5, 0xED04, 0xECE3, // row 25, 832 pixels
  0xECC3, 0xF5AB, 0xFE6F, 0xF501, 0xF4E1, 0xF501, 0xF502, 0xF502, 0xF522, 0xF522, 0xF523, 0xF523, 0xF523, 0xFD84, 0xC504, 0x20E1, 0x18E1, 0xC4E4, 0xFD84, 0xF543, 0xF523, 0xF523, 0xF523, 0xF522, 0xF522, 0xF502, 0xF502, 0xF501, 0xF501, 0xFDC9, 0xF62F, 0xECC3, // row 26, 864 pixels
  0xECC2, 0xFE92, 0xF523, 0xF4E0, 0xF4E0, 0xF4E1, 0xF4E1, 0xF501, 0xF501, 0xF502, 0xF502, 0xF522, 0xF522, 0xF543, 0xFDE3, 0xFEA5, 0xF6A4, 0xFE04, 0xF543, 0xF522, 0xF522, 0xF522, 0xF502, 0xF502, 0xF501, 0xF501, 0xF4E1, 0xF4E1, 0xF4E0, 0xF4E1, 0xFED4, 0xECC2, // row 27, 896 pixels
  0xECA2, 0xF5EC, 0xF4E0, 0xF4C0, 0xF4E0, 0xF4E0, 0xF4E0, 0xF4E1, 0xF4E1, 0xF501, 0xF501, 0xF501, 0xF502, 0xF502, 0xF542, 0xFDA2, 0xFDA2, 0xF542, 0xF502, 0xF502, 0xF502, 0xF501, 0xF501, 0xF501, 0xF4E1, 0xF4E1, 0xF4E0, 0xF4E0, 0xF4E0, 0xF4C0, 0xF5A9, 0xECA2, // row 28, 928 pixels
  0xECA2, 0xECA2, 0xECC2, 0xF4C1, 0xF4C1, 0xF4C1, 0xF4C1, 0xF4C1, 0xF4C1, 0xF4C1, 0xF4C1, 0xF4E1, 0xF4E2, 0xF4E2, 0xF4E2, 0xF4E2, 0xF4E2, 0xF4E2, 0xF4E2, 0xF4E2, 0xF4E2, 0xF4E1, 0xF4C1, 0xF4C1, 0xF4C1, 0xF4C1, 0xF4C1, 0xF4C1, 0xF4C1, 0xECC2, 0xECC3, 0xECA2, // row 29, 960 pixels
  0x8AC1, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0xEC82, 0x9B01, // row 30, 992 pixels
  0x0000, 0x1880, 0x51A0, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x8AA1, 0x61E0, 0x28E0, 0x0000
}; // row 31, 1024 pixels

const byte VeryBigFont[] PROGMEM = {
  21, // ymax
  5, // descender
  ' ', // first char
  10, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  //  
  3, 0xE0,0xF8,0x0F,0xE0,0xFC,0x0F,0xE0,0xF0,0x0F,  // !
  9, 0x00,0xC0,0x0F,0x00,0xE0,0x0F,0x00,0xC0,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0x0F,0x00,0xE0,0x0F,0x00,0xC0,0x0F,  // "
  11, 0x00,0x82,0x00,0xE0,0x83,0x00,0xC0,0xFF,0x00,0x00,0xFA,0x07,0x00,0x82,0x0F,0x00,0x82,0x00,0xE0,0x83,0x00,0xC0,0xBF,0x00,0x00,0xFE,0x07,0x00,0x82,0x0F,0x00,0x82,0x00,  // #
  9, 0xE0,0xC1,0x07,0xE0,0xE1,0x0F,0x60,0xF0,0x0D,0x20,0xF8,0x08,0xF0,0xFF,0x1F,0x60,0x3C,0x0C,0x60,0x1E,0x0F,0xE0,0x1F,0x0F,0xC0,0x0F,0x00,  // $
  21, 0x00,0xF0,0x07,0x00,0xF8,0x0F,0x00,0xF8,0x0F,0x00,0x18,0x0C,0x00,0x18,0x0C,0x20,0xF8,0x0F,0xE0,0xF8,0x0F,0x80,0xF1,0x07,0x00,0x07,0x00,0x00,0x0C,0x00,0x00,0x38,0x00,0x00,0x60,0x00,0x00,0xC0,0x01,0xC0,0x1F,0x03,0xE0,0x3F,0x0E,0xE0,0x3F,0x08,0x60,0x30,0x00,0x60,0x30,0x00,0xE0,0x3F,0x00,0xE0,0x3F,0x00,0x80,0x1F,0x00,  // %
  17, 0xC0,0x07,0x00,0xE0,0x0F,0x00,0xE0,0x1F,0x00,0xE0,0x3B,0x00,0xE0,0xF0,0x07,0xE0,0xF8,0x0F,0x60,0xFC,0x0F,0x60,0xFE,0x0C,0xC0,0x7F,0x08,0xC0,0xCF,0x0E,0xC0,0xC7,0x0F,0xE0,0xB3,0x07,0xE0,0x3F,0x00,0xE0,0x3C,0x00,0x60,0x38,0x00,0xE0,0x30,0x00,0xC0,0x00,0x00,  // &
  3, 0x00,0xC0,0x0F,0x00,0xE0,0x0F,0x00,0xC0,0x0F,  // '
  7, 0xC0,0x7F,0x00,0xF0,0xFF,0x00,0xF8,0xFF,0x03,0x7C,0xE0,0x03,0x0E,0x00,0x07,0x03,0x00,0x0C,0x03,0x00,0x0C,  // (
  7, 0x03,0x00,0x0C,0x03,0x00,0x0C,0x0E,0x00,0x07,0x7C,0xE0,0x03,0xF8,0xFF,0x01,0xF0,0xFF,0x00,0xC0,0x3F,0x00,  // )
  7, 0x00,0x60,0x03,0x00,0xE0,0x03,0x00,0xE0,0x0F,0x00,0x80,0x0F,0x00,0xE0,0x0F,0x00,0xE0,0x03,0x00,0x60,0x03,  // *
  11, 0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x80,0xFF,0x03,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,  // +
  3, 0xE6,0x00,0x00,0xEE,0x00,0x00,0xFC,0x00,0x00,  // ,
  7, 0x00,0x02,0x00,0x00,0x02,0x00,0x00,0x02,0x00,0x00,0x02,0x00,0x00,0x02,0x00,0x00,0x02,0x00,0x00,0x02,0x00,  // -
  3, 0xE0,0x00,0x00,0xE0,0x00,0x00,0xE0,0x00,0x00,  // .
  6, 0xE0,0x00,0x00,0xE0,0x07,0x00,0x00,0x3F,0x00,0x00,0xF8,0x01,0x00,0xC0,0x0F,0x00,0x00,0x0E,  // /
  10, 0x00,0xFF,0x01,0xC0,0xFF,0x07,0xE0,0xFF,0x0F,0xE0,0x00,0x0E,0x20,0x00,0x08,0x20,0x00,0x08,0xE0,0x00,0x0E,0xE0,0xFF,0x0F,0xC0,0xFF,0x07,0x00,0xFF,0x01,  // 0
  8, 0x20,0x00,0x06,0x20,0x00,0x06,0x60,0x00,0x06,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x60,0x00,0x00,0x20,0x00,0x00,  // 1
  10, 0x20,0x80,0x01,0xE0,0x80,0x07,0xE0,0x01,0x0F,0xE0,0x03,0x0E,0xE0,0x07,0x0F,0xE0,0xBE,0x0F,0xE0,0xFC,0x0F,0xE0,0xF0,0x0F,0xE0,0xE1,0x07,0xC0,0x01,0x00,  // 2
  10, 0xE0,0x01,0x00,0xE0,0x01,0x02,0xE0,0x01,0x06,0xE0,0x00,0x0C,0x60,0x70,0x0C,0x60,0xF0,0x0E,0xE0,0xFC,0x0F,0xC0,0xFF,0x0F,0xC0,0xBF,0x07,0x00,0x1F,0x00,  // 3
  10, 0x00,0x07,0x00,0x00,0x1F,0x00,0x00,0x3B,0x00,0x00,0x63,0x00,0x00,0xC3,0x00,0x00,0x83,0x03,0xE0,0xFF,0x07,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x00,0x03,0x00,  // 4
  10, 0xE0,0x60,0x00,0xE0,0xE0,0x03,0xE0,0xE0,0x0F,0xE0,0xF0,0x0E,0x60,0xF0,0x0E,0x60,0xF0,0x0E,0x20,0xF8,0x0E,0x60,0x78,0x0E,0xC0,0x3E,0x0E,0xC0,0x1F,0x0E,  // 5
  10, 0x80,0x7F,0x00,0xC0,0xFF,0x00,0xE0,0xFF,0x01,0xE0,0xF9,0x03,0x60,0xF0,0x07,0x20,0x20,0x0F,0x60,0x3C,0x0E,0xE0,0x3F,0x0C,0xC0,0x3F,0x0C,0x80,0x1F,0x0C,  // 6
  10, 0x00,0x80,0x07,0x00,0x00,0x0F,0x00,0x00,0x0F,0x20,0x00,0x0E,0xE0,0x01,0x0E,0x80,0x07,0x0E,0x00,0x3E,0x0E,0x00,0xF0,0x0F,0x00,0x80,0x0F,0x00,0x00,0x0E,  // 7
  10, 0xC0,0xC7,0x07,0xE0,0xEF,0x07,0xE0,0xFF,0x0F,0xE0,0xFC,0x0D,0x60,0x78,0x08,0x20,0x7C,0x08,0x60,0x7E,0x0E,0xE0,0xFF,0x0F,0xC0,0xDF,0x0F,0x80,0x8F,0x07,  // 8
  10, 0x60,0xF0,0x03,0x60,0xF8,0x07,0x60,0xF8,0x0F,0xE0,0x78,0x0C,0xE0,0x18,0x08,0xC0,0x09,0x0C,0x80,0x1F,0x0F,0x80,0xFF,0x0F,0x00,0xFE,0x07,0x00,0xFC,0x03,  // 9
  3, 0xE0,0xE0,0x00,0xE0,0xE0,0x00,0xE0,0xE0,0x00,  // :
  3, 0xE6,0xE0,0x00,0xEE,0xE0,0x00,0xFC,0xE0,0x00,  // ;
  12, 0x00,0x38,0x00,0x00,0x38,0x00,0x00,0x28,0x00,0x00,0x6C,0x00,0x00,0x6C,0x00,0x00,0xC6,0x00,0x00,0xC6,0x00,0x00,0x83,0x01,0x00,0x83,0x01,0x00,0x01,0x01,0x80,0x01,0x03,0x80,0x01,0x03,  // <
  11, 0x00,0x48,0x00,0x00,0x48,0x00,0x00,0x48,0x00,0x00,0x48,0x00,0x00,0x48,0x00,0x00,0x48,0x00,0x00,0x48,0x00,0x00,0x48,0x00,0x00,0x48,0x00,0x00,0x48,0x00,0x00,0x48,0x00,  // =
  12, 0x80,0x01,0x03,0x80,0x01,0x03,0x00,0x01,0x01,0x00,0x83,0x01,0x00,0x83,0x01,0x00,0xC6,0x00,0x00,0xC6,0x00,0x00,0x6C,0x00,0x00,0x6C,0x00,0x00,0x28,0x00,0x00,0x38,0x00,0x00,0x38,0x00,  // >
  9, 0x00,0x00,0x07,0x00,0x00,0x0F,0x00,0x00,0x0F,0xE0,0x00,0x08,0xE0,0x1C,0x0C,0xE0,0x78,0x0E,0x00,0xF0,0x0F,0x00,0xE0,0x0F,0x00,0xC0,0x07,  // ?
  20, 0xE0,0x3F,0x00,0xF0,0xF8,0x00,0x18,0xC0,0x01,0x0C,0x80,0x03,0xE6,0x0F,0x07,0xE3,0x1F,0x06,0xE3,0x7F,0x0C,0x63,0x7F,0x0C,0x61,0xF0,0x0C,0xC1,0xC1,0x08,0xE1,0xCF,0x08,0xE1,0x7F,0x08,0xE3,0xFF,0x0C,0x63,0xFF,0x0C,0x63,0xF0,0x0C,0x66,0x80,0x06,0xC6,0x00,0x07,0x8C,0xC7,0x03,0x18,0xFE,0x00,0x30,0x00,0x00,  // @
  16, 0x60,0x00,0x00,0x60,0x00,0x00,0xE0,0x01,0x00,0xE0,0x07,0x00,0x20,0x3E,0x00,0x00,0xF2,0x00,0x00,0xC2,0x07,0x00,0xC2,0x0F,0x00,0xFA,0x0F,0x20,0xFE,0x07,0xE0,0xFF,0x00,0xE0,0x3F,0x00,0xE0,0x07,0x00,0xE0,0x01,0x00,0x60,0x00,0x00,0x20,0x00,0x00,  // A
  14, 0x20,0x00,0x08,0x60,0x00,0x0C,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x60,0x10,0x08,0x20,0x10,0x08,0x20,0x30,0x0C,0x60,0x30,0x0C,0x60,0x78,0x0E,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xC0,0xCF,0x07,0x80,0x07,0x00,  // B
  14, 0x00,0xFE,0x00,0x80,0xFF,0x01,0xC0,0xFF,0x03,0xC0,0xFF,0x07,0xE0,0x01,0x0F,0x60,0x00,0x0C,0x60,0x00,0x0C,0x20,0x00,0x08,0x20,0x00,0x08,0x20,0x00,0x0C,0x60,0x00,0x0C,0x60,0x00,0x0E,0xC0,0x80,0x0F,0xC0,0x81,0x0F,  // C
  15, 0x20,0x00,0x08,0x60,0x00,0x0C,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x60,0x00,0x08,0x20,0x00,0x08,0x20,0x00,0x0C,0x60,0x00,0x0C,0x60,0x00,0x0C,0xE0,0x01,0x0F,0xC0,0xFF,0x07,0xC0,0xFF,0x07,0x80,0xFF,0x03,0x00,0xFE,0x00,  // D
  13, 0x20,0x00,0x08,0x60,0x00,0x0C,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x60,0x10,0x08,0x20,0x38,0x08,0x20,0xFE,0x08,0x20,0xFF,0x0D,0x60,0x00,0x0C,0x60,0x00,0x0F,0xE0,0x80,0x0F,0xE0,0x03,0x00,  // E
  13, 0x20,0x00,0x08,0x60,0x00,0x0C,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x60,0x10,0x08,0x20,0x38,0x08,0x00,0xFE,0x08,0x00,0xFF,0x09,0x00,0x00,0x0C,0x00,0x00,0x0C,0x00,0x00,0x0F,0x00,0x80,0x0F,  // F
  17, 0x00,0xFE,0x00,0x80,0xFF,0x01,0x80,0xFF,0x03,0xC0,0xFF,0x07,0xE0,0x01,0x0F,0x60,0x00,0x0C,0x60,0x00,0x0C,0x60,0x00,0x08,0x20,0x00,0x08,0x20,0x00,0x0C,0x20,0x04,0x0C,0x60,0x06,0x0C,0xE0,0x07,0x0E,0xE0,0x87,0x0F,0xC0,0x87,0x0F,0x00,0x06,0x00,0x00,0x04,0x00,  // G
  16, 0x20,0x00,0x08,0x60,0x00,0x0C,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x60,0x10,0x0C,0x20,0x10,0x08,0x00,0x10,0x00,0x00,0x10,0x00,0x20,0x10,0x08,0x60,0x10,0x0C,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x60,0x00,0x0C,0x20,0x00,0x08,  // H
  7, 0x20,0x00,0x08,0x60,0x00,0x0C,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x60,0x00,0x0C,0x20,0x00,0x08,  // I
  12, 0xC0,0x03,0x00,0xE0,0x03,0x00,0xE0,0x03,0x00,0x20,0x00,0x00,0x20,0x00,0x00,0x20,0x00,0x08,0x60,0x00,0x0C,0xE0,0xFF,0x0F,0xC0,0xFF,0x0F,0xC0,0xFF,0x0F,0x00,0x00,0x0C,0x00,0x00,0x08,  // J
  17, 0x20,0x00,0x08,0x60,0x00,0x0C,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x60,0x18,0x0C,0x20,0x3C,0x08,0x00,0x7E,0x00,0x00,0xFF,0x00,0xA0,0x8F,0x01,0xE0,0x87,0x09,0xE0,0x03,0x0F,0xE0,0x01,0x0E,0xE0,0x00,0x0C,0x60,0x00,0x0C,0x60,0x00,0x08,0x20,0x00,0x08,  // K
  14, 0x20,0x00,0x08,0x60,0x00,0x0C,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x60,0x00,0x0C,0x20,0x00,0x08,0x20,0x00,0x00,0x20,0x00,0x00,0x60,0x00,0x00,0x60,0x00,0x00,0xE0,0x00,0x00,0xE0,0x07,0x00,0xC0,0x07,0x00,  // L
  21, 0x20,0x00,0x08,0x60,0x00,0x0C,0xE0,0xFF,0x0F,0x60,0x00,0x0E,0x20,0xC0,0x0F,0x00,0xF0,0x0F,0x00,0xFC,0x07,0x80,0xFF,0x01,0xE0,0x3F,0x00,0xE0,0x0F,0x00,0xC0,0x03,0x00,0x00,0x0F,0x00,0x00,0x38,0x00,0x00,0xE0,0x01,0x20,0x80,0x07,0x60,0x00,0x0E,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x60,0x00,0x0C,0x20,0x00,0x08,  // M
  15, 0x20,0x00,0x08,0x60,0x00,0x08,0xE0,0xFF,0x0F,0x60,0x00,0x0E,0x20,0x80,0x0F,0x00,0xC0,0x0F,0x00,0xE0,0x07,0x00,0xF8,0x01,0x00,0xFC,0x00,0x00,0x7E,0x00,0x80,0x1F,0x08,0xC0,0x0F,0x0C,0xE0,0xFF,0x0F,0x00,0x00,0x0C,0x00,0x00,0x08,  // N
  16, 0x00,0xFE,0x00,0x80,0xFF,0x03,0xC0,0xFF,0x07,0xC0,0xFF,0x07,0xE0,0x01,0x0F,0x60,0x00,0x0C,0x60,0x00,0x0C,0x20,0x00,0x08,0x20,0x00,0x08,0x60,0x00,0x0C,0x60,0x00,0x0C,0xE0,0x00,0x0E,0xC0,0xFF,0x07,0xC0,0xFF,0x07,0x80,0xFF,0x03,0x00,0xFE,0x00,  // O
  13, 0x20,0x00,0x08,0x60,0x00,0x0C,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x60,0x10,0x08,0x20,0x10,0x08,0x00,0x30,0x0C,0x00,0x30,0x0C,0x00,0x70,0x0E,0x00,0xF0,0x0F,0x00,0xE0,0x07,0x00,0xC0,0x03,  // P
  16, 0x00,0xFE,0x00,0x80,0xFF,0x03,0xC0,0xFF,0x07,0xC0,0xFF,0x07,0xE0,0x01,0x0F,0x60,0x00,0x0C,0x70,0x00,0x0C,0x38,0x00,0x08,0x3C,0x00,0x08,0x7E,0x00,0x0C,0x6E,0x00,0x0C,0xE6,0x01,0x0E,0xC6,0xFF,0x07,0xC2,0xFF,0x07,0x82,0xFF,0x03,0x00,0xFE,0x00,  // Q
  16, 0x20,0x00,0x08,0x60,0x00,0x0C,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x60,0x10,0x08,0x20,0x18,0x08,0x00,0x1C,0x08,0x00,0x3F,0x0C,0x80,0x3F,0x0C,0xC0,0x7F,0x0E,0xE0,0xF3,0x0F,0xE0,0xE1,0x07,0xE0,0xE0,0x03,0x60,0x00,0x00,0x20,0x00,0x00,  // R
  11, 0xE0,0xC7,0x03,0xE0,0xE7,0x07,0xC0,0xF0,0x0F,0x60,0xF8,0x0D,0x60,0xF8,0x08,0x20,0x7C,0x08,0x60,0x7C,0x0C,0xE0,0x3E,0x0E,0xE0,0x9F,0x0F,0xC0,0x9F,0x0F,0x80,0x0F,0x00,  // S
  13, 0x00,0x00,0x0E,0x00,0x00,0x0E,0x00,0x00,0x0C,0x20,0x00,0x08,0x60,0x00,0x08,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x60,0x00,0x08,0x20,0x00,0x08,0x00,0x00,0x0C,0x00,0x00,0x0E,0x00,0x00,0x0E,  // T
  15, 0x00,0x00,0x08,0x00,0x00,0x0C,0x80,0xFF,0x0F,0xC0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0x00,0x0C,0x60,0x00,0x08,0x20,0x00,0x00,0x20,0x00,0x00,0x60,0x00,0x00,0x60,0x00,0x08,0xC0,0x01,0x0C,0x80,0xFF,0x0F,0x00,0x00,0x0C,0x00,0x00,0x08,  // U
  17, 0x00,0x00,0x08,0x00,0x00,0x0C,0x00,0x00,0x0E,0x00,0x80,0x0F,0x00,0xF0,0x0F,0x00,0xFC,0x0F,0x80,0xFF,0x09,0xE0,0x7F,0x08,0xE0,0x0F,0x00,0xE0,0x03,0x00,0x00,0x0F,0x00,0x00,0x7C,0x08,0x00,0xE0,0x0D,0x00,0x00,0x0F,0x00,0x00,0x0E,0x00,0x00,0x0C,0x00,0x00,0x08,  // V
  23, 0x00,0x00,0x08,0x00,0x00,0x0C,0x00,0x00,0x0F,0x00,0xE0,0x0F,0x00,0xFC,0x0F,0x00,0xFF,0x0F,0xE0,0xFF,0x08,0xE0,0x1F,0x00,0xC0,0x07,0x08,0x00,0x1F,0x0C,0x00,0xF8,0x0E,0x00,0xC0,0x0F,0x00,0xF8,0x0F,0x00,0xFF,0x0F,0xE0,0xFF,0x09,0xE0,0x3F,0x00,0xC0,0x07,0x00,0x00,0x1F,0x00,0x00,0xF8,0x08,0x00,0xC0,0x0F,0x00,0x00,0x0E,0x00,0x00,0x08,0x00,0x00,0x08,  // W
  15, 0x60,0x00,0x08,0x60,0x00,0x0C,0xE0,0x00,0x0E,0xE0,0x81,0x0F,0x20,0xC3,0x0F,0x20,0xF6,0x0F,0x00,0xFC,0x09,0x00,0xFE,0x00,0xA0,0x7F,0x00,0xE0,0xDF,0x09,0xE0,0x87,0x0F,0xE0,0x01,0x0E,0xE0,0x00,0x0C,0x60,0x00,0x0C,0x20,0x00,0x08,  // X
  15, 0x00,0x00,0x08,0x00,0x00,0x0C,0x00,0x00,0x0E,0x00,0x80,0x0F,0x20,0xE0,0x0F,0x60,0xF8,0x0F,0xE0,0xFF,0x09,0xE0,0x7F,0x00,0xE0,0x1F,0x00,0x60,0x78,0x00,0x20,0xE0,0x09,0x00,0x80,0x0F,0x00,0x00,0x0F,0x00,0x00,0x0C,0x00,0x00,0x08,  // Y
  14, 0x60,0x00,0x00,0xE0,0x80,0x0F,0xE0,0x03,0x0F,0xE0,0x07,0x0C,0xE0,0x1F,0x0C,0xA0,0x3F,0x08,0x20,0xFE,0x08,0x20,0xF8,0x0B,0x20,0xF0,0x0F,0x60,0xC0,0x0F,0x60,0x80,0x0F,0xE0,0x00,0x0E,0xE0,0x03,0x0C,0xE0,0x03,0x00,  // Z
  5, 0xFF,0xFF,0x0F,0xFF,0xFF,0x0F,0xFF,0xFF,0x0F,0x03,0x00,0x0C,0x01,0x00,0x08,  // [
  6, 0x00,0x00,0x0E,0x00,0xC0,0x0F,0x00,0xF8,0x01,0x00,0x3F,0x00,0xE0,0x07,0x00,0xE0,0x00,0x00,
  5, 0x01,0x00,0x08,0x03,0x00,0x0C,0xFF,0xFF,0x0F,0xFF,0xFF,0x0F,0xFF,0xFF,0x0F,  // ]
  11, 0x00,0x20,0x00,0x00,0xE0,0x00,0x00,0x80,0x03,0x00,0x00,0x07,0x00,0x00,0x1C,0x00,0x00,0x18,0x00,0x00,0x1C,0x00,0x00,0x07,0x00,0x80,0x03,0x00,0xE0,0x00,0x00,0x20,0x00,  // ^
  12, 0x01,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,  // _
  4, 0x00,0x00,0x08,0x00,0x00,0x0C,0x00,0x00,0x0E,0x00,0x00,0x0E,  // `
  10, 0xE0,0x71,0x00,0xE0,0xFB,0x00,0xE0,0xFF,0x00,0xE0,0x86,0x00,0x60,0x8C,0x00,0xC0,0xCC,0x00,0xE0,0xFF,0x00,0xE0,0xFF,0x00,0xE0,0x7F,0x00,0x20,0x00,0x00,  // a
  11, 0x00,0x00,0x08,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x60,0xC0,0x00,0x20,0x80,0x00,0x60,0x80,0x00,0xE0,0xE1,0x00,0xE0,0xFF,0x00,0xC0,0x7F,0x00,0x80,0x3F,0x00,  // b
  9, 0x80,0x3F,0x00,0xC0,0x7F,0x00,0xE0,0xFF,0x00,0xE0,0xE3,0x00,0x60,0x80,0x00,0x20,0xC0,0x00,0x60,0xE0,0x00,0xE0,0xE0,0x00,0xC0,0x60,0x00,  // c
  11, 0x80,0x3F,0x00,0xC0,0x7F,0x00,0xE0,0xFF,0x00,0xE0,0xE1,0x00,0x20,0x80,0x00,0x60,0x80,0x00,0xC0,0xC0,0x08,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x60,0x00,0x00,  // d
  9, 0x80,0x3F,0x00,0xC0,0x7F,0x00,0xE0,0xFF,0x00,0xE0,0xEB,0x00,0x60,0x88,0x00,0x20,0xC8,0x00,0x60,0xF8,0x00,0xC0,0x78,0x00,0x80,0x38,0x00,  // e
  9, 0x00,0x80,0x00,0x20,0x80,0x00,0xE0,0xFF,0x07,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x20,0x80,0x08,0x00,0x80,0x0E,0x00,0x00,0x0E,0x00,0x00,0x0E,  // f
  10, 0x7F,0x7E,0x00,0xFF,0xFF,0x00,0xFB,0xFF,0x00,0x39,0xE3,0x00,0x39,0x81,0x00,0x39,0xE3,0x00,0x39,0xFF,0x00,0x3B,0xFE,0x00,0x3F,0xFC,0x00,0x1E,0x80,0x00,  // g
  11, 0x20,0x00,0x08,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x20,0xC0,0x00,0x00,0x80,0x00,0x60,0xC0,0x00,0xE0,0xFF,0x00,0xE0,0xFF,0x00,0xE0,0x7F,0x00,0x20,0x00,0x00,  // h
  5, 0x20,0x80,0x00,0xE0,0xFF,0x0E,0xE0,0xFF,0x0E,0xE0,0xFF,0x0E,0x20,0x00,0x00,  // i
  5, 0x07,0x00,0x00,0x03,0x80,0x00,0xFF,0xFF,0x0E,0xFF,0xFF,0x0E,0xFE,0xFF,0x0E,  // j
  12, 0x20,0x00,0x08,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x20,0x06,0x00,0x00,0x0F,0x00,0xA0,0x9F,0x00,0xE0,0xFF,0x00,0xE0,0xF3,0x00,0xE0,0xE1,0x00,0xE0,0xC0,0x00,0x60,0x80,0x00,  // k
  5, 0x20,0x00,0x08,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0xE0,0xFF,0x0F,0x20,0x00,0x00,  // l
  17, 0x20,0x80,0x00,0xE0,0xFF,0x00,0xE0,0xFF,0x00,0xE0,0xFF,0x00,0x20,0xC0,0x00,0x00,0x80,0x00,0x20,0xC0,0x00,0xE0,0xFF,0x00,0xE0,0xFF,0x00,0xE0,0xFF,0x00,0x20,0xE0,0x00,0x00,0xC0,0x00,0x60,0xC0,0x00,0xE0,0xFF,0x00,0xE0,0xFF,0x00,0xE0,0x7F,0x00,0x20,0x00,0x00,  // m
  11, 0x20,0x80,0x00,0xE0,0xFF,0x00,0xE0,0xFF,0x00,0xE0,0xFF,0x00,0x20,0xC0,0x00,0x00,0x80,0x00,0x60,0xC0,0x00,0xE0,0xFF,0x00,0xE0,0xFF,0x00,0xE0,0x7F,0x00,0x20,0x00,0x00,  // n
  10, 0x80,0x3F,0x00,0xC0,0x7F,0x00,0xE0,0xFF,0x00,0xE0,0xE0,0x00,0x20,0x80,0x00,0x20,0x80,0x00,0xE0,0xE0,0x00,0xE0,0xFF,0x00,0xC0,0x7F,0x00,0x80,0x3F,0x00,  // o
  11, 0x01,0x80,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x61,0xE0,0x00,0x60,0xC0,0x00,0x20,0x80,0x00,0xE0,0xF0,0x00,0xE0,0xFF,0x00,0xC0,0x7F,0x00,0x80,0x3F,0x00,  // p
  11, 0x80,0x3F,0x00,0xC0,0x7F,0x00,0xE0,0xFF,0x00,0xE0,0xF0,0x00,0x20,0xC0,0x00,0x60,0x80,0x00,0x61,0xE0,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x01,0x00,0x00,  // q
  8, 0x20,0x80,0x00,0xE0,0xFF,0x00,0xE0,0xFF,0x00,0xE0,0xFF,0x00,0x20,0x70,0x00,0x00,0xC0,0x00,0x00,0xE0,0x00,0x00,0xE0,0x00,  // r
  7, 0xE0,0x78,0x00,0xE0,0xFC,0x00,0x60,0xFE,0x00,0x20,0xDF,0x00,0x60,0xEF,0x00,0xE0,0xE7,0x00,0xC0,0x07,0x00,  // s
  7, 0x00,0x80,0x00,0xC0,0xFF,0x01,0xE0,0xFF,0x07,0xE0,0xFF,0x0F,0x60,0x80,0x00,0x60,0x80,0x00,0x40,0x00,0x00,  // t
  11, 0x00,0x80,0x00,0xC0,0xFF,0x00,0xE0,0xFF,0x00,0xE0,0xFF,0x00,0x60,0x00,0x00,0x20,0x00,0x00,0xE0,0x80,0x00,0xE0,0xFF,0x00,0xE0,0xFF,0x00,0xE0,0xFF,0x00,0x20,0x00,0x00,  // u
  12, 0x00,0xC0,0x00,0x00,0xE0,0x00,0x00,0xFC,0x00,0x80,0xFF,0x00,0xE0,0xFF,0x00,0xE0,0x9F,0x00,0xC0,0x03,0x00,0x00,0x8F,0x00,0x00,0xFC,0x00,0x00,0xE0,0x00,0x00,0xC0,0x00,0x00,0x80,0x00,  // v
  17, 0x00,0x80,0x00,0x00,0xE0,0x00,0x00,0xF8,0x00,0x00,0xFF,0x00,0xE0,0xFF,0x00,0xE0,0x9F,0x00,0xE0,0x87,0x00,0x00,0xFF,0x00,0x00,0xF8,0x00,0x00,0xFF,0x00,0xE0,0xFF,0x00,0xE0,0xBF,0x00,0xE0,0x03,0x00,0x00,0x9F,0x00,0x00,0xFC,0x00,0x00,0xE0,0x00,0x00,0x80,0x00,  // w
  11, 0x20,0x80,0x00,0x60,0xC0,0x00,0xE0,0xF0,0x00,0xE0,0xFD,0x00,0x00,0xFF,0x00,0xE0,0x1F,0x00,0xE0,0xFF,0x00,0xE0,0xE3,0x00,0xE0,0xC0,0x00,0x60,0x80,0x00,0x20,0x00,0x00,  // x
  11, 0x07,0xC0,0x00,0x07,0xF0,0x00,0x07,0xFC,0x00,0x03,0xFF,0x00,0xDE,0xFF,0x00,0xFC,0x9F,0x00,0xE0,0x03,0x00,0x00,0x8F,0x00,0x00,0xFC,0x00,0x00,0xE0,0x00,0x00,0xC0,0x00,  // y
  8, 0x60,0x00,0x00,0xE0,0xE1,0x00,0xE0,0xE7,0x00,0xE0,0x9F,0x00,0xA0,0xFF,0x00,0x60,0xFE,0x00,0xE0,0xF9,0x00,0xE0,0xE1,0x00,  // z
  7, 0x00,0x04,0x00,0x00,0x0E,0x00,0x7E,0xDF,0x07,0xFE,0xFB,0x0F,0xFF,0xFB,0x0F,0xF7,0xF1,0x0D,0x03,0x00,0x0C,  // {
  1, 0xFF,0xFF,0x0F,  // |
  7, 0x03,0x00,0x0C,0xF7,0xF1,0x0C,0xFF,0xFB,0x0F,0xFE,0xFB,0x0F,0x7E,0xDF,0x07,0x00,0x0E,0x00,0x00,0x04,0x00,  // }
  12, 0x00,0x06,0x00,0x00,0x0C,0x00,0x00,0x08,0x00,0x00,0x08,0x00,0x00,0x0C,0x00,0x00,0x0C,0x00,0x00,0x06,0x00,0x00,0x06,0x00,0x00,0x02,0x00,0x00,0x02,0x00,0x00,0x06,0x00,0x00,0x0C,0x00,  // ~
  12, 0xE0,0xFF,0x0F,0x20,0x00,0x08,0x20,0x00,0x08,0x20,0x00,0x08,0x20,0x00,0x08,0x20,0x00,0x08,0x20,0x00,0x08,0x20,0x00,0x08,0x20,0x00,0x08,0x20,0x00,0x08,0x20,0x00,0x08,0xE0,0xFF,0x0F,  // 
  0};

const uint8_t pnp[] PROGMEM={
  42,128, // width run-length encoded
  47,0, // height
  0x00,0x1C,0x01,0x29,0x01,0x29,0x01,0x29,0x01,0x29,0x01,0x29,0x01,0x29,0x01,0x20,0x0B,0x1D,0x04,0x06,
  0x04,0x1A,0x03,0x0A,0x05,0x16,0x03,0x0C,0x01,0x03,0x03,0x13,0x02,0x0D,0x02,0x05,0x02,0x11,0x02,0x0D,
  0x02,0x07,0x02,0x0F,0x02,0x09,0x01,0x03,0x02,0x09,0x02,0x0E,0x01,0x07,0x01,0x02,0x05,0x0B,0x01,0x0D,
  0x02,0x07,0x01,0x02,0x04,0x0C,0x02,0x0C,0x01,0x08,0x01,0x01,0x06,0x0C,0x01,0x0B,0x02,0x08,0x01,0x01,
  0x06,0x0C,0x02,0x0A,0x01,0x09,0x01,0x01,0x06,0x0D,0x01,0x09,0x02,0x09,0x05,0x10,0x02,0x08,0x02,0x09,
  0x02,0x13,0x02,0x08,0x01,0x0A,0x01,0x15,0x01,0x08,0x01,0x0A,0x01,0x15,0x01,0x01,0x13,0x15,0x01,0x01,
  0x13,0x15,0x01,0x08,0x01,0x09,0x02,0x15,0x01,0x08,0x01,0x0A,0x01,0x15,0x01,0x08,0x02,0x09,0x02,0x13,
  0x02,0x08,0x02,0x09,0x03,0x12,0x02,0x09,0x01,0x09,0x01,0x01,0x02,0x11,0x01,0x0A,0x02,0x08,0x01,0x02,
  0x02,0x0F,0x02,0x0B,0x01,0x08,0x01,0x03,0x02,0x0E,0x02,0x0B,0x02,0x07,0x01,0x04,0x02,0x0C,0x02,0x0D,
  0x01,0x06,0x02,0x05,0x02,0x0B,0x01,0x0E,0x02,0x06,0x01,0x06,0x02,0x09,0x02,0x0F,0x02,0x0D,0x02,0x07,
  0x02,0x11,0x02,0x0D,0x02,0x05,0x02,0x13,0x03,0x0C,0x01,0x03,0x03,0x16,0x02,0x0B,0x02,0x01,0x02,0x19,
  0x04,0x08,0x04,0x1D,0x0B,0x28,0x01,0x29,0x01,0x29,0x01,0x29,0x01,0x29,0x01,0x29,0x01,0x0D 
};

const uint8_t smiley[] PROGMEM={
  21,0, // width
  21,0, // height
  0xFC,0x01,0xFF,0x80,0x03,0xF8,0x7F,0x0F,0x8F,0xFE,0x3C,0xFF,0xF9,0xCF,0xFF,0xE4,
  0x7F,0xFF,0x07,0x0E,0x1C,0x3B,0x76,0xE1,0xDB,0xB7,0x0E,0x1C,0x38,0x7F,0xFF,0xC3,
  0xFF,0xFE,0x1D,0xFF,0x70,0x67,0xF3,0x13,0x80,0x39,0xCE,0x03,0x9E,0x3F,0xF8,0xF8,
  0x7F,0x0F,0xE0,0x00,0xFF,0xC0,0x1F,0x80 };

void TestAll(void) {
  ClearDisplay(TFT_BLACK);
  DrawLine(0, 0, tft_width - 1, tft_height - 1, TFT_DARKGREY);
  DrawLine(0, tft_height - 1, tft_width - 1, 0, TFT_DARKGREY);
  DrawBox(5, 5, 4, 6, TFT_WHITE);
  DrawFrame(11, 5, 4, 6, TFT_RED);
  DrawHLine(5, 15, 6, TFT_YELLOW);
  DrawVLine(5, 17, 6, TFT_GREEN);
  DrawCircle(12, 22, 4, TFT_WHITE);
  DrawDisc(6, 32, 4, TFT_BLUE);
  DrawEllipse(12 + 16, 22, 8, 4, TFT_WHITE);
  DrawFilledEllipse(6 + 16, 32, 8, 4, TFT_ORANGE);
  DrawLine(20, 2, 30, 7, TFT_WHITE);
  DrawPixel(20, 7, TFT_WHITE);
  DrawTriangle(10, 40, 2, 45, 5, 50, TFT_WHITE);
  ILI9341SetCursor(35, 15);  DrawString("Hi World", LargeFont, TFT_RED);

  ILI9341SetCursor(35, 55);  DrawString("Hi World", MediumFont, TFT_YELLOW);
  ILI9341SetCursor(35, 35);  DrawString("Hi World ", SmallFont, TFT_GREEN);
  DrawString("01234", DigitsFont, TFT_WHITE);

  int i = Cursorx;
  execDrawChar = false;
  DrawFloat(3.1415927, 3, MediumFont, TFT_BLUE);
  execDrawChar = true;
  ILI9341SetCursor(tft_width - (Cursorx - i), 100);  DrawFloat(3.1415927, 3, MediumFont, TFT_MAGENTA);

  DrawBitmap(100, 100, alert);
  DrawBitmapMono(140, 100, smiley, TFT_CYAN);
  DrawBitmapMono(180, 100, pnp, TFT_CYAN);

  ILI9341SetCursor(10, 130);  DrawString("Hello", VeryBigFont, TFT_GREEN);
}

//#include <SPI.h>

void setup(void) {
//  ILI9341Begin();
//  BeginTouch();
  ILI9341Begin(2, 4, 3, 320, 240, ILI9341_Rotation3);
  BeginTouch(7, 3);

  //  ILI9341Begin(10, 8, 7, 240, 320, ILI9341_Rotation0);
  //  BeginTouch(2, 0);

  //  ILI9341Begin(10, 8, 7, 240, 320, ILI9341_Rotation2);
  //  BeginTouch(2, 2);

  //  ILI9341Begin(10, 8, 7, 320, 240, ILI9341_Rotation3);
  //  BeginTouch(2, 3);

  TestAll();
}

void loop(void) {
  int x, y;
  static int prevx = -1;
  static int prevy = -1;

  if (GetTouch(&x, &y)) {
    //    pen_width = 3;
    if (prevx > 0)
      DrawLine(prevx, prevy, x, y, TFT_YELLOW);
    prevx = x;
    prevy = y;
  } else {
    prevx = -1;
  }
}


