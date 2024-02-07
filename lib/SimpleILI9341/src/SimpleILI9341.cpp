/***************************************************
  Arduino TFT graphics library for the ILI9341

  Derived from the TFT_ILI9341 library which was
  derived from the Adafruit_GFX library and the
  associated driver library.

 ****************************************************/

#include "SimpleILI9341.h"

#include <avr/pgmspace.h>
#include <SPI.h>

int16_t  Cursorx, Cursory, win_xe, win_ye;

uint16_t tft_width = 320;
uint16_t tft_height = 240;

uint8_t pen_width = 1; // if pen_width > 1 then draws with a disc of radius pen_width-1 // the overall width is pen_width*2-1

uint8_t letter_gap = 1;

uint16_t addr_row, addr_col;
uint8_t  mySPCR;

bool execDrawChar = true;
bool ILI9341fast = false;

uint8_t tft_CS   = 10; // Chip select control pin
uint8_t tft_DC   = 8;  // Data Command control pin
uint8_t tft_RST  = 7;  // Reset pin (could connect to Arduino RESET pin)

int touch_xmin = 320;
int touch_ymin = 320;
int touch_xmax = 3900;
int touch_ymax = 3900;
uint8_t touch_Rotation = 1;
uint8_t touch_CS = 2; // Chip select control pin

static inline void spiWait17(void) __attribute__((always_inline));
static inline void spiWait15(void) __attribute__((always_inline));
static inline void spiWait14(void) __attribute__((always_inline));
static inline void spiWait12(void) __attribute__((always_inline));
static inline void spiWrite16(uint16_t data, int16_t count) __attribute__((always_inline));

void tft_fastSetup(void);
void tft_fastPixel(uint16_t x, uint16_t y, uint16_t color);
void tft_setAddrWindow(int16_t x0, int16_t y0, int16_t x1, int16_t y1);

#define ILI9341_INIT_DELAY 0x80

// ILI9341 control registers
#define ILI9341_NOP     0x00
#define ILI9341_SWRESET 0x01
#define ILI9341_RDDID   0x04
#define ILI9341_RDDST   0x09
#define ILI9341_RDIMGFMT  0x0A
#define ILI9341_RDMADCTL  0x0B
#define ILI9341_RDPIXFMT  0x0C
#define ILI9341_RD_DISP_IMG_FMT 0x0D
#define ILI9341_RD_DISP_SIG_MODE 0x0E
#define ILI9341_RDSELFDIAG  0x0F
#define ILI9341_SLPIN   0x10
#define ILI9341_SLPOUT  0x11
#define ILI9341_PTLON   0x12
#define ILI9341_NORON   0x13
#define ILI9341_INVOFF  0x20
#define ILI9341_INVON   0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON  0x29
#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C
#define ILI9341_COLOR_SET 0x2D
#define ILI9341_RAMRD   0x2E
#define ILI9341_PTLAR   0x30
#define ILI9341_VSCRDEF 0x33
#define ILI9341_TEAR_EFF_OFF 0x34
#define ILI9341_TEAR_EFF_ON 0x35
#define ILI9341_MADCTL  0x36
#define ILI9341_VSCRSADD 0x37
#define ILI9341_IDLE_MODE_OFF 0x38
#define ILI9341_IDLE_MODE_ON 0x39
#define ILI9341_PIXFMT  0x3A
#define ILI9341_WRT_MEM_CTRL 0x3C
#define ILI9341_RD_MEM_CTRL 0x3E
#define ILI9341_SET_TEAR_SCANLINE 0x44
#define ILI9341_GET_SCANLINE 0x45
#define ILI9341_WRT_DISP_BRIGHT 0x51
#define ILI9341_RD_DISP_BRIGHT 0x52
#define ILI9341_WRT_CTRL_DISP 0x53
#define ILI9341_RD_CTRL_DISP 0x54
#define ILI9341_WRT_ADAPT_BRIGHT_CTRL 0x55
#define ILI9341_RD_ADAPT_BRIGHT_CTRL 0x56
#define ILI9341_WRT_CABC_MIN_BRIGHT 0x5E
#define ILI9341_RD_CABC_MIN_BRIGHT 0x5F
#define ILI9341_RGB_INTF_SIG_CTRL 0xB0
#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR  0xB4
#define ILI9341_BLANKING_PORCH_CTRL 0xB5
#define ILI9341_DFUNCTR 0xB6
#define ILI9341_ENTRY_MODE_SET 0xB7
#define ILI9341_BKLGT_CTRL_1 0xB8
#define ILI9341_BKLGT_CTRL_2 0xB9
#define ILI9341_BKLGT_CTRL_3 0xBA
#define ILI9341_BKLGT_CTRL_4 0xBB
#define ILI9341_BKLGT_CTRL_5 0xBC
#define ILI9341_BKLGT_CTRL_7 0xBE
#define ILI9341_BKLGT_CTRL_8 0xBF
#define ILI9341_PWCTR1  0xC0
#define ILI9341_PWCTR2  0xC1
#define ILI9341_PWCTR3  0xC2
#define ILI9341_PWCTR4  0xC3
#define ILI9341_PWCTR5  0xC4
#define ILI9341_VMCTR1  0xC5
#define ILI9341_VMCTR2  0xC7
#define ILI9341_NV_MEM_WRT 0xD0
#define ILI9341_NV_MEM_PROT_KEY 0xD1
#define ILI9341_NV_MEM_STAT_RD 0xD2
#define ILI9341_RD_ID4 0xD3
#define ILI9341_RDID1   0xDA
#define ILI9341_RDID2   0xDB
#define ILI9341_RDID3   0xDC
#define ILI9341_RDID4   0xDD
#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1
#define ILI9341_GAMMA_CTRL_1 0xE2
#define ILI9341_GAMMA_CTRL_2 0xE3
#define ILI9341_INTF_CTRL 0xF6

/***************************************************************************************
** Function name:           tft_spiwrite
** Description:             Write 8 bits to SPI port
***************************************************************************************/
static void tft_spiwrite(uint8_t c)
{
  if (ILI9341fast) {
    uint8_t backupSPCR = SPCR;
    SPCR = mySPCR;
    SPDR = c;
    asm volatile( "nop\n\t" ::); // Sync SPIF and some commands need this delay otherwise they get lost!
    while (!(SPSR & _BV(SPIF)));
    SPCR = backupSPCR;
  } else
    SPI.transfer(c);
}

/***************************************************************************************
** Function name:           tft_writecommand
** Description:             Send an 8 bit command to the TFT
***************************************************************************************/
static void tft_writecommand(uint8_t c)
{
  digitalWrite(tft_DC, LOW);
  digitalWrite(tft_CS, LOW);
  tft_spiwrite(c);
  digitalWrite(tft_CS, HIGH);
}

/***************************************************************************************
** Function name:           tft_writedata
** Description:             Send a 8 bit data value to the TFT
***************************************************************************************/
static void tft_writedata(uint8_t c)
{
  digitalWrite(tft_DC, HIGH);
  digitalWrite(tft_CS, LOW);
  tft_spiwrite(c);
  digitalWrite(tft_CS, HIGH);
}

/***************************************************************************************
** Function name:           spi_begin
** Description:             Prepare for SPI communication to TFT
***************************************************************************************/
// If the SPI library has transaction support, these functions
// establish settings and protect from interference from other
// libraries.  Otherwise, they simply do nothing.

#ifdef SPI_HAS_TRANSACTION
  #ifdef SUPPORT_TRANSACTIONS

static inline void spi_begin(void) __attribute__((always_inline));

static inline void spi_begin(void) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
}

static inline void spi_end(void) __attribute__((always_inline));

static inline void spi_end(void) {
  while (!(SPSR & _BV(SPIF))); // wait for everything SPI to finish before freeing up the bus
  SPI.endTransaction();
}
  #else // we do not want to SUPPORT_TRANSACTIONS

#define spi_begin()
#define spi_end()

  #endif // SUPPORT_TRANSACTIONS

#else
#define spi_begin()
#define spi_end()
#endif

/***************************************************************************************
** Description:             Get initialisation commands from FLASH and send to TFT
***************************************************************************************/
void tft_commandList (const uint8_t *addr)
{
  uint8_t  numCommands, numArgs;
  uint8_t  ms;

  spi_begin();
  numCommands = pgm_read_byte(addr++);            // Number of commands to follow
  while (numCommands--)                           // For each command...
  {
    tft_writecommand(pgm_read_byte(addr++));    // Read, issue command
    numArgs = pgm_read_byte(addr++);        // Number of args to follow
    ms = numArgs & ILI9341_INIT_DELAY;      // If hibit set, delay follows args
    numArgs &= ~ILI9341_INIT_DELAY;         // Mask out delay bit
    while (numArgs--)                       // For each argument...
    {
            tft_writedata(pgm_read_byte(addr++)); // Read, issue argument
    }

    if (ms)
    {
            ms = pgm_read_byte(addr++);     // Read post-command delay time (ms)
            delay( (ms==255 ? 500 : ms) );
    }
  }
  spi_end();
}

/***************************************************************************************
** Function name:           DrawBitmap
** Description:             Draw an image stored in an array on the TFT
***************************************************************************************/
void DrawBitmap(int16_t x, int16_t y, const unsigned short *bitmap) {
  int16_t i, j, w,h;
  w = pgm_read_word(bitmap++);
  h = pgm_read_word(bitmap++);

  tft_fastSetup();
  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++ ) {
        tft_fastPixel(x + i, y + j, pgm_read_word(bitmap++));
    }
  }
}

/***************************************************************************************
** Function name:           DrawBitmapMonoBits
** Description:             Draw a black and white image stored as an array of bits
***************************************************************************************/
void DrawBitmapMonoBits(int16_t x, int16_t y, const uint8_t *bitmap, uint16_t color) {
  int16_t i, j, bits, n, w,h;
  w = pgm_read_byte(bitmap++);
  w = w | (pgm_read_byte(bitmap++) << 8);
  h = pgm_read_byte(bitmap++);
  h = h | (pgm_read_byte(bitmap++) << 8);

  tft_fastSetup();
  n = 0;
  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++ ) {
      if (n % 8 == 0)
        bits = pgm_read_byte(bitmap++);

      if ((bits & 0x80) == 0)
        tft_fastPixel(x + i, y + j, color);

      bits = bits << 1;
      n++;
    }
  }
}

/***************************************************************************************
** Function name:           DrawBitmapMonoRLE
** Description:             Draw a black and white image stored as RLE
***************************************************************************************/
void DrawBitmapMonoRLE(int16_t x, int16_t y, const uint8_t *bitmap, uint16_t color) {
  int16_t i,j,nb, w,h;
  bool CurIsBlack;

  w = pgm_read_byte(bitmap++);
  w = w | (pgm_read_byte(bitmap++) << 8);
  w = w & 0x7FFF;
  h = pgm_read_byte(bitmap++);
  h = h | (pgm_read_byte(bitmap++) << 8);

  tft_fastSetup();

  nb = 0;
  CurIsBlack = true;
  j = 0;
  i = 0;

  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++ ) {
      while (nb == 0) {
        nb = pgm_read_byte(bitmap++);
        CurIsBlack = !CurIsBlack;
      }

      if (!CurIsBlack)
        tft_fastPixel(x + i, y + j, color);
      nb--;
    }
  }
}

/***************************************************************************************
** Function name:           DrawBitmapMono
** Description:             Draw a black and white image stored in an array on the TFT
***************************************************************************************/
void DrawBitmapMono(int16_t x, int16_t y, const uint8_t *bitmap, uint16_t color) {
  uint8_t *bmp;
  int16_t w;

  bmp = bitmap;
  w = pgm_read_byte(bmp++);
  w = pgm_read_byte(bmp++);

  if ((w & 0x80) > 0)
    DrawBitmapMonoRLE(x, y, bitmap, color);
  else
    DrawBitmapMonoBits(x, y, bitmap, color);
}

/***************************************************************************************
** Function name:           setCursor
** Description:             Set the text cursor x,y position
***************************************************************************************/
void tft_setCursor(int16_t x, int16_t y)
{
  Cursorx = x;
  Cursory = y;
}

/***************************************************************************************
** Function name:           tft_setAddrWindow
** Description:             define an area to receive a stream of pixels
***************************************************************************************/
// Chip select stays low, use setWindow() from sketches

static void tft_setAddrWindow(int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
  if (ILI9341fast) {
    // Column addr set
    digitalWrite(tft_DC, LOW);
    digitalWrite(tft_CS, LOW);
    SPDR = ILI9341_CASET;
    spiWait15();

    digitalWrite(tft_DC, HIGH);
    SPDR = x0 >> 8;; spiWait12();
    addr_col = 0xFFFF;
    SPDR = x0; spiWait12();
    if(x1!=win_xe) {
      SPDR = x1 >> 8; spiWait12();
      asm volatile( "nop\n\t" ::);
      win_xe=x1;
      SPDR = x1; spiWait14();
    }

    // Row addr set
    digitalWrite(tft_DC, LOW);
    SPDR = ILI9341_PASET; spiWait15();

    digitalWrite(tft_DC, HIGH);
    SPDR = y0 >> 8; spiWait12();
    addr_row = 0xFFFF;
    SPDR = y0; spiWait12();
    if(y1!=win_ye) {
      SPDR = y1 >> 8; spiWait12();
      asm volatile( "nop\n\t" ::);
      win_ye=y1;
      SPDR = y1; spiWait14();
    }

    // write to RAM
    digitalWrite(tft_DC, LOW);
    SPDR = ILI9341_RAMWR; spiWait14();

    //CS, HIGH;
    //digitalWrite(tft_CS, HIGH);
    digitalWrite(tft_DC, HIGH);
  } else {
    tft_writecommand(ILI9341_CASET);        // Column addr set
    tft_writedata(x0 >> 8);
    tft_writedata(x0);                    // XSTART
    tft_writedata(x1 >> 8);
    tft_writedata(x1);                    // XEND
    tft_writecommand(ILI9341_PASET);      // Row addr set
    tft_writedata(y0 >> 8);
    tft_writedata(y0);                    // YSTART
    tft_writedata(y1 >> 8);
    tft_writedata(y1);                    // YEND
    tft_writecommand(ILI9341_RAMWR);      // write to RAM
    digitalWrite(tft_CS, LOW);
    digitalWrite(tft_DC, HIGH);
  }
}

/***************************************************************************************
** Function name:           DrawPixel
** Description:             push a single pixel at an arbitrary position
***************************************************************************************/
void DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  // Faster range checking, possible because x and y are unsigned
  if ((x >= tft_width) || (y >= tft_height)) return;
  spi_begin();

  digitalWrite(tft_CS, LOW);

  if (ILI9341fast) {
    if (addr_col != x) {
      digitalWrite(tft_DC, LOW);
      SPDR = ILI9341_CASET;
      spiWait12();
      addr_col = x;
      digitalWrite(tft_DC, HIGH);
      SPDR = x >> 8; spiWait17();
      SPDR = x; spiWait17();

      SPDR = x >> 8; spiWait17();
      SPDR = x; spiWait12();
    }

    if (addr_row != y) {
      digitalWrite(tft_DC, LOW);
      SPDR = ILI9341_PASET;
      spiWait12();
      addr_row = y;
      digitalWrite(tft_DC, HIGH);
      SPDR = y >> 8; spiWait17();
      SPDR = y; spiWait17();

      SPDR = y >> 8; spiWait17();
      SPDR = y; spiWait14();
    }

    digitalWrite(tft_DC, LOW);

    SPDR = ILI9341_RAMWR; spiWait15();

    digitalWrite(tft_DC, HIGH);

    SPDR = color >> 8; spiWait15();
    win_xe=x;
    SPDR = color; spiWait12();
    win_ye=y;
  } else {
    tft_setAddrWindow(x, y, x, y);
    spiWrite16(color, 1);
  }

  digitalWrite(tft_CS, HIGH);

  spi_end();
}

/***************************************************************************************
** Function name:           tft_fastPixel
***************************************************************************************/
static void tft_fastPixel(uint16_t x, uint16_t y, uint16_t color)
{
  if (ILI9341fast) {
    // Faster range checking, possible because x and y are unsigned
    if ((x >= tft_width) || (y >= tft_height)) return;
    spi_begin();

    digitalWrite(tft_CS, LOW);

    if (addr_col != x) {
      digitalWrite(tft_DC, LOW);
      SPDR = ILI9341_CASET;
      spiWait14();
      addr_col = x;
      digitalWrite(tft_DC, HIGH);

      SPDR = x >> 8;; spiWait17();
      SPDR = x; spiWait12();
    }

    if (addr_row != y) {
      digitalWrite(tft_DC, LOW);
      SPDR = ILI9341_PASET;
      spiWait14();
      addr_row = y;
      digitalWrite(tft_DC, HIGH);

      SPDR = y >> 8; spiWait17();
      SPDR = y; spiWait14();
    }

    digitalWrite(tft_DC, LOW);

    SPDR = ILI9341_RAMWR; spiWait15();

    digitalWrite(tft_DC, HIGH);

    SPDR = color >> 8; spiWait17();
    SPDR = color; spiWait14();

    digitalWrite(tft_CS, HIGH);

    spi_end();
  } else {
    DrawPixel(x, y, color);
  }
}

/***************************************************************************************
** Function name:           tft_fastSetup
***************************************************************************************/
static void tft_fastSetup(void)
{
  if (ILI9341fast) {
    spi_begin();

    digitalWrite(tft_DC, LOW);
    digitalWrite(tft_CS, LOW);

    SPDR = ILI9341_CASET;
    spiWait15();
    digitalWrite(tft_DC, HIGH);
    SPDR = 0; spiWait14();
    addr_col = 0;
    SPDR = 0; spiWait12();
    win_xe=tft_width-1;
    SPDR = win_xe >> 8; spiWait15();
    SPDR = win_xe; spiWait14();

    digitalWrite(tft_DC, LOW);

    SPDR = ILI9341_PASET;
    spiWait15();
    digitalWrite(tft_DC, HIGH);
    SPDR = 0; spiWait14();
    addr_row = 0;
    SPDR = 0; spiWait12();
    win_ye=tft_height-1;
    SPDR = win_ye >> 8; spiWait15();
    SPDR = win_ye; spiWait14();

    digitalWrite(tft_CS, HIGH);

    spi_end();
  }
}

/***************************************************************************************
** Function name:           DrawVLine
** Description:             draw a vertical line
***************************************************************************************/
void DrawVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color)
{
#ifdef CLIP_CHECK
  // Rudimentary clipping
  if ((x >= tft_width) || (y >= tft_height)) return;
  if ((y + h - 1) >= tft_height) h = tft_height - y;
#endif

  if (pen_width > 1) {
    DrawLine(x, y, x, y+h-1, color);
  } else {
    spi_begin();
    tft_setAddrWindow(x, y, x, y + h - 1);
    spiWrite16(color, h);
    digitalWrite(tft_CS, HIGH);
    spi_end();
  }
}

/***************************************************************************************
** Function name:           DrawHLineSingle
** Description:             draw a horizontal line with pen width 1
***************************************************************************************/
void DrawHLineSingle(uint16_t x, uint16_t y, uint16_t w, uint16_t color)
{
#ifdef CLIP_CHECK
  // Rudimentary clipping
  if ((x >= tft_width) || (y >= tft_height)) return;
  if ((x + w - 1) >= tft_width)  w = tft_width - x;
#endif

  spi_begin();
  tft_setAddrWindow(x, y, x + w - 1, y);

  spiWrite16(color, w);
  digitalWrite(tft_CS, HIGH);

  spi_end();
}

/***************************************************************************************
** Function name:           DrawHLine
** Description:             draw a horizontal line
***************************************************************************************/
void DrawHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color)
{
  if (pen_width > 1)
    DrawLine(x, y, x+w-1, y, color);
  else
    DrawHLineSingle(x, y, w, color);
}

/***************************************************************************************
** Function name:           DrawBox
** Description:             draw a filled rectangle
***************************************************************************************/
void DrawBox(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
#ifdef CLIP_CHECK
  if ((x > tft_width) || (y > tft_height) || (w==0) || (h==0)) return;
  if ((x + w - 1) > tft_width)  w = tft_width  - x;
  if ((y + h - 1) > tft_height) h = tft_height - y;
#endif

  spi_begin();
  tft_setAddrWindow(x, y, x + w - 1, y + h - 1);

  while (h--) spiWrite16(color, w);
  digitalWrite(tft_CS, HIGH);

  spi_end();
}

/***************************************************************************************
** Function name:           DrawRoundRect
** Description:             draw a filled rectangle with round corners
***************************************************************************************/
void DrawRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t rad, uint16_t color) {
  DrawBox(x, y+rad+1, w, h-rad*2-2, color);
  DrawDisc(x+rad, y+rad, rad, color);
  DrawDisc(x+w-rad-1, y+rad, rad, color);
  DrawDisc(x+rad, y+h-rad-1, rad, color);
  DrawDisc(x+w-rad-1, y+h-rad-1, rad, color);
  DrawBox(x+rad+1, y, w-rad*2-2, rad*2, color);
  DrawBox(x+rad+1, y+h-rad*2, w-rad*2-2, rad*2, color);
}

/***************************************************************************************
** Function name:           invertDisplay
** Description:             invert the display colours i = 1 invert, i = 0 normal
***************************************************************************************/
void InvertDisplay(boolean i)
{
  spi_begin();
  // Send the command twice as otherwise it does not always work!
  tft_writecommand(i ? ILI9341_INVON : ILI9341_INVOFF);
  tft_writecommand(i ? ILI9341_INVON : ILI9341_INVOFF);
  spi_end();
}

/***************************************************************************************
** Function name:           drawFloat
** Descriptions:            drawFloat, prints 7 non zero digits maximum
***************************************************************************************/
void DrawFloat(float floatNumber, int dp, word Font, uint16_t color)
{
  char str[14];               // Array to contain decimal string
  uint8_t ptr = 0;            // Initialise pointer for array
  int8_t  digits = 1;         // Count the digits to avoid array overflow
  float rounding = 0.5;       // Round up down delta

  if (dp > 7) dp = 7; // Limit the size of decimal portion

  // Adjust the rounding value
  for (uint8_t i = 0; i < dp; ++i) rounding /= 10.0;

  if (floatNumber < -rounding)    // add sign, avoid adding - sign to 0.0!
  {
    str[ptr++] = '-'; // Negative number
    str[ptr] = 0; // Put a null in the array as a precaution
    digits = 0;   // Set digits to 0 to compensate so pointer value can be used later
    floatNumber = -floatNumber; // Make positive
  }

  floatNumber += rounding; // Round up or down

  // For error put ... in string and return (all TFT_ILI9341 library fonts contain . character)
  if (floatNumber >= 2147483647) {
    strcpy(str, "...");
    DrawString(str, Font, color);
    return;
  }
  // No chance of overflow from here on

  // Get integer part
  unsigned long temp = (unsigned long)floatNumber;

  // Put integer part into array
  ltoa(temp, str + ptr, 10);

  // Find out where the null is to get the digit count loaded
  while ((uint8_t)str[ptr] != 0) ptr++; // Move the pointer along
  digits += ptr;                  // Count the digits

  str[ptr++] = '.'; // Add decimal point
  str[ptr] = '0';   // Add a dummy zero
  str[ptr + 1] = 0; // Add a null but don't increment pointer so it can be overwritten

  // Get the decimal portion
  floatNumber = floatNumber - temp;

  // Get decimal digits one by one and put in array
  // Limit digit count so we don't get a false sense of resolution
  uint8_t i = 0;
  while ((i < dp) && (digits < 9)) // while (i < dp) for no limit but array size must be increased
  {
    i++;
    floatNumber *= 10;       // for the next decimal
    temp = floatNumber;      // get the decimal
    ltoa(temp, str + ptr, 10);
    ptr++; digits++;         // Increment pointer and digits count
    floatNumber -= temp;     // Remove that digit
  }

  // Finally we can plot the string and return pixel length
  DrawString(str, Font, color);
}

/***************************************************************************************
** Function name:           spiWrite16
** Descriptions:            Delay based assembler loop for fast SPI write
***************************************************************************************/
static inline void spiWrite16(uint16_t data, int16_t count)
{
// We can enter this loop with 0 pixels to draw, so we need to check this
  if (ILI9341fast) {
    uint8_t temp;
    asm volatile
    (
      "	sbiw	%[count],0\n"		// test count
      //"	brmi	2f\n"			// if < 0 then done
      "	breq	2f\n"			// if == 0 then done

      "1:	out	%[spi],%[hi]\n"		// write SPI data
      "	rcall	3f      \n" // 7
      "	rcall	3f      \n" // 14
      "	rjmp 	4f      \n" // 16
      "3:	ret     \n" //
      "4:	nop     \n" // 17

      "	out	%[spi],%[lo]\n"		// write SPI data

      "	adiw	r24,0	  \n"	// 2
      "	adiw	r24,0  \n"	// 4
      "	rcall	5f     \n"	// 11
      "	rjmp 	6f     \n"	// 13
      "5:	ret    \n"	//
      "6:	       \n"

      "	sbiw	%[count],1 \n" // 15 decrement count
      "	brne	1b         \n" // 17 if != 0 then loop

      "2:\n"

      : [temp] "=d" (temp), [count] "+w" (count)
      : [spi] "i" (_SFR_IO_ADDR(SPDR)), [lo] "r" ((uint8_t)data), [hi] "r" ((uint8_t)(data>>8))
      :
    );
  } else {
    for (;count > 0; count--)
      SPI.transfer16(data);
  }
}

/***************************************************************************************
** Function name:           spiWait
** Descriptions:            17 cycle delay
***************************************************************************************/
static inline void spiWait17(void)
{
  asm volatile
  (
    "	rcall	1f    \n" // 7
    "	rcall	1f    \n" // 14
    "	rjmp 	2f    \n" // 16
    "1:	ret   \n" //
    "2:	nop	 \n" // 17
  );
}

/***************************************************************************************
** Function name:           spiWait
** Descriptions:            15 cycle delay
***************************************************************************************/
static inline void spiWait15(void)
{
  asm volatile
  (
    "	adiw	r24,0  \n"	// 2
    "	adiw	r24,0  \n"	// 4
    "	adiw	r24,0  \n"	// 6
    "	rcall	1f     \n"	// 13
    "	rjmp 	2f     \n"	// 15
    "1:	ret    \n"	//
    "2:	       \n"	//
  );
}

/***************************************************************************************
** Function name:           spiWait
** Descriptions:            14 cycle delay
***************************************************************************************/
static inline void spiWait14(void)
{
  asm volatile
  (
    "	nop         \n"	// 1
    "	adiw	r24,0  \n"	// 3
    "	adiw	r24,0  \n"	// 5
    "	rcall	1f     \n"	// 12
    "	rjmp 	2f     \n"	// 14
    "1:	ret    \n"	//
    "2:	       \n"	//
  );
}

/***************************************************************************************
** Function name:           spiWait
** Descriptions:            12 cycle delay
***************************************************************************************/
static inline void spiWait12(void)
{
  asm volatile
  (
    "	nop         \n"	// 1
    "	adiw	r24,0  \n"	// 3
    "	rcall	1f     \n"	// 10
    "	rjmp 	2f     \n"	// 12
    "1:	ret    \n"	//
    "2:	       \n"	//
  );
}

/***************************************************************************************
** Function name:           ILI9341Begin
** Descriptions:            initialise ILI9341
***************************************************************************************/
void ILI9341Begin(uint8_t CS = 10, uint8_t CD = 8, uint8_t RST = 7, uint16_t w = 320, uint16_t h = 240, uint8_t Rotation = ILI9341_Rotation1) {

  tft_CS   = CS;
  tft_DC   = CD;
  tft_RST  = RST;

  SPI.begin();

  if (tft_RST > 0) {
    digitalWrite(tft_RST, LOW);
    pinMode(tft_RST, OUTPUT);
  }

  digitalWrite(tft_DC, HIGH);
  pinMode(tft_DC, OUTPUT);

  digitalWrite(tft_CS, HIGH);
  pinMode(tft_CS, OUTPUT);

  tft_width    = w;
  tft_height   = h;
  Cursory  = Cursorx    = 0;

  addr_row = 0xFFFF;
  addr_col = 0xFFFF;
  win_xe = 0xFFFF;
  win_ye = 0xFFFF;

  SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  mySPCR = SPCR;

  // toggle RST low to reset
  if (tft_RST > 0) {
    digitalWrite(tft_RST, HIGH);
    delay(5);
    digitalWrite(tft_RST, LOW);
    delay(20);
    digitalWrite(tft_RST, HIGH);
    delay(150);
  }

  if (tft_RST <= 0) {
    spi_begin();

    tft_writecommand(ILI9341_SWRESET);
    delay(10);
    spi_end();
  };

  // Initialization commands for ILI9341 screens
  static const uint8_t ILI9341_cmds[] PROGMEM =
  {
    21,               /* num commands             */
    0xEF,             /*                          */ 3, 0x03, 0x80, 0x02,
    0xCF,             /*                          */ 3, 0x00, 0xC1, 0x30,
    0xED,             /*                          */ 4, 0x64, 0x03, 0x12, 0x81,
    0xE8,             /*                          */ 3, 0x85, 0x00, 0x78,
    0xCB,             /*                          */ 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
    0xF7,             /*                          */ 1, 0x20,
    0xEA,             /*                          */ 2, 0x00, 0x00,
    ILI9341_PWCTR1,   /* power control            */ 1, 0x23,                           // VRH[5:0]
    ILI9341_PWCTR2,   /* power control           */ 1, 0x10,                           // SAP[2:0];BT[3:0]
    ILI9341_VMCTR1,   /* VCM control             */ 2, 0x3e, 0x28,
    ILI9341_VMCTR2,   /* VCM control2            */ 1, 0x86,                           // --
    ILI9341_MADCTL,   /*                         */ 1, (ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR),
    ILI9341_PIXFMT,   /*                         */ 1, 0x55,
    ILI9341_FRMCTR1,  /*                         */ 2, 0x00, 0x18,
    ILI9341_DFUNCTR,  /*                         */ 3, 0x08, 0x82, 0x27,
    0xF2,             /* 3Gamma Function Disable */ 1, 0x00,
    ILI9341_GAMMASET, /* Gamma curve selected    */ 1, 0x01,
    ILI9341_GMCTRP1,  /* Set Gamma               */ 15,0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
    ILI9341_GMCTRN1,  /*                         */ 15,0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
    ILI9341_SLPOUT,   /*                         */ ILI9341_INIT_DELAY, 120,
    ILI9341_DISPON,   /*                         */ 0,
  };

  tft_commandList(ILI9341_cmds);


  spi_begin();
  tft_writecommand(ILI9341_MADCTL);
  tft_writedata(Rotation);
  spi_end();

}

void ILI9341SetCursor(uint16_t x, uint16_t y) {
  tft_setCursor(x, y);
}

void ClearDisplay(uint16_t color) {
  DrawBox(0, 0, tft_width, tft_height, color);
}

void DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
  int tmp, x, y, dx, dy, err, ystep, swapxy;
  swapxy = 0;

  if (x1 > x2) dx = x1 - x2;
  else dx = x2 - x1;
  if (y1 > y2) dy = y1 - y2;
  else dy = y2 - y1;

  if (dy > dx) {
    swapxy = 1;
    tmp = dx;
    dx = dy;
    dy = tmp;
    tmp = x1;
    x1 = y1;
    y1 = tmp;
    tmp = x2;
    x2 = y2;
    y2 = tmp;
  }
  if ((x1 > x2)) {
    tmp = x1;
    x1 = x2;
    x2 = tmp;
    tmp = y1;
    y1 = y2;
    y2 = tmp;
  }
  err = dx >> 1;
  if ((y2 > y1)) ystep = 1;
  else ystep = -1;
  y = y1;
  for (x = x1; x <= x2; x++) {
    if (pen_width > 1) {
      if ((swapxy == 0))
        DrawDisc(x, y, pen_width-1, color);
      else
        DrawDisc(y, x, pen_width-1, color);
    } else{
      if ((swapxy == 0)) {
        DrawPixel(x, y, color);
      } else {
        DrawPixel(y, x, color);
      }
    }

    err -= dy;
    if (err < 0) {
      y += ystep;
      err += dx;
    }
  }
}

void DrawFrame(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
  DrawHLine(x, y, w, color);
  DrawHLine(x, y + h - 1, w, color);
  DrawVLine(x, y, h, color);
  DrawVLine(x + w - 1, y, h, color);
}

void DrawCircle(uint16_t x0, uint16_t y0, uint16_t rad, uint16_t color) {
  DrawEllipse(x0, y0, rad, rad, color);
}

void DrawDisc(uint16_t x0, uint16_t y0, uint16_t rad, uint16_t color) {
  DrawFilledEllipse(x0, y0, rad, rad, color);
}

void DrawFilledEllipse(int x0, int y0, int rx, int ry, uint16_t color) {
// the overall width is rad*2+1
  if (rx < 1) return;
  if (ry < 1) return;
  int16_t x, y;
  int32_t rx2 = rx * rx;
  int32_t ry2 = ry * ry;
  int32_t fx2 = 4 * rx2;
  int32_t fy2 = 4 * ry2;
  int32_t s;

  for (x = 0, y = ry, s = 2*ry2+rx2*(1-2*ry); ry2*x <= rx2*y; x++)
  {
    DrawHLineSingle(x0 - x, y0 - y, x + x + 1, color);
    DrawHLineSingle(x0 - x, y0 + y, x + x + 1, color);

    if (s >= 0)
    {
      s += fx2 * (1 - y);
      y--;
    }
    s += ry2 * ((4 * x) + 6);
  }

  for (x = rx, y = 0, s = 2*rx2+ry2*(1-2*rx); rx2*y <= ry2*x; y++)
  {
    DrawHLineSingle(x0 - x, y0 - y, x + x + 1, color);
    DrawHLineSingle(x0 - x, y0 + y, x + x + 1, color);

    if (s >= 0)
    {
      s += fy2 * (1 - x);
      x--;
    }
    s += rx2 * ((4 * y) + 6);
  }
}

void DrawEllipse(int x0, int y0, int rx, int ry, uint16_t color) {
  if (rx<2) return;
  if (ry<2) return;
  int16_t x, y;
  int32_t rx2 = rx * rx;
  int32_t ry2 = ry * ry;
  int32_t fx2 = 4 * rx2;
  int32_t fy2 = 4 * ry2;
  int32_t s;

  tft_fastSetup();

  for (x = 0, y = ry, s = 2*ry2+rx2*(1-2*ry); ry2*x <= rx2*y; x++)
  {
    tft_fastPixel(x0 + x, y0 + y, color);
    tft_fastPixel(x0 - x, y0 + y, color);
    tft_fastPixel(x0 - x, y0 - y, color);
    tft_fastPixel(x0 + x, y0 - y, color);
    if (s >= 0)
    {
      s += fx2 * (1 - y);
      y--;
    }
    s += ry2 * ((4 * x) + 6);
  }

  for (x = rx, y = 0, s = 2*rx2+ry2*(1-2*rx); rx2*y <= ry2*x; y++)
  {
    tft_fastPixel(x0 + x, y0 + y, color);
    tft_fastPixel(x0 - x, y0 + y, color);
    tft_fastPixel(x0 - x, y0 - y, color);
    tft_fastPixel(x0 + x, y0 - y, color);
    if (s >= 0)
    {
      s += fy2 * (1 - x);
      x--;
    }
    s += rx2 * ((4 * y) + 6);
  }
}

static void swap(uint16_t *a, uint16_t *b) {
  uint16_t c;
  c = *a;
  *a = *b;
  *b = c;
}

void DrawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
  int16_t a, b, y, last;

  // Sort coordinates by Y order (y2 >= y1 >= y0)
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }
  if (y1 > y2) {
    swap(y2, y1); swap(x2, x1);
  }
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }

  if (y0 == y2) { // Handle awkward all-on-same-line case as its own thing
    a = b = x0;
    if (x1 < a)      a = x1;
    else if (x1 > b) b = x1;
    if (x2 < a)      a = x2;
    else if (x2 > b) b = x2;
    DrawHLineSingle(a, y0, b - a + 1, color);
    return;
  }

  int16_t
  dx01 = x1 - x0,
  dy01 = y1 - y0,
  dx02 = x2 - x0,
  dy02 = y2 - y0,
  dx12 = x2 - x1,
  dy12 = y2 - y1,
  sa   = 0,
  sb   = 0;

  // For upper part of triangle, find scanline crossings for segments
  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y1 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y0=y1
  // (flat-topped triangle).
  if (y1 == y2) last = y1;  // Include y1 scanline
  else         last = y1 - 1; // Skip it

  for (y = y0; y <= last; y++) {
    a   = x0 + sa / dy01;
    b   = x0 + sb / dy02;
    sa += dx01;
    sb += dx02;

    if (a > b) swap(a, b);
    DrawHLineSingle(a, y, b - a + 1, color);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 0-2 and 1-2.  This loop is skipped if y1=y2.
  sa = dx12 * (y - y1);
  sb = dx02 * (y - y0);
  for (; y <= y2; y++) {
    a   = x1 + sa / dy12;
    b   = x0 + sb / dy02;
    sa += dx12;
    sb += dx02;

    if (a > b) swap(a, b);
    DrawHLineSingle(a, y, b - a + 1, color);
  }
}

void DrawChar(uint8_t c, word Font, uint16_t color) {
  word n, j, px;
  byte ymax, desc;
  unsigned long b,d;

  ymax = pgm_read_byte_near(Font);
  Font++;
  desc = pgm_read_byte_near(Font);
  Font++;

  j = pgm_read_byte_near(Font); // first char
  Font++;
  if (c < j) return;

  while (c > j) {
    b = pgm_read_byte_near(Font);
    if (b == 0)
      return;
    if (ymax > 15)
      Font += b * 3 + 1;
    else if (ymax > 7)
      Font += b * 2 + 1;
    else
      Font += b + 1;
    c--;
  }
  px = Cursorx;
  n = pgm_read_byte_near(Font);
  Font++;
  while (n > 0) {
    b = pgm_read_byte_near(Font);
    b &= 0xFF;
    Font++;
    if (ymax > 7) {
      d = pgm_read_byte_near(Font);
      d &= 0xFF;
      b |= d << 8;
      Font++;
    }
    if (ymax > 15) {
      d = pgm_read_byte_near(Font);
      d &= 0xFF;
      b |= d << 16;
      Font++;
    }
    if (execDrawChar) {
      for (j = 0; j <= ymax; j++) {
        if (b & 1)
          DrawPixel(Cursorx, Cursory + desc - j, color);
        b = b >> 1;
      }
    }
    Cursorx++;
    n--;
  }
  Cursorx += letter_gap;
}

void DrawString(char * s, word Font, uint16_t color) {
  for (char * t = s; * t; t++) {
    DrawChar( * t, Font, color);
  }
}

void DrawStringAt(int16_t x, int16_t y, char * s, word Font, uint16_t color) {
  ILI9341SetCursor(x, y);
  DrawString(s, Font, color);
}

void DrawInt(int i, word Font, uint16_t color) {
  if (i < 0) {
    i=-i;
    DrawChar('-',Font, color);
  }

  bool HasDigit = false;
  int n =  10000;
  if (i == 0) {
    DrawChar('0',Font, color);
  } else {
    while (n > 0) {
      if ((i >= n) or HasDigit) {
        DrawChar('0' + (i / n),Font, color);
        HasDigit = true;
      }
      i %= n;
      n /= 10;
    }
  }
}

/***************************************************************************************
** Function name:           rgb
** Description:             convert three 8 bit RGB levels to a 16 bit colour value
***************************************************************************************/
uint16_t rgb(uint8_t r, uint8_t g, uint8_t b)
{
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


/***************************************************************************************
** Function name:           cmp_swap
** Description:             if a > b then swap a and b
***************************************************************************************/
static inline void cmp_swap(int16_t *a, int16_t *b)
{
  if (*a > *b) {
    int16_t c = *a;
    *a = *b;
    *b = c;
  }
}

/***************************************************************************************
** Function name:           median_filter
** Description:             return median value of five
***************************************************************************************/
static int median_filter(int16_t t[]) {
  cmp_swap(&t[0],&t[4]);
  cmp_swap(&t[2],&t[4]);
  cmp_swap(&t[1],&t[3]);
  cmp_swap(&t[0],&t[2]);
  cmp_swap(&t[3],&t[4]);
  cmp_swap(&t[1],&t[2]);
  cmp_swap(&t[0],&t[3]);
  cmp_swap(&t[2],&t[3]);

  return t[2];
}

/***************************************************************************************
** Function name:           GetTouch
** Description:             get touch coordinates
**                          return true if touched
***************************************************************************************/
bool GetTouch(int *x, int *y) {
  int16_t z,z1,z2,ax[5],ay[5];

  //SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));

  digitalWrite(touch_CS, LOW);
  delayMicroseconds(100);
//  z = SPI.transfer(0xB1);
//  z1 = SPI.transfer16(0xC1) >> 3;
//  z2 = SPI.transfer16(0x91) >> 3;
  z = SPI.transfer16(0xB4) >> 3;
  z1 = SPI.transfer16(0xB4) >> 3;
  z2 = SPI.transfer16(0xC4) >> 3;

  SPI.transfer16(0x91);  // switch on drivers
  ax[0] = SPI.transfer16(0x91) >> 3;
  ax[1] = SPI.transfer16(0x91) >> 3;
  ax[2] = SPI.transfer16(0x91) >> 3;
  ax[3] = SPI.transfer16(0x91) >> 3;
  ax[4] = SPI.transfer16(0x91) >> 3;

  SPI.transfer16(0xD1);  // switch on drivers
  ay[0] = SPI.transfer16(0xD1) >> 3;
  ay[1] = SPI.transfer16(0xD1) >> 3;
  ay[2] = SPI.transfer16(0xD1) >> 3;
  ay[3] = SPI.transfer16(0xD1) >> 3;
  ay[4] = SPI.transfer16(0xD1) >> 3;

//  data[4] = SPI.transfer16(0xD0) >> 3;  // Last Y touch power down
//  data[5] = SPI.transfer16(0) >> 3;
  SPI.transfer16(0);  // power down
  digitalWrite(touch_CS, HIGH);
  delayMicroseconds(100);
  SPI.endTransaction();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  SPI.endTransaction();

  //dlf Touch pad was getting into funky state of returning 4096 with no key presses so trap and ignore it
  if (z1+z2 != 4096 && z1+z2 > 0x80) {    
    switch (touch_Rotation) {
      case 1:
        *x = round(long(touch_xmax-median_filter(ax))*tft_width / long(touch_xmax-touch_xmin));
        *y = round(long(touch_ymax-median_filter(ay))*tft_height / long(touch_ymax-touch_ymin));
        break;
      case 2:
        *x = round(long(touch_xmax-median_filter(ay))*tft_width / long(touch_xmax-touch_xmin));
        *y = tft_height-round(long(touch_ymax-median_filter(ax))*tft_height / long(touch_ymax-touch_ymin));
        break;
      case 3:
        *x = round(long(median_filter(ax)-touch_xmin)*tft_width / long(touch_xmax-touch_xmin));
        *y = round(long(median_filter(ay)-touch_ymin)*tft_height / long(touch_ymax-touch_ymin));
        break;
      default:
        *x = round(long(median_filter(ay)-touch_xmin)*tft_width / long(touch_xmax-touch_xmin));
        *y = tft_height-round(long(median_filter(ax)-touch_ymin)*tft_height / long(touch_ymax-touch_ymin));
        break;
    }
  //  Serial.print("Screen Touched "); Serial.println(z1+z2); //dlf
    return true;
  } else {
    *x = 0;
    *y = 0;
    return false;
  }
}

/***************************************************************************************
** Function name:           BeginTouch
** Descriptions:            initialise ILI9341
***************************************************************************************/
void BeginTouch(uint8_t CS = 2, uint8_t Rotation = 1, int xmin = 320, int ymin = 320, int xmax = 3900, int ymax = 3900) {
  int x,y;

  touch_xmin = xmin;
  touch_xmax = xmax;
  touch_ymin = ymin;
  touch_ymax = ymax;
  touch_Rotation = Rotation % 4;
  touch_CS = CS;

  if (touch_CS > 0) {
    pinMode(touch_CS, OUTPUT);
    digitalWrite(touch_CS, LOW);
  }

  GetTouch(&x, &y);
}


