/***** Includes *****/
#include <Arduino.h>
#include <Wire.h>
#include <imxrt.h>

/***** Definitions *****/
#define OV7670_I2C_ADDR 0x42 >> 1
#define DATAPORT GPIO7_PSR  // using GPIO port 2/7 - 7 is the high-speed version
// data port are not in sequential order (see DATA_PINS config below). After receiving the data, the bits must be reordered.

/* Registers */
#define REG_PID 0x0a  // Product ID MSB
#define REG_VER 0x0b  // Product ID LSB

#define COM7 0x12 // Common control 7 output format option
#define COM13 0x3D // Common control 13 Gamme, UV saturation, UV swap
#define COM15 0x40 // Common control 15 RGB 555/565 option
#define MVFP 0x1E // Scan direction control
#define SCALING_XSC 0x70
#define SCALING_YSC 0x71

#define CLKRC 0x11

/***** Globals *****/
const uint8_t DATA_PINS[8] = { 10, 12, 11, 13, 6, 9, 8, 7 };  // port 2 pins, arranged for GPIO to be as continuous as possible
const uint8_t MCLK = 5;

/* Horizontal sync, vertical sync, pixel clock pins*/
const uint8_t HS = 4;
const uint8_t PCLK = 3;
const uint8_t VS = 2;

/* Dimensions */
const int width = 320;
const int height = 240;
const int bytesPerPixel = 2;
uint8_t buffer[width * height * bytesPerPixel];



/***** Functions *****/

void setup() {
  // Ignore NACK (camera uses SCCB)
  IMXRT_LPI2C1.MCFGR1 |= LPI2C_MCFGR1_IGNACK;

  // Start MCLK
  analogWriteFrequency(MCLK, 16000000);
  analogWrite(MCLK, 128);

  Wire.begin();
  Serial.begin(6000000);

  // set data pins as inputs
  for (int pin : DATA_PINS) {
    pinMode(pin, INPUT);
  }
  pinMode(HS, INPUT);
  pinMode(VS, INPUT);
  pinMode(PCLK, INPUT);

  // test_i2c_read();
  configureCamera();
  // setTestPattern();
}

void loop() {
  if (digitalReadFast(23) == HIGH) {
    getPicture();
    printPicture();
  }
  
  // Serial.println("********************************************\n********************************************");
  // delay(500);
}

void printPicture() {
  Serial.write("FRAME", 5);
  Serial.write(buffer, width * height * bytesPerPixel);
}

void getPicture() {
  noInterrupts();
  int byte_count = 0;

  // wait for next frame to start
  while (digitalReadFast(VS) == LOW);
  while (digitalReadFast(VS) == HIGH);

  for (int row = 0; row < height; row++) {
    // wait for href to start
    while (digitalReadFast(HS) == LOW);

    for (int pixel = 0; pixel < width * bytesPerPixel; pixel++) {
      // wait for pclk to read data
      while (digitalReadFast(PCLK) == LOW); // only read after it goes from LOW to HIGH
      
      buffer[byte_count] = readDataPort();
      byte_count++;

      while (digitalReadFast(PCLK) == HIGH); // wait until clock ticks to LOW to continue 

    }
  }
  interrupts();

}

void configureCamera() {
  // reset registers
  if (!writeToRegister(COM7, 0x80)) {
    return;
  }
  
  delay(1000);

  // decrease pclk
  uint8_t clkrc_byte = readFromRegister(CLKRC);
  if (!writeToRegister(CLKRC, (clkrc_byte | 0b00000001))) {
    return;
  }

  // Set to RGB output mode
  if (!writeToRegister(COM7, 0b00010000)) {
    return;
  }

  // // Set to RGB565
  // uint8_t com15byte = readFromRegister(COM15);
  // if (!writeToRegister(COM15, (com15byte | 0b00010000))) {
  //   return;
  // }

  // set to yuv
  uint8_t com15byte = readFromRegister(COM15);
  if (!writeToRegister(COM15, (com15byte | 0b00000000))) {
    return;
  }

  // flip vertical and horizontal image
  if (!writeToRegister(MVFP, (0b00110000))) {
    return;
  }

  // writeToRegister(0x4f, 0x80);
  // writeToRegister(0x50, 0x80);
  // writeToRegister(0x51, 0x00);
  // writeToRegister(0x52, 0x22);
  // writeToRegister(0x53, 0x5e);
  // writeToRegister(0x54, 0x80);

  // uint8_t mtxsbyte = readFromRegister(0x58);
  // if (!writeToRegister(0x58, (mtxsbyte | 0b00011010))) {
  //   return;
  // }

  // uint8_t com13byte = readFromRegister(COM13);
  // if (!writeToRegister(COM13, (com13byte | 0b11000000))) {
  //   return;
  // }

  // setTestPattern();
// writeToRegister(0x3A, 0x04);
// writeToRegister(0x12, 0x00);

writeToRegister(0x17, 0x13);  // HSTART- changing these flips color channels - probably has to do with, if 
writeToRegister(0x18, 0x01);  // HSTOP - start/stop are off, we chop off bits and the alignment becomes wrong? 
// Commenting these out seems to gives right RGB order and gets rid of weird borders on sides

writeToRegister(0x32, 0xB6); // HREF - seems like can be left commented without issue

writeToRegister(0x19, 0x02); // VSTRT - more output frame settings, seems can be commented but maybe images looked a bit better?
writeToRegister(0x1A, 0x7A); // VSTOP - Try both these settings again later

writeToRegister(0x03, 0x0A); // VREF - seems can be commented

writeToRegister(0x0C, 0x00); // COM3 - this is default value in datasheet so can probably be commented
writeToRegister(0x3E, 0x00); // COM14 - datasheet says to set this to 0x19; seems necessary
writeToRegister(0x3E, 0x19); // COM14 - datasheet says to set this to 0x19; but setting to 0 seems necessary; seems
                            // key part is setting bit 3 to 0, so that scaling cannot be adjusted manually

writeToRegister(0x70, 0x3A);
writeToRegister(0x71, 0x35);
writeToRegister(0x72, 0x11); // SCALING_DCWCTR DCW Control - down sampling?
writeToRegister(0x73, 0xF0); // SCALING_PCLK_DIV Clock scaling for DSP?
writeToRegister(0xA2, 0x01); // SCALING_PCLK_DELAY Pixel Clock Delay, seems necessary
writeToRegister(0x15, 0x00); // COM10, polarities and options for *SYNC, *REF, PCLK pins

// GAMMA CURVE, seems important for good image but isn't necessary for good colors
writeToRegister(0x7A, 0x20);
writeToRegister(0x7B, 0x10);
writeToRegister(0x7C, 0x1E);
writeToRegister(0x7D, 0x35);
writeToRegister(0x7E, 0x5A);
writeToRegister(0x7F, 0x69);
writeToRegister(0x80, 0x76);
writeToRegister(0x81, 0x80);
writeToRegister(0x82, 0x88);
writeToRegister(0x83, 0x8F);
writeToRegister(0x84, 0x96);
writeToRegister(0x85, 0xA3);
writeToRegister(0x86, 0xAF);
writeToRegister(0x87, 0xC4);
writeToRegister(0x88, 0xD7);
writeToRegister(0x89, 0xE8);

// writeToRegister(0x13, 0xC0); // COM8 AGC/AEC/AWB enables
// writeToRegister(0x00, 0x00); // GAIN AGC Gain control gain setting
writeToRegister(0x10, 0x00); // AECH Exposure Value
writeToRegister(0x0D, 0x40); // COM4 average window option; this is default setting
writeToRegister(0x14, 0x18); // COM9 Automatic Gain Ceiling
writeToRegister(0xA5, 0x05); // AECGMAX Maximum Banding Filter Step
writeToRegister(0xAB, 0x07); // RSVD RESERVED
writeToRegister(0x24, 0x95); // AEW AGC AEC Stable Operating Region Upper
writeToRegister(0x25, 0x33); // AEB AGC AEC Stable Operating Region Lower
writeToRegister(0x26, 0xE3); // VPT AGC AEC Fast Mode Operating Region
writeToRegister(0x9F, 0x78); // HRL High Reference Luminance
writeToRegister(0xA0, 0x68); // LRL Low Reference Luminance
writeToRegister(0xA1, 0x03); // DSPC3 DSP Control 3
writeToRegister(0xA6, 0xD8); // LPH Lower Limit of Probability for HRL
writeToRegister(0xA7, 0xD8); // UPL Upper Limit of Probability for LRL
writeToRegister(0xA8, 0xF0); // TPL Probability Threshold for LRL to control AEC/AGC speed
writeToRegister(0xA9, 0x90); // TPH  Probability Threshold for HRL to control AEC/AGC speed
writeToRegister(0xAA, 0x94); // NALG AEC algorithm selection, reserved
writeToRegister(0x13, 0xC5); // COM8 !! repeat AGC/AEC/AWB enables; probably disabling before changing settings and re-enabling
writeToRegister(0x30, 0x00); // HSYST HSYNC Rising Edge Delay
writeToRegister(0x31, 0x00); // HSYEN HSYNC Falling Edge Delay
writeToRegister(0x0E, 0x61); // COM5 RESERVED
writeToRegister(0x0F, 0x4B); // COM6 reset all timing when format changes; output of optical black row option
writeToRegister(0x16, 0x02); // RSVD RESERVED
//// writeToRegister(0x1E, 0x07);
writeToRegister(0x21, 0x02); // ADCCTR1 RESERVED
writeToRegister(0x22, 0x91); // ADCCTR2 RESERVED
writeToRegister(0x29, 0x07); // RSVD RESERVED 
writeToRegister(0x33, 0x0B); // CHLF Array Current Control RESERVED
writeToRegister(0x35, 0x0B); // RSVD RESERVed
writeToRegister(0x37, 0x1D); // ADC ADC Control RESERVED
writeToRegister(0x38, 0x71); // ACOM ADC and Analog Common Mode Control RESERVED
writeToRegister(0x39, 0x2A); // OFON ADC Offset Control RESERVED
writeToRegister(0x3C, 0x78); // COM12 HREF option when VSYNC low or always
writeToRegister(0x4D, 0x40); // DM_POS Dummy row position
writeToRegister(0x4E, 0x20); // RSVD RESERVED
writeToRegister(0x69, 0x00); // GFIX Fix Gain Control
writeToRegister(0x74, 0x10); // REG74 digital gain control select and manual controll
writeToRegister(0x8D, 0x4F); // RSVD RESERVED
writeToRegister(0x8E, 0x00); // RSVD RESERVED
writeToRegister(0x8F, 0x00); // RSVD RESERVED
writeToRegister(0x90, 0x00); // RSVD RESERVED
writeToRegister(0x91, 0x00); // RSVD RESERVED
writeToRegister(0x96, 0x00); // RSVD RESERVED
writeToRegister(0x9A, 0x00); // RSVD RESERVED
writeToRegister(0xB0, 0x84); // RSVD RESERVED
writeToRegister(0xB1, 0x0C); // ABLC1 ALBC enable/disable
writeToRegister(0xB2, 0x0E); // RSVD RESERVED
writeToRegister(0xB3, 0x82); // THL_ST ABLC Target
writeToRegister(0xB8, 0x0A); // RSVD RESERVED

// AWBC AWB Control
writeToRegister(0x43, 0x0A);
writeToRegister(0x44, 0xF0);
writeToRegister(0x45, 0x34);
writeToRegister(0x46, 0x58);
writeToRegister(0x47, 0x28);
writeToRegister(0x48, 0x3A);
writeToRegister(0x59, 0x88);
writeToRegister(0x5A, 0x88);
writeToRegister(0x5B, 0x44);
writeToRegister(0x5C, 0x67);
writeToRegister(0x5D, 0x49);
writeToRegister(0x5E, 0x0E);
writeToRegister(0x6C, 0x0A);
writeToRegister(0x6D, 0x55);
writeToRegister(0x6E, 0x11);
writeToRegister(0x6F, 0x9E);


writeToRegister(0x6A, 0x40); // GGAIN G Channel AWB Gain
writeToRegister(0x01, 0x40); // BLUE AWB Blue channel gain
writeToRegister(0x02, 0x60); // RED AWN REd channel gain

writeToRegister(0x13, 0xC7); // COM8 AGC/AEC/Banding/enable REPEAT

// MTX Color Matrix Coefficients
writeToRegister(0x4F, 0x80);
writeToRegister(0x50, 0x80);
writeToRegister(0x51, 0x00);
writeToRegister(0x52, 0x22);
writeToRegister(0x53, 0x5E);
writeToRegister(0x54, 0x80);
writeToRegister(0x58, 0x9E); // MTXS signs, auto contrast

writeToRegister(0x41, 0x08); // COM16 double color matrix, AWB gain enable, denoise, edge enhancement
writeToRegister(0x3F, 0x00); // EDGE Edge Enhancement Adjustment
writeToRegister(0x75, 0x05); // REG75 Edge enhancement lower limit
writeToRegister(0x76, 0xE1); // REG76 Edge enhancement higher limit, black/white pixel correction
writeToRegister(0x4C, 0x00); // DNSTH De-noise Strength
writeToRegister(0x77, 0x01); // REG77 Offset, de-noise range control
//// writeToRegister(0x3D, 0x48);
writeToRegister(0x4B, 0x09); // REG4B, RESERVED, UV average unable
writeToRegister(0xC9, 0x60); // SATCTR Saturation Control
writeToRegister(0x56, 0x40); // CONTRAS Contrast Control
writeToRegister(0x34, 0x11); // ARBLM Array Reference Control
writeToRegister(0x3B, 0x12); // COM11 Night mode, auto 50/60Hz, exposure timing
writeToRegister(0xA4, 0x82); // NT_CTRL auto frame adjustment dummy row
writeToRegister(0x96, 0x00); // RSVD RESERVED
writeToRegister(0x97, 0x30); // RSVD RESERVED
writeToRegister(0x98, 0x20); // RSVD RESERVED
writeToRegister(0x99, 0x30); // RSVD RESERVED
writeToRegister(0x9A, 0x84); // RSVD RESERVED
writeToRegister(0x9B, 0x29); // RSVD RESERVED
writeToRegister(0x9C, 0x03); // RSVD RESERVED
writeToRegister(0x9D, 0x4C); // BD50ST 50 Hz Banding Filter Value 
writeToRegister(0x9E, 0x3F); // BD60ST ^0 Hz Banding Filter Value
writeToRegister(0x78, 0x04); // RSVD RESERVED
writeToRegister(0x79, 0x01); // RSVD RESERVED
writeToRegister(0xC8, 0xF0); // RSVD RESERVED
writeToRegister(0x79, 0x0F); // RSVD RESERVED
writeToRegister(0xC8, 0x00); // RSVD RESERVED
writeToRegister(0x79, 0x10); // RSVD RESERVED
writeToRegister(0xC8, 0x7E); // RSVD RESERVED
writeToRegister(0x79, 0x0A); // RSVD RESERVED
writeToRegister(0xC8, 0x80); // RSVD RESERVED
writeToRegister(0x79, 0x0B); // RSVD RESERVED
writeToRegister(0xC8, 0x01); // RSVD RESERVED
writeToRegister(0x79, 0x0C); // RSVD RESERVED
writeToRegister(0xC8, 0x0F); // RSVD RESERVED
writeToRegister(0x79, 0x0D); // RSVD RESERVED
writeToRegister(0xC8, 0x20); // RSVD RESERVED
writeToRegister(0x79, 0x09); // RSVD RESERVED
writeToRegister(0xC8, 0x80); // RSVD RESERVED
writeToRegister(0x79, 0x02); // RSVD RESERVED
writeToRegister(0xC8, 0xC0); // RSVD RESERVED
writeToRegister(0x79, 0x03); // RSVD RESERVED
writeToRegister(0xC8, 0x40); // RSVD RESERVED
writeToRegister(0x79, 0x05); // RSVD RESERVED
writeToRegister(0xC8, 0x30); // RSVD RESERVED
writeToRegister(0x79, 0x26); // RSVD RESERVED
writeToRegister(0xFF, 0xFF);

writeToRegister(0x15, 0x20); // COM10 *REF, *SYNC, PCLK polarities and settings
writeToRegister(0x0C, 0x04); // COM3 output data MSB LSB swap, tristate options, scsale/DCW enable
writeToRegister(0x3E, 0x19); // COM14 DCW and scaling PCLK, manual scaling enable, PCLK divider
writeToRegister(0x72, 0x11); // SCALING_DCWCTR DCW Control
writeToRegister(0x73, 0xF1); // SCALING PCLK_DIV
writeToRegister(0x17, 0x16); // HSTART
writeToRegister(0x18, 0x04); // HSTOP
writeToRegister(0x32, 0xA4); // HREF
writeToRegister(0x19, 0x02); // VSTRT
writeToRegister(0x1A, 0x7A); // VSTOP
writeToRegister(0x03, 0x0A); // VREF
writeToRegister(0xFF, 0xFF); 

writeToRegister(0x12, 0x00); // COM7 output format - YUV?
writeToRegister(0x8C, 0x00); // RSVD RESERVED
writeToRegister(0x04, 0x00); // COM1 CCIR656, AEC low 2 LSB
// writeToRegister(0x40, 0xC0); // COM15 data format output range, RGB555/565 option
writeToRegister(0x14, 0x6A); // COM9 Automatic Gain Ceiling

// MTX Matrix Coefficients
writeToRegister(0x4F, 0x80);
writeToRegister(0x50, 0x80);
writeToRegister(0x51, 0x00);
writeToRegister(0x52, 0x22);
writeToRegister(0x53, 0x5E);
writeToRegister(0x54, 0x80);

//// writeToRegister(0x3D, 0x40); // COM13 Gamma enable, UV saturation auto adjustment, UV swap
writeToRegister(0xFF, 0xFF);

// writeToRegister(0x11, 0x1F);
writeToRegister(0x0C, 0x08); // COM3 output data MSB/LSB swap, tri state options, scale/DCW enable
writeToRegister(0x3E, 0x19); // COM14 DCW and scaling PCLK enable, manual scaling enable, PCLK divider
writeToRegister(0x73, 0xF1); // SCALING_PCLK_DIV
writeToRegister(0x12, 0x10); // COM7 SCCB register reset, output format

delay(1000); // wait for registers to be set (because it is quite slow)
}

uint8_t readFromRegister(int reg) {
  uint8_t value;
  
  Wire.beginTransmission(OV7670_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(); 

  Wire.requestFrom(OV7670_I2C_ADDR, 1); // request one byte of data
  while (Wire.available() < 1); // wait for data
  value = Wire.read();

  Serial.println(value, 2);
  return value;
}

bool writeToRegister(int reg, int value) {
  uint8_t ret;
  Wire.beginTransmission(OV7670_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  ret = Wire.endTransmission();
  if (!handleWriteRegister(ret)) {
    Serial.printf("%d register write failed, Error: %d\nAbort Camera configuration\n", reg, ret);
    return false;
  }
  Serial.printf("%d register write success!\n", reg);
  return true;
}

bool handleWriteRegister(uint8_t ret) {
  if (ret != 0 ) {
    // Serial.printf("ERROR: Wire write failed, error %d\n", ret);
    return false;
  }
  return true;
}

uint8_t readDataPort() {
  uint32_t read = DATAPORT;
  int r0123;
  int r45;
  int r67;

  // rearranging bits to be in order
  // 0,1,2,3 OK
  // 4,5 -> 2.10, 2.11
  // 6,7 -> 2.16, 2.17
  r0123 = read & 0b1111;
  r45 = ((read >> 10) & 0b11) << 4;
  r67 = ((read >> 16) & 0b11) << 6;
  return (r0123 | r45 | r67);

}

bool test_i2c_read() {
  uint8_t pid;
  uint8_t ret;

  Wire.beginTransmission(OV7670_I2C_ADDR);
  Wire.write(REG_PID);
  ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.printf("ERROR: Wire write failed, error %d\n", ret);
    return false;
  }

  Wire.requestFrom(OV7670_I2C_ADDR, 1);
  pid = Wire.read();
  if (pid != 0x76) {
    Serial.printf("ERROR: PID read 0x%X, expected 0x76\n", pid);
    return false;
  }

  Serial.println("test_read passed");

  // reset
  Wire.beginTransmission(OV7670_I2C_ADDR);
  Wire.write(COM7);
  Wire.write(0x80);
  ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.printf("ERROR: COM7 write failed, error %d\n", ret);
    return false;
  }
  delay(1);

  return true;
}

bool setTestPattern() {
  uint8_t ret;
  // set test patterns
  Wire.beginTransmission(OV7670_I2C_ADDR);
  Wire.write(SCALING_XSC);
  Wire.write(0x3A | 0b10000000);
  ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.printf("ERROR: Set test_pattern write failed, error %d\n", ret);
    return false;
  }

  Wire.beginTransmission(OV7670_I2C_ADDR);
  Wire.write(SCALING_YSC);
  Wire.write(0x35 | 0b10000000);
  ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.printf("ERROR: Set test_pattern write failed, error %d\n", ret);
    return false;
  }
  return true;
}
