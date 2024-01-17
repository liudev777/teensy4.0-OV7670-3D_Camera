/***** Includes *****/
#include <Arduino.h>
#include <Wire.h>
#include <imxrt.h>

/***** Definitions *****/
#define OV7670_I2C_ADDR 0x42 >> 1
#define DATAPORT GPIO7_PSR  // using GPIO port 2/7 - 7 is the high-speed version

/* Registers */
#define REG_PID 0x0a  // Product ID MSB
#define REG_VER 0x0b  // Product ID LSB
#define COM7 0x12
#define COM15 0x40
#define SCALING_XSC 0x70
#define SCALING_YSC 0x71

/***** Globals *****/
const uint8_t DATA_PINS[8] = { 10, 12, 11, 13, 6, 9, 8, 7 };  // port 2 pins, arranged for GPIO to be as continuous as possible
const uint8_t MCLK = 5;
const uint8_t HS = 4;
const uint8_t PCLK = 3;
const uint8_t VS = 2;

const int width = 320;
const int height = 240;
const int bytesPerPixel = 2;
const int bytesPerRow = width * bytesPerPixel;
uint8_t buffer[320 * 240 * 2];

/***** Functions *****/

void setup() {
  // Ignore NACK (camera uses SCCB)
  IMXRT_LPI2C1.MCFGR1 |= LPI2C_MCFGR1_IGNACK;

  // Start MCLK
  analogWriteFrequency(MCLK, 16000000);
  analogWrite(MCLK, 128);

  Wire.begin();
  Serial.begin(9600);
  Serial.println("########################################");

  test_i2c_read();
  setFormat();

  // set data pins as inputs
  for (int pin : DATA_PINS) {
    pinMode(pin, INPUT);
  }
  pinMode(HS, INPUT);
  pinMode(PCLK, INPUT);
  pinMode(VS, INPUT);
}

void loop() {
  noInterrupts();
  int pixels = 0;

  // wait for frame start
  while (digitalReadFast(VS) == LOW);
  while (digitalReadFast(VS) == HIGH);

  for (int i = 0; i < height; i++) {
    // wait for HREF start
    while (digitalReadFast(HS) == LOW);

    for (int j = 0; j < bytesPerRow; j++) {
      // wait for PCLK low to read data
      while (digitalReadFast(PCLK) == HIGH);

      buffer[pixels] = readDataport();
      pixels++;

      while (digitalReadFast(PCLK) == LOW);
    }
    while (digitalReadFast(HS) == HIGH);
  }
  interrupts();


  for (int i = 0; i < height * width * bytesPerPixel; i++) {
    uint8_t p = buffer[i];
    if (p < 0x10) {
      // pad to two digits
      Serial.print('0');
    }
    Serial.print(p, HEX);
  }
  Serial.println();

  delay(1000);
}

uint8_t readDataport() {
  int read;
  int r0;
  int r45;
  int r67;

  read = DATAPORT;
  r0 = read & 0b1111;
  r45 = ((read >> 10) & 0b11) << 4;
  r67 = ((read >> 16) & 0b11) << 6;

  return (r0 | r45 | r67);
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
  return true;
}

bool setFormat() {
  uint8_t ret;
  // set to RGB output mode
  Wire.beginTransmission(OV7670_I2C_ADDR);
  Wire.write(COM7);
  Wire.write(0b00010100);
  ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.printf("ERROR: COM7 write failed, error %d\n", ret);
    return false;
  }

  // set to RGB565 output
  Wire.beginTransmission(OV7670_I2C_ADDR);
  Wire.write(COM15);
  Wire.write(0xC0 | 0b010000);
  ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.printf("ERROR: COM15 write failed, error %d\n", ret);
    return false;
  }
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

void printAscii() {
  // use with YUV mode
  Serial.println("===");

  for (int row = 0; row < height; row++) {
    for (int col = 0; col < bytesPerRow; col += 2) {
      Serial.print(grayscaleToAscii(shift(buffer[bytesPerRow * row + col])));
    }
    Serial.println();
  }
  Serial.println("===");
}

uint8_t shift(uint8_t value) {
  return ((value & 0x01) << 7) | ((value & 0x02) << 5) | ((value & 0x04) << 3) | ((value & 0x08) << 1) | ((value & 0x10) >> 1) | ((value & 0x20) >> 3) | ((value & 0x40) >> 5) | ((value & 0x80) >> 7);
}

char grayscaleToAscii(int grayscale) {
  const char* asciiScale = "@%#*+=-:. ";
  int index = grayscale * 9 / 255;
  return asciiScale[index];
}

// uint8_t read_register(uint8_t reg, uint8_t* buffer) {
//   uint8_t ret;

//   Wire.beginTransmission(OV7670_I2C_ADDR);
//   Wire.write(reg);
//   ret = Wire.endTransmission();
//   if (ret != 0) {
//     Serial.printf("ERROR: read_register write failed, error %d", ret);
//     return ret;
//   }

//   Wire.requestFrom(OV7670_I2C_ADDR, 1);
//   *buffer = Wire.read();
//   return 0;
// }

// uint8_t write_register(uint8_t reg, uint8_t val) {
//   uint8_t ret;

//   Wire.beginTransmission(OV7670_I2C_ADDR);
//   Wire.write(reg);
//   Wire.write(val);
//   ret = Wire.endTransmission();
//   if (ret != 0) {
//     Serial.printf("ERROR: write_register write failed, error %d", ret);
//     return ret;
//   }
//   return 0;
// }
