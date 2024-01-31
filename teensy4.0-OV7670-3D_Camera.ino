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
#define COM15 0x40 // Common control 15 RGB 555/565 option
#define MVFP 0x1E // Scan direction control

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
  Serial.begin(9600);
  Serial.println("########################################");

  test_i2c_read();
  configureCamera();

  // set data pins as inputs
  for (int pin : DATA_PINS) {
    pinMode(pin, INPUT);
  }
  pinMode(HS, INPUT);
  pinMode(VS, INPUT);
  pinMode(PCLK, INPUT);
}

void loop() {
  
  getPicture();
  printPicture();
  Serial.println("********************************************\n********************************************");
  delay(1000);
}

void printPicture() {
  for (int b = 0; b < height * width * bytesPerPixel; b++) {
    uint8_t pixel = buffer[b];
    if (pixel < 0x10) {
      Serial.print('0'); // ex: pads B C A to 0B 0C 0A
    }
    Serial.print(pixel, HEX);
  }
  Serial.println();

  delay(500);
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
    while (digitalReadFast(HS) == HIGH);
  }
  interrupts();
}

void configureCamera() {
  // reset registers
  if (!writeToRegister(COM7, 0x80)) {
    return;
  }
  
  delay(1000);

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
  return true;
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
