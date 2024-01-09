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

/***** Globals *****/
const uint8_t DATA_PINS[8] = { 10, 12, 11, 13, 6, 9, 8, 7 };  // port 2 pins, arranged for GPIO to be as continuous as possible
const uint8_t MCLK = 5;
const uint8_t HS = 4;
const uint8_t PCLK = 3;
const uint8_t VC = 2;

/***** Functions *****/

void setup() {
  // Ignore NACK (camera uses SCCB)
  IMXRT_LPI2C1.MCFGR1 |= LPI2C_MCFGR1_IGNACK;

  // Start XCLK signal
  analogWriteFrequency(5, 16000000);
  analogWrite(5, 128);

  Wire.begin();
  Serial.begin(9600);
  Serial.println("########################################");

  test_i2c_read();

  // set data pins as inputs
  for (int pin : DATA_PINS) {
    pinMode(pin, INPUT);
  }
}

void loop() {
  int read = DATAPORT;
  int r0;
  int r45;
  int r67;

  // rearranging bits to be in order
  // 0,1,2,3 OK
  // 4,5 -> 2.10, 2.11
  // 6,7 -> 2.16, 2.17
  r0 = read & 0b1111;
  r45 = ((read >> 10) & 0b11) << 4;
  r67 = ((read >> 16) & 0b11) << 6;
  read = r0 | r45 | r67;
  Serial.println(read, BIN);

  delay(100);
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
