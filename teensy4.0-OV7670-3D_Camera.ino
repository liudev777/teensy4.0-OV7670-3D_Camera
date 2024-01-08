/***** Includes *****/
#include <Arduino.h>
#include <Wire.h>
#include <imxrt.h>

/***** Definitions *****/
#define OV7670_I2C_ADDR 0x42 >> 1

/* Registers */
#define REG_PID 0x0a  // Product ID MSB
#define REG_VER 0x0b  // Product ID LSB

/***** Globals *****/

/***** Functions *****/

void setup() {
  // Ignore NACK (camera uses SCCB)
  IMXRT_LPI2C1.MCFGR1 |= LPI2C_MCFGR1_IGNACK;
  pinMode(LED_BUILTIN, OUTPUT);

  // Start XCLK signal
  analogWriteFrequency(9, 16000000);
  analogWrite(9, 128);

  Wire.begin();
  Serial.begin(9600);

  if (test_read()) {
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}

bool test_read() {
  uint8_t pid;
  uint8_t ret;

  Wire.beginTransmission(OV7670_I2C_ADDR);
  Wire.write(REG_PID);
  ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.printf("ERROR: Wire write failed, error %d", ret);
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
