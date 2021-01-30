/*

  Simple library to use a BH1730FVI Digital Light Sensor.

  Uses I2C communication. 

  Written by Janco Kock, Jan 2021.

*/

#ifndef BH1730_h
#define BH1730_h

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include "Wire.h"

#define BH1730_DEBUG 0

#define BH1730_ADDR 0x29
#define BH1730_PART_NUMBER 0x7

#define BH1730_CMD 0x80
#define BH1730_CMD_SPECIAL 0x60
#define BH1730_CMD_SPECIAL_SOFT_RESET 0x4

#define BH1730_REG_CONTROL 0x00
#define BH1730_REG_GAIN 0x7
#define BH1730_REG_TIMING 0x01
#define BH1730_REG_PART_ID 0x12
#define BH1730_REG_DATA0_LOW 0x14
#define BH1730_REG_DATA0_HIGH 0x15
#define BH1730_REG_DATA1_LOW 0x16
#define BH1730_REG_DATA1_HIGH 0x17

#define BH1730_REG_CONTROL_POWER 0x1
#define BH1730_REG_CONTROL_ADC_EN 0x2
#define BH1730_REG_CONTROL_ONE_TIME 0x8
#define BH1730_REG_CONTROL_ADC_VALID 0x10

#define BH1730_GAIN_X1_MODE 0x00
#define BH1730_GAIN_X2_MODE 0x01
#define BH1730_GAIN_X64_MODE 0x02
#define BH1730_GAIN_X128_MODE 0x03

#define BH1730_RET_TIMEOUT 50

#define BH1730_ITIME 218
#define BH1730_T_INT 2.8
#define BH1730_ITIME_MS ((BH1730_T_INT/1000.0) * 964.0 * (256.0 - BH1730_ITIME))

typedef enum{
  GAIN_X1 = 1,
  GAIN_X2 = 2,
  GAIN_X64 = 64,
  GAIN_X128 = 128
} BH1730_GAIN;

class BH1730 {
  public:
    BH1730();
    bool begin();
    void setGain(BH1730_GAIN gain);
    float readLux();

  private:
    BH1730_GAIN gain = BH1730_GAIN::GAIN_X1;

    uint8_t read8(uint8_t addr);
    uint16_t read16(uint8_t addr);
    void write8(uint8_t addr, uint8_t data);
};

#endif
