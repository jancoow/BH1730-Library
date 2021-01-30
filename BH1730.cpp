/*

  Simple library to use a BH1730FVI Digital Light Sensor.

  Uses I2C communication. 

  Written by Janco Kock, Jan 2021.

*/

#include "BH1730.h"

/**
 * Constructor
 *
 */
BH1730::BH1730() {}


/**
 * Configure sensor
 */
bool BH1730::begin() {
  Wire.begin();

  // Verify if there is a BH1730 on this address
  if((read8(BH1730_REG_PART_ID) >> 4) != BH1730_PART_NUMBER){
  #if BH1730_DEBUG == 1
    Serial.println("BH1730 not found");
  #endif
    return false;
  };

  // Reset 
  write8(BH1730_CMD_SPECIAL | BH1730_CMD_SPECIAL_SOFT_RESET, (1<<7));

  // Check if gain is set before begin
  if(gain != GAIN_X1){
    setGain(gain);
  }

  return true;
}

/**
 *  Set gain of the internal ADC
 * 
 */
void BH1730::setGain(BH1730_GAIN gain){
  if(gain == GAIN_X1){
    write8(BH1730_REG_GAIN, BH1730_GAIN_X1_MODE); 
  }else if(gain == GAIN_X2){
    write8(BH1730_REG_GAIN, BH1730_GAIN_X2_MODE); 
  }else if(gain == GAIN_X64){
    write8(BH1730_REG_GAIN, BH1730_GAIN_X64_MODE); 
  }else if(gain == GAIN_X128){
    write8(BH1730_REG_GAIN, BH1730_GAIN_X128_MODE); 
  }else {
  #if BH1730_DEBUG == 1
    Serial.println("Gain invalid");
  #endif
    return;
  }
  BH1730::gain = gain;
}

/**
 * Read lux level from sensor.
 * Returns -1 if read is timed out
 * 
 */
float BH1730::readLux() {
  // Start one time measurement
  write8(BH1730_REG_CONTROL, BH1730_REG_CONTROL_POWER | BH1730_REG_CONTROL_ADC_EN | BH1730_REG_CONTROL_ONE_TIME);

  // Wait for ADC data is valid
  uint8_t ret = 0;

  while(((read8(BH1730_REG_CONTROL) & BH1730_REG_CONTROL_ADC_VALID) == 0) && ++ret < BH1730_RET_TIMEOUT){
    delay(10);
  }

  if(ret == BH1730_RET_TIMEOUT){
  #if BH1730_DEBUG == 1
    Serial.println("Read timed out");
  #endif
    return -1;
  }

  // Read real light and IR light from registers
  float data0 = (float)read16(BH1730_REG_DATA0_LOW);
  float data1 = (float)read16(BH1730_REG_DATA1_LOW);

  // Calculate lux based on formula in datasheet.
  if(data0 == 0)
    return 0;
  
  float lx = 0;
  float div = data1/data0;

  if(div < 0.26) {
      lx = ((1.29 * data0) - (2.733 * data1)) / gain * 102.6 / BH1730_ITIME_MS;
  }else if(div < 0.55) {
      lx = ((0.795 * data0) - (0.859 * data1)) / gain * 102.6 / BH1730_ITIME_MS;
  }else if(div < 1.09) {
      lx = ((0.51 * data0) - (0.345 * data1)) / gain * 102.6 / BH1730_ITIME_MS;
  }else if(div < 2.13) {
      lx = ((0.276 * data0) - (0.13 * data1)) / gain * 102.6 / BH1730_ITIME_MS;
  }

  return lx;
}


uint8_t BH1730::read8(uint8_t a) {
  uint8_t ret;

  Wire.beginTransmission(BH1730_ADDR); 
  #if (ARDUINO >= 100)
    Wire.write(a | BH1730_CMD); // Send register to read from with CMD field
  #else
    Wire.send(a | BH1730_CMD); // Send register to read from with CMD field
  #endif
    Wire.endTransmission();
    
    Wire.beginTransmission(BH1730_ADDR); 
    Wire.requestFrom(BH1730_ADDR, 1); // Wait for 1 byte
  #if (ARDUINO >= 100)
    ret = Wire.read();
  #else
    ret = Wire.receive(); 
  #endif
    Wire.endTransmission(); 

  return ret;
}

uint16_t BH1730::read16(uint8_t a) {
  uint16_t ret;

  Wire.beginTransmission(BH1730_ADDR);
  #if (ARDUINO >= 100)
    Wire.write(a | BH1730_CMD); // Send register to read from with CMD field
  #else
    Wire.send(a | BH1730_CMD); // Send register to read from with CMD field
  #endif
    Wire.endTransmission();
    
    Wire.beginTransmission(BH1730_ADDR);
    Wire.requestFrom(BH1730_ADDR, 2); // Wait for 2 bytes
  #if (ARDUINO >= 100)
    ret = Wire.read(); // receive LOW DATA first
    ret |= (Wire.read() << 8); // receive HIGH DATA and shift to the left
  #else
    ret = Wire.receive(); // receive LOW DATA first
    ret |= (Wire.receive() << 8); // receive HIGH DATA and shift to the left
  #endif
    Wire.endTransmission(); 

  return ret;
}

void BH1730::write8(uint8_t a, uint8_t d) {
  Wire.beginTransmission(BH1730_ADDR); 
  #if (ARDUINO >= 100)
    Wire.write(a | BH1730_CMD); // Send register to write with CMD field
    Wire.write(d);  // Write data
  #else
    Wire.send(a | BH1730_CMD); // Send register to write with CMD field
    Wire.send(d);  // Write data
  #endif
  Wire.endTransmission(); 
}

