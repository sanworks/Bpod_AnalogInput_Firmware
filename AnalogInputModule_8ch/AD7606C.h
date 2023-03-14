/*
----------------------------------------------------------------------------

This file is part of the Sanworks Bpod repository
Copyright (C) 2023 Sanworks LLC, Rochester, New York, USA

----------------------------------------------------------------------------

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3.

This program is distributed  WITHOUT ANY WARRANTY and without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

// Simplified Library for programming the AD7606C ADC as installed on the Bpod Analog Input Module v2

#ifndef AD7606C_h
#define AD7606C_h
#include "Arduino.h"
#include <SPI.h>  

// Registers
// Note: All registers are 8 bits wide

#define REG_READ B01000000
#define REG_WRITE B00000000
#define REG_STATUS 1
#define REG_CONFIG 2
#define REG_RANGE_CH1_CH2 3
#define REG_RANGE_CH3_CH4 4
#define REG_RANGE_CH5_CH6 5
#define REG_RANGE_CH7_CH8 6
#define REG_BANDWIDTH 7
#define REG_OVERSAMPLING 8
const byte REG_GAIN[8] = {9, 10, 11, 12, 13, 14, 15, 16};
const byte REG_OFFSET[8] = {17, 18, 19, 20, 21, 22, 23, 24};
const byte REG_PHASE[8] = {25, 26, 27, 28, 29, 30, 31, 32};
#define REG_CRC 33
#define REG_CRCERROR 34
#define REG_OPEN_DETECT_ENABLE 35
#define REG_OPEN_DETECT 36
#define REG_DIAG_MUX_CH1_CH2 40
#define REG_DIAG_MUX_CH3_CH4 41
#define REG_DIAG_MUX_CH5_CH6 42
#define REG_DIAG_MUX_CH7_CH8 43
#define REG_OPEN_DETECT_QUEUE 44
#define REG_FS_CLK_COUNTER 45
#define REG_OS_CLK_COUNTER 46
#define REG_DEVICEID 47

class AD7606C 
{
public:
  // Constructor
  AD7606C(byte clockEnable, byte intB, byte shutDown);
  union {
    byte byteArray[16];
    uint16_t uint16[8];
    int16_t int16[8];
  } analogData;
  
  void writeRegister(byte regID, byte value);
  uint8_t readRegister(byte regID);
  void readADC();
  void setRange(byte chan, byte rangeCode);
  void setOffset(byte chan, byte newOffset); // newOffset is a byte. 128 = no offset. 0-127 are negative offsets, 129-255 are positive.
  
private:
  byte csPin = 0;
  byte resetPin = 0;
  byte convStartPin = 0;
  byte oversampling = 2; // 0 = None, 1 = 2x, 2 = 4x
  // Range Codes are:
  // 0: +/-2.5V, Single Ended
  // 1: +/-5V, Single Ended
  // 2: +/-6.25V, Single Ended
  // 3: +/-10V, Single Ended
  // 4: +/-12.5V, Single Ended
  // 5: 0-5V, Single Ended
  // 6: 0-10V, Single Ended
  // 7: 0-12.5V, Single Ended
  // 8: +/-5V, Differential
  // 9: +/-10V, Differential
  // 10: +/-12.5V, Differential
  // 11: +/-20V, Differential
  byte rangeCodes[8] = {3,3,3,3,3,3,3,3};
  boolean isBipolarRange[8] = {true,true,true,true,true,true,true,true};
  const byte conversionTime[3] = {10, 20, 35}; // Time to wait for conversion for each oversampling mode. unit = us
  union {
    byte byteArray[4];
    uint16_t uint16[2];
    uint32_t uint32[1];
  } typeBuffer;
};
#endif
