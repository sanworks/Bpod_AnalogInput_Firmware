/*
----------------------------------------------------------------------------

This file is part of the Sanworks repository
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


#include <Arduino.h>
#include <SPI.h>
#include "AD7606C.h"

SPISettings ADCSettings(25000000, MSBFIRST, SPI_MODE2);

AD7606C::AD7606C(byte csPin_In, byte resetPin_In, byte convStartPin_In) {
  csPin = csPin_In;
  resetPin = resetPin_In;
  convStartPin = convStartPin_In;
}

void AD7606C::init() {
  pinMode(csPin, OUTPUT);
  pinMode(resetPin, OUTPUT);
  pinMode(convStartPin, OUTPUT);
  digitalWrite(csPin, HIGH);
  digitalWrite(convStartPin, LOW);
  digitalWrite(resetPin, HIGH);
  delay(1);
  digitalWrite(resetPin, LOW);
  SPI.begin();
  writeRegister(REG_CONFIG, 0);
  for (int i = 0; i < 8; i++) {
    setRange(i, DEFAULT_RANGE_CODE);
  }
  writeRegister(REG_OVERSAMPLING, oversampling); // Set to 4x by default
}

void AD7606C::writeRegister(byte regID, byte value) {
  typeBuffer.uint32[0] = 0;
  typeBuffer.byteArray[0] = regID + REG_READ;
  typeBuffer.byteArray[1] = value;
  SPI.beginTransaction(ADCSettings);
  digitalWriteFast(csPin, LOW);
  SPI.transfer(typeBuffer.byteArray, 2);
  typeBuffer.uint32[0] = 0;
  typeBuffer.byteArray[0] = regID + REG_WRITE;
  typeBuffer.byteArray[1] = value;
  SPI.transfer(typeBuffer.byteArray, 2);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}

uint8_t AD7606C::readRegister(byte regID) {
  typeBuffer.uint32[0] = 0;
  typeBuffer.byteArray[0] = regID + REG_READ;
  SPI.beginTransaction(ADCSettings);
  digitalWriteFast(csPin, LOW);
  SPI.transfer(typeBuffer.byteArray, 2);
  typeBuffer.uint32[0] = 0;
  SPI.transfer(typeBuffer.byteArray, 2);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  return typeBuffer.byteArray[1];
}

void AD7606C::readADC() {
  digitalWriteFast(convStartPin, HIGH);
  delayMicroseconds(1);
  digitalWriteFast(convStartPin, LOW);
  delayMicroseconds(conversionTime[oversampling]);
  SPI.beginTransaction(ADCSettings);
  for (int i = 0; i < 8; i++) {
    digitalWriteFast(csPin, LOW);
    analogData.uint16[i] = SPI.transfer16(0);
    digitalWrite(csPin, HIGH);
    if (isBipolarRange[i]) {
      bitWrite(analogData.uint16[i], 15, 1-bitRead(analogData.uint16[i], 15)); // Convert from two's compliment to straight binary
    }
  }
  SPI.endTransaction();
}

void AD7606C::setRange(byte chan, byte rangeCode) {
  uint8_t rangeRegValue = 0;
  rangeCodes[chan] = rangeCode;
  switch (chan) {
    case 0:
    case 1:
      rangeRegValue = rangeCodes[0] + (rangeCodes[1] << 4);
      writeRegister(REG_RANGE_CH1_CH2, rangeRegValue);
    break;
    case 2:
    case 3:
      rangeRegValue = rangeCodes[2] + (rangeCodes[3] << 4);
      writeRegister(REG_RANGE_CH3_CH4, rangeRegValue);
    break;
    case 4:
    case 5:
      rangeRegValue = rangeCodes[4] + (rangeCodes[5] << 4);
      writeRegister(REG_RANGE_CH5_CH6, rangeRegValue);
    break;
    case 6:
    case 7:
      rangeRegValue = rangeCodes[6] + (rangeCodes[7] << 4);
      writeRegister(REG_RANGE_CH7_CH8, rangeRegValue);
    break;
  }
  isBipolarRange[chan] = true;
  if ((rangeCode > 4) && (rangeCode < 8)) {
    isBipolarRange[chan] = false;
  }
}

void AD7606C::setOffset(byte chan, byte newOffset) {
  writeRegister(REG_OFFSET[chan], newOffset);
}
