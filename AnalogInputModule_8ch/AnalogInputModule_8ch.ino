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
// Analog Module firmware
// Federico Carnevale, October 2016
// Revised by Josh Sanders, April 2018 - June 2023

// **NOTE** previous versions of this firmware required dependencies and modifications to the Teensy core files. As of firmware v4, these are no longer necessary.
// **NOTE** Requires Arduino 1.8.13 or newer, and Teensyduino 1.5.4

#include "ArCOM.h" // A wrapper for Arduino serial interfaces. See https://sites.google.com/site/sanworksdocs/arcom
#include <SPI.h>
#include "SdFat.h"

#define FIRMWARE_VERSION 6

// SETUP MACROS TO COMPILE FOR TARGET DEVICE:
#define HARDWARE_VERSION 2 // Use: 1 = AIM rev 1.0-1.2 (as marked on PCB), 2 = AIM rev 2.0
//-------------------------------------------

// Validate macros
#if (HARDWARE_VERSION < 1) || (HARDWARE_VERSION > 2)
#error Error! HARDWARE_VERSION must be either 1 or 2
#endif

#if (HARDWARE_VERSION == 1)
  #include "AD7327.h" // Library for the AD7327 Analog to digital converter IC
  #define N_RANGES 4 // Number of ranges supported
#else
  #include "AD7606C.h" // Library for the AD7606C Analog to digital acquisition system IC
  #define CS_PIN 41 // Chip select 
  #define RESET_PIN 14 // Hard reset
  #define CONV_START_PIN 15 // Conversion start signal - ADC samples 8ch simultaneously on rising edge of CONV_START
  #define N_RANGES 12 // Number of ranges supported
#endif
byte circuitRevisionArray[5] = {28,29,30,31,32};
SdFs SDcard;
bool ready = false; // Indicates if SD is busy (for use with SDBusy() funciton)

// Module setup
char moduleName[] = "AnalogIn"; // Name of module for manual override UI and state machine assembler

#if HARDWARE_VERSION == 1
  AD7327 AD(39); // Create AD, an AD7327 ADC object.
  const uint32_t bitMax = 8191;
#else
  AD7606C AD(CS_PIN, RESET_PIN, CONV_START_PIN);
  const uint32_t bitMax = 65535;
#endif

ArCOM USBCOM(SerialUSB); // Creates an ArCOM object called USBCOM, wrapping SerialUSB. See https://sites.google.com/site/sanworksdocs/arcom
#if HARDWARE_VERSION == 1
  ArCOM StateMachineCOM(Serial3); // Creates an ArCOM object for the state machine
#else
  ArCOM StateMachineCOM(Serial1); // Creates an ArCOM object for the state machine
#endif
ArCOM OutputStreamCOM(Serial2); // Creates an ArCOM object for the output stream

// Extra memory for state machine serial buffer
byte StateMachineSerialBuf[192] = {0};

// Digital i/o pins available from side of enclosure (not currently used; can be configured as an I2C interface)
byte DigitalPin1 = 18;
byte DigitalPin2 = 19;

// System objects
IntervalTimer hardwareTimer; // Hardware timer to ensure even sampling
FsFile DataFile; // File on microSD card, to store waveform data
FsFile CalFile; // File on microSD card, to store calibration data

// Op menu variable
byte opCode = 0; // Serial inputs access an op menu. The op code byte stores the intended operation.
byte opSource = 0; // 0 = op from USB, 1 = op from UART1, 2 = op from UART2. More op code menu options are exposed for USB.
boolean newOpCode = 0; // this flag is true if an opCode was read from one of the ports
byte OpMenuByte = 213; // This byte must be the first byte in any USB serial transmission. Reduces the probability of interference from port-scanning software
byte inByte = 0; // General purpose temporary byte

// Channel counts
const byte nPhysicalChannels = 8; // Number of physical channels on device
byte nActiveChannels = 8; // Number of channels currently being read (consecutive, starting at ch1)

// State Flags
boolean StreamSignalToUSB = false; // Stream to USB
boolean StreamSignalToModule = false; // Send adc reads to output or DDS module through serial port
volatile boolean LoggingDataToSD = false; // Logs active channels to SD card
boolean SendEventsToUSB = false; // Send threshold crossing events to USB
boolean SendEventsToStateMachine = false; // Send threshold crossing events to state machine
boolean AppConnected = false; // True if PC-side software is connected to the device
volatile boolean sd2USBflag = false; // True if data is in cue to be returned from the microSD card to USB

// State variables
byte streamChan2Module[nPhysicalChannels] = {0}; // List of channels streaming to module
byte streamChan2USB[nPhysicalChannels] = {0}; // List of channels streaming to USB
byte voltageRanges[nPhysicalChannels] = {0}; // Voltage range indexes of channels
uint32_t samplingRate = 1000; // in Hz 
double timerPeriod = 0;

// Voltage threshold-crossing detection variables
byte eventChannels[nPhysicalChannels] = {0}; // Indicates channels that generate events when events are turned on globally
boolean eventEnabled[nPhysicalChannels*2] = {1}; // Events are disabled after a threshold crossing, until the value crosses resetValue
uint16_t thresholdValue[nPhysicalChannels*2] = {0}; // Threshold voltage (in bits).
uint16_t resetValue[nPhysicalChannels*2] = {0}; // Voltage (in bits) of reset event
boolean thresholdDirection[nPhysicalChannels*2] = {0}; // Indicates whether resetValue is less than (0) or greater than (1) thresholdValue
                                                     // This also determines whether a voltage greater than (0) or less than (1) threshold
                                                     // will trigger an event.
boolean thresholdEventDetected[nPhysicalChannels*2] = {0};

// SD variables
volatile uint32_t nFullBufferReads = 0; // Number of full buffer reads in transmission
volatile uint32_t nRemainderBytes = 0; // Number of bytes remaining after full transmissions
#if HARDWARE_VERSION == 1
  const uint32_t sdReadBufferSize = 4096;  //2048
#else
  const uint32_t sdReadBufferSize = 1024; // HW version 2 must transmit microSD ==> USB more slowly so the PC can keep up
#endif
uint8_t sdReadBuffer[sdReadBufferSize] = {0};
const uint32_t sdWriteBufferSize = 2048; // in bytes
volatile uint16_t sdWriteBuffer[nPhysicalChannels*sdWriteBufferSize*2] = {0}; // These two buffers store data to be written to microSD. 
                                                                     // One is dumped to microSD in the main loop,
                                                                     // while the other is filled in the timer callback.
volatile uint16_t sdWriteBuffer2[nPhysicalChannels*sdWriteBufferSize*2] = {0};
volatile uint32_t writeBufferPos = 0;
volatile uint32_t writeBuffer2Pos = 0;
volatile byte currentBuffer = 0; // Current buffer being written to microSD

// PSRAM setup (for bench testing only, PSRAM is not used in current firmware)
#if HARDWARE_VERSION > 1
  extern "C" uint8_t external_psram_size;
  uint32_t *memory_begin, *memory_end;
  boolean memOK = false;
#endif

// Other variables
int16_t zeroCodeOffset[12] = {0}; // Zero-code offset corrections for each range
int16_t thisZCC = 0; // Temporary variable to store zero-code correction during computation
uint32_t nSamplesAcquired = 0; // Number of samples acquired since logging started
uint32_t maxSamplesToAcquire = 0; // maximum number of samples to acquire on startLogging command. 0 = infinite
uint32_t sum = 0; // Sum for calculating zero-code correction
volatile byte writeFlag = 0; // Incremented if a write buffer contains samples to be written to SD. Decremented as buffers are written.
byte streamPrefix = 'R'; // Byte sent before each sample of data when streaming to output module
byte usbDataPrefix = 'R'; // Byte sent before each sample of data when streaming to usb
boolean usbSyncFlag = false; // True if a sync signal arrived from the state machine. If streaming, this is relayed to PC along with the current samples
byte usbSyncData = 0; // A byte relayed with each sync message
byte circuitRevision = 0; // A byte containing the circuit revision, read from an array of pins on the board
boolean throttleUSB = false; // If true, USB transfer is throttled to 300kB/s (necessary for MATLAB built-in serial interface)
union { // Union for conversion of calibration values to <-> from SD card
    byte byteArray[2];
    int16_t int16;
} typeBuffer;
union { // Union for streaming adc values to USB
    byte byteArray[16000];
    uint16_t uint16[8000];
} usbBufferA;
union { // Union for streaming adc values to USB (used to capture data while USBBufferA is being transmitted + vice versa)
    byte byteArray[16000];
    uint16_t uint16[8000];
} usbBufferB;
byte currentUSBBuffer = 0; // Which USB buffer is currently being used to capture data (0 or 1)
uint16_t usbBufferPos[2] = {0}; // Current position in each USB buffer (number of samples captured)
boolean usbBufferFlag = false; // True if new data is available in the current USB buffer

void setup() {
  AD.init();
  pinMode(DigitalPin1, OUTPUT);
  digitalWrite(DigitalPin1, LOW);
  #if HARDWARE_VERSION == 1
    pinMode(DigitalPin1, OUTPUT);
    digitalWrite(DigitalPin1, HIGH); // This allows a potentiometer to be powered from the board, for coarse diagnostics. V2 has a 5V reference output for this.
    Serial3.addMemoryForRead(StateMachineSerialBuf, 192);
    Serial3.begin(1312500);
  #else
    memory_begin = (uint32_t *)(0x70000000); // PSRAM start address
    memory_end = (uint32_t *)(0x70000000 + external_psram_size * 1048576); // PSRAM end address
    Serial1.addMemoryForRead(StateMachineSerialBuf, 192);
    Serial1.begin(1312500);
  #endif
  Serial2.begin(1312500); // Select the highest value your CAT5e/CAT6 cable supports without dropped bytes: 1312500, 2457600, 3686400, 7372800
  
  // Read hardware revision from circuit board (an array of grounded pins indicates revision in binary, grounded = 1, floating = 0)
  circuitRevision = 0;
  for (int i = 0; i < 5; i++) {
    pinMode(circuitRevisionArray[i], INPUT_PULLUP);
    circuitRevision += pow(2, i)*digitalRead(circuitRevisionArray[i]);
    pinMode(circuitRevisionArray[i], INPUT);
  }
  circuitRevision = 31-circuitRevision;

  //SPI.begin();
  SDcard.begin(SdioConfig(FIFO_SDIO)); // Initialize microSD card
  if (SDcard.exists("Cal.wfm")) {
    CalFile = SDcard.open("Cal.wfm", O_RDWR | O_CREAT);
    for (int i = 0; i < N_RANGES; i++) {
      CalFile.read(typeBuffer.byteArray, 2);
      zeroCodeOffset[i] = typeBuffer.int16;
    }
    CalFile.close();
  }
  SDcard.remove("Data.wfm");
  DataFile = SDcard.open("Data.wfm", O_RDWR | O_CREAT);  
  #if HARDWARE_VERSION == 2
    for (int i = 0; i < 8; i++) {
      AD.setRange(i, 3);
      AD.setOffset(i, 128+zeroCodeOffset[3]);
    }
  #endif 
  for (int i = 0; i < nPhysicalChannels*2; i++) {
    thresholdValue[i] = bitMax;
  }
  timerPeriod = (1/(double)samplingRate)*1000000;
  hardwareTimer.begin(handler, timerPeriod); // hardwareTimer is an interval timer object - Teensy 3.6's hardware timer
}

void loop() {
  if (writeFlag > 0) { // If data is available to be written to microSD
    if (currentBuffer == 0) { // If the data was loaded into buffer 0
      currentBuffer = 1; // Make the current buffer = 1
      DataFile.write(sdWriteBuffer, writeBufferPos*2); // Write to microSD
      writeBufferPos = 0; // Reset buffer write position
    } else {
      currentBuffer = 0;
      DataFile.write(sdWriteBuffer2, writeBuffer2Pos*2);
      writeBuffer2Pos = 0;
    }
    writeFlag--;
  }
  if (usbBufferFlag) {
    currentUSBBuffer = 1-currentUSBBuffer;
    usbBufferPos[currentUSBBuffer] = 0;
    if (currentUSBBuffer == 1) {
      USBCOM.writeByteArray(usbBufferA.byteArray, usbBufferPos[0]*2);
    } else {
      USBCOM.writeByteArray(usbBufferB.byteArray, usbBufferPos[1]*2);
    }
    Serial.flush();
    usbBufferFlag = false;
  }
  if (sd2USBflag) {
    for (int i = 0; i < nFullBufferReads; i++) { // Full buffer transfers; skipped if nFullBufferReads = 0
      while (sdBusy()) {}
      DataFile.read(sdReadBuffer, sdReadBufferSize);
      while (sdBusy()) {}
      Serial.write(sdReadBuffer, sdReadBufferSize);
      if (throttleUSB) {
        delayMicroseconds(500);
      }    
    }
    if (nRemainderBytes > 0) {
      while (sdBusy()) {}
      DataFile.read(sdReadBuffer, nRemainderBytes);
      while (sdBusy()) {}
      Serial.write(sdReadBuffer, nRemainderBytes);
      Serial.send_now();   
    }
    sd2USBflag = false;
  }
}

void handler(void) {
  if (AppConnected) {
    AD.readADC(); // Reads all active channels and stores the result in a buffer in the AD object: AD.analogData[]
  }
  if (StateMachineCOM.available() > 0) { // If bytes arrived from the state machine
    opCode = StateMachineCOM.readByte(); // Read in an op code
    opSource = 1; // 0 = USB, 1 = State machine, 2 = output stream (DDS, Analog output module, etc)
    newOpCode = true;
  } else if (OutputStreamCOM.available() > 0) {
    opCode = OutputStreamCOM.readByte();
    opSource = 2; // UART 2
    newOpCode = true;
  } else if (USBCOM.available() > 0) {
    if (USBCOM.readByte() == OpMenuByte) { // This extra menu access byte avoids most issues with auto-polling software (some versions of Ubuntu)
      opCode = USBCOM.readByte();
      opSource = 0; // USB
    };
    newOpCode = true;
  }

  if (newOpCode) { // If an op byte arrived from one of the serial interfaces
    newOpCode = false;
    switch (opCode) {
      case 'O': // USB initiated new connection; reset all state variables
        if (opSource == 0) {
          USBCOM.writeByte(161); // Send acknowledgement byte
          USBCOM.writeUint32(FIRMWARE_VERSION); // Send firmware version
          AppConnected = true;
          StreamSignalToUSB = false;
          SendEventsToUSB = false;
          LoggingDataToSD = false;
          usbSyncFlag = false;
          SendEventsToStateMachine = false;
          StreamSignalToModule = false;
          samplingRate = 1000;
          nActiveChannels = 8;
          #if HARDWARE_VERSION == 1
            AD.setNchannels(nActiveChannels);
          #endif
          for (int i = 0; i > nPhysicalChannels; i++) {
            streamChan2Module[i] = 0;
            streamChan2USB[i] = 0;
            voltageRanges[i] = 0;
            #if HARDWARE_VERSION == 1
              AD.setRange(i, 0);
            #else
              AD.setRange(i, 3);
            #endif
            eventChannels[i] = 0;
            eventEnabled[i] = 0;
            thresholdValue[i] = 0;
            thresholdDirection[i] = 0;
            resetValue[i] = 0;
            thresholdDirection[i] = 0;
            thresholdEventDetected[i] = 0;
          }
          hardwareTimer.end();
          timerPeriod = (1/(double)samplingRate)*1000000;
          hardwareTimer.begin(handler, timerPeriod);
        }
      break;

      case 255: // Return Bpod module info
        if (opSource == 1) { // Only returns this info if requested from state machine device
          returnModuleInfo();
        } 
      break;

      case 254: // Relay test byte from USB to echo module, or from echo module back to USB 
        if (opSource == 0) {
          OutputStreamCOM.writeByte(254);
        }
        if (opSource == 2) {
          USBCOM.writeByte(254);
        }
      break;

      case 'H':
        USBCOM.writeByte(HARDWARE_VERSION);
      break;

      case 'S': // Start/Stop data streaming
        inByte = readByteFromSource(opSource);
        switch (inByte) {
          case 0:
            StreamSignalToUSB = (boolean)readByteFromSource(opSource);
          break;
          case 1:
            StreamSignalToModule = (boolean)readByteFromSource(opSource);
              if (opSource == 0) {
                USBCOM.writeByte(1); // Send confirm byte
              }
          break;
        }
      break;

      case '#': // Relay sync byte and the current AIM timestamp to PC during streaming
        if (opSource == 1) {
          usbSyncData = StateMachineCOM.readByte();
          usbSyncFlag = true;
        }
      break;

      case 'E': // Start/Stop threshold event detection + transmission
        inByte = readByteFromSource(opSource);
        switch (inByte) {
          case 0:
            SendEventsToUSB = (boolean)readByteFromSource(opSource);
          break;
          case 1:
            SendEventsToStateMachine = (boolean)readByteFromSource(opSource);
          break;
        }
        for (int i = 0; i < nPhysicalChannels*2; i++) {
          eventEnabled[i] = true; // All channels start out enabled until first threshold crossing (though only eventChannels are evaluated)
        }
        if (opSource == 0) {
          USBCOM.writeByte(1); // Send confirm byte
        }
      break;

      case 'L': // Start/Stop logging data from active channels to microSD card
        inByte = readByteFromSource(opSource);
        switch (inByte) {
          case 0: // Stop logging
            stopLogging();
          break;
          case 1: // Start logging
            DataFile.seek(0);
            LoggingDataToSD = true;
            nSamplesAcquired = 0;
            writeBufferPos = 0;
            writeBuffer2Pos = 0;
            currentBuffer = 0;
          break;
        }
        if (opSource == 0) {
          USBCOM.writeByte(1); // Send confirm byte
        }
      break;
      
      case 'C': // Set subset of channels to stream raw data (USB and module)
        if (opSource == 0) {
          USBCOM.readByteArray(streamChan2USB, nPhysicalChannels);
          USBCOM.readByteArray(streamChan2Module, nPhysicalChannels);
          USBCOM.writeByte(1); // Send confirm byte
        }
      break;

      case 'R': // Select ADC Voltage range for each channel
          if (opSource == 0) {
            if (!LoggingDataToSD){
              USBCOM.readByteArray(voltageRanges, nPhysicalChannels);
              for (int i = 0; i < nPhysicalChannels; i++) {
                AD.setRange(i, voltageRanges[i]);
                #if HARDWARE_VERSION == 2
                  AD.setOffset(i, 128+zeroCodeOffset[voltageRanges[i]]);
                #endif
              }
              USBCOM.writeByte(1); // Send confirm byte
            } else {
              for (int i = 0; i < nPhysicalChannels; i++) { // Clear input buffer (more elegantly in future ArCOM versions)
                USBCOM.readByte();
              }
              USBCOM.writeByte(0); // Send error byte
            }
          }
      break;

      case 'A': // Set max number of actively sampled channels
        if (opSource == 0) {
          if (!LoggingDataToSD) {
            nActiveChannels = USBCOM.readByte();
            #if HARDWARE_VERSION == 1
              AD.setNchannels(nActiveChannels);
            #endif
            USBCOM.writeByte(1); // Send confirm byte
          } else {
            USBCOM.readByte();
            USBCOM.writeByte(0); // Send error byte
          }
        }
      break;

      case 'T': // Set thresholds and reset values
        if (opSource == 0) {
          for (int i = 0; i < nPhysicalChannels*2; i++) { // Read in threshold values (in bits)
            thresholdValue[i] = USBCOM.readUint16();
          }
          for (int i = 0; i < nPhysicalChannels*2; i++) { // Read in reset values (in bits)
            resetValue[i] = USBCOM.readUint16();
          }
          for (int i = 0; i < nPhysicalChannels*2; i++) { // Set sign of reset value with respect to threshold
            if (resetValue[i] < thresholdValue[i]) {
              thresholdDirection[i] = 0;
            } else {
              thresholdDirection[i] = 1;
            }
          }
          USBCOM.writeByte(1); // Send confirm byte
        }
      break;
      
      case 'K': // Set channels that generate events
        if (opSource == 0) {
          USBCOM.readByteArray(eventChannels, nPhysicalChannels);
          USBCOM.writeByte(1); // Send confirm byte
        }
      break;

      case 'D': // Read SD card and send data to USB
        if (opSource == 0) {
            while (sdBusy()) {}
            DataFile.seek(0);
            if (nSamplesAcquired*nActiveChannels*2 > sdReadBufferSize) {
              nFullBufferReads = (uint32_t)(floor(((double)nSamplesAcquired)*double(nActiveChannels)*2 / (double)sdReadBufferSize));
            } else {
              nFullBufferReads = 0;
            } 
            USBCOM.writeUint32(nSamplesAcquired); 
            nRemainderBytes = (nSamplesAcquired*nActiveChannels*2)-(nFullBufferReads*sdReadBufferSize);    
            LoggingDataToSD = false;
            sd2USBflag = true;
          }
      break;

      case 'F': // Change sampling frequency
          if (opSource == 0) {
            if (!LoggingDataToSD) {
              samplingRate = USBCOM.readUint32();
              hardwareTimer.end();
              timerPeriod = (1/(double)samplingRate)*1000000;
              hardwareTimer.begin(handler, timerPeriod);
              USBCOM.writeByte(1); // Confirm byte
            } else {
              USBCOM.readUint32();
              USBCOM.writeByte(0); // Error byte
            }
          }
      break;

      case 'W': // Set maximum number of samples to acquire on command to log data
        if (opSource == 0) {
          if (!LoggingDataToSD) {
            maxSamplesToAcquire = USBCOM.readUint32();
            USBCOM.writeByte(1);
          } else {
            USBCOM.readUint32();
            USBCOM.writeByte(0); // Error byte
          }
        }
      break;

      case 'P': // Set output stream prefix byte (sent once before each sample)
        if (opSource == 0) {
          streamPrefix = USBCOM.readByte();
          USBCOM.writeByte(1); // Send confirm byte
        }
      break;

      case 'o': // Set ADC voltage offset (HW version 2 only)
         #if HARDWARE_VERSION == 2
          if (opSource == 0) {
            byte offsetChan = USBCOM.readByte();
            byte offsetVal = USBCOM.readByte();
            AD.setOffset(offsetChan, offsetVal);
          }
        #endif
      break;

      case 't': // throttle USB
        throttleUSB = USBCOM.readByte();
      break;

      case '%': // test PSRAM (HW v2 only)
        #if HARDWARE_VERSION > 1
           if (opSource == 0) {
             USBCOM.writeByte(external_psram_size);
             memOK = true;
             if (!check_fixed_pattern(0x55555555)) {memOK = false;}
             if (!check_fixed_pattern(0x33333333)) {memOK = false;}
             if (!check_fixed_pattern(0x0F0F0F0F)) {memOK = false;}
             if (!check_fixed_pattern(0x00FF00FF)) {memOK = false;}
             if (!check_fixed_pattern(0x0000FFFF)) {memOK = false;}
             if (!check_fixed_pattern(0xAAAAAAAA)) {memOK = false;}
             if (!check_fixed_pattern(0xCCCCCCCC)) {memOK = false;}
             if (!check_fixed_pattern(0xF0F0F0F0)) {memOK = false;}
             if (!check_fixed_pattern(0xFF00FF00)) {memOK = false;}
             if (!check_fixed_pattern(0xFFFF0000)) {memOK = false;}
             if (!check_fixed_pattern(0xFFFFFFFF)) {memOK = false;}
             if (!check_fixed_pattern(0x00000000)) {memOK = false;}
             USBCOM.writeByte(memOK);
           }
        #endif
      break;

      case 'X': // Disconnect from PC-side app
        AppConnected = false;
      break;

      case 'Z': // Measure and set zero-code offset (first, connect a wire between channel 1 signal and ground; ch1 on device = ch0 in code)
                // Note that on the AD7606C, the zero code offset is programmed into the ADC chip, not handled in firmware
      if ((opSource == 0) && (!LoggingDataToSD)) {
        #if HARDWARE_VERSION == 2
          AD.setOffset(0, 128);
        #endif
        for (int i = 0; i < N_RANGES; i++) {
          AD.setRange(0, i);
          sum = 0;
          for (int j = 0; j < 1000; j++) {
            AD.readADC();
            sum += AD.analogData.uint16[0];
          }
          thisZCC = 0;
          #if HARDWARE_VERSION == 1
            if (i < 4) { // Exclude single-ended ranges
              thisZCC = (int16_t)(4095-(sum/1000));
            }
          #else
            if ((i < 5) || (i > 7)) { // Exclude single-ended ranges
              thisZCC = (int16_t)(32768-(sum/1000));
            }
          #endif
          zeroCodeOffset[i] = thisZCC;
        }
        AD.setRange(0, voltageRanges[0]);
        #if HARDWARE_VERSION == 2
          for (int i = 0; i < 8; i++) {
            AD.setOffset(i, 128+zeroCodeOffset[voltageRanges[i]]);
          }
        #endif        
        DataFile.close();
        SDcard.remove("Cal.wfm");
        CalFile = SDcard.open("Cal.wfm", O_RDWR | O_CREAT);
        for (int i = 0; i < N_RANGES; i++) {
          typeBuffer.int16 = zeroCodeOffset[i];
          CalFile.write(typeBuffer.byteArray, 2);
        }
        CalFile.close();
        DataFile = SDcard.open("Data.wfm", O_RDWR | O_CREAT);
      }
      break;
    }// end switch(opCode)
  }// end newOpCode
  
  for (int i = 0; i < nActiveChannels; i++) { // Detect threshold crossings and send event bytes to targets
    #if HARDWARE_VERSION == 1
      AD.analogData.uint16[i] += zeroCodeOffset[voltageRanges[i]];
    #endif
    if (eventChannels[i]) { // If event reporting is enabled for this channel
      for (int j = 0; j < 2; j++) {
        byte thisEvent = j*8;
        thresholdEventDetected[i+thisEvent] = false;
        if (eventEnabled[i+thisEvent]) { // Check for threshold crossing
          if (thresholdDirection[i+thisEvent] == 0) { // If crossing is from low to high voltage
            if (AD.analogData.uint16[i] >= thresholdValue[i+thisEvent]) {
              thresholdEventDetected[i+thisEvent] = true;
            }
          } else { // If crossing is from high to low voltage
            if (AD.analogData.uint16[i] <= thresholdValue[i+thisEvent]) {
              thresholdEventDetected[i+thisEvent] = true;
            }
          }
          if (thresholdEventDetected[i+thisEvent]) {
            if (SendEventsToUSB) {
              USBCOM.writeByte(i+1+thisEvent); // Convert to event code (event codes are indexed by 1)
            }
            if (SendEventsToStateMachine) {
              StateMachineCOM.writeByte(i+1+thisEvent); // Convert to event code (indexed by 1)
            }
            eventEnabled[i+thisEvent] = false;
          }
        } else { // Check for re-enable
          if (thresholdDirection[i+thisEvent] == 0) {
            if (AD.analogData.uint16[i] <=  resetValue[i+thisEvent]) {
              eventEnabled[i+thisEvent] = true;
            }
          } else {
            if (AD.analogData.uint16[i] >=  resetValue[i+thisEvent]) {
              eventEnabled[i+thisEvent] = true;
            }
          }
        }
      }
    }
  }
  
  if (LoggingDataToSD) {
    LogData();
  } 
  if (StreamSignalToUSB) { // Stream data to USB
    if (usbSyncFlag) {
      usbDataPrefix = '#'; // Sync + Read
      usbSyncFlag = false;
    } else {
      usbDataPrefix = 'R'; // Read
    }
    if (currentUSBBuffer == 0) {
      usbBufferA.byteArray[usbBufferPos[currentUSBBuffer]*2] = usbDataPrefix;
      usbBufferA.byteArray[usbBufferPos[currentUSBBuffer]*2 + 1] = usbSyncData;
    } else {
      usbBufferB.byteArray[usbBufferPos[currentUSBBuffer]*2] = usbDataPrefix;
      usbBufferB.byteArray[usbBufferPos[currentUSBBuffer]*2 + 1] = usbSyncData;
    }
    usbSyncData = 0;
    usbBufferPos[currentUSBBuffer]++; 
    for (int i = 0; i < nActiveChannels; i++) {
        if (streamChan2USB[i]) {
          if (currentUSBBuffer == 0) {
            usbBufferA.uint16[usbBufferPos[0]] = AD.analogData.uint16[i];
            usbBufferPos[0]++;
          } else {
            usbBufferB.uint16[usbBufferPos[1]] = AD.analogData.uint16[i];
            usbBufferPos[1]++;
          }
        }
    }
    usbBufferFlag = true;
  }

  if (StreamSignalToModule) {
    OutputStreamCOM.writeByte(streamPrefix);
    for (int i = 0; i < nActiveChannels; i++) {
      if (streamChan2Module[i]) {
        OutputStreamCOM.writeUint16(AD.analogData.uint16[i]);
      }
    }
  }
} // End main timer loop

// Log data
void LogData() {
  if (currentBuffer == 0) {
    for (int i = 0; i < nActiveChannels; i++) {
      sdWriteBuffer[writeBufferPos] = AD.analogData.uint16[i]; writeBufferPos++;
    }
  } else {
    for (int i = 0; i < nActiveChannels; i++) {
      sdWriteBuffer2[writeBuffer2Pos] = AD.analogData.uint16[i]; writeBuffer2Pos++;
    }
  }
  writeFlag++;
  nSamplesAcquired++;
  if (nSamplesAcquired == maxSamplesToAcquire) {
    stopLogging();
  }
}

void stopLogging() {
  LoggingDataToSD = false;
}

byte readByteFromSource(byte opSource) {
  switch (opSource) {
    case 0:
      return USBCOM.readByte();
    break;
    case 1:
      return StateMachineCOM.readByte();
    break;
    case 2:
      return OutputStreamCOM.readByte();
    break;
  }
}

#if HARDWARE_VERSION > 1
   // This memory test was adopted from PJRC's teensy41_psram_memtest repository : https://github.com/PaulStoffregen/teensy41_psram_memtest
   bool check_fixed_pattern(uint32_t pattern)
   {
     volatile uint32_t *p;
     for (p = memory_begin; p < memory_end; p++) {
       *p = pattern;
     }
     arm_dcache_flush_delete((void *)memory_begin,
       (uint32_t)memory_end - (uint32_t)memory_begin);
     for (p = memory_begin; p < memory_end; p++) {
       uint32_t actual = *p;
       if (actual != pattern) return false;
     }
     return true;
   }
#endif

bool sdBusy() {
  return ready ? SDcard.card()->isBusy() : false;
}

void returnModuleInfo() {
  boolean fsmSupportsHwInfo = false;
  delayMicroseconds(100);
  if (StateMachineCOM.available() == 1) { // FSM firmware v23 or newer sends a second info request byte to indicate that it supports additional ops
    if (StateMachineCOM.readByte() == 255) {fsmSupportsHwInfo = true;}
  }
  StateMachineCOM.writeByte('A'); // Acknowledge
  StateMachineCOM.writeUint32(FIRMWARE_VERSION); // 4-byte firmware version
  StateMachineCOM.writeByte(sizeof(moduleName)-1); // Length of module name
  StateMachineCOM.writeCharArray(moduleName, sizeof(moduleName)-1); // Module name
  StateMachineCOM.writeByte(1); // 1 if more info follows, 0 if not
  StateMachineCOM.writeByte('#'); // Op code for: Number of behavior events this module can generate
  StateMachineCOM.writeByte(16); // 8 channels, 2 thresholds each
  if (fsmSupportsHwInfo) {
    StateMachineCOM.writeByte(1); // 1 if more info follows, 0 if not
    StateMachineCOM.writeByte('V'); // Op code for: Hardware major version
    StateMachineCOM.writeByte(HARDWARE_VERSION); 
    StateMachineCOM.writeByte(1); // 1 if more info follows, 0 if not
    StateMachineCOM.writeByte('v'); // Op code for: Hardware minor version
    StateMachineCOM.writeByte(circuitRevision); 
  }
  StateMachineCOM.writeByte(0); // 1 if more info follows, 0 if not
} 
