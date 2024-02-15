/* Copyright (c) 2023 Martin Dorfner (for md_PZEM017)
  Copyright (c) 2020 Maxz Maxzerker (for PZEM-017)
  Copyright (c) 2019 Jakub Mandula

  Permission is hereby granted, free of charge, to any person obtaining a copy of
  this software and associated documentation files (the “Software”), to deal in
  the Software without restriction, including without limitation the rights to
  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
  the Software, and to permit persons to whom the Software is furnished to do so,
  subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
  FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
  IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* Authors *********************************
     ******************************************
      PZEM-004Tv30.h
      Interface library for the upgraded version of PZEM-004T v3.0
      Based on the PZEM004T library by @olehs https://github.com/olehs/PZEM004T
      Author: Jakub Mandula https://github.com/mandulaj
     ******************************************
      PZEM-017Tv1.h
      Interface library for PZEM-017 v1.0
      Based on the PZEM004T library by Jakub Mandula https://github.com/mandulaj
      Author: Maxz Maxzerker https://github.com/maxzerker
     ******************************************
      md_PZEM017.h
      Interface library for PZEM-003, PZEM-017
      Extended library for PZEM-017 v1.0 from Maxz Maxzerker
      Author: Martin Dorfner https://github.com/mdfreak
     */
/* Description md_PZEM017 V0.1.0 ************
 ** additional features V0.1.0
    - multiple slave usage with single RS485 bus
      - shared Serial usage for 2 md_PZEM017 objects
      - data structure change to slave specific structure
        data static memory is provided from main module via constructor
      -> see class md_PZEM017 constructor
    - introduce RTS Handshake using MAX485/RE-DE
      keep communication without RTS (default) -> RS485_rtsPin = 255

 ** TODO in future
    - introduce task used for measurement cycling

 ** changes to PZEM-017 V0.1.0 from Maxz Maxzerker
    - remove SoftwareSerial - not necessary for ESP32 TODO ongoing
 */


// TODO This project si stopped because Multi-HW does not run stable

#pragma once

#if defined(USE_PZEM017_RS485)
  #ifndef _MD_PZEM017_H_
      #define _MD_PZEM017_H_

      #define MD_PZEM017_VERSION   "V0.1.0"
      #include <Arduino.h>

      // #define PZEM004_NO_SWSERIAL not used
      // device types
        #define PZEM_SHUNT_100A      0x0000
        #define PZEM_SHUNT_50A       0x0001
        #define PZEM_SHUNT_200A      0x0002
        #define PZEM_SHUNT_300A      0x0003
        #define PZEM_SHUNT_10A       0x0004
        #define PZEM003         0x0300
        #define PZEM004         0x0400
        #define PZEM017         0x1100

      #define PZEM_DEFAULT_ADDR    0x01
      typedef struct // _SlaveData
        { // device params
            uint16_t  addr;      // slave address of device
            uint16_t  devaddr;   // slave address of device read from device
            uint16_t  devtype;   // includes shunt type
            float     HVAlarmVoltage;
            float     LVAlarmVoltage;
          // device actual values
            float     voltage;
            float     current;
            float     power;
            float     energy;
            float     frequency;
            float     pf;
            uint16_t  HVAlarms;
            uint16_t  LVAlarms;
          // runtime data
            uint64_t  lastInputRead; // Last time input values were updated
        } RS485_SlaveData_t; // Parameter values
      class md_PZEM017
        {
          public:
            /*
                // Negative Pin Number will keep it unmodified, thus this function can set individual pins
                // SetPins shall be called after Serial begin()
                void HardwareSerial::begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin, bool invert, unsigned long timeout_ms, uint8_t rxfifo_full_thrhd)
                bool setPins(int8_t rxPin, int8_t txPin, int8_t ctsPin = -1, int8_t rtsPin = -1);
                // Enables or disables Hardware Flow Control using RTS and/or CTS pins (must use setAllPins() before)
                bool setHwFlowCtrlMode(uint8_t mode = HW_FLOWCTRL_CTS_RTS, uint8_t threshold = 64);   // 64 is half FIFO Length
                // Used to set RS485 modes such as UART_MODE_RS485_HALF_DUPLEX for Auto RTS function on ESP32
             */
            md_PZEM017(HardwareSerial& port, /* RS485_SlaveData_t* slaveData, uint8_t slaveCount,*/ uint32_t config = SERIAL_8N1,
                       uint8_t rtsPin = -1,   uint8_t rxPin = -1,  uint8_t txPin = -1);
            //md_PZEM017(HardwareSerial* port, uint8_t addr, uint8_t pin_dir = -1);
            ~md_PZEM017() {}

            uint8_t   config(RS485_SlaveData_t* slaveData, uint16_t addr = PZEM_DEFAULT_ADDR); // Init common to all constructors
            uint16_t  getParameters(RS485_SlaveData_t* slaveData, uint16_t addr);
            uint8_t   updateValues(RS485_SlaveData_t* slaveData);    // Get most up to date values from device registers and cache them
            uint8_t   setAddress(uint16_t old_addr, uint16_t new_addr);
            uint8_t   setShuntType(uint16_t type, RS485_SlaveData_t* slaveData);
            uint8_t   resetEnergy(RS485_SlaveData_t* slaveData);
            uint8_t   search(uint8_t minaddr, uint8_t maxaddr = 0xF8, uint8_t autoStop = false);
                //float     voltage(RS485_SlaveData_t* slaveData);
                //float     current(uint8_t slaveIdx = 0);
                //float     power(uint8_t slaveIdx = 0);
                //float     energy(uint8_t slaveIdx = 0);
                //----------------
                //void      begin();
                //uint8_t   getAddress(uint8_t slaveIdx = 0);
                //bool      setHighvoltAlarm(uint16_t volts, uint8_t slaveIdx = 0);
                //float     getHighvoltAlarmValue(uint8_t slaveIdx = 0);
                //bool      isHighvoltAlarmOn(uint8_t slaveIdx = 0);
                //bool      setLowvoltAlarm(uint16_t volts, uint8_t slaveIdx = 0);
                //float     getLowvoltAlarmValue(uint8_t slaveIdx = 0);
                //bool      isLowvoltAlarmOn(uint8_t slaveIdx = 0);
                //uint16_t  getDevicetype(uint8_t slaveIdx = 0);
          private:
            Stream*            _serial      = NULL;;  // Serial interface
              //uint8_t            _slavecount  = 0; // count of slaves
              //RS485_SlaveData_t* _slavedata   = NULL;
            uint8_t            _rtsPin      = -1;
                  //uint8_t   _addr;    // Device address
                  //struct // _currentValues
                  //  {
                  //    float     voltage;
                  //    float     current;
                  //    float     power;
                  //    float     energy;
                  //    uint16_t  HVAlarms;
                  //    uint16_t  LVAlarms;
                  //  } _currentValues; // Measured values
                  //struct // _parameterValues
                  //  {
                  //    float     HVAlarmVoltage;
                  //    float     LVAlarmVoltage;
                  //    uint16_t  address;
                  //    uint16_t  shunttype;
                  //  } _parameterValues; // Parameter values
              //uint64_t  _lastInputRead;   // Last time input values were updated
              //uint64_t  _lastHoldingRead; // Last time input values were updated
            void      getFree(); // Send 8 byte command
            uint8_t   sendCmd8(uint8_t cmd, uint16_t rAddr, uint16_t val, uint16_t slave_addr, bool check=false); // Send 8 byte command
            uint16_t  receive(uint8_t *resp, uint16_t len); // Receive len bytes into a buffer
            void      setCRC(uint8_t *buf, uint16_t len);           // Set the CRC for a buffer
            uint8_t   checkCRC(const uint8_t *buf, uint16_t len);   // Check CRC of buffer
            uint16_t  CRC16(const uint8_t *data, uint16_t len); // Calculate CRC of buffer
            void      preTransmission();
            void      postTransmission();
        };
      #endif // _MD_PZEM017_H_
  #endif // _USE_PZEM017_RS485_H_