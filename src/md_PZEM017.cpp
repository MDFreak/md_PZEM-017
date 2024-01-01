/*
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
#if defined(USE_PZEM017_RS485) // to be defined in platformio.ini
  #include <md_PZEM017.h>
  #include <md_defines.h>
  #include <stdio.h>
  // addresses read registers
    #define REG_VOLTAGE     0x0000
    #define REG_CURRENT     0x0001
    #define REG_POWER_L     0x0002
    #define REG_POWER_H     0x0003
    #define REG_ENERGY_L    0x0004
    #define REG_ENERGY_H    0x0005
    #define REG_HVALARM     0x0006
    #define REG_LVALARM     0x0007
  // addresses write registers
    #define WREG_HV_ALARM_THR   0x0000
    #define WREG_LV_ALARM_THR   0x0001
    #define WREG_ADDR           0x0002
    #define WREG_SHUNT          0x0003
  // commands
    #define CMD_RHR         0x03
    #define CMD_RIR         0X04
    #define CMD_WSR         0x06
    #define CMD_CAL         0x41
    #define CMD_REST        0x42
  // device types
    #define SHUNT_100A      0x0000
    #define SHUNT_50A       0x0001
    #define SHUNT_200A      0x0002
    #define SHUNT_300A      0x0003
    #define SHUNT_10A       0x0004
    #define PZEM003         0x0300
    #define PZEM004         0x0400
    #define PZEM017         0x1100

  // comm defines
    #define UPDATE_TIME     200
    #define RESPONSE_SIZE   32
    #define READ_TIMEOUT    1000 //100
    #define PZEM_BAUD_RATE  9600
    #ifndef DEBUG
        #define DEBUG       DEBUG_MODE
      #endif
  // externals
    extern HardwareSerial Serial;
  /* implementation RS485 multi slave

   */
  uint8_t _pbusy = FALSE;
  void printBuf(uint8_t* buffer, uint16_t len)
    {
      #ifdef DEBUG
          for(uint16_t i = 0; i < len; i++)
            {
                char temp[6];
                sprintf(temp, "%.2x ", buffer[i]);
                Serial.print(temp);
            }
          Serial.println();
        #endif
    }
  /* class md_PZEM017 -> only Hardware serial constructor
   * @param port Hardware serial to use
   * @param addr Slave address of device
   */
  //md_PZEM017::md_PZEM017(HardwareSerial* port, uint8_t addr, uint8_t pin_dir)//, uint8_t addr)
//              md_PZEM017(HardwareSerial* port, uint8_t pin_dir = 255, uint8_t addr=PZEM_DEFAULT_ADDR);
  //md_PZEM017::md_PZEM017(HardwareSerial& port, uint32_t config, uint8_t rxPin, uint8_t txPin)
  md_PZEM017::md_PZEM017(HardwareSerial& port,  //RS485_SlaveData_t* slaveData, uint8_t slaveCount,
                         uint32_t config, uint8_t rtsPin,   uint8_t rxPin,  uint8_t txPin)
    {
      port.begin(PZEM_BAUD_RATE, config, rxPin, txPin);
      this->_serial  = (Stream*)&port;
      //this->_slavedata = slaveData;
      //this->_slavecount = slaveCount;
      if (rtsPin < 34)
        {
          _rtsPin = rtsPin;
          pinMode(_rtsPin, OUTPUT);
          digitalWrite(_rtsPin, 1);             // receive
                //S3VAL("   conf PZeM ready #", _addr, "val", digitalRead(RS485_rtsPin));
        }
    }
  uint8_t   md_PZEM017::config(RS485_SlaveData_t* slaveData, uint16_t addr)
    {
      uint8_t _ret = false;
      //if(addr < 0x01 || addr > 0xF8) { addr = PZEM_DEFAULT_ADDR; } // Sanity check of address
        S2VAL("PZEM.config addr &slaveData", addr, (uint32_t)slaveData);
      slaveData->devaddr = addr;
      _ret = getParameters(slaveData, addr);
          S4VAL("  config devaddr devtype ULV UHV ", (uint16_t )(slaveData->devaddr),
                 slaveData->devtype, slaveData->LVAlarmVoltage, slaveData->HVAlarmVoltage );
      // Set initial lastRed time so that we read right away
      //lastInputRead    = 0;
      //lastInputRead   -= UPDATE_TIME;
      return _ret;
    }
  uint16_t   md_PZEM017::getParameters(RS485_SlaveData_t* slaveData, uint16_t addr)
    {
      static uint8_t  response[13];
             uint16_t ret = FALSE;
      //if (!slaveData) { slaveData = &_slavedata[slaveIdx]; }
      S4VAL("  getParameters devaddr devtype ULV UHV ", addr, slaveData->devtype, slaveData->LVAlarmVoltage, slaveData->HVAlarmVoltage );
      // If we read before the update time limit, do not update

      for (uint8_t i = 0 ; i < 3 ; i++)
        { // try 3x to contact
          // Read 3 registers starting at 0x00
          sendCmd8(CMD_RHR, 0x00, 0x04, addr, false);
          ret = receive(response, 13);
          if( ret == 13) // Something went wrong
            { // Update the current paramaters
              slaveData->HVAlarmVoltage = (  (uint32_t)response[3] << 8    // Raw voltage in 0.01V
                                           | (uint32_t)response[4])/100.0;
              slaveData->LVAlarmVoltage = (  (uint32_t)response[5] << 8    // Raw voltage in 0.01V
                                           | (uint32_t)response[6])/100.0;
              slaveData->devaddr =        (  (uint32_t)response[7] << 8    // Raw address 0x00-0xf7
                                           | (uint32_t)response[8]);
              slaveData->devtype =        (  (uint32_t)response[9] << 8    // Shunt type 0x0000 - 0x0003 (100A/50A/200A/300A)
                                           | (uint32_t)response[10]);
              break;
            }
          else
            {
              //S3VAL("ERROR trial", i, "getParameters.receive went wrong -> ret ", ret);
            }
        }
     // Record current time as _lastHoldingRead
      //S4VAL("  getParameters devaddr devtype ULV UHV ", addr, slaveData->devtype, slaveData->LVAlarmVoltage, slaveData->HVAlarmVoltage );
      return (uint8_t) ret;
    }
      /*
        float     md_PZEM017::getHighvoltAlarmValue(uint8_t slaveIdx)
          {
            //if(!getParameters())
            //    return NAN;
            return _slavedata[slaveIdx].HVAlarmVoltage;
          }
        float     md_PZEM017::getLowvoltAlarmValue(uint8_t slaveIdx)
          {
            //if(!getParameters())
            //    return NAN;
            return _slavedata[slaveIdx].LVAlarmVoltage;
          }
        uint16_t  md_PZEM017::getDevicetype(uint8_t slaveIdx)
          {
            //if(!getParameters())
            //  { return NAN; }
            return _slavedata[slaveIdx].devtype;
          }
       */
  uint8_t   md_PZEM017::updateValues(RS485_SlaveData_t* slaveData)
    {
      //static uint8_t buffer[] = {0x00, CMD_RIR, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00};
      static uint8_t response[21];
             uint8_t ret = FALSE;
      // If we read before the update time limit, do not update
      //if(_slavedata[slaveIdx].lastInputRead + UPDATE_TIME > millis())
      //  { return true; }
      for (uint8_t i=0 ; i<3 ; i++)
        {
          // Read 10 registers starting at 0x00 (no check)
              //sendCmd8(CMD_RIR, 0x00, 0x08, false, _slavedata[slaveIdx].devaddr);
          sendCmd8(CMD_RIR, 0x00, 0x08, slaveData->devaddr, false);
          ret = receive(response, 21);
          if( ret == 21)
            { // Update the current values
              slaveData->voltage =  ((uint32_t)response[3] << 8 | // Raw voltage in 0.01V
                                     (uint32_t)response[4])/100.0;
              slaveData->current =  ((uint32_t)response[5] << 8 | // Raw voltage in 0.01A
                                     (uint32_t)response[6])/100.0;
              slaveData->power =    ((uint32_t)response[7] << 8 | // Raw power in 0.1W
                                     (uint32_t)response[8] |
                                     (uint32_t)response[9] << 24 |
                                     (uint32_t)response[10] << 16) / 10.0;
              slaveData->energy =   ((uint32_t)response[11] << 8 | // Raw Energy in 1Wh
                                     (uint32_t)response[12] |
                                     (uint32_t)response[13] << 24 |
                                     (uint32_t)response[14] << 16) / 1000.0;
              slaveData->HVAlarms = ((uint32_t)response[15] << 8 | // Raw alarm value
                                     (uint32_t)response[16]);
              slaveData->LVAlarms = ((uint32_t)response[17] << 8 | // Raw alarm value
                                     (uint32_t)response[18]);
              slaveData->lastInputRead = millis();
              S4VAL("  updateValues U I P E", slaveData->voltage, slaveData->current, slaveData->power, slaveData->energy);
              break;
            }
          else
            { // Something went wrong
              S2VAL("  ERROR updateValues i ret ", i, ret);
            }
        }
      //for(int i = 0; i < sizeof(response); i++)
        //{
        //  Serial.println(response[i]);
        //}
      return ret;
    }
      /*
        float     md_PZEM017::voltage(uint8_t slaveIdx)
          { // no automatic update
              // if(!updateValues()) // Update vales if necessary
              //   { return NAN; }// Update did not work, return NAN
            return _slavedata[slaveIdx].voltage;
          }
        float     md_PZEM017::current(uint8_t slaveIdx)
          { // no automatic update
              //if(!updateValues())// Update vales if necessary
              //    return NAN; // Update did not work, return NAN
            return _slavedata[slaveIdx].current;
          }
        float     md_PZEM017::power(uint8_t slaveIdx)
          { // no automatic update
              // if(!updateValues()) // Update vales if necessary
              //    return NAN; // Update did not work, return NAN
            return _slavedata[slaveIdx].power;
          }
        // get Active energy in kWh since last reset
        float     md_PZEM017::energy(uint8_t slaveIdx)
          { // no automatic update
              //if(!updateValues()) // Update vales if necessary
              //    return NAN; // Update did not work, return NAN
            return _slavedata[slaveIdx].energy;
          }
       */
  void      md_PZEM017::preTransmission()
    {
      if (_rtsPin != -1)
        { // RTS in use
          for (uint8_t i = 0 ; i < 3; i++)
            {
              //if(RS485_is_inuse)
              //  { // wait for line
              //    STXT("    preTransmission PZeM wait RTS");
              //    usleep(500);
              //  }
              //else
                {
                  //RS485_is_inuse = _addr;
                        //S3VAL("    preTransmission PZeM set RTS", RS485_rtsPin, "value", digitalRead(RS485_rtsPin));
                  digitalWrite(_rtsPin, 1);
                        //SVAL("    preTransmission PZeM RTS set value", digitalRead(RS485_rtsPin));
                  break;
                }
            }
        }
    }
  void      md_PZEM017::postTransmission()
    {
            //S3VAL("    postTransmission PZeM reset RTS", RS485_rtsPin, "value", digitalRead(RS485_rtsPin));
      digitalWrite(_rtsPin, 0);
            //SVAL("    postTransmission PZeM RTS reset value", digitalRead(RS485_rtsPin));
      //RS485_is_inuse = 0;
    }
  /* Prepares the 8 byte command buffer and sends
   * @param[in] cmd - Command to send (position 1)
   * @param[in] rAddr - Register address (postion 2-3)
   * @param[in] val - Register value to write (positon 4-5)
   * @param[in] check - perform a simple read check after write
   *
   * @return success
   */
  uint8_t   md_PZEM017::sendCmd8(uint8_t cmd, uint16_t rAddr, uint16_t val, uint16_t slave_addr, bool check)
    {
      uint8_t sendBuffer[8]; // Send buffer
      uint8_t respBuffer[8]; // Response buffer (only used when check is true)
      //if (   (slave_addr == 0xFFFF)
      //    || (slave_addr < 0x01)
      //    || (slave_addr > 0xF7))
      //  {
      //    slave_addr = addr;
      //  }
      sendBuffer[0] = slave_addr;              // Set slave address
      sendBuffer[1] = cmd;                     // Set command
      sendBuffer[2] = (rAddr >> 8) & 0xFF;     // Set high byte of register address
      sendBuffer[3] = (rAddr) & 0xFF;          // Set low byte =//=
      sendBuffer[4] = (val >> 8) & 0xFF;       // Set high byte of register value
      sendBuffer[5] = (val) & 0xFF;            // Set low byte =//=
      setCRC(sendBuffer, 8);                   // Set CRC of frame
      //S2HEXVAL("  sendCmd8", (uint32_t) sendBuffer[0], (uint32_t) sendBuffer[4]);
      preTransmission();
      //usleep(20);
      _serial->write(sendBuffer, 8); // send frame
      _serial->flush();
      //usleep(100);
      postTransmission();
      if(check)
        {
          if(!receive(respBuffer, 8)) // if check enabled, read the response
            { return false; }
          // Check if response is same as send
          for(uint8_t i = 0; i < 8; i++)
            {
              if(sendBuffer[i] != respBuffer[i])
                { return false; }
            }
        }
      return true;
    }
  /* Set a new device address and update the device
   * WARNING - should be used to set up devices once.
   * Code initializtion will still have old address on next run!
   * @param[in] addr New device address 0x01-0xF7
   * @return success
   */
  uint8_t   md_PZEM017::setAddress(uint16_t slave_addr, RS485_SlaveData_t* slaveData)
    {
      //if(addr < 0x01 || addr > 0xF7) // sanity check
      //  { return false; }
      // Write the new address to the address register
      if(!sendCmd8(CMD_WSR, WREG_ADDR, slaveData->devaddr, slave_addr, true))
        { return false; }
      slaveData->devaddr = slave_addr; // If successful, update the current slave address
      return true;
    }
      /* Get the current device address
        uint8_t   md_PZEM017::getAddress(uint8_t slaveIdx)
          { return _slavedata[slaveIdx].devaddr; }
        // Is the alarm set
        bool      md_PZEM017::isHighvoltAlarmOn(uint8_t slaveIdx)
          {
            //if(!updateValues()) // Update vales if necessary
            //    return NAN; // Update did not work, return NAN
            return _slavedata[slaveIdx].HVAlarms != 0x0000;
          }
        bool      md_PZEM017::isLowvoltAlarmOn(uint8_t slaveIdx)
          {
            //if(!updateValues()) // Update vales if necessary
            //    return NAN; // Update did not work, return NAN
            return _slavedata[slaveIdx].LVAlarms != 0x0000;
          }
       */
  uint8_t   md_PZEM017::resetEnergy(RS485_SlaveData_t* slaveData)
    {
      uint8_t buffer[] = {0x00, CMD_REST, 0x00, 0x00};
      uint8_t reply[5];
      buffer[0] = slaveData->devaddr;
      setCRC(buffer, 4);
      _serial->write(buffer, 4);
      uint16_t length = receive(reply, 5);
      if(length == 0 || length == 5)
        { return false; }
      return true;
    }
  uint8_t   md_PZEM017::setShuntType(uint16_t type, RS485_SlaveData_t* slaveData)
    {
      uint8_t ret = 0;
      if (type < 0 ) // Sanity check
        { type = 0; }
      if (type > 3)
        { type = 3; }
      // Write shunt type to the holding register
      ret = sendCmd8(CMD_WSR, WREG_SHUNT, type, slaveData->devaddr, true);
      return ret;
    }
      /*
        // Set alarm threshold in volts
        bool      md_PZEM017::setHighvoltAlarm(uint16_t volts, uint8_t slaveIdx)
          {
            if (volts < 500) // Sanity check
              { volts = 500; }
            if (volts > 34999) // Sanity check
              { volts = 34999; }
            // Write the volts threshold to the alarm register
            if(!sendCmd8(CMD_WSR, WREG_HV_ALARM_THR, volts, true, slaveIdx))
              { return false; }
            return true;
          }
        bool      md_PZEM017::setLowvoltAlarm(uint16_t volts, uint8_t slaveIdx)
          {
            if (volts < 100) // Sanity check
              { volts = 100; }
            if (volts > 34999) // Sanity check
              { volts = 34999; }
            // Write the volts threshold to the alarm register
            if(!sendCmd8(CMD_WSR, WREG_LV_ALARM_THR, volts, true, slaveIdx))
              { return false; }
            return true;
          }
       */
  /* Receive data from serial with buffer limit and timeout
   *
   * @param[out] resp Memory buffer to hold response. Must be at least `len` long
   * @param[in] len Max number of bytes to read
   * @return number of bytes read
   */
  uint16_t  md_PZEM017::receive(uint8_t *resp, uint16_t len)
    {
      unsigned long startTime = millis(); // Start time for Timeout
      uint8_t index = 0; // Bytes we have read
      while((index < len) && (millis() - startTime < READ_TIMEOUT))
        {
          if(_serial->available() > 0)
            {
              uint8_t c = (uint8_t)_serial->read();
              resp[index++] = c;
            }
          yield();	// do background netw tasks while blocked for IO (prevents ESP watchdog trigger)
        }
      // Check CRC with the number of bytes read
      if(!checkCRC(resp, index))
        { return 0; }
      return index;
    }
  /* md_PZEM017::checkCRC
   * Performs CRC check of the buffer up to len-2 and compares check sum to last two bytes
   * @param[in] data Memory buffer containing the frame to check
   * @param[in] len  Length of the respBuffer including 2 bytes for CRC
   * @return is the buffer check sum valid
   */
  bool      md_PZEM017::checkCRC(const uint8_t *buf, uint16_t len)
    {
      if(len <= 2) // Sanity check
        { return false; }
      uint16_t crc = CRC16(buf, len - 2); // Compute CRC of data
      return ((uint16_t)buf[len-2]  | (uint16_t)buf[len-1] << 8) == crc;
    }
  /* md_PZEM017::setCRC
   * Set last two bytes of buffer to CRC16 of the buffer up to byte len-2
   * Buffer must be able to hold at least 3 bytes
   * @param[out] data Memory buffer containing the frame to checksum and write CRC to
   * @param[in] len  Length of the respBuffer including 2 bytes for CRC
   */
  void      md_PZEM017::setCRC(uint8_t *buf, uint16_t len)
    {
      if(len <= 2) // Sanity check
        { return; }
      uint16_t crc = CRC16(buf, len - 2); // CRC of data
      // Write high and low byte to last two positions
      buf[len - 2] = crc & 0xFF; // Low byte first
      buf[len - 1] = (crc >> 8) & 0xFF; // High byte second
    }
  // Pre computed CRC table
  static const uint16_t crcTable[] PROGMEM =
    {
      0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
      0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
      0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
      0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
      0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
      0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
      0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
      0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
      0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
      0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
      0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
      0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
      0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
      0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
      0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
      0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
      0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
      0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
      0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
      0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
      0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
      0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
      0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
      0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
      0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
      0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
      0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
      0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
      0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
      0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
      0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
      0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
    };
  /* Calculate the CRC16-Modbus for a buffer
   * Based on https://www.modbustools.com/modbus_crc16.html
   * @param[in] data Memory buffer containing the data to checksum
   * @param[in] len  Length of the respBuffer
   * @return Calculated CRC
   */
  uint16_t  md_PZEM017::CRC16(const uint8_t *data, uint16_t len)
    {
      uint8_t nTemp; // CRC table index
      uint16_t crc = 0xFFFF; // Default value
      while (len--)
        {
          nTemp = *data++ ^ crc;
          crc >>= 8;
          crc ^= (uint16_t)pgm_read_word(&crcTable[nTemp]);
        }
      return crc;
    }
  /* md_PZEM017::search
   * Search for available devices. This should be used only for debugging!
   * Prints any found device addresses on the bus.
   * Can be disabled by defining PZEM017_DISABLE_SEARCH
   */
  uint8_t   md_PZEM017::search(uint8_t maxaddr, uint8_t* paddrList, uint8_t* pshuntList)
    {
      #if ( not defined(PZEM017_DISABLE_SEARCH))
          static uint8_t response[7];
          uint8_t* pList = paddrList;
          if (pList)
            {
              pList[0] = 0;
              pList++;
            }
          for(uint16_t addr = 0x01; addr <= maxaddr; addr++)
          //for(uint16_t addr = 0x01; addr <= 0xF8; addr++)
            {
              //Serial.println(addr);
              sendCmd8(CMD_RIR, 0x00, 0x01, addr, false);
              if(receive(response, 7) != 7) // Something went wrong
                {
                  //STXT("ERROR search.receive went wrong -> continue");
                  continue;
                }
              else
                {
                  RS485_SlaveData_t _data;
                  SVAL("  Device on addr: ",(uint16_t )(addr));
                  getParameters(&_data, addr);
                  S4VAL("  devaddr devtype ULV UHV ", (uint16_t )(_data.devaddr), _data.devtype, _data.LVAlarmVoltage, _data.HVAlarmVoltage );
                  if (pList)
                    {
                      *pList = _data.devaddr;
                      pList++;
                      paddrList[0]++;
                    }
                  if (pshuntList)
                    {
                      *pshuntList = _data.devtype;
                      pshuntList++;
                    }
                }
            }
          return paddrList[0];
        #endif
      return -1;
    }
#endif // USE_PZEM017_RS485