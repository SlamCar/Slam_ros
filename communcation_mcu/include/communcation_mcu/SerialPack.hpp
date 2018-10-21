#pragma once

#include "UART_Interface.hpp"
#include "Protocal.h"

/**
 * \class ProtocalPack
 * \brief base protocal for socket communication
 */
class ProtocalPack
{
  public:
    /**
     * \brief return underlayer raw data
     */
    virtual uint8_t *data() = 0;

    /**
     * brief return data size
     */
    virtual std::size_t size() = 0;

    /**
     * \biref It's CRC16-Modbus checking.
     *
     * \param  ucBuffer [Input buffer.]
     * \param  uiBufLen [Input bufferAL_HPP_ length.]
     * \return          [crc result]
     */
    uint16_t crcVerify(const uint8_t *buf, std::size_t size)
    {
        uint16_t wCrc = 0;
        for(int i = 0; i < size; i++)
            wCrc += buf[i]; 
        return wCrc;
    }
};

class DataPack : public ProtocalPack
{
  public:
    /**
     * \brief default constructor
     */
    DataPack() {}

    /**
     * \brief constructor with command
     * \param cmd command type
     */
    explicit DataPack(uint16_t cmd) : msg_{0} 
    {
        msg_.head_.moduleId = MODULEID; 
        msg_.head_.dataId = cmd;
        msg_.head_.recvLen = 0;
    }

    inline void setDataId(uint16_t cmd) { msg_.head_.dataId = cmd; }
    inline void setLen(uint16_t len) { msg_.head_.dataLen = len; }
    inline void setBody(const uint8_t *data, size_t length)
    {
        std::memcpy(&msg_.byData_, data, length);
        if (length != msg_.head_.dataLen)
        {
            msg_.head_.dataLen = length;
        }
    }

    inline uint16_t dataId() const { return msg_.head_.dataId; }
    inline uint16_t len() const { return msg_.head_.dataLen; }

    inline uint8_t *data() { return reinterpret_cast<uint8_t *>(&msg_); }
    inline uint8_t *body() 
    { 
        for(int t = 0; t < 8; t++)
            ROS_INFO("[0x%02x]",msg_.byData_[t]);    
        return msg_.byData_; 
    }

    inline std::size_t size()
    {
        return HEADER_BYTESIZE + BODY_MAX_BYTESIZE + CRC_BYTESIZE;
    }

    inline void generateCrc()
    {
        uint16_t crc = crcVerify(reinterpret_cast<uint8_t *>(&msg_), HEADER_BYTESIZE + msg_.head_.dataLen);
        msg_.check_ = crc;
        //std::memcpy(&msg_.byData[msg_.head_.dataLen], reinterpret_cast<uint8_t *>(&crc), sizeof(uint16_t));
    }

    bool checkCrc()
    {
        //uint16_t crcRcv = 0;
        //std::memcpy(reinterpret_cast<uint8_t *>(&crcRcv), &msg_.byData[msg_.head_.dataLen], sizeof(uint16_t));

        uint16_t crc = crcVerify(reinterpret_cast<uint8_t *>(&msg_), HEADER_BYTESIZE + len());
        //return crcRcv == crc;
        return msg_.check_ == crc;
    }

  private: 
    SerialPackage msg_;
};
