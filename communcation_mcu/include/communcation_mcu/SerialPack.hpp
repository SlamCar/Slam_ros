#pragma once

#include "Serial.hpp"
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
        uint16_t wCrc = 0xFFFF;
        uint16_t wPolynom = 0xA001;

        for (uint32_t i = 0; i < size; ++i)
        {
            wCrc ^= buf[i];
            for (uint32_t j = 0; j < 8; ++j)
            {
                if (wCrc & 0x0001)
                {
                    wCrc = (wCrc >> 1) ^ wPolynom;
                }
                else
                {
                    wCrc = wCrc >> 1;
                }
            }
        }

        return wCrc;
    }
};

class SerialPack : public ProtocalPack
{
public:
    /**
     * \brief default constructor
     */
    SerialPack() {}

    /**
     * \brief constructor with command
     * \param cmd command type
     */
    explicit SerialPack(uint16_t cmd) : msg_{0}
    {
        msg_.wCmd = cmd;
    }

    /**
     * \brief set command
     * \param cmd command type
     */
    inline void setCmd(uint16_t cmd) { msg_.wCmd = cmd; }

    /**
     * \brief wCmd getter
     */
    inline uint16_t cmd() const { return msg_.wCmd; }

    /**
     * \brief set excution status of the command
     * \param cO excution state
     */

    /**
     * \brief set body length of the package
     * \param len body length
     */
    inline void setLen(uint16_t len) { msg_.wLen = len; }

    /**
     * \brief wLen getter
     */
    inline uint16_t len() const { return msg_.wLen; }

    /**
     * \brief append data content to tail of body
     * \param data data to be added
     * \param len data length
     */
    inline void setBody(const uint8_t *data, size_t length)
    {
        std::memcpy(&msg_.byData[0], data, length);
        if (length != msg_.wLen)
        {
            msg_.wLen = length;
        }
    }

    /**
     * \brief return underlayer raw data
     * \return raw data
     */
    inline uint8_t *data() { return reinterpret_cast<uint8_t *>(&msg_); }

        /**
     * brief return data size
     */
    inline std::size_t size() {}

    inline uint8_t *body() { return msg_.byData; }

    /**
     * \brief generate crc code and append to the tail of data body
     */
    inline void generateCrc()
    {
        uint16_t crc = crcVerify(reinterpret_cast<uint8_t *>(&msg_), HEADER_BYTESIZE + msg_.wLen);
        std::memcpy(&msg_.byData[msg_.wLen], reinterpret_cast<uint8_t *>(&crc), sizeof(uint16_t));
    }

    /**
     * \brief check crc
     */
    bool checkCrc()
    {
        uint16_t crcRcv = 0;
        std::memcpy(reinterpret_cast<uint8_t *>(&crcRcv), &msg_.byData[msg_.wLen], sizeof(uint16_t));

        uint16_t crc = crcVerify(reinterpret_cast<uint8_t *>(&msg_), len() + HEADER_BYTESIZE);
        return crcRcv == crc;
    }
private: 
    SerialProtocal msg_;
};
