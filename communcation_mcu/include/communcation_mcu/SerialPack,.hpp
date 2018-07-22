#pragma once

#include "Serial.hpp"
#include "Protocal.h"

class SerialPack
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
