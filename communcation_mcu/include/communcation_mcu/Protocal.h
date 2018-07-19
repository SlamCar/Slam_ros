#ifndef PROTOCAL_H_
#define PROTOCAL_H_
#include <stdint.h>

#define BODY_MAX_BYTESIZE           1024
#define CRC_BYTESIZE                2


typedef struct SerialProtocal_
{
    uint16_t wCmd;                                    // command code
    uint16_t wLen;                                    // current pask's data length, 1024 max
    uint8_t byData[BODY_MAX_BYTESIZE + CRC_BYTESIZE]; // data content, max size is 1024, append crc16
} SerialProtocal;

typedef struct TaskRequest_
{
    float driverVelocity;
    float steeringAngle;
} TaskRequest_;

#endif