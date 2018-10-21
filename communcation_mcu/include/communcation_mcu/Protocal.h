#ifndef PROTOCAL_H_
#define PROTOCAL_H_
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HEADER_BYTESIZE             6
#define BODY_MAX_BYTESIZE           8
#define CRC_BYTESIZE                2

#define MODULEID                    0x039Cu

enum CmdId
{
    /**
     * TEST
     */
    DEBUG_TEST_COMMOND    = 0x0001u,
    /**
     * QT -> STM32
     */
    DEBUG_QT_COMMOND      = 0xA010u,
    /**
     * QT || IPC -> STM32
     */
    CMD_GET_VERSION       = 0x1010u,
    CMD_IPC_COMMOND       = 0x2010u,
    CMD_RESET             = 0x2020u,
    /**
     * STM32 -> IPC || QT
     */
    STM32_FEED_BACK       = 0X5010,
    STM32_HEART_BEAT      = 0x6010,
    STM32_TASK_FINISH     = 0x7010,
};

typedef struct DataHead_
{
    uint16_t moduleId;
    uint16_t dataId;
    uint8_t  dataLen;
    uint8_t  recvLen;
} Head;

typedef struct SerialPackage_
{
    Head head_;      // fixed for MODULEID  9.24  birthday
    uint8_t byData_[BODY_MAX_BYTESIZE]; // data content, max size is 50
    uint16_t check_;//append crc16
} SerialPackage;

typedef struct IpcCommand_
{
    float driverVelocity;
    float steeringAngle;
} IpcCommand;



#ifdef __cplusplus
}
#endif

#endif