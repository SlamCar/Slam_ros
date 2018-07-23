#ifndef PROTOCAL_H_
#define PROTOCAL_H_
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HEADER_BYTESIZE             16
#define BODY_MAX_BYTESIZE           1024
#define CRC_BYTESIZE                2


enum CmdType
{
    /**
     * Server -> AGV
     */
    CMD_GET_VERSION     = 0x3030u,
    CMD_GET_RTC_TIME    = 0x3130u,
    CMD_GET_DIAGNOSE    = 0x3230u,
    CMD_GET_CONFIG      = 0x3330u,
    CMD_GET_USER_REAL   = 0x3430u,
    CMD_READ_MEM        = 0x3530u,
    CMD_SET_TIME        = 0x3140u,
    CMD_SET_CONFIG      = 0x3340u,
    CMD_WRITE_MEM       = 0x3540u,
    CMD_COM_CAN1TXD     = 0x3050u,
    CMD_COM_CAN2TXD     = 0x3150u,
    CMD_UPDATE_PREPARE  = 0x3060u,
    CMD_UPDATE_DOWN     = 0x3160u,
    CMD_UPDATE_UPCHECK  = 0x3260u,
    CMD_UPDATE_REFLASH  = 0x3360u,
    CMD_RESET           = 0x3070u,

    /**
     * Service protocal
     */
    HEART_REQ           = 0x0002,
    HEART_RSP           = 0XF002,
    ENTER_BUNDLE_REQ    = 0x0003,
    ENTER_BUNDLE_RSP    = 0xF003,
    TASK_FINISH_REQ     = 0x0004,
    TASK_FINISH_RSP     = 0xF004,
    TASK_REQ            = 0x200A,
    TASK_RSP            = 0xF00A,
    REMOTE_CONTROL_REQ  = 0x200B,
    REMOTE_CONTROL_RSP  = 0xF00B,

    /**
     * IPC releated
     */
    IPC_REQ             = 0x0005,
    IPC_RSP             = 0xF005,
};

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
} TaskRequest;

#ifdef __cplusplus
}
#endif

#endif