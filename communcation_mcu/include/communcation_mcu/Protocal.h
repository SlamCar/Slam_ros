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

enum RECEIVE_STATE{
    STATE_RECV_NONE = 0,
    STATE_RECV_HEARD,
    STATE_RECV_ID_H,
    STATE_RECV_ID_L,
    STATE_RECV_LEN,
    STATE_RECV_DATA,
    STATE_RECV_CHECK
};

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
} Head;

typedef struct SerialPackage_
{
    Head head_;      // fixed for MODULEID  9.24  birthday
    uint8_t byData_[BODY_MAX_BYTESIZE]; // data content, max size is 50
    uint16_t check_;//append crc16
} SerialPackage;

// typedef struct SerialPackage_
// {
//     Head head_;      // fixed for MODULEID  9.24  birthday
//     uint8_t byData_[BODY_MAX_BYTESIZE]; // data content, max size is 50
//     uint16_t check_;//append crc16
   
//     SerialPackage_(){}
//     SerialPackage_(uint16_t cmd_id, uint8_t* data = 0,uint16_t len = 0)
//     {
//         head_.moduleId = MODULEID;
//         head_.dataId = cmd_id;
//         head_.dataLen = len;
//         //head_.recv_len = 0;

//         check_ = 0;
        
//         if(data != 0 && len != 0)
//             memcpy(byData_, data, len);
        
//         uint8_t*  _send_buffer = (uint8_t*)this;

//         unsigned int i = 0;
//         for(i = 0; i < sizeof(head_)+head_.dataLen; i++)
//             check_ += _send_buffer[i];
        
//         _send_buffer[sizeof(head_)+head_.dataLen] = check_;
//     }

// } SerialPackage;


typedef struct IpcCommand_
{
    float driverVelocity;
    float steeringAngle;
} IpcCommand;



#ifdef __cplusplus
}
#endif

#endif