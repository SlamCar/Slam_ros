#include <stdio.h>

#include "Transport.hpp"
#include "IPC_Protocalframe.hpp"
#include "DataBase.hpp"

Ipc_dataframe::Ipc_dataframe(Transport* _trans): trans(_trans){
    recv_state = STATE_RECV_NONE;
}

Ipc_dataframe::~Ipc_dataframe(){
}

bool Ipc_dataframe::init(){
    trans->setTimeout(5000);
    return true;
}

bool Ipc_dataframe::data_recv(unsigned char c)
{
    //ROS_DEBUG("%02x ", c);

    switch (recv_state)
    {
        case STATE_RECV_NONE:
            if (c == 0x03)
            {
                memset(&active_rx_msg,0, sizeof(active_rx_msg));
                active_rx_msg.head_.moduleId = c << 8;
                active_rx_msg.check_ += c;
                
                recv_state = STATE_RECV_HEARD;
            }   
            else
                recv_state = STATE_RECV_NONE;
            break;

        case STATE_RECV_HEARD:
            if (c == 0x9c)
            {
                active_rx_msg.head_.moduleId = c;
                active_rx_msg.check_ += c;
                recv_state = STATE_RECV_ID_H;
            }   
            else
                recv_state = STATE_RECV_NONE;
            break;

        case STATE_RECV_ID_H:   
            active_rx_msg.head_.dataId = c << 8;
            active_rx_msg.check_ += c;
            recv_state = STATE_RECV_ID_L;
            break;

        case STATE_RECV_ID_L:
            active_rx_msg.head_.dataId = c ;
            active_rx_msg.check_ += c;
            recv_state = STATE_RECV_ID_H;
            break;

        case STATE_RECV_LEN:
            active_rx_msg.head_.dataLen = c;
            active_rx_msg.check_ += c;
            if (active_rx_msg.head_.dataLen == 0)
                recv_state = STATE_RECV_CHECK;
            else
                recv_state = STATE_RECV_DATA;
            break;

        case STATE_RECV_DATA:
            active_rx_msg.byData_[active_rx_msg.head_.recv_len++] = c;
            active_rx_msg.check_ += c;
            if (active_rx_msg.head_.recv_len >= active_rx_msg.head_.dataLen)
                recv_state  = STATE_RECV_CHECK;
            break;

        case STATE_RECV_CHECK:
            recv_state = STATE_RECV_NONE;
            if (active_rx_msg.check_ == c)
            {
                return true;
            }
            else
            {
                ROS_ERROR("RX DATA ERROR");
            }
            break;

        default:
            recv_state = STATE_RECV_NONE;
    }

    return false;
}

bool Ipc_dataframe::data_parse()
{
    CmdId id = (CmdId)active_rx_msg.head_.dataId;
    printf("data_parse:id=%d\r\n", id);

    DataBase* dc = DataBase::get();

    switch (id)
    {
        case STM32_FEED_BACK:
            memcpy(&dc->feedbackData_, active_rx_msg.byData_, sizeof(dc->feedbackData_));
            break;
    
        default:
            break;
    }

    return true;
}

bool Ipc_dataframe::interact(const CmdId id)
{
   // printf("Send ... CmdId: %d \r\n", id);

    DataBase* dc = DataBase::get();
    // dc->cmdvelData_.driverVelocity = 1.0;
    // dc->cmdvelData_.steeringAngle = 2.0;

    switch (id)
    {
        case DEBUG_TEST_COMMOND:
            send_message(id);
            break;

        case CMD_IPC_COMMOND:
            send_message(id, (uint8_t *)&dc->cmdvelData_, sizeof(dc->cmdvelData_));
            ROS_DEBUG("aaaaaaaaaaaaaaaaaaaaaaaaaaaa");
            // ROS_DEBUG("data size is %d: ",sizeof(dc->cmdvel_)); 
            break;
        case STM32_FEED_BACK:
            send_message(id);
            break;
    }

    if (!recv_proc())
        return false;
    
    ROS_DEBUG("aaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    return true;
}

bool Ipc_dataframe::recv_proc()
{
    int i=0;
    trans->setTimeout(150);
    bool got=false;
    ROS_DEBUG("recv_proc!!!");
    while(true)
    {
        Buffer data = trans->read();
        
        for (int i = 0;i < data.size(); i++)
        {
            if (data_recv(data[i]))
            {
                got = true;
                break;
            }
        }

        if (got)
            break;
        
        if (trans->isTimeout())
        {
            ROS_ERROR("[TIME OUT !!!]");
            return false;
        }
    }

    if (!data_parse())
        return false;

    return true;
}

bool Ipc_dataframe::send_message(const CmdId id)
{
    SerialPackage msg(id);

    send_message(&msg);

    return true;
}

bool Ipc_dataframe::send_message(const CmdId id, unsigned char* data, unsigned char len)
{
    SerialPackage msg(id, data, len);

    send_message(&msg);

    return true;
}

bool Ipc_dataframe::send_message(SerialPackage* msg)
{
    if (trans == 0)
        return true;
    
    Buffer data((unsigned char*)msg, (unsigned char*)msg+sizeof(msg->head_)+msg->head_.dataLen+1);
    trans->write(data);

    // for(int i=0;i < data.size();i++)
	// {
	// 	std::cout << data[i] - '0' << "     "; // ASII '0' = 48
	// }
    // std::cout << std::endl;
    return true;
}
