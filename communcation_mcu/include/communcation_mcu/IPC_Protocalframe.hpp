#pragma once

#include <stdio.h>
#include <ros/ros.h>

#include "Transport.hpp"
#include "Protocal.h"
#include "DataBase.hpp"

typedef std::vector<uint8_t> Buffer;

class Transport;

class Ipc_dataframe : public Protocalframe
{
    public:
        Ipc_dataframe(Transport* trans=0);
        ~Ipc_dataframe();
        bool init();
        
        void register_notify(const CmdId id, Notify* _nf){}

        bool data_recv(unsigned char c);
        bool data_parse();
        bool interact(const CmdId id);
    
    private:
    
        bool recv_proc();
        bool send_message(const CmdId id);
        bool send_message(const CmdId id, unsigned char* data, unsigned char len);
        bool send_message(SerialPackage* msg);

        bool databuffer(uint8_t data, Buffer &rxpack);
    
    private:
        SerialPackage active_rx_msg;

        RECEIVE_STATE recv_state;
		Transport* trans; 

       /* boost::thread* recv_thread;
        boost::condition cond_start_recv;
        boost::condition cond_end_recv;
        boost::mutex _lock;
        bool is_run;*/
};
