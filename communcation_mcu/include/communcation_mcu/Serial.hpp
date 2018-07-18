#pragma once

#include <serial/serial.h>
#include <ros/ros.h>
#include <string>

class McuSerial
{
public:
    /**
     * constructor.
     */
    McuSerial()
    {
        ros::NodeHandle private_nh("~");

        private_nh.param("baudrate", baudrate_, 115200);
        private_nh.param("timeout", timeout_, 1000);
        private_nh.param("portId", portId_, std::string("/dev/ttyUSB0"));
    }

    /**
     * destructor.
     */
    ~CotekSerial()
    {
        if (serialPort_.isOpen())
        {
            serialPort_.close();
        }
    }
private:
    // using serial lib for ros to handle serial communication.
    serial::Serial serialPort_;
    
    // communication baudrate.
    int32_t baudrate_;

    // recevive timeout threshold which cause read timeout.
    int32_t timeout_;

    // port id like "/dev/ttyUSB0"
    std::string portId_;
}
