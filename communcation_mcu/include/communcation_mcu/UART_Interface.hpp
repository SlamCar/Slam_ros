#pragma once

#include <serial/serial.h>
#include <ros/ros.h>
#include <string>

class TransPort
{
  public:
    virtual bool connect() = 0;
    
    virtual bool state() = 0;
    
    virtual size_t read(uint8_t *buff, size_t size) = 0;
    virtual void write(const uint8_t *buff, size_t size) = 0;
};

class McuSerial
{
public:
    /**
     * constructor.
     */
    McuSerial():timeout_(1000),baudrate_(256000),portId_("/dev/ttyUSB0")
    {
        ROS_DEBUG("[McuSerial]");
        // ros::NodeHandle private_nh("~");

        // private_nh.param("baudrate", baudrate_, 115200);
        // private_nh.param("timeout", timeout_, 1000);
        // private_nh.param("portId", portId_, std::string("/dev/ttyUSB0"));
    }

    /**
     * destructor.
     */
    ~McuSerial()
    {
        if (serialPort_.isOpen())
        {
            serialPort_.close();
        }
    }

    /**
     * connect to serial port.
     * 
     * @return [true if succeed, false otherwise]
     */
    bool connect()
    {
        if (std::string::npos != portId_.find(std::string("USB")))
        {
            std::string cmd = "ls -l /dev/ttyUSB* | awk '{print $10}'";

            FILE *fp = popen(cmd.data(), "r");
            if (nullptr == fp)
            {
                ROS_ERROR("Excute command: (%s) failed", cmd.data());
                return false;
            }

            char line[20] = {0};
            if (nullptr == fgets(line, sizeof(line) - 1, fp))
            {
                ROS_ERROR_STREAM("Cannot find any ttyUSB device!");
                pclose(fp);
                return false;
            }

            pclose(fp);
            portId_ = std::string(line);
            // boost::algorithm::trim((portId_ = std::string(line)));
            ROS_INFO_STREAM("Find usb device: " << portId_);
            ROS_INFO_STREAM("Connecting to serail port...");
        }

        // open serial port for r&w
        try
        {
            // serialPort_.setPort(portId_);
            // serialPort_.setBaudrate(baudrate_);
            // serial::Timeout to = serial::Timeout::simpleTimeout(timeout_);
            // serialPort_.setTimeout(to);
            // serialPort_.open();

            serialPort_.setPort("/dev/ttyUSB0");
            serialPort_.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            serialPort_.setTimeout(to);
            serialPort_.open();
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Unable to open serial port." << e.what());
            return false;
        }
        // catch (serial::SerialException &e)
        // {
        //     ROS_ERROR_STREAM(e.what());
        //     return false;
        // }
        // catch (std::invalid_argument &e)
        // {
        //     ROS_ERROR_STREAM(e.what());
        //     return false;
        // }
        if(serialPort_.isOpen())
        {
            ROS_INFO_STREAM("Connected.");
        }
        return true;
    }

    /**
     * read size bytes data to buff.
     * 
     * @param buff buffer to save the data
     * @param size size to read
     */
    size_t read(uint8_t *buff, size_t size)
    {
        if (serialPort_.isOpen())
        {
            try
            {
                size_t read_size = serialPort_.read(buff, size);
                if (read_size != size)
                {
                    ROS_WARN("Serial read %d bytes data, but we need %d bytes.", (int)read_size, (int)size);
                    // serialPort_.flushInput();
                }

                return read_size;
            }
            catch (serial::PortNotOpenedException &e)
            {
                ROS_ERROR_STREAM(e.what());
                serialPort_.close();
                return 0;
            }
            catch (serial::SerialException &e)
            {
                ROS_ERROR_STREAM(e.what());
                serialPort_.close();
                return 0;
            }
            catch (std::exception &e)
            {
                ROS_ERROR_STREAM(e.what());
                serialPort_.close();
                return 0;
            }
        }
        else
        {
            ROS_ERROR_STREAM("Serial closed!");

            while (!connect())
            {
                ROS_WARN_STREAM("Reconnecting...");
                ros::Duration(1.0).sleep();
            }
        }

        return 0;
    }

    /**
     * write buff data to serial port.
     * 
     * @param buff buffer to send
     * @param size size to write
     */
    void write(const uint8_t *buff, size_t size)
    {
        if (serialPort_.isOpen())
        {
            try
            {
                while (size != serialPort_.write(buff, size) && ros::ok())
                {
                    ROS_ERROR_STREAM("Write size doesn't match requested!");
                    serialPort_.flushOutput();
                }
            }
            catch (serial::PortNotOpenedException &e)
            {
                ROS_ERROR_STREAM(e.what());
                serialPort_.close();
            }
            catch (serial::SerialException &e)
            {
                ROS_ERROR_STREAM(e.what());
                serialPort_.close();
            }
            catch (serial::IOException &e)
            {
                ROS_ERROR_STREAM(e.what());
                serialPort_.close();
            }
        }
    }

    /**
     * \brief data sync
     * \param buff data to sync
     * \param sync data size
     */
    void sync(const uint8_t* buff, size_t size)
    {
        for (size_t i = 0; i < size && ros::ok();)
        {
            uint8_t byte = 0;

            if (sizeof(uint8_t) == read(&byte, sizeof(uint8_t)) && buff[i] == byte)
            {
                i++;
                continue;
            }

            ROS_WARN_STREAM("Sync header failed!");
            i = 0;
        }
    }

    bool isfree()
    {
        return serialPort_.available();
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
};
