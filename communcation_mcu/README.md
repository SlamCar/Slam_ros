# communcation_mcu_node

## ROS publish and advertise
### advertise topic
      name                 type        
    /cmd_vel            msgs/CmdVel

### publish topic
       name                type                frate
    /feedBack           msgs/FeedBack          50hz

## Serial 

### input param   
    name                  default
    port               /dev/ttyUSB0
    baud                  115200

    

## Usage:
```
roslaunch communcation_mcu communcation.launch
```
