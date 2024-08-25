/*

MIT License

Copyright (c) 2024 FSC Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
#ifndef ROS_ARDUINO_ENCODER_ROS_ARDUINO_ENCODER_HPP
#define ROS_ARDUINO_ENCODER_ROS_ARDUINO_ENCODER_HPP

#include <iostream>
#include <ros/ros.h>
#include <cstdint>
#include <ctime>
#include <cmath>
#include <vector>
#include "serial/serial.h"
#include "geometry_msgs/Vector3Stamped.h"

#include <thread>
#include <chrono>

class RosArduinoEncoderNode
{
public:
    union MsgUint32_t
    {
        uint32_t value;
        uint8_t bits[4];
    };

    void Update(void);
    RosArduinoEncoderNode(ros::NodeHandle &nh);
    ~RosArduinoEncoderNode() = default;

private:
    void InitializeSerial(serial::Serial &serial);
    void ReadEncoder(serial::Serial &serial);
    void PubEncoderRaw(void);

    std::unique_ptr<serial::Serial> serialPort;
    std::vector<uint8_t> buffer{13, 0};
    MsgUint32_t posX{0};
    MsgUint32_t posY{0};
    MsgUint32_t posZ{0};

    // ros pubs
    ros::Publisher encoderPub;
    geometry_msgs::Vector3Stamped encoderRawMsg;
};

#endif