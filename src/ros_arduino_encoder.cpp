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

#include "ros_arduino_encoder.hpp"

RosArduinoEncoderNode::RosArduinoEncoderNode(ros::NodeHandle &nh)
{
    serialPort = std::make_unique<serial::Serial>("/dev/ttyUSB0", 115200UL, serial::Timeout::simpleTimeout(10000));
    InitializeSerial(*serialPort);

    encoderPub = nh.advertise<geometry_msgs::Vector3Stamped>("/encoder/position_raw", 1);
}

void RosArduinoEncoderNode::InitializeSerial(serial::Serial &serial)
{
    constexpr uint32_t inter_byte_timeout = 1000;
    constexpr uint32_t read_timeout_constant = 1000;
    constexpr uint32_t read_timeout_multiplier = 200;
    constexpr uint32_t write_timeout_constant = 1000;
    constexpr uint32_t write_timeout_multiplier = 200;

    serial.setTimeout(inter_byte_timeout,
                      read_timeout_constant,
                      read_timeout_multiplier,
                      write_timeout_constant,
                      write_timeout_multiplier);

    serial.flushInput();
}

void RosArduinoEncoderNode::ReadEncoder(serial::Serial &serial)
{
    serial.flushInput();
    auto flag = serial.waitReadable();
    if (!flag)
    { // check for message received
        std::cout << "Timeout - No message received \n";
        return;
    }
    [[maybe_unused]] size_t length = serial.read(buffer.data(), 13); // read to buffer, byte length is 13 because header + 3x4 bytes

    if (buffer[0] == 0xEF)
    {
        posX.bits[0] = buffer[1];
        posX.bits[1] = buffer[2];
        posX.bits[2] = buffer[3];
        posX.bits[3] = buffer[4];

        posY.bits[0] = buffer[5];
        posY.bits[1] = buffer[6];
        posY.bits[2] = buffer[7];
        posY.bits[3] = buffer[8];

        posZ.bits[0] = buffer[9];
        posZ.bits[1] = buffer[10];
        posZ.bits[2] = buffer[11];
        posZ.bits[3] = buffer[12];

        encoderRawMsg.vector.x = posX.value;
        encoderRawMsg.vector.y = posY.value;
        encoderRawMsg.vector.z = posZ.value;
    }
}

// publish msgs

void RosArduinoEncoderNode::Update(void)
{
    ReadEncoder(*serialPort);
    PubEncoderRaw();
}

void RosArduinoEncoderNode::PubEncoderRaw(void)
{
    encoderPub.publish(encoderRawMsg);
}
