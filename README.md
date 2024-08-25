# ROS Arduino Encoder

[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://GitHub.com/Naereen/StrapDown.js/graphs/commit-activity)

- A ros node that reads serial data from an encoder and outputs to a topic for further processing.

## Authors

- Dr. Longhao Qian
- Tommy Zhang
- Ji Tong (Kevin) Chen

## How to use

Subscribes to / Input:

- ttyACM0 @ baudrate of 115200

Publishes to:

- /serial/encoder

Requires:

- Place this in /include https://github.com/wjwwood/serial
