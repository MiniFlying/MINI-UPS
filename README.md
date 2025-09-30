| Supported Targets | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | -------- | -------- | -------- |

# TinyUSB Human Interface Device Example

(See the README.md file in the upper level 'examples' directory for more information about examples.)

Human interface devices (HID) are one of the most common USB devices, it is implemented in various devices such as keyboards, mice, game controllers, sensors and alphanumeric display devices.
In this example, we implement USB keyboard and mouse.
Upon connection to USB host (PC), the example application will sent 'key a/A pressed & released' events and move mouse in a square trajectory. To send these HID reports again, press the BOOT button, that is present on most ESP development boards (GPIO0).

As a USB stack, a TinyUSB component is used.

## How to use example

### Hardware Required

Any ESP board that have USB-OTG supported.


2025-09-30
1、使用ESP32S3开发板初步测试，当前版本，QNAP NAS可以正常识别为UPS, 且可以正常读取剩余电量和剩余运行时间, 后面继续优化;
