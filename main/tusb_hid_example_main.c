#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "tusb.h"
#include "class/hid/hid.h"
#include "tinyusb.h"
#include "device/usbd.h"

static const char *TAG = "UPS";

// 宏定义
#define LO8(x) ((x) & 0xFF)
#define HI8(x) ((x) >> 8)

// 报告ID定义
#define HID_PD_IPRODUCT              0x01 // FEATURE ONLY
#define HID_PD_SERIAL                0x02 // FEATURE ONLY
#define HID_PD_MANUFACTURER          0x03 // FEATURE ONLY
#define IDEVICECHEMISTRY             0x04
#define IOEMVENDOR                   0x05

#define HID_PD_RECHARGEABLE          0x06 // FEATURE ONLY
#define HID_PD_PRESENTSTATUS         0x07 // INPUT OR FEATURE(required by Windows)
#define HID_PD_REMAINTIMELIMIT       0x08
#define HID_PD_MANUFACTUREDATE       0x09
#define HID_PD_CONFIGVOLTAGE         0x0A // 10 FEATURE ONLY
#define HID_PD_VOLTAGE               0x0B // 11 INPUT (NA) OR FEATURE(implemented)
#define HID_PD_REMAININGCAPACITY     0x0C // 12 INPUT OR FEATURE(required by Windows)
#define HID_PD_RUNTIMETOEMPTY        0x0D 
#define HID_PD_FULLCHARGECAPACITY    0x0E // 14 FEATURE ONLY. Last Full Charge Capacity 
#define HID_PD_WARNCAPACITYLIMIT     0x0F
#define HID_PD_CPCTYGRANULARITY1     0x10
#define HID_PD_REMNCAPACITYLIMIT     0x11
#define HID_PD_DELAYBE4SHUTDOWN      0x12 // 18 FEATURE ONLY
#define HID_PD_DELAYBE4REBOOT        0x13
#define HID_PD_AUDIBLEALARMCTRL      0x14 // 20 INPUT OR FEATURE
#define HID_PD_CURRENT               0x15 // 21 FEATURE ONLY
#define HID_PD_CAPACITYMODE          0x16
#define HID_PD_DESIGNCAPACITY        0x17
#define HID_PD_CPCTYGRANULARITY2     0x18
#define HID_PD_AVERAGETIME2FULL      0x1A
#define HID_PD_AVERAGECURRENT        0x1B
#define HID_PD_AVERAGETIME2EMPTY     0x1C

#define HID_PD_IDEVICECHEMISTRY      0x1F // Feature
#define HID_PD_IOEMINFORMATION       0x20 // Feature



// 字符串索引定义
#define IMANUFACTURER               0x01
#define IPRODUCT                    0x02
#define ISERIAL                     0x03
#define IDEVICECHEMISTRY            0x04

// HID 协议类型定义
#define HID_PROTOCOL_NONE 0


// 电源状态结构体
struct PresentStatus {
  uint8_t Charging : 1;                   // bit 0x00
  uint8_t Discharging : 1;                // bit 0x01
  uint8_t ACPresent : 1;                  // bit 0x02
  uint8_t BatteryPresent : 1;             // bit 0x03
  uint8_t BelowRemainingCapacityLimit : 1;// bit 0x04
  uint8_t RemainingTimeLimitExpired : 1;  // bit 0x05
  uint8_t NeedReplacement : 1;            // bit 0x06
  uint8_t VoltageNotRegulated : 1;        // bit 0x07
  
  uint8_t FullyCharged : 1;               // bit 0x08
  uint8_t FullyDischarged : 1;            // bit 0x09
  uint8_t ShutdownRequested : 1;          // bit 0x0A
  uint8_t ShutdownImminent : 1;           // bit 0x0B
  uint8_t CommunicationLost : 1;          // bit 0x0C
  uint8_t Overload : 1;                   // bit 0x0D
  uint8_t unused1 : 1;
  uint8_t unused2 : 1;
};

// 将PresentStatus结构体转换为uint16_t
static inline uint16_t PresentStatus_to_uint16(const struct PresentStatus* ps) {
    return *(const uint16_t*)(ps);
}

// 全局状态变量
static struct PresentStatus UPS = {
    .Charging = 1,                      // 充电中
    .Discharging = 0,                    // 放电中
    .ACPresent = 1,                      // AC电源存在
    .BatteryPresent = 1,                 // 电池存在
    .BelowRemainingCapacityLimit = 0,    // 低于容量限制
    .RemainingTimeLimitExpired = 0,      // 时间限制到期
    .NeedReplacement = 0,                // 需要更换电池
    .VoltageNotRegulated = 0,            // 电压未调节
    .FullyCharged = 0,                   // 电池充满
    .FullyDischarged = 0,                // 电池放空
    .ShutdownRequested = 0,              // 关机请求
    .ShutdownImminent = 0,               // 即将关机
    .CommunicationLost = 0,              // 通信丢失
    .Overload = 0,                       // 过载
    .unused1 = 0,                        // 保留位
    .unused2 = 0                         // 保留位
};

uint16_t manufacture_date = 12345;      // 生产日期（自1990-01-01的天数）
uint16_t config_voltage = 12000;        // 配置电压, 指数5 = 10^-5伏  示例值：120.00V   
uint16_t voltage = 11850;               // 当前电压, 指数5 = 10^-5伏, 示例值：118.50V  
uint8_t remaining_capacity = 60;        // 剩余容量, 示例值：85.00%
uint16_t runtime_to_empty = 3600;       // 运行至空的时间, 示例值：60分钟
uint16_t full_charge_capacity = 10000;  // 充满电容量, 示例值：100.00%
uint8_t warring_capacity_limit = 20;    // 警告容量限制,示例值：20.00%
uint8_t remaining_capacity_limit = 10;  // 剩余容量限制,示例值：10.00%
int16_t delay_before_shutdown = 300;    // 关机前延迟（秒）,示例值：300秒
int16_t delay_before_reboot = 60;       // 重启前延迟（秒）, 示例值：60秒
uint16_t design_capacity = 100;         // 设计容量（单位：%）, 示例值：100.00%
uint16_t avg_time_to_full = 7200;       // 平均充满时间（秒）, 示例值：2小时
uint16_t avg_time_to_empty = 14400;     // 平均放空时间（秒）, 示例值：4小时

// A.6 Report Descriptorr  报告描述符 
const uint8_t hid_report_descriptor_github[] = {

    0x05, 0x84, // USAGE_PAGE (Power Device)
    0x09, 0x04, // USAGE (UPS)
    0xA1, 0x01, // COLLECTION (Application)

        0x09, 0x24, //   USAGE (Sink)
        0xA1, 0x02, //   COLLECTION (Logical)
        0x75, 0x08, //     REPORT_SIZE (8)
        0x95, 0x01, //     REPORT_COUNT (1)
        0x15, 0x00, //     LOGICAL_MINIMUM (0)
        0x26, 0xFF, 0x00, //     LOGICAL_MAXIMUM (255)
        0x85, HID_PD_IPRODUCT, //     REPORT_ID (1)
        0x09, 0xFE, //     USAGE (iProduct)
        0x79, IPRODUCT, //     STRING INDEX (2)
        0xB1, 0x23, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
        0x85, HID_PD_SERIAL, //     REPORT_ID (2)
        0x09, 0xFF, //     USAGE (iSerialNumber)
        0x79, ISERIAL, //  STRING INDEX (3)
        0xB1, 0x23, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
        0x85, HID_PD_MANUFACTURER, //     REPORT_ID (3)
        0x09, 0xFD, //     USAGE (iManufacturer)
        0x79, IMANUFACTURER, //     STRING INDEX (1)
        0xB1, 0x23, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
        0x05, 0x85, //     USAGE_PAGE (Battery System) ====================
        0x85, HID_PD_RECHARGEABLE, //     REPORT_ID (6)
        0x09, 0x8B, //     USAGE (Rechargable)                  
        0xB1, 0x23, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
        0x85, HID_PD_IDEVICECHEMISTRY, //     REPORT_ID (31)
        0x09, 0x89, //     USAGE (iDeviceChemistry)
        0x79, IDEVICECHEMISTRY, //     STRING INDEX (4)
        0xB1, 0x23, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
        0x85, HID_PD_IOEMINFORMATION,  //     REPORT_ID (32)
        0x09, 0x8F, //     USAGE (iOEMInformation)
        0x79, IOEMVENDOR, //     STRING INDEX (5)
        0xB1, 0x23, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
        0x85, HID_PD_CAPACITYMODE, //     REPORT_ID (22)
        0x09, 0x2C, //     USAGE (CapacityMode)
        0xB1, 0x23, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
        0x85, HID_PD_CPCTYGRANULARITY1, //     REPORT_ID (16)
        0x09, 0x8D, //     USAGE (CapacityGranularity1)
        0x26, 0x64,0x00, //     LOGICAL_MAXIMUM (100)    
        0xB1, 0x22, //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
        0x85, HID_PD_CPCTYGRANULARITY2, //     REPORT_ID (24)
        0x09, 0x8E, //     USAGE (CapacityGranularity2)
        0xB1, 0x23, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
        0x85, HID_PD_FULLCHARGECAPACITY, //     REPORT_ID (14)        
        0x09, 0x67, //     USAGE (FullChargeCapacity)
        0xB1, 0x83, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x85, HID_PD_DESIGNCAPACITY, //     REPORT_ID (23)
        0x09, 0x83, //     USAGE (DesignCapacity)
        0xB1, 0x83, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x85, HID_PD_REMAININGCAPACITY, //     REPORT_ID (12)
        0x09, 0x66, //     USAGE (RemainingCapacity)
        0x81, 0xA3, //     INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0x66, //     USAGE (RemainingCapacity)
        0xB1, 0xA3, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x85, HID_PD_WARNCAPACITYLIMIT, //     REPORT_ID (15)
        0x09, 0x8C, //     USAGE (WarningCapacityLimit)
        0xB1, 0xA2, //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x85, HID_PD_REMNCAPACITYLIMIT, //     REPORT_ID (17)
        0x09, 0x29, //     USAGE (RemainingCapacityLimit)
        0xB1, 0xA2, //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x85, HID_PD_MANUFACTUREDATE, //     REPORT_ID (9)
        0x09, 0x85, //     USAGE (ManufacturerDate)
        0x75, 0x10, //     REPORT_SIZE (16)
        0x27, 0xFF, 0xFF, 0x00, 0x00, //     LOGICAL_MAXIMUM (65534)
        0xB1, 0xA3, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x85, HID_PD_AVERAGETIME2FULL, //     REPORT_ID (26)
        0x09, 0x6A, //     USAGE (AverageTimeToFull)
        0x27, 0xFF, 0xFF, 0x00, 0x00, //     LOGICAL_MAXIMUM (65534)
        0x66, 0x01, 0x10, //     UNIT (Seconds)
        0x55, 0x00, //     UNIT_EXPONENT (0)
        0xB1, 0xA3, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield) 
        0x85, HID_PD_AVERAGETIME2EMPTY, //     REPORT_ID (28)
        0x09, 0x69, //     USAGE (AverageTimeToEmpty)  
        0x81, 0xA3, //     INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0x69, //     USAGE (AverageTimeToEmpty)
        0xB1, 0xA3, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x85, HID_PD_RUNTIMETOEMPTY, //     REPORT_ID (13)    
        0x09, 0x68, //     USAGE (RunTimeToEmpty)  
        0x81, 0xA3, //     INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0x68, //     USAGE (RunTimeToEmpty)
        0xB1, 0xA3, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)      
        0x85, HID_PD_REMAINTIMELIMIT, //     REPORT_ID (8)
        0x09, 0x2A, //     USAGE (RemainingTimeLimit)
        0x75, 0x10, //     REPORT_SIZE (16)
        0x27, 0x64, 0x05, 0x00, 0x00, //     LOGICAL_MAXIMUM (1380)
        0x16, 0x78, 0x00, //     LOGICAL_MINIMUM (120)
        0x81, 0x22, //     INPUT (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0x2A, //     USAGE (RemainingTimeLimit)
        0xB1, 0xA2, //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x05, 0x84, //     USAGE_PAGE (Power Device) ====================
        0x85, HID_PD_DELAYBE4SHUTDOWN, //     REPORT_ID (18)
        0x09, 0x57, //     USAGE (DelayBeforeShutdown)
        0x16, 0x00, 0x80, //     LOGICAL_MINIMUM (-32768)
        0x27, 0xFF, 0x7F, 0x00, 0x00, //     LOGICAL_MAXIMUM (32767)
        0xB1, 0xA2, //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x85, HID_PD_DELAYBE4REBOOT, //     REPORT_ID (19)
        0x09, 0x55, //     USAGE (DelayBeforeReboot)
        0xB1, 0xA2, //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x85, HID_PD_CONFIGVOLTAGE, //     REPORT_ID (10)
        0x09, 0x40, //     USAGE (ConfigVoltage)
        0x15, 0x00, //     LOGICAL_MINIMUM (0)
        0x27, 0xFF, 0xFF, 0x00, 0x00, //     LOGICAL_MAXIMUM (65535)
        0x67, 0x21, 0xD1, 0xF0, 0x00, //     UNIT (Centivolts)
        0x55, 0x05, //     UNIT_EXPONENT (5)
        0xB1, 0x23, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
        0x85, HID_PD_VOLTAGE, //     REPORT_ID (11)
        0x09, 0x30, //     USAGE (Voltage)
        0x81, 0xA3, //     INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0x30, //     USAGE (Voltage)
        0xB1, 0xA3, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x85, HID_PD_AUDIBLEALARMCTRL, //     REPORT_ID (20)
        0x09, 0x5A, //     USAGE (AudibleAlarmControl)
        0x75, 0x08, //     REPORT_SIZE (8)
        0x15, 0x01, //     LOGICAL_MINIMUM (1)
        0x25, 0x03, //     LOGICAL_MAXIMUM (3)
        0x65, 0x00, //     UNIT (0)
        0x55, 0x00, //     UNIT_EXPONENT (0)
        0x81, 0x22, //     INPUT (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0x5A, //     USAGE (AudibleAlarmControl)
        0xB1, 0xA2, //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x09, 0x02, //     USAGE (PresentStatus)
        0xA1, 0x02, //     COLLECTION (Logical)
        0x85, HID_PD_PRESENTSTATUS, //       REPORT_ID (7)
        0x05, 0x85, //       USAGE_PAGE (Battery System) =================
        0x09, 0x44, //       USAGE (Charging)
        0x75, 0x01, //       REPORT_SIZE (1)
        0x15, 0x00, //       LOGICAL_MINIMUM (0)
        0x25, 0x01, //       LOGICAL_MAXIMUM (1)
        0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0x44, //       USAGE (Charging)
        0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x09, 0x45, //       USAGE (Discharging)
        0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0x45, //       USAGE (Discharging)
        0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x09, 0xD0, //       USAGE (ACPresent)
        0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0xD0, //       USAGE (ACPresent)
        0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x09, 0xD1, //       USAGE (BatteryPresent)
        0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0xD1, //       USAGE (BatteryPresent)
        0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x09, 0x42, //       USAGE (BelowRemainingCapacityLimit)
        0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0x42, //       USAGE (BelowRemainingCapacityLimit)
        0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x09, 0x43, //       USAGE (RemainingTimeLimitExpired)
        0x81, 0xA2, //       INPUT (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0x43, //       USAGE (RemainingTimeLimitExpired)
        0xB1, 0xA2, //       FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)  
        0x09, 0x4B, //       USAGE (NeedReplacement)
        0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0x4B, //       USAGE (NeedReplacement)
        0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)    
        0x09, 0xDB, //       USAGE (VoltageNotRegulated)
        0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0xDB, //       USAGE (VoltageNotRegulated)
        0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x09, 0x46, //       USAGE (FullyCharged)
        0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0x46, //       USAGE (FullyCharged)
        0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x09, 0x47, //       USAGE (FullyDischarged)
        0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0x47, //       USAGE (FullyDischarged)
        0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)    
        0x05, 0x84, //       USAGE_PAGE (Power Device) =================
        0x09, 0x68, //       USAGE (ShutdownRequested)
        0x81, 0xA2, //       INPUT (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0x68, //       USAGE (ShutdownRequested)
        0xB1, 0xA2, //       FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x09, 0x69, //       USAGE (ShutdownImminent)
        0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0x69, //       USAGE (ShutdownImminent)
        0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x09, 0x73, //       USAGE (CommunicationLost)
        0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0x73, //       USAGE (CommunicationLost)
        0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x09, 0x65, //       USAGE (Overload)
        0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
        0x09, 0x65, //       USAGE (Overload)
        0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
        0x95, 0x02, //       REPORT_COUNT (2) // padding bits to make the report byte aligned
        0x81, 0x01, //       INPUT (Constant, Array, Absolute)
        0xB1, 0x01, //       FEATURE (Constant, Array, Absolute, No Wrap, Linear, Preferred State, No Null Position, Nonvolatile, Bitfield)
        0xC0,       //     END_COLLECTION
        0xC0,       //   END_COLLECTION
    0xC0        // END_COLLECTION
};

// A.6 Report Descriptor  报告描述符
const uint8_t hid_report_descriptor[] = {
    // ==================== 根集合：定义整个设备 ====================
    0x05, 0x84, // USAGE_PAGE (Power Device)  // 主用途页面：电源设备
    0x09, 0x04, // USAGE (UPS)                // 具体用途：不间断电源 (UPS)
    0xA1, 0x01, // COLLECTION (Application)   // 开始一个应用集合（根集合）

    // ==================== 逻辑集合：信息类特性 ====================
    // 这个集合主要包含设备标识和静态信息，使用字符串索引
    0x09, 0x24, //   USAGE (Sink)             // 用途：接收器（表示此设备消耗电源）
    0xA1, 0x02, //   COLLECTION (Logical)     // 开始一个逻辑集合
        0x75, 0x08, //     REPORT_SIZE (8)    // 每个字段大小为8位（1字节）
        0x95, 0x01, //     REPORT_COUNT (1)   // 字段数量为1
        0x15, 0x00, //     LOGICAL_MINIMUM (0) // 逻辑最小值：0
        0x26, 0xFF, 0x00, // LOGICAL_MAXIMUM (255) // 逻辑最大值：255

        // --- 产品字符串 (Report ID 1) ---
        0x85, HID_PD_IPRODUCT, //     REPORT_ID (1) // 报告ID：1
        0x09, 0xFE, //     USAGE (iProduct)        // 用途：产品字符串
        0x79, IPRODUCT, //     STRING INDEX (2)    // 字符串描述符索引：2
        0xB1, 0x23, //     FEATURE (Const, Var, Abs, NonVol) // 特性报告（常量，非易失）

        // --- 序列号字符串 (Report ID 2) ---
        0x85, HID_PD_SERIAL, //     REPORT_ID (2) // 报告ID：2
        0x09, 0xFF, //     USAGE (iSerialNumber)   // 用途：序列号字符串
        0x79, ISERIAL, //  STRING INDEX (3)        // 字符串描述符索引：3
        0xB1, 0x23, //     FEATURE (Const, Var, Abs, NonVol) // 特性报告（常量，非易失）

        // --- 制造商字符串 (Report ID 3) ---
        0x85, HID_PD_MANUFACTURER, // REPORT_ID (3) // 报告ID：3
        0x09, 0xFD, //     USAGE (iManufacturer)   // 用途：制造商字符串
        0x79, IMANUFACTURER, // STRING INDEX (1)   // 字符串描述符索引：1
        0xB1, 0x23, //     FEATURE (Const, Var, Abs, NonVol) // 特性报告（常量，非易失）

    // ==================== 电池系统特性 ====================
    // 切换用途页面到电池系统，定义电池相关特性
        0x05, 0x85, //     USAGE_PAGE (Battery System) // 用途页面：电池系统

        // --- 是否可充电 (Report ID 6) ---
        0x85, HID_PD_RECHARGEABLE, // REPORT_ID (6) // 报告ID：6
        0x09, 0x8B, //     USAGE (Rechargable)      // 用途：可充电
        0xB1, 0x23, //     FEATURE (Const, Var, Abs, NonVol) // 特性报告（常量，非易失）

        // --- 电池化学类型字符串 (Report ID 31) ---
        0x85, HID_PD_IDEVICECHEMISTRY, // REPORT_ID (31) // 报告ID：31
        0x09, 0x89, //     USAGE (iDeviceChemistry) // 用途：电池化学类型字符串
        0x79, IDEVICECHEMISTRY, // STRING INDEX (4) // 字符串描述符索引：4
        0xB1, 0x23, //     FEATURE (Const, Var, Abs, NonVol) // 特性报告（常量，非易失）

        // --- OEM信息字符串 (Report ID 32) ---
        0x85, HID_PD_IOEMINFORMATION,  // REPORT_ID (32) // 报告ID：32
        0x09, 0x8F, //     USAGE (iOEMInformation)  // 用途：OEM信息字符串
        0x79, IOEMVENDOR, // STRING INDEX (5)       // 字符串描述符索引：5
        0xB1, 0x23, //     FEATURE (Const, Var, Abs, NonVol) // 特性报告（常量，非易失）

        // --- 容量模式 (Report ID 22) ---
        0x85, HID_PD_CAPACITYMODE, // REPORT_ID (22) // 报告ID：22
        0x09, 0x2C, //     USAGE (CapacityMode)     // 用途：容量模式（如mAh/mWh）
        0xB1, 0x23, //     FEATURE (Const, Var, Abs, NonVol) // 特性报告（常量，非易失）

        // --- 容量粒度1 (Report ID 16) ---
        0x85, HID_PD_CPCTYGRANULARITY1, // REPORT_ID (16) // 报告ID：16
        0x09, 0x8D, //     USAGE (CapacityGranularity1) // 用途：容量粒度（步进值）
        0x26, 0x64,0x00, // LOGICAL_MAXIMUM (100)    // 逻辑最大值：100
        0xB1, 0x22, //     FEATURE (Data, Var, Abs, NonVol) // 特性报告（数据，非易失）

        // --- 容量粒度2 (Report ID 24) ---
        0x85, HID_PD_CPCTYGRANULARITY2, // REPORT_ID (24) // 报告ID：24
        0x09, 0x8E, //     USAGE (CapacityGranularity2) // 用途：容量粒度2
        0xB1, 0x23, //     FEATURE (Const, Var, Abs, NonVol) // 特性报告（常量，非易失）

        // --- 充满电容量 (Report ID 14) ---
        0x85, HID_PD_FULLCHARGECAPACITY, // REPORT_ID (14) // 报告ID：14
        0x09, 0x67, //     USAGE (FullChargeCapacity) // 用途：充满电容量
        0xB1, 0x83, //     FEATURE (Const, Var, Abs, Vol) // 特性报告（常量，易失-可能变化）

        // --- 设计容量 (Report ID 23) ---
        0x85, HID_PD_DESIGNCAPACITY, // REPORT_ID (23) // 报告ID：23
        0x09, 0x83, //     USAGE (DesignCapacity)    // 用途：设计容量（标称值）
        0xB1, 0x83, //     FEATURE (Const, Var, Abs, Vol) // 特性报告（常量，易失）

    // ==================== 电池容量与状态数据 ====================
        // --- 剩余容量 (Report ID 12) ---
        0x85, HID_PD_REMAININGCAPACITY, // REPORT_ID (12) // 报告ID：12
        0x09, 0x66, //     USAGE (RemainingCapacity) // 用途：剩余容量
        0x81, 0xA3, //     INPUT (Const, Var, Abs)   // 输入报告（常量）
        0x09, 0x66, //     USAGE (RemainingCapacity)
        0xB1, 0xA3, //     FEATURE (Const, Var, Abs, Vol) // 特性报告（常量，易失）

        // --- 警告容量限制 (Report ID 15) ---
        0x85, HID_PD_WARNCAPACITYLIMIT, // REPORT_ID (15) // 报告ID：15
        0x09, 0x8C, //     USAGE (WarningCapacityLimit) // 用途：低电量警告阈值
        0xB1, 0xA2, //     FEATURE (Data, Var, Abs, Vol) // 特性报告（数据，易失-可设置）

        // --- 剩余容量限制 (Report ID 17) ---
        0x85, HID_PD_REMNCAPACITYLIMIT, // REPORT_ID (17) // 报告ID：17
        0x09, 0x29, //     USAGE (RemainingCapacityLimit) // 用途：剩余容量限制阈值
        0xB1, 0xA2, //     FEATURE (Data, Var, Abs, Vol) // 特性报告（数据，易失-可设置）

    // ==================== 时间相关数据 ====================
        // --- 生产日期 (Report ID 9) ---
        0x85, HID_PD_MANUFACTUREDATE, // REPORT_ID (9) // 报告ID：9
        0x09, 0x85, //     USAGE (ManufacturerDate)  // 用途：生产日期
        0x75, 0x10, //     REPORT_SIZE (16)         // 字段大小改为16位
        0x27, 0xFF, 0xFF, 0x00, 0x00, // LOGICAL_MAXIMUM (65534) // 逻辑最大值
        0xB1, 0xA3, //     FEATURE (Const, Var, Abs, Vol) // 特性报告（常量，易失）

        // --- 平均充满时间 (Report ID 26) ---
        0x85, HID_PD_AVERAGETIME2FULL, // REPORT_ID (26) // 报告ID：26
        0x09, 0x6A, //     USAGE (AverageTimeToFull) // 用途：预估充满所需时间
        0x27, 0xFF, 0xFF, 0x00, 0x00, // LOGICAL_MAXIMUM (65534) // 逻辑最大值
        0x66, 0x01, 0x10, //     UNIT (Seconds)      // 单位：秒
        0x55, 0x00, //     UNIT_EXPONENT (0)         // 单位指数：0 (10^0 = 1)
        0xB1, 0xA3, //     FEATURE (Const, Var, Abs, Vol) // 特性报告（常量，易失）

        // --- 平均耗尽时间 (Report ID 28) ---
        0x85, HID_PD_AVERAGETIME2EMPTY, // REPORT_ID (28) // 报告ID：28
        0x09, 0x69, //     USAGE (AverageTimeToEmpty)  // 用途：预估耗尽所需时间
        0x81, 0xA3, //     INPUT (Const, Var, Abs)    // 输入报告（常量）
        0x09, 0x69, //     USAGE (AverageTimeToEmpty)
        0xB1, 0xA3, //     FEATURE (Const, Var, Abs, Vol) // 特性报告（常量，易失）

        // --- 运行至耗尽时间 (Report ID 13) ---
        0x85, HID_PD_RUNTIMETOEMPTY, // REPORT_ID (13) // 报告ID：13
        0x09, 0x68, //     USAGE (RunTimeToEmpty)     // 用途：运行至耗尽时间
        0x81, 0xA3, //     INPUT (Const, Var, Abs)    // 输入报告（常量）
        0x09, 0x68, //     USAGE (RunTimeToEmpty)
        0xB1, 0xA3, //     FEATURE (Const, Var, Abs, Vol) // 特性报告（常量，易失）

        // --- 剩余时间限制 (Report ID 8) ---
        0x85, HID_PD_REMAINTIMELIMIT, // REPORT_ID (8) // 报告ID：8
        0x09, 0x2A, //     USAGE (RemainingTimeLimit) // 用途：剩余时间限制（可设置）
        0x75, 0x10, //     REPORT_SIZE (16)          // 字段大小：16位
        0x27, 0x64, 0x05, 0x00, 0x00, // LOGICAL_MAXIMUM (1380) // 逻辑最大值：1380分钟（23小时）
        0x16, 0x78, 0x00, //     LOGICAL_MINIMUM (120) // 逻辑最小值：120分钟（2小时）
        0x81, 0x22, //     INPUT (Data, Var, Abs)     // 输入报告（数据，可设置）
        0x09, 0x2A, //     USAGE (RemainingTimeLimit)
        0xB1, 0xA2, //     FEATURE (Data, Var, Abs, Vol) // 特性报告（数据，易失-可设置）

    // ==================== 电源设备控制与状态 ====================
    // 切换回电源设备用途页面，定义UPS特定控制
        0x05, 0x84, //     USAGE_PAGE (Power Device) // 用途页面：电源设备

        // --- 关机前延迟 (Report ID 18) ---
        0x85, HID_PD_DELAYBE4SHUTDOWN, // REPORT_ID (18) // 报告ID：18
        0x09, 0x57, //     USAGE (DelayBeforeShutdown) // 用途：关机前延迟时间
        0x16, 0x00, 0x80, // LOGICAL_MINIMUM (-32768) // 逻辑最小值：-32768
        0x27, 0xFF, 0x7F, 0x00, 0x00, // LOGICAL_MAXIMUM (32767) // 逻辑最大值：32767
        0xB1, 0xA2, //     FEATURE (Data, Var, Abs, Vol) // 特性报告（数据，易失-可设置）

        // --- 重启前延迟 (Report ID 19) ---
        0x85, HID_PD_DELAYBE4REBOOT, // REPORT_ID (19) // 报告ID：19
        0x09, 0x55, //     USAGE (DelayBeforeReboot)  // 用途：重启前延迟时间
        0xB1, 0xA2, //     FEATURE (Data, Var, Abs, Vol) // 特性报告（数据，易失-可设置）

        // --- 配置电压 (Report ID 10) ---
        0x85, HID_PD_CONFIGVOLTAGE, // REPORT_ID (10) // 报告ID：10
        0x09, 0x40, //     USAGE (ConfigVoltage)      // 用途：配置/额定电压
        0x15, 0x00, //     LOGICAL_MINIMUM (0)       // 逻辑最小值：0
        0x27, 0xFF, 0xFF, 0x00, 0x00, // LOGICAL_MAXIMUM (65535) // 逻辑最大值：65535
        0x67, 0x21, 0xD1, 0xF0, 0x00, // UNIT (Centivolts) // 单位：百分之一伏特 (cV)
        0x55, 0x05, //     UNIT_EXPONENT (5)         // 单位指数：5 (10^5, 结合单位cV，实际为V * 1000)
        0xB1, 0x23, //     FEATURE (Const, Var, Abs, NonVol) // 特性报告（常量，非易失）

        // --- 当前电压 (Report ID 11) ---
        0x85, HID_PD_VOLTAGE, // REPORT_ID (11) // 报告ID：11
        0x09, 0x30, //     USAGE (Voltage)           // 用途：当前电压
        0x81, 0xA3, //     INPUT (Const, Var, Abs)   // 输入报告（常量）
        0x09, 0x30, //     USAGE (Voltage)
        0xB1, 0xA3, //     FEATURE (Const, Var, Abs, Vol) // 特性报告（常量，易失）

        // --- 声音警报控制 (Report ID 20) ---
        0x85, HID_PD_AUDIBLEALARMCTRL, // REPORT_ID (20) // 报告ID：20
        0x09, 0x5A, //     USAGE (AudibleAlarmControl) // 用途：声音警报控制
        0x75, 0x08, //     REPORT_SIZE (8)           // 字段大小改回8位
        0x15, 0x01, //     LOGICAL_MINIMUM (1)       // 逻辑最小值：1
        0x25, 0x03, //     LOGICAL_MAXIMUM (3)       // 逻辑最大值：3（可能代表关/开/静音等状态）
        0x65, 0x00, //     UNIT (0)                  // 单位：无
        0x55, 0x00, //     UNIT_EXPONENT (0)         // 单位指数：0
        0x81, 0x22, //     INPUT (Data, Var, Abs)    // 输入报告（数据，可设置）
        0x09, 0x5A, //     USAGE (AudibleAlarmControl)
        0xB1, 0xA2, //     FEATURE (Data, Var, Abs, Vol) // 特性报告（数据，易失-可设置）

    // ==================== 状态位集合 (重要！) ====================
    // 这是一个位图集合，每个位代表一个不同的状态标志
        0x09, 0x02, //     USAGE (PresentStatus)     // 用途：当前状态
        0xA1, 0x02, //     COLLECTION (Logical)     // 开始一个逻辑集合（状态位集合）
        0x85, HID_PD_PRESENTSTATUS, // REPORT_ID (7) // 报告ID：7
        0x05, 0x85, //       USAGE_PAGE (Battery System) // 用途页面：电池系统（用于电池状态位）
        0x75, 0x01, //       REPORT_SIZE (1)        // 每个字段大小为1位（标志位）
        0x15, 0x00, //       LOGICAL_MINIMUM (0)    // 逻辑最小值：0 (False)
        0x25, 0x01, //       LOGICAL_MAXIMUM (1)    // 逻辑最大值：1 (True)

        // 以下每个USAGE定义一个状态标志位，INPUT和FEATURE报告各有一个
        0x09, 0x44, //       USAGE (Charging)        // 正在充电
        0x81, 0xA3, //       INPUT (Const, Var, Abs) // 输入报告（常量）
        0x09, 0x44, //       USAGE (Charging)
        0xB1, 0xA3, //       FEATURE (Const, Var, Abs, Vol) // 特性报告（常量，易失）

        0x09, 0x45, //       USAGE (Discharging)     // 正在放电
        0x81, 0xA3, //       INPUT (Const, Var, Abs)
        0x09, 0x45, //       USAGE (Discharging)
        0xB1, 0xA3, //       FEATURE (Const, Var, Abs, Vol)

        0x09, 0xD0, //       USAGE (ACPresent)       // AC电源 present/连接
        0x81, 0xA3, //       INPUT (Const, Var, Abs)
        0x09, 0xD0, //       USAGE (ACPresent)
        0xB1, 0xA3, //       FEATURE (Const, Var, Abs, Vol)

        0x09, 0xD1, //       USAGE (BatteryPresent)  // 电池 present/安装
        0x81, 0xA3, //       INPUT (Const, Var, Abs)
        0x09, 0xD1, //       USAGE (BatteryPresent)
        0xB1, 0xA3, //       FEATURE (Const, Var, Abs, Vol)

        0x09, 0x42, //       USAGE (BelowRemainingCapacityLimit) // 低于剩余容量限制
        0x81, 0xA3, //       INPUT (Const, Var, Abs)
        0x09, 0x42, //       USAGE (BelowRemainingCapacityLimit)
        0xB1, 0xA3, //       FEATURE (Const, Var, Abs, Vol)

        0x09, 0x43, //       USAGE (RemainingTimeLimitExpired) // 剩余时间限制已过期
        0x81, 0xA2, //       INPUT (Data, Var, Abs)  // 注意：这个是Data，可能主机可写？
        0x09, 0x43, //       USAGE (RemainingTimeLimitExpired)
        0xB1, 0xA2, //       FEATURE (Data, Var, Abs, Vol)

        0x09, 0x4B, //       USAGE (NeedReplacement) // 需要更换电池
        0x81, 0xA3, //       INPUT (Const, Var, Abs)
        0x09, 0x4B, //       USAGE (NeedReplacement)
        0xB1, 0xA3, //       FEATURE (Const, Var, Abs, Vol)

        0x09, 0xDB, //       USAGE (VoltageNotRegulated) // 电压未调节（异常）
        0x81, 0xA3, //       INPUT (Const, Var, Abs)
        0x09, 0xDB, //       USAGE (VoltageNotRegulated)
        0xB1, 0xA3, //       FEATURE (Const, Var, Abs, Vol)

        0x09, 0x46, //       USAGE (FullyCharged)    // 已充满
        0x81, 0xA3, //       INPUT (Const, Var, Abs)
        0x09, 0x46, //       USAGE (FullyCharged)
        0xB1, 0xA3, //       FEATURE (Const, Var, Abs, Vol)

        0x09, 0x47, //       USAGE (FullyDischarged) // 已放空
        0x81, 0xA3, //       INPUT (Const, Var, Abs)
        0x09, 0x47, //       USAGE (FullyDischarged)
        0xB1, 0xA3, //       FEATURE (Const, Var, Abs, Vol)

        // 切换回电源设备用途页面（用于UPS特定状态位）
        0x05, 0x84, //       USAGE_PAGE (Power Device)

        0x09, 0x68, //       USAGE (ShutdownRequested) // 已请求关机
        0x81, 0xA2, //       INPUT (Data, Var, Abs)    // 数据（可能由主机触发）
        0x09, 0x68, //       USAGE (ShutdownRequested)
        0xB1, 0xA2, //       FEATURE (Data, Var, Abs, Vol)

        0x09, 0x69, //       USAGE (ShutdownImminent) // 关机即将发生（最终警告）
        0x81, 0xA3, //       INPUT (Const, Var, Abs)
        0x09, 0x69, //       USAGE (ShutdownImminent)
        0xB1, 0xA3, //       FEATURE (Const, Var, Abs, Vol)

        0x09, 0x73, //       USAGE (CommunicationLost) // 与UPS通信丢失
        0x81, 0xA3, //       INPUT (Const, Var, Abs)
        0x09, 0x73, //       USAGE (CommunicationLost)
        0xB1, 0xA3, //       FEATURE (Const, Var, Abs, Vol)

        0x09, 0x65, //       USAGE (Overload)         // 过载
        0x81, 0xA3, //       INPUT (Const, Var, Abs)
        0x09, 0x65, //       USAGE (Overload)
        0xB1, 0xA3, //       FEATURE (Const, Var, Abs, Vol)

        // --- 填充位和对齐 ---
        // 前面的状态位可能不是8的整数倍，这里添加填充位使整个报告按字节对齐
        0x95, 0x02, //       REPORT_COUNT (2)        // 2个填充位
        0x81, 0x01, //       INPUT (Constant, Array) // 输入报告（常量，数组-填充用）
        0xB1, 0x01, //       FEATURE (Constant, Array, NonVol) // 特性报告（常量，数组，非易失-填充用）

        0xC0,       //     END_COLLECTION // 结束 状态位逻辑集合
    // ==================== 结束集合 ====================
        0xC0,       //   END_COLLECTION // 结束 信息类逻辑集合
    0xC0        // END_COLLECTION // 结束 根应用集合
};

// A.1 Device Descriptor  USB设备描述符 
tusb_desc_device_t descriptor_dev = {
    .bLength = sizeof(tusb_desc_device_t),  //Numeric expression specifying the size of this descriptor.
    .bDescriptorType = TUSB_DESC_DEVICE,   //Device descriptor type (assigned by USB).
    .bcdUSB = 0x0200,       //USB HID Specification Release 1.0
    .bDeviceClass = 0x00,   //Class code (assigned by USB). Note that the HID class is defined in the Interface descriptor.
    .bDeviceSubClass = 0x00,  //Subclass code (assigned by USB). These codes are qualified by the value of the bDeviceClass field.
    .bDeviceProtocol = 0x00,  //Protocol code. These codes are qualified by the value of the bDeviceSubclass field
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,  //Maximum packet size for endpoint zero (only 8, 16, 32, or 64 are valid).
    .idVendor = 0x04d8,    //Vendor ID (assigned by USB).
    .idProduct = 0xd005,   //Product ID (assigned by manufacturer).
    .bcdDevice = 0x0100,   //Device release number (assigned by manufacturer
    .iManufacturer = IMANUFACTURER,  //Index of String descriptor describing manufacturer.
    .iProduct = IPRODUCT,       //Index of String descriptor describing product.
    .iSerialNumber = ISERIAL,   //Index of String descriptor describing the device’s serial number.
    .bNumConfigurations = 0x01  //Number of possible configurations
};

// A.3 + A.4 + A.5 HID描述符
const uint8_t hid_interface_desc[] = {
    
    // A.3 Interface Descriptor 
    // 核心作用：告诉主机“这里有一个接口，它是HID类的”。
    0x09,                    // bLength, Size of this descriptor in bytes.
    0x04,                    // bDescriptorType, Interface descriptor type (assigned by USB).
    0x00,                    // bInterfaceNumber
    0x00,                    // bAlternateSetting
    0x01,                    // bNumEndpoints, Number of endpoints used by this interface
    0x03,                    // bInterfaceClass (HID), Class code (HID code assigned by USB).
    0x00,                    // bInterfaceSubClass, 0 No subclass; 1 Boot Interface subclass
    0x00,                    // bInterfaceProtocol, 0 None
    0x00,                    // iInterface, Index of string descriptor describing this interface
    
    // A.5 HID Descriptor
    0x09,                    // bLength, Size of this descriptor in bytes
    0x21,                    // bDescriptorType (HID), HID descriptor type (assigned by USB).
    0x11, 0x01,              // bcdHID (1.11), HID Class Specification release number in binarycoded decimal.
    0x00,                    // bCountryCode, Hardware target country.
    0x01,                    // bNumDescriptors, Number of HID class descriptors to follow
    0x22,                    // bDescriptorType (Report),Report descriptor type.
    LO8(sizeof(hid_report_descriptor)),  // wDescriptorLength (low), Total length of Report descriptor.
    HI8(sizeof(hid_report_descriptor)),  // wDescriptorLength (high)
    
    // A.4 Endpoint Descriptor
    0x07,                    // bLength, Size of this descriptor in bytes.
    0x05,                    // bDescriptorType, Endpoint descriptor type (assigned by USB).
    0x81,                    // bEndpointAddress (IN endpoint 1)
    // The address of the endpoint on the USB device described by this descriptor
    // Bit 0..3 The endpoint number
    // Bit 4..6 Reserved, reset to zero
    // Bit 7    Direction, ignored for Control endpoints:
    // 0        OUT endpoint
    // 1        IN endpoint
    0x03,                    // bmAttributes (Interrupt)
    // This field describes the endpoint’s attributes when it is configured using the bConfigurationValue
    // Bit 0..1 Transfer type:
    // 00 Control
    // 01 Isochronous
    // 10 Bulk
    // 11 Interrupt
    LO8(CFG_TUD_HID_EP_BUFSIZE),  // wMaxPacketSize (low)
    HI8(CFG_TUD_HID_EP_BUFSIZE),  // wMaxPacketSize (high)
    0x0A                     // bInterval (10ms) , Interval for polling endpoint for data transfers, expressed in milliseconds.
};

// A.2 Configuration Descriptor  配置描述符
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + sizeof(hid_interface_desc))
uint8_t const desc_configuration[] = {

    // 配置描述符，具体配置顺序由宏自行调配
    TUD_CONFIG_DESCRIPTOR(
        // bLength          由宏自动生成
        // bDescriptorType  由宏自动生成
        1,      // bConfigurationValue , Value to use as an argument to Set Configuration to select this configuration.
        1,      // bNumInterfaces ,Number of interfaces supported by this configuration.
        0,      // iConfiguration , Index of string descriptor describing this configuration.
        TUSB_DESC_TOTAL_LEN, // wTotalLength 
        // Total length of data returned for this configuration.
        // Includes the combined length of all returned
        // descriptors (configuration, interface, endpoint, and
        // HID) returned for this configuration. This value
        // includes the HID descriptor but none of the other
        // HID class descriptors (report or designator).
        TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, // bmAttributes ,Bus Powered + Self Powered
        // 7 Bus Powered
        // 6 Self Powered
        // 5 Remote Wakeup
        // 4..0 Reserved (reset to 0)
        100     // bMaxPower 
    ),
    
    // HID接口描述符
    hid_interface_desc[0], hid_interface_desc[1], hid_interface_desc[2],
    hid_interface_desc[3], hid_interface_desc[4], hid_interface_desc[5],
    hid_interface_desc[6], hid_interface_desc[7], hid_interface_desc[8],
    hid_interface_desc[9], hid_interface_desc[10], hid_interface_desc[11],
    hid_interface_desc[12], hid_interface_desc[13], hid_interface_desc[14],
    hid_interface_desc[15], hid_interface_desc[16], hid_interface_desc[17],
    hid_interface_desc[18], hid_interface_desc[19], hid_interface_desc[20],
    hid_interface_desc[21], hid_interface_desc[22], hid_interface_desc[23],
    hid_interface_desc[24]
};

// 字符串描述符
const char* descriptor_str[] = {
    [0] = "en",
    [IMANUFACTURER] = "DINGHUGANG",
    [IPRODUCT] = "DHG.UPS",
    [ISERIAL] = "383503417",
    [IDEVICECHEMISTRY] = "Li-ion",
    [IOEMVENDOR] = "DHG"
};

// TinyUSB回调函数
uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
    // ESP_LOGI(TAG, "return hid_report_descriptor;");
    return hid_report_descriptor;
}


// Report ID	功能描述	                            请求长度	频率	        重要性	            备注
// 0x0B	    电压测量 (Voltage)	                    2字节	    每2秒	    ⭐⭐⭐⭐⭐	    监控供电状态
// 0x0C	    剩余容量 (RemainingCapacity)	        1字节	    每2秒	    ⭐⭐⭐⭐⭐	    电池百分比
// 0x0D	    运行至空时间 (RunTimeToEmpty)	        2字节	    每2秒	    ⭐⭐⭐⭐⭐	     剩余时间预测
// 0x07	    当前状态 (PresentStatus)	            2字节	    每2秒	    ⭐⭐⭐⭐⭐	    充电/放电状态
// 0x11	    剩余容量限制 (RemainingCapacityLimit)	1字节	    每2秒	    ⭐⭐⭐⭐	      低电量阈值
// 0x0F	    警告容量限制 (WarningCapacityLimit)	    1字节	    每30秒	    ⭐⭐⭐	           警告阈值
// 0x20	    OEM信息 (iOEMInformation)	            1字节	    每30秒	    ⭐⭐	            设备标识
// 0x1F	    设备化学类型 (iDeviceChemistry)	        1字节	    仅初始化	  ⭐	               电池类型
// 0x17	    设计容量 (DesignCapacity)	            1字节	    仅初始化	  ⭐	               标称容量



uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) 
{
    // ESP_LOGI(TAG, "Get report: ID=0x%02X, Type=%d, ReqLen=%d", report_id, report_type, reqlen);

    // 只处理Feature Report（类型3）
    if (report_type != HID_REPORT_TYPE_FEATURE) {
        ESP_LOGW(TAG, "Unsupported report type: %d", report_type);
        return 0;
    }

    // 根据报告ID返回相应的数据
    switch (report_id) {
        case HID_PD_IPRODUCT: // REPORT_ID (1)
            if (reqlen >= 1) {
                buffer[0] = IPRODUCT; // 字符串索引
                return 1;
            }
            break;

        case HID_PD_SERIAL: // REPORT_ID (2)
            if (reqlen >= 1) {
                buffer[0] = ISERIAL; // 字符串索引
                return 1;
            }
            break;

        case HID_PD_MANUFACTURER: // REPORT_ID (3)
            if (reqlen >= 1) {
                buffer[0] = IMANUFACTURER; // 字符串索引
                return 1;
            }
            break;

        case HID_PD_RECHARGEABLE: // REPORT_ID (6)
            if (reqlen >= 1) {
                buffer[0] = 0x01; // 可充电：是
                return 1;
            }
            break;
        
        // [0x07] - [Every 2s] 当前状态 (PresentStatus)
        case HID_PD_PRESENTSTATUS: // REPORT_ID (7)
            if (reqlen >= 2) {
                // 当前状态（位字段）
                // UPS.Charging = 1;        // 充电状态
                // UPS.Discharging = 0;     // 放电状态
                // UPS.ACPresent = 1;       // AC状态
                // UPS.BatteryPresent = 1;  // 电池存在状态
                // UPS.FullyCharged = 0;    // 充满状态
                
                // 将结构体内容拷贝到缓冲区
                memcpy(buffer, &UPS, 2);
                return 2;
            }
            break;

        case HID_PD_MANUFACTUREDATE: // REPORT_ID (9)
            if (reqlen >= 2) {
                // 生产日期（自1990-01-01的天数）
                buffer[0] = manufacture_date & 0xFF;
                buffer[1] = (manufacture_date >> 8) & 0xFF;
                return 2;
            }
            break;

        case HID_PD_CONFIGVOLTAGE: // REPORT_ID (10)
            if (reqlen >= 2) {
                // 配置电压（单位：厘伏，指数5 = 10^-5伏）
                buffer[0] = config_voltage & 0xFF;
                buffer[1] = (config_voltage >> 8) & 0xFF;
                return 2;
            }
            break;
        
        // [0x0B] - [Every 2s] 电压测量 (Voltage)
        case HID_PD_VOLTAGE: // REPORT_ID (11)
            if (reqlen >= 2) {
                // 当前电压（单位：厘伏，指数5 = 10^-5伏）
                buffer[0] = voltage & 0xFF;
                buffer[1] = (voltage >> 8) & 0xFF;
                return 2;
            }
            break;

        // [0x0C] - [Every 2s] 剩余容量 (RemainingCapacity) 
        case HID_PD_REMAININGCAPACITY: // REPORT_ID (12)
            if (reqlen >= 1) {
                // 剩余容量（单位：%）
                buffer[0] = remaining_capacity;
                return 1;
            }
            break;
        
        // [0x0D] - [Every 2s] 运行至空时间 (RunTimeToEmpty) 
        case HID_PD_RUNTIMETOEMPTY: // REPORT_ID (13)
            if (reqlen >= 2) {
                // 运行至空的时间（分钟）
                buffer[0] = runtime_to_empty & 0xFF;
                buffer[1] = (runtime_to_empty >> 8) & 0xFF;
                return 2;
            }
            break;

        case HID_PD_FULLCHARGECAPACITY: // REPORT_ID (14)
            if (reqlen >= 2) {
                // 充满电容量（单位：%）
                buffer[0] = full_charge_capacity & 0xFF;
                buffer[1] = (full_charge_capacity >> 8) & 0xFF;
                return 2;
            }
            break;

        // [0x0F] - [Every 30s] 警告容量限制 (WarningCapacityLimit)
        case HID_PD_WARNCAPACITYLIMIT: // REPORT_ID (15)
            if (reqlen >= 1) {
                // 警告容量限制（百分比）
                buffer[0] = 20; // 示例值：20%
                return 1;
            }
            break;

        case HID_PD_CPCTYGRANULARITY1: // REPORT_ID (16)
            if (reqlen >= 1) {
                // 容量粒度1（百分比）
                buffer[0] = 1; // 示例值：1%
                return 1;
            }
            break;
        
        // [0x11] - [Every 2s] 剩余容量限制 (RemainingCapacityLimit)
        case HID_PD_REMNCAPACITYLIMIT: // REPORT_ID (17)
            if (reqlen >= 1) {
                // 剩余容量限制（百分比）
                buffer[0] = 10; // 示例值：10%
                return 1;
            }
            break;

        case HID_PD_DELAYBE4SHUTDOWN: // REPORT_ID (18)
            if (reqlen >= 2) {
                // 关机前延迟（秒）
                buffer[0] = delay_before_shutdown & 0xFF;
                buffer[1] = (delay_before_shutdown >> 8) & 0xFF;
                return 2;
            }
            break;

        case HID_PD_DELAYBE4REBOOT: // REPORT_ID (19)
            if (reqlen >= 2) {
                // 重启前延迟（秒）
                buffer[0] = delay_before_reboot & 0xFF;
                buffer[1] = (delay_before_reboot >> 8) & 0xFF;
                return 2;
            }
            break;

        case HID_PD_AUDIBLEALARMCTRL: // REPORT_ID (20)
            if (reqlen >= 1) {
                // 声音报警控制（1-3）
                buffer[0] = 2; // 示例值：启用
                return 1;
            }
            break;

        case HID_PD_CAPACITYMODE: // REPORT_ID (22)
            if (reqlen >= 1) {
                // 容量模式
                buffer[0] = 0x01; // 示例值：相对模式
                return 1;
            }
            break;
        
        // [0x17] - [Only Init] 设计容量 (DesignCapacity)
        case HID_PD_DESIGNCAPACITY: // REPORT_ID (23)
            if (reqlen >= 1) {
                // 设计容量（单位：%）
                buffer[0] = design_capacity;
                return 1;
            }
            break;

        case HID_PD_CPCTYGRANULARITY2: // REPORT_ID (24)
            if (reqlen >= 1) {
                // 容量粒度2
                buffer[0] = 0x00; // 示例值：未定义
                return 1;
            }
            break;

        case HID_PD_AVERAGETIME2FULL: // REPORT_ID (26)
            if (reqlen >= 2) {
                // 平均充满时间（秒）
                buffer[0] = avg_time_to_full & 0xFF;
                buffer[1] = (avg_time_to_full >> 8) & 0xFF;
                return 2;
            }
            break;

        case HID_PD_AVERAGETIME2EMPTY: // REPORT_ID (28)
            if (reqlen >= 2) {
                // 平均放空时间（秒）
                buffer[0] = avg_time_to_empty & 0xFF;
                buffer[1] = (avg_time_to_empty >> 8) & 0xFF;
                return 2;
            }
            break;

        // [0x1F] - [Only Init] 设备化学类型 (iDeviceChemistry) 
        case HID_PD_IDEVICECHEMISTRY: // REPORT_ID (31)
            if (reqlen >= 1) {
                // 设备化学类型字符串索引
                buffer[0] = IDEVICECHEMISTRY;
                return 1;
            }
            break;

        // [0x20] - OEM信息 (iOEMInformation) [Every 30s]
        case HID_PD_IOEMINFORMATION: // REPORT_ID (32)
            if (reqlen >= 1) {
                // OEM信息字符串索引
                buffer[0] = IOEMVENDOR;
                return 1;
            }
            break;

        default:
            ESP_LOGW(TAG, "Unknown feature report ID: 0x%02X", report_id);
            return 0;
    }

    ESP_LOGW(TAG, "Request length too short for report ID: 0x%02X", report_id);
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) 
{
    ESP_LOGI(TAG, "Set report: ID=0x%02X, Type=%u, Size=%u", report_id, report_type, bufsize);

    if (report_type != HID_REPORT_TYPE_FEATURE || bufsize < 1) {
        return;
    }

    uint8_t const* data = buffer + 1;
    uint16_t data_size = bufsize - 1;

    switch(report_id) {
        // ==================== Report ID 1: 主交流输入配置 ====================
        case 0x01: // 主交流输入配置
            if (data_size >= 4) {
                uint8_t config_voltage = data[2];
                uint8_t config_frequency = data[3];
                ESP_LOGI(TAG, "AC Input Config - Voltage:%dV, Frequency:%dHz", 
                        config_voltage * 128, config_frequency);
                // 硬件控制代码
            }
            break;

        // ==================== Report ID 2: 备份直流流配置 ====================
        case 0x02: // 备份直流流配置
            if (data_size >= 5) {
                uint16_t config_voltage = (data[3] << 8) | data[2];
                ESP_LOGI(TAG, "DC Backup Config - Voltage:%dmV", config_voltage * 32);
                // 硬件控制代码
            }
            break;

        case 0x03: // 输出交流流配置
            if (data_size >= 6) {
                uint8_t config_voltage = data[2];
                uint8_t config_frequency = data[3];
                uint16_t config_apparent_power = (data[5] << 8) | data[4];
                ESP_LOGI(TAG, "AC Output Config - Voltage:%dV, Freq:%dHz, Power:%dVA", 
                        config_voltage * 128, config_frequency, config_apparent_power * 128);
                // 硬件控制代码
            }
            break;

        case 0x06: // 电池配置
            if (data_size >= 11) {
                uint16_t config_voltage = (data[6] << 8) | data[5];
                ESP_LOGI(TAG, "Battery Config - Voltage:%dmV", config_voltage * 32);
                // 硬件控制代码
            }
            break;

        case 0x0B: // 电源摘要配置
            if (data_size >= 28) {
                uint16_t config_voltage = (data[16] << 8) | data[15];
                ESP_LOGI(TAG, "Power Summary Config - Voltage:%dmV", config_voltage * 32);
                // 硬件控制代码
            }
            break;

        default:
            ESP_LOGW(TAG, "Unknown report ID: 0x%02X", report_id);
            break;
    }
}

uint8_t tud_hid_get_protocol_cb(uint8_t instance) {
    return HID_PROTOCOL_NONE;
}

void tud_hid_set_protocol_cb(uint8_t instance, uint8_t protocol) {
    ESP_LOGI(TAG, "Protocol set to: %u", protocol);
}

// 更新UPS状态
static void update_ups_state(void) {
    // 模拟AC电源断开/连接（60秒切换一次）
    static uint32_t ac_timer = 0;
    if (xTaskGetTickCount() - ac_timer > pdMS_TO_TICKS(60000)) {
        UPS.ACPresent ^= 1;   //每次翻转
        ac_timer = xTaskGetTickCount();
        
        if (UPS.ACPresent) {
            UPS.Charging = 1;
            UPS.Discharging = 0;
            ESP_LOGI(TAG, "AC Connected - Charging");
        } else {
            UPS.Charging = 0;
            UPS.Discharging = 1;
            ESP_LOGI(TAG, "AC Disconnected - Discharging");
        }
    }

    // 模拟电池容量变化
    if (UPS.Charging && remaining_capacity < 100) {
        remaining_capacity++;
        if (remaining_capacity >= 100) {
            UPS.FullyCharged = 1;
            UPS.Charging = 0;
        }
    } else if (UPS.Discharging && remaining_capacity > 0) {
        remaining_capacity--;
        UPS.FullyCharged = 0;
        if (remaining_capacity <= 15) {
            UPS.FullyDischarged = 15;
        }
    }

    // 更新剩余时间
    runtime_to_empty = (remaining_capacity * 72) ;

    ESP_LOGI(TAG, "ACPresent: %d, Charging: %d, Discharging: %d, FullyCharged: %d, RemainingCapacity: %d%%",
        UPS.ACPresent, UPS.Charging, UPS.Discharging, UPS.FullyCharged, remaining_capacity);

// uint16_t manufacture_date = 12345;      // 生产日期（自1990-01-01的天数）
// uint16_t config_voltage = 12000;        // 配置电压, 指数5 = 10^-5伏  示例值：120.00V   
// uint16_t voltage = 11850;               // 当前电压, 指数5 = 10^-5伏, 示例值：118.50V  
// uint8_t remaining_capacity = 85;        // 剩余容量, 示例值：85.00%
// uint16_t runtime_to_empty = 3600;       // 运行至空的时间, 示例值：60分钟
// uint16_t full_charge_capacity = 10000;  // 充满电容量, 示例值：100.00%
// uint8_t warring_capacity_limit = 20;    // 警告容量限制,示例值：20.00%
// uint8_t remaining_capacity_limit = 10;  // 剩余容量限制,示例值：10.00%
// int16_t delay_before_shutdown = 300;    // 关机前延迟（秒）,示例值：300秒
// int16_t delay_before_reboot = 60;       // 重启前延迟（秒）, 示例值：60秒
// uint16_t design_capacity = 100;         // 设计容量（单位：%）, 示例值：100.00%
// uint16_t avg_time_to_full = 7200;       // 平均充满时间（秒）, 示例值：2小时
// uint16_t avg_time_to_empty = 14400;     // 平均放空时间（秒）, 示例值：4小时
}

// USB HID初始化
static void usb_hid_init(void) {
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &descriptor_dev,
        .configuration_descriptor = desc_configuration,
        .string_descriptor = descriptor_str,
        .string_descriptor_count = sizeof(descriptor_str) / sizeof(descriptor_str[0]),
        .external_phy = false,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "TinyUSB initialized");

    while (!tud_mounted()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "USB connected");
}

// 主函数
void app_main(void) {
    ESP_LOGI(TAG, "UPS Device starting");

    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 初始化USB HID
    usb_hid_init();
    
    // 主循环
    while (1) {
        update_ups_state();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}



























//使用Github的配置， 可以识别UPS，识别电池容量和时间，进一步更新
// #include <stdio.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "esp_err.h"
// #include "nvs_flash.h"
// #include "tusb.h"
// #include "class/hid/hid.h"
// #include "tinyusb.h"
// #include "device/usbd.h"



// static const char *TAG = "DC-UPS";

// // 宏定义
// #define LO8(x) ((x) & 0xFF)
// #define HI8(x) ((x) >> 8)

// // 报告ID定义

// #define HID_PD_IDEVICECHEMISTRY      0x04
// #define HID_PD_RECHARGEABLE          0x06



// #define HID_PD_IPRODUCT              0x01 // FEATURE ONLY
// #define HID_PD_SERIAL                0x02 // FEATURE ONLY
// #define HID_PD_MANUFACTURER          0x03 // FEATURE ONLY
// #define IDEVICECHEMISTRY             0x04
// #define IOEMVENDOR                   0x05

// #define HID_PD_RECHARGEABLE          0x06 // FEATURE ONLY
// #define HID_PD_PRESENTSTATUS         0x07 // INPUT OR FEATURE(required by Windows)
// #define HID_PD_REMAINTIMELIMIT       0x08
// #define HID_PD_MANUFACTUREDATE       0x09
// #define HID_PD_CONFIGVOLTAGE         0x0A // 10 FEATURE ONLY
// #define HID_PD_VOLTAGE               0x0B // 11 INPUT (NA) OR FEATURE(implemented)
// #define HID_PD_REMAININGCAPACITY     0x0C // 12 INPUT OR FEATURE(required by Windows)
// #define HID_PD_RUNTIMETOEMPTY        0x0D 
// #define HID_PD_FULLCHARGECAPACITY     0x0E // 14 FEATURE ONLY. Last Full Charge Capacity 
// #define HID_PD_WARNCAPACITYLIMIT     0x0F
// #define HID_PD_CPCTYGRANULARITY1     0x10
// #define HID_PD_REMNCAPACITYLIMIT     0x11
// #define HID_PD_DELAYBE4SHUTDOWN      0x12 // 18 FEATURE ONLY
// #define HID_PD_DELAYBE4REBOOT        0x13
// #define HID_PD_AUDIBLEALARMCTRL      0x14 // 20 INPUT OR FEATURE
// #define HID_PD_CURRENT               0x15 // 21 FEATURE ONLY
// #define HID_PD_CAPACITYMODE          0x16
// #define HID_PD_DESIGNCAPACITY        0x17
// #define HID_PD_CPCTYGRANULARITY2     0x18
// #define HID_PD_AVERAGETIME2FULL      0x1A
// #define HID_PD_AVERAGECURRENT        0x1B
// #define HID_PD_AVERAGETIME2EMPTY     0x1C

// #define HID_PD_IDEVICECHEMISTRY      0x1F // Feature
// #define HID_PD_IOEMINFORMATION       0x20 // Feature



// // 字符串索引定义
// #define IMANUFACTURER               0x01
// #define IPRODUCT                    0x02
// #define ISERIAL                     0x03
// #define IDEVICECHEMISTRY            0x04

// // HID 协议类型定义
// #define HID_PROTOCOL_NONE 0

// // 电源状态结构体
// struct PresentStatus {
//     uint8_t Charging : 1;
//     uint8_t Discharging : 1;
//     uint8_t ACPresent : 1;
//     uint8_t BatteryPresent : 1;
//     uint8_t BelowRemainingCapacityLimit : 1;
//     uint8_t RemainingTimeLimitExpired : 1;
//     uint8_t NeedReplacement : 1;
//     uint8_t VoltageNotRegulated : 1;
//     uint8_t FullyCharged : 1;
//     uint8_t FullyDischarged : 1;
//     uint8_t ShutdownRequested : 1;
//     uint8_t ShutdownImminent : 1;
//     uint8_t CommunicationLost : 1;
//     uint8_t Overload : 1;
//     uint8_t unused1 : 1;
//     uint8_t unused2 : 1;
// };

// // 将PresentStatus结构体转换为uint16_t
// static inline uint16_t PresentStatus_to_uint16(const struct PresentStatus* ps) {
//     return *(const uint16_t*)(ps);
// }

// // 全局状态变量
// static struct PresentStatus UPS = {
//     .ACPresent = 1,
//     .BatteryPresent = 1,
//     .Charging = 1,
//     .FullyCharged = 0
// };
// static uint16_t ac_input_voltage = 22000;
// static uint16_t battery_voltage = 1200;
// static uint16_t dc_output_voltage = 1200;
// static uint16_t dc_output_current = 1000;
// static uint16_t design_capacity = 2000;
// static uint8_t battery_capacity = 100;
// static uint16_t runtime_to_empty = 120;
// static uint8_t device_capability = 0x07;




// const uint8_t hid_report_descriptor[] = {

//     0x05, 0x84, // USAGE_PAGE (Power Device)
//     0x09, 0x04, // USAGE (UPS)
//     0xA1, 0x01, // COLLECTION (Application)
//     0x09, 0x24, //   USAGE (Sink)
//     0xA1, 0x02, //   COLLECTION (Logical)
//     0x75, 0x08, //     REPORT_SIZE (8)
//     0x95, 0x01, //     REPORT_COUNT (1)
//     0x15, 0x00, //     LOGICAL_MINIMUM (0)
//     0x26, 0xFF, 0x00, //     LOGICAL_MAXIMUM (255)
//     0x85, HID_PD_IPRODUCT, //     REPORT_ID (1)
//     0x09, 0xFE, //     USAGE (iProduct)
//     0x79, IPRODUCT, //     STRING INDEX (2)
//     0xB1, 0x23, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
//     0x85, HID_PD_SERIAL, //     REPORT_ID (2)
//     0x09, 0xFF, //     USAGE (iSerialNumber)
//     0x79, ISERIAL, //  STRING INDEX (3)
//     0xB1, 0x23, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
//     0x85, HID_PD_MANUFACTURER, //     REPORT_ID (3)
//     0x09, 0xFD, //     USAGE (iManufacturer)
//     0x79, IMANUFACTURER, //     STRING INDEX (1)
//     0xB1, 0x23, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
//     0x05, 0x85, //     USAGE_PAGE (Battery System) ====================
//     0x85, HID_PD_RECHARGEABLE, //     REPORT_ID (6)
//     0x09, 0x8B, //     USAGE (Rechargable)                  
//     0xB1, 0x23, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
//     0x85, HID_PD_IDEVICECHEMISTRY, //     REPORT_ID (31)
//     0x09, 0x89, //     USAGE (iDeviceChemistry)
//     0x79, IDEVICECHEMISTRY, //     STRING INDEX (4)
//     0xB1, 0x23, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
//     0x85, HID_PD_IOEMINFORMATION,  //     REPORT_ID (32)
//     0x09, 0x8F, //     USAGE (iOEMInformation)
//     0x79, IOEMVENDOR, //     STRING INDEX (5)
//     0xB1, 0x23, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
//     0x85, HID_PD_CAPACITYMODE, //     REPORT_ID (22)
//     0x09, 0x2C, //     USAGE (CapacityMode)
//     0xB1, 0x23, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
//     0x85, HID_PD_CPCTYGRANULARITY1, //     REPORT_ID (16)
//     0x09, 0x8D, //     USAGE (CapacityGranularity1)
//     0x26, 0x64,0x00, //     LOGICAL_MAXIMUM (100)    
//     0xB1, 0x22, //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
//     0x85, HID_PD_CPCTYGRANULARITY2, //     REPORT_ID (24)
//     0x09, 0x8E, //     USAGE (CapacityGranularity2)
//     0xB1, 0x23, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
//     0x85, HID_PD_FULLCHARGECAPACITY, //     REPORT_ID (14)        
//     0x09, 0x67, //     USAGE (FullChargeCapacity)
//     0xB1, 0x83, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x85, HID_PD_DESIGNCAPACITY, //     REPORT_ID (23)
//     0x09, 0x83, //     USAGE (DesignCapacity)
//     0xB1, 0x83, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x85, HID_PD_REMAININGCAPACITY, //     REPORT_ID (12)
//     0x09, 0x66, //     USAGE (RemainingCapacity)
//     0x81, 0xA3, //     INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0x66, //     USAGE (RemainingCapacity)
//     0xB1, 0xA3, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x85, HID_PD_WARNCAPACITYLIMIT, //     REPORT_ID (15)
//     0x09, 0x8C, //     USAGE (WarningCapacityLimit)
//     0xB1, 0xA2, //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x85, HID_PD_REMNCAPACITYLIMIT, //     REPORT_ID (17)
//     0x09, 0x29, //     USAGE (RemainingCapacityLimit)
//     0xB1, 0xA2, //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x85, HID_PD_MANUFACTUREDATE, //     REPORT_ID (9)
//     0x09, 0x85, //     USAGE (ManufacturerDate)
//     0x75, 0x10, //     REPORT_SIZE (16)
//     0x27, 0xFF, 0xFF, 0x00, 0x00, //     LOGICAL_MAXIMUM (65534)
//     0xB1, 0xA3, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x85, HID_PD_AVERAGETIME2FULL, //     REPORT_ID (26)
//     0x09, 0x6A, //     USAGE (AverageTimeToFull)
//     0x27, 0xFF, 0xFF, 0x00, 0x00, //     LOGICAL_MAXIMUM (65534)
//     0x66, 0x01, 0x10, //     UNIT (Seconds)
//     0x55, 0x00, //     UNIT_EXPONENT (0)
//     0xB1, 0xA3, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield) 
//     0x85, HID_PD_AVERAGETIME2EMPTY, //     REPORT_ID (28)
//     0x09, 0x69, //     USAGE (AverageTimeToEmpty)  
//     0x81, 0xA3, //     INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0x69, //     USAGE (AverageTimeToEmpty)
//     0xB1, 0xA3, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x85, HID_PD_RUNTIMETOEMPTY, //     REPORT_ID (13)    
//     0x09, 0x68, //     USAGE (RunTimeToEmpty)  
//     0x81, 0xA3, //     INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0x68, //     USAGE (RunTimeToEmpty)
//     0xB1, 0xA3, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)      
//     0x85, HID_PD_REMAINTIMELIMIT, //     REPORT_ID (8)
//     0x09, 0x2A, //     USAGE (RemainingTimeLimit)
//     0x75, 0x10, //     REPORT_SIZE (16)
//     0x27, 0x64, 0x05, 0x00, 0x00, //     LOGICAL_MAXIMUM (1380)
//     0x16, 0x78, 0x00, //     LOGICAL_MINIMUM (120)
//     0x81, 0x22, //     INPUT (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0x2A, //     USAGE (RemainingTimeLimit)
//     0xB1, 0xA2, //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x05, 0x84, //     USAGE_PAGE (Power Device) ====================
//     0x85, HID_PD_DELAYBE4SHUTDOWN, //     REPORT_ID (18)
//     0x09, 0x57, //     USAGE (DelayBeforeShutdown)
//     0x16, 0x00, 0x80, //     LOGICAL_MINIMUM (-32768)
//     0x27, 0xFF, 0x7F, 0x00, 0x00, //     LOGICAL_MAXIMUM (32767)
//     0xB1, 0xA2, //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x85, HID_PD_DELAYBE4REBOOT, //     REPORT_ID (19)
//     0x09, 0x55, //     USAGE (DelayBeforeReboot)
//     0xB1, 0xA2, //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x85, HID_PD_CONFIGVOLTAGE, //     REPORT_ID (10)
//     0x09, 0x40, //     USAGE (ConfigVoltage)
//     0x15, 0x00, //     LOGICAL_MINIMUM (0)
//     0x27, 0xFF, 0xFF, 0x00, 0x00, //     LOGICAL_MAXIMUM (65535)
//     0x67, 0x21, 0xD1, 0xF0, 0x00, //     UNIT (Centivolts)
//     0x55, 0x05, //     UNIT_EXPONENT (5)
//     0xB1, 0x23, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
//     0x85, HID_PD_VOLTAGE, //     REPORT_ID (11)
//     0x09, 0x30, //     USAGE (Voltage)
//     0x81, 0xA3, //     INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0x30, //     USAGE (Voltage)
//     0xB1, 0xA3, //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x85, HID_PD_AUDIBLEALARMCTRL, //     REPORT_ID (20)
//     0x09, 0x5A, //     USAGE (AudibleAlarmControl)
//     0x75, 0x08, //     REPORT_SIZE (8)
//     0x15, 0x01, //     LOGICAL_MINIMUM (1)
//     0x25, 0x03, //     LOGICAL_MAXIMUM (3)
//     0x65, 0x00, //     UNIT (0)
//     0x55, 0x00, //     UNIT_EXPONENT (0)
//     0x81, 0x22, //     INPUT (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0x5A, //     USAGE (AudibleAlarmControl)
//     0xB1, 0xA2, //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x09, 0x02, //     USAGE (PresentStatus)
//     0xA1, 0x02, //     COLLECTION (Logical)
//     0x85, HID_PD_PRESENTSTATUS, //       REPORT_ID (7)
//     0x05, 0x85, //       USAGE_PAGE (Battery System) =================
//     0x09, 0x44, //       USAGE (Charging)
//     0x75, 0x01, //       REPORT_SIZE (1)
//     0x15, 0x00, //       LOGICAL_MINIMUM (0)
//     0x25, 0x01, //       LOGICAL_MAXIMUM (1)
//     0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0x44, //       USAGE (Charging)
//     0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x09, 0x45, //       USAGE (Discharging)
//     0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0x45, //       USAGE (Discharging)
//     0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x09, 0xD0, //       USAGE (ACPresent)
//     0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0xD0, //       USAGE (ACPresent)
//     0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x09, 0xD1, //       USAGE (BatteryPresent)
//     0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0xD1, //       USAGE (BatteryPresent)
//     0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x09, 0x42, //       USAGE (BelowRemainingCapacityLimit)
//     0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0x42, //       USAGE (BelowRemainingCapacityLimit)
//     0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x09, 0x43, //       USAGE (RemainingTimeLimitExpired)
//     0x81, 0xA2, //       INPUT (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0x43, //       USAGE (RemainingTimeLimitExpired)
//     0xB1, 0xA2, //       FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)  
//     0x09, 0x4B, //       USAGE (NeedReplacement)
//     0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0x4B, //       USAGE (NeedReplacement)
//     0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)    
//     0x09, 0xDB, //       USAGE (VoltageNotRegulated)
//     0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0xDB, //       USAGE (VoltageNotRegulated)
//     0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x09, 0x46, //       USAGE (FullyCharged)
//     0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0x46, //       USAGE (FullyCharged)
//     0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x09, 0x47, //       USAGE (FullyDischarged)
//     0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0x47, //       USAGE (FullyDischarged)
//     0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)    
//     0x05, 0x84, //       USAGE_PAGE (Power Device) =================
//     0x09, 0x68, //       USAGE (ShutdownRequested)
//     0x81, 0xA2, //       INPUT (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0x68, //       USAGE (ShutdownRequested)
//     0xB1, 0xA2, //       FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x09, 0x69, //       USAGE (ShutdownImminent)
//     0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0x69, //       USAGE (ShutdownImminent)
//     0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x09, 0x73, //       USAGE (CommunicationLost)
//     0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0x73, //       USAGE (CommunicationLost)
//     0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x09, 0x65, //       USAGE (Overload)
//     0x81, 0xA3, //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
//     0x09, 0x65, //       USAGE (Overload)
//     0xB1, 0xA3, //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
//     0x95, 0x02, //       REPORT_COUNT (2) // padding bits to make the report byte aligned
//     0x81, 0x01, //       INPUT (Constant, Array, Absolute)
//     0xB1, 0x01, //       FEATURE (Constant, Array, Absolute, No Wrap, Linear, Preferred State, No Null Position, Nonvolatile, Bitfield)
//     0xC0,       //     END_COLLECTION
//     0xC0,       //   END_COLLECTION
//     0xC0        // END_COLLECTION
// };


// // A.6 HID报告描述符
// // const uint8_t hid_report_descriptor[] = {
// //     // ==================== [6.1] UPS应用集合 ====================
// //     0x05, 0x84,        // USAGE_PAGE (Power Device)
// //     0x09, 0x04,        // USAGE (UPS) - 正确
// //     0xA1, 0x01,        // COLLECTION (Application)
    
// //         // ==================== [6.2] 主交流输入物理集合 [ID1] ====================
// //         // 主交流电流程包含交流电的流程 ID (1)、名称、配置电压和配置频率。功能报告 ID 1 开始。
// //         0x05, 0x84,        // USAGE_PAGE (Power Device)
// //         0x09, 0x1E,        // USAGE (Flow) - 修正: 0x1E (原错误: 0x2F)
// //         0xA1, 0x00,        // COLLECTION (Physical)

// //         // Report ID 1 - Main AC Flow配置报告
// //         0x85, 0x01,        // REPORT_ID (1) - 固定为1

// //             // FlowID字段 (4位) - 固定值1
// //             0x09, 0x1F,        // USAGE (FlowID) - 修正: 0x1F (原错误: 0x71)
// //             0x75, 0x04,        // REPORT_SIZE (4)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //             0x65, 0x00,        // UNIT (None)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)

// //             // Pad字段 (4位) - 填充位
// //             0x75, 0x04,        // REPORT_SIZE (4)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             // 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             // 0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //             // 0x65, 0x00,        // UNIT (None)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)

// //             // iName字段 (8位) - 字符串索引
// //             0x09, 0xFE,        // USAGE (iName)
// //             0x75, 0x08,        // REPORT_SIZE (8)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0xFF,        // LOGICAL_MAXIMUM (255)
// //             // 0x65, 0x00,        // UNIT (None)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
            
// //             // ConfigVoltage字段 (8位) - 配置电压
// //             0x09, 0x40,        // USAGE (ConfigVoltage) - 修正: 0x40 (原错误: 0x50)
// //             0x75, 0x08,        // REPORT_SIZE (8)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0xFA,        // LOGICAL_MAXIMUM (250)
// //             0x65, 0x00,        // UNIT (None)
// //             0x55, 0x07,        // UNIT_EXPONENT (7)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
            
// //             // ConfigFrequency字段 (8位) - 配置频率
// //             0x09, 0x42,        // USAGE (ConfigFrequency) - 修正: 0x42 (原错误: 0x51)
// //             0x75, 0x08,        // REPORT_SIZE (8)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0x3C,        // LOGICAL_MAXIMUM (60)
// //             0x66, 0x01, 0x10,  // UNIT (Hertz)
// //             0x55, 0x00,        // UNIT_EXPONENT (0)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
        
// //         0xC0,              // END_COLLECTION (Main AC Physical)
        
// //         // ==================== [6.3] Backup DC Flow Physical Collection ====================
// //         // 备份直流流包含直流电的流程 ID (2)、名称、配置电压和配置频率。功能报告 ID 2 开始。
// //         0x05, 0x84,        // USAGE_PAGE (Power Device)
// //         0x09, 0x1E,        // USAGE (Flow) - 修正: 0x1E
// //         0xA1, 0x00,        // COLLECTION (Physical)

// //         // Report ID 2 - Backup DC Flow配置报告
// //         0x85, 0x02,        // REPORT_ID (2)
            
// //             // FlowID字段 (4位) - 固定值2
// //             0x09, 0x1F,        // USAGE (FlowID) - 修正: 0x1F
// //             0x75, 0x04,        // REPORT_SIZE (4)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //             0x65, 0x00,        // UNIT (None)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
            
// //             // Pad字段 (4位)
// //             0x75, 0x04,        // REPORT_SIZE (4)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             // 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             // 0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //             // 0x65, 0x00,        // UNIT (None)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
            
// //             // iName字段 (8位)
// //             0x09, 0x01,        // USAGE (iName) - 修正: 0x01
// //             0x75, 0x08,        // REPORT_SIZE (8)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0xFF,        // LOGICAL_MAXIMUM (255)
// //             // 0x65, 0x00,        // UNIT (None)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
            
// //             // ConfigVoltage字段 (16位)
// //             0x09, 0x40,        // USAGE (ConfigVoltage) - 修正: 0x40
// //             0x75, 0x10,        // REPORT_SIZE (16)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x26, 0xFE, 0xFF,  // LOGICAL_MAXIMUM (65534)
// //             0x66, 0x01, 0x10,  // UNIT (Volts)
// //             0x55, 0x05,        // UNIT_EXPONENT (5)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
            
// //             // ConfigFrequency字段 (8位)
// //             0x09, 0x42,        // USAGE (ConfigFrequency) - 修正: 0x42
// //             0x75, 0x08,        // REPORT_SIZE (8)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0x3C,        // LOGICAL_MAXIMUM (60)
// //             0x66, 0x01, 0x10,  // UNIT (Hertz)
// //             0x55, 0x00,        // UNIT_EXPONENT (0)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)

// //         0xC0,              // END_COLLECTION - 结束备份直流流集合
        
// //         // ==================== [6.4] 输出交流物理集合 [ID4] ====================
// //         // 输出交流电流程包含交流电的流程 ID (3)、名称、配置电压、配置频率和配置功率。
// //         // 功能报告 ID 3 开始。
// //         0x05, 0x84,        // USAGE_PAGE (Power Device)
// //         0x09, 0x1E,        // USAGE (Flow) - 修正: 0x1E
// //         0xA1, 0x00,        // COLLECTION (Physical)

// //         // Report ID 3
// //         0x85, 0x03,        // REPORT_ID (3)
            
// //             // FlowID字段 (4位) - 固定值3
// //             0x09, 0x1F,        // USAGE (FlowID) - 修正: 0x1F
// //             0x75, 0x04,        // REPORT_SIZE (4)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //             0x65, 0x00,        // UNIT (None)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
            
// //             // Pad字段 (4位)
// //             0x75, 0x04,        // REPORT_SIZE (4)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             // 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             // 0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //             // 0x65, 0x00,        // UNIT (None)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
            
// //             // iName字段 (8位)
// //             0x09, 0x01,        // USAGE (iName) - 修正: 0x01
// //             0x75, 0x08,        // REPORT_SIZE (8)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0xFF,        // LOGICAL_MAXIMUM (255)
// //             // 0x65, 0x00,        // UNIT (None)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
            
// //             // ConfigVoltage字段 (8位)
// //             0x09, 0x40,        // USAGE (ConfigVoltage) - 修正: 0x40
// //             0x75, 0x08,        // REPORT_SIZE (8)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0xFA,        // LOGICAL_MAXIMUM (250)
// //             0x66, 0x01, 0x10,  // UNIT (Volts)
// //             0x55, 0x07,        // UNIT_EXPONENT (7)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
            
// //             // ConfigFrequency字段 (8位)
// //             0x09, 0x42,        // USAGE (ConfigFrequency) - 修正: 0x42
// //             0x75, 0x08,        // REPORT_SIZE (8)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0x3C,        // LOGICAL_MAXIMUM (60)
// //             0x66, 0x01, 0x10,  // UNIT (Hertz)
// //             0x55, 0x00,        // UNIT_EXPONENT (0)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
            
// //             // ConfigApparentPower字段 (16位)
// //             0x09, 0x43,        // USAGE (ConfigApparentPower) - 修正: 0x43 (原错误: 0x5A)
// //             0x75, 0x10,        // REPORT_SIZE (16)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x26, 0xFE, 0xFF,  // LOGICAL_MAXIMUM (65534)
// //             0x66, 0x01, 0x10,  // UNIT (Watt)
// //             0x55, 0x07,        // UNIT_EXPONENT (7)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)

// //         0xC0,              // END_COLLECTION - 结束输出交流流集合

// //         // 直流UPS的输出是直流，所以这里应该是DC Output [待确认]


// //         // ==================== [6.5] 电池系统物理集合 [ID1] ====================
// //         //电池系统的标题包含电池系统 ID (1)，后跟与其子模块对应的集合。功能报告 ID 4 开始。
// //         0x05, 0x84,        // USAGE_PAGE (Power Device)
// //         0x09, 0x10,        // USAGE (BatterySystem) - 修正: 0x10 (原错误: 0x01)
// //         0xA1, 0x00,        // COLLECTION (Physical)
            
// //         // Report ID 4 - Battery System配置报告
// //         0x85, 0x04,        // REPORT_ID (4) - 固定为4，电池系统

// //             // BatterySystemID字段 (4位)
// //             0x09, 0x11,        // USAGE (BatterySystemID) - 修正: 0x11 (原错误: 0x72)
// //             0x75, 0x04,        // REPORT_SIZE (4)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //             0x65, 0x00,        // UNIT (None)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
            
// //             // Pad字段 (4位)
// //             0x75, 0x04,        // REPORT_SIZE (4)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             // 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             // 0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //             // 0x65, 0x00,        // UNIT (None)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)

// //             // ================== [6.6] 交流输入物理集合 [ID1] ==================
// //             // 电池系统交流电输入包含输入 ID (1)、已连接流程的 ID (1) 以及两个状态项：“已使用”和“良好”。
// //             // 功能报告 ID 4 继续。输入报告 ID 4 开始。
// //             // 电池系统AC输入包含输入ID (1)、连接的流ID (1)和两个状态项：Used和Good
// //             0x09, 0x1A,        // USAGE (Input) - 修正: 0x1A (原错误: 0x2E)
// //             0xA1, 0x00,        // COLLECTION (Physical)

// //                 // InputID和FlowID字段
// //                 0x09, 0x1B,        // USAGE (InputID) - 修正: 0x1B (原错误: 0x73)
// //                 0x09, 0x1F,        // USAGE (FlowID) - 修正: 0x1F
// //                 0x75, 0x04,        // REPORT_SIZE (4)
// //                 0x95, 0x02,        // REPORT_COUNT (2)
// //                 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                 0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //                 0x65, 0x00,        // UNIT (None)
// //                 0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)

// //                 // ==================== Present Status集合 ====================
// //                 0x09, 0x02,        // USAGE (PresentStatus) - 正确
// //                 0xA1, 0x02,        // COLLECTION (Logical)
                
// //                     // Used和Good状态位 (各1位，共2位)
// //                     0x09, 0x6D,        // USAGE (Used) - 修正: 0x6D (原错误: 0x74)
// //                     0x09, 0x61,        // USAGE (Good) - 修正: 0x61 (原错误: 0x75)
// //                     0x75, 0x01,        // REPORT_SIZE (1)
// //                     0x95, 0x02,        // REPORT_COUNT (2)
// //                     0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                     0x25, 0x01,        // LOGICAL_MAXIMUM (1)
// //                     // 0x65, 0x00,        // UNIT (None)
// //                     0xB1, 0x83,        // FEATURE (Const,Var,Abs,Vol)  - 特性(常量, 变量, 绝对值, 易失性)
                
// //                 0xC0,              // END_COLLECTION - 结束当前状态集合

// //                 // ==================== Changed Status集合 ====================
// //                 0x09, 0x03,        // USAGE (ChangedStatus) - 正确
// //                 0xA1, 0x02,        // COLLECTION (Logical)
                
// //                     // Used和Good变化状态 (各1位，但作为INPUT报告)
// //                     0x09, 0x6D,        // USAGE (Used) - 修正: 0x6D
// //                     0x09, 0x61,        // USAGE (Good) - 修正: 0x61
// //                     0x75, 0x02,        // REPORT_SIZE (2)
// //                     0x95, 0x02,        // REPORT_COUNT (2)
// //                     0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                     0x25, 0x01,        // LOGICAL_MAXIMUM (1)
// //                     // 0x65, 0x00,        // UNIT (None)
// //                     0x81, 0x82,        // INPUT (Data,Var,Abs,Vol)  - 输入(数据, 变量, 绝对值, 易失性)
                
// //                 0xC0,              // END_COLLECTION - 结束变化状态集合

// //             0xC0,              // END_COLLECTION - 结束AC输入集合

// //             // =================== [6.7] 充电器物理集合 ===================
// //             // 电池系统充电器包含充电器 ID (1)。功能报告 ID 5 开始。
// //             0x09, 0x14,        // USAGE (Charger) - 修正: 0x14 (原错误: 0x2C)
// //             0xA1, 0x00,        // COLLECTION (Physical)

// //             // Report ID 5 - Charger配置报告
// //             0x85, 0x05,        // REPORT_ID (5) - 固定为5，充电器
                
// //                 // ChargerID字段 (4位) - 固定值1
// //                 0x09, 0x15,        // USAGE (ChargerID) - 修正: 0x15 (原错误: 0x77)
// //                 0x75, 0x04,        // REPORT_SIZE (4)
// //                 0x95, 0x01,        // REPORT_COUNT (1)
// //                 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                 0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //                 0x65, 0x00,        // UNIT (None)
// //                 0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
                
// //                 // Pad字段 (4位)    // 待定是否增加
// //                 0x75, 0x04,        // REPORT_SIZE (4)
// //                 0x95, 0x01,        // REPORT_COUNT (1)
// //                 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                 0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //                 0x65, 0x00,        // UNIT (None)
// //                 0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)

// //             0xC0,              // END_COLLECTION - 结束充电器集合


// //             // =================== [6.8] 直流输出物理集合 ===================
// //             // 电池系统直流输出包含输出ID (1)和连接的流ID (2)。功能报告ID 5继续。
// //             0x09, 0x1C,        // USAGE (Output) - 修正: 0x1C (原错误: 0x22)
// //             0xA1, 0x00,        // COLLECTION (Physical)

// //                 // OutputID和FlowID字段 (各4位，共1字节)
// //                 0x09, 0x1D,        // USAGE (OutputID) - 修正: 0x1D (原错误: 0x70)
// //                 0x09, 0x1F,        // USAGE (FlowID) - 修正: 0x1F
// //                 0x75, 0x04,        // REPORT_SIZE (4)
// //                 0x95, 0x02,        // REPORT_COUNT (2)
// //                 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                 0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //                 0x65, 0x00,        // UNIT (None)
// //                 0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)

// //             0xC0,              // END_COLLECTION - 结束直流输出集合

// //             // ==================== [6.9] 电池物理集合 ====================
// //             //电池系统电池包含电池 ID (1)、容量模式、设计容量、配置电压、剩余容量以及四个状态项：良好、低于剩余容量限制、充电中和放电中。
// //             //功能报告 ID 6 和输入报告 ID 6 开始。
// //             0x09, 0x12,        // USAGE (Battery) - 修正: 0x12 (原错误: 0x03)
// //             0xA1, 0x00,        // COLLECTION (Physical)

// //             // Report ID 6 - Battery配置报告
// //             0x85, 0x06,        // REPORT_ID (6) - 固定为6
                
// //                 // BatteryID字段 (4位) - 固定值1
// //                 0x09, 0x13,        // USAGE (BatteryID) - 修正: 0x13 (原错误: 0x78)
// //                 0x75, 0x04,        // REPORT_SIZE (4)
// //                 0x95, 0x01,        // REPORT_COUNT (1)
// //                 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                 0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //                 0x65, 0x00,        // UNIT (None)
// //                 0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
                
// //                 // ==================== Battery System页面 ====================
// //                 0x05, 0x85,        // USAGE_PAGE (Battery System)
                
// //                 // CapacityMode字段 (1位)
// //                 0x09, 0x52,        // USAGE (CapacityMode) - 修正: 0x52 (原错误: 0x86)
// //                 0x75, 0x01,        // REPORT_SIZE (1)
// //                 0x95, 0x01,        // REPORT_COUNT (1)
// //                 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                 0x25, 0x01,        // LOGICAL_MAXIMUM (1)
// //                 0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
                
// //                 // Pad字段 (3位)
// //                 0x75, 0x03,        // REPORT_SIZE (3)
// //                 0x95, 0x01,        // REPORT_COUNT (1)
// //                 // 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                 // 0x25, 0x07,        // LOGICAL_MAXIMUM (7)
// //                 0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
                
// //                 // DesignCapacity字段 (24位) - 设计容量
// //                 0x09, 0x4B,        // USAGE (DesignCapacity) - 修正: 0x4B (原错误: 0x83)
// //                 0x75, 0x18,        // REPORT_SIZE (24)
// //                 0x95, 0x01,        // REPORT_COUNT (1)
// //                 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                 0x26, 0xFE, 0xFF, 0xFF,  // LOGICAL_MAXIMUM (16777214)
// //                 0x66, 0x01, 0x10,  // UNIT (Amp.s)
// //                 0x55, 0x00,        // UNIT_EXPONENT (0)
// //                 0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
                
// //                 // ==================== Power Device页面 ====================
// //                 0x05, 0x84,        // USAGE_PAGE (Power Device)
                
// //                 // ConfigVoltage字段 (16位) - 配置电压
// //                 0x09, 0x40,        // USAGE (ConfigVoltage) - 修正: 0x40
// //                 0x75, 0x10,        // REPORT_SIZE (16)
// //                 0x95, 0x01,        // REPORT_COUNT (1)
// //                 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                 0x26, 0xFE, 0xFF,  // LOGICAL_MAXIMUM (65534)
// //                 0x66, 0x01, 0x10,  // UNIT (Volt)
// //                 0x55, 0x05,        // UNIT_EXPONENT (5)
// //                 0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
                
// //                 // ==================== Battery System页面 ====================
// //                 0x05, 0x85,        // USAGE_PAGE (Battery System)
                
// //                 // RemainingCapacity字段 (24位) - 剩余容量
// //                 0x09, 0x66,        // USAGE (RemainingCapacity) - 正确
// //                 0x75, 0x18,        // REPORT_SIZE (24)
// //                 0x95, 0x01,        // REPORT_COUNT (1)
// //                 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                 0x26, 0xFE, 0xFF, 0xFF,  // LOGICAL_MAXIMUM (16777214)
// //                 0x66, 0x01, 0x10,  // UNIT (mAh)
// //                 0x55, 0x00,        // UNIT_EXPONENT (0)
// //                 0xB1, 0x83,        // FEATURE (Const,Var,Abs,Vol)  - 特性(常量, 变量, 绝对值, 易失性)
                
// //                 // ==================== Present Status集合 ====================
// //                 0x05, 0x84,        // USAGE_PAGE (Power Device)
// //                 0x09, 0x02,        // USAGE (PresentStatus) - 正确
// //                 0xA1, 0x02,        // COLLECTION (Logical)
// //                 0x65, 0x00,        // UNIT (None)
                
// //                     // 状态位 (各1位，共4位)
// //                     0x09, 0x61,        // USAGE (Good) - 修正: 0x61
// //                     0x05, 0x85,        // USAGE_PAGE (Battery System)
// //                     0x09, 0x4C,        // USAGE (BelowRemainingCapacityLimit) - 修正: 0x4C (原错误: 0x87)
// //                     0x09, 0x44,        // USAGE (Charging) - 修正: 0x44 (原错误: 0x8C)
// //                     0x09, 0x4B,        // USAGE (Discharging) - 修正: 0x4B (原错误: 0x8E)
// //                     0x75, 0x01,        // REPORT_SIZE (1)
// //                     0x95, 0x04,        // REPORT_COUNT (4)
// //                     0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                     0x25, 0x01,        // LOGICAL_MAXIMUM (1)
// //                     0x65, 0x00,        // UNIT (None)
// //                     0xB1, 0x83,        // FEATURE (Const,Var,Abs,Vol)  - 特性(常量, 变量, 绝对值, 易失性)
                
// //                 0xC0,              // END_COLLECTION - 结束当前状态集合
                
// //                 // ==================== Changed Status集合 ====================
// //                 0x05, 0x84,        // USAGE_PAGE (Power Device)
// //                 0x09, 0x03,        // USAGE (ChangedStatus) - 正确
// //                 0xA1, 0x02,        // COLLECTION (Logical)
                
// //                     // 变化状态位 (各1位，但作为INPUT报告)
// //                     0x09, 0x61,        // USAGE (Good) - 修正: 0x61
// //                     0x05, 0x85,        // USAGE_PAGE (Battery System)
// //                     0x09, 0x4C,        // USAGE (BelowRemainingCapacityLimit) - 修正: 0x4C
// //                     0x09, 0x44,        // USAGE (Charging) - 修正: 0x44
// //                     0x09, 0x4B,        // USAGE (Discharging) - 修正: 0x4B
// //                     0x75, 0x02,        // REPORT_SIZE (2)
// //                     0x95, 0x04,        // REPORT_COUNT (4)
// //                     0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                     0x25, 0x01,        // LOGICAL_MAXIMUM (1)
// //                     // 0x65, 0x00,        // UNIT (None)
// //                     0x81, 0x82,        // INPUT (Data,Var,Abs,Vol)  - 输入(数据, 变量, 绝对值, 易失性)
                
// //                 0xC0,              // END_COLLECTION - 结束变化状态集合

// //             0xC0,              // END_COLLECTION - 结束电池集合

// //         0xC0,              // END_COLLECTION (Battery System Physical)


// // /*  ====================================无电源适配器==============================

// //         //==================== [6.10] 电源转换器物理集合 ====================
// //         // 电源转换器的标题包含电源转换器 ID，后跟与其子模块对应的集合。
// //         // 功能报告 ID 8 和输入报告 ID 8 开始。
// //         0x05, 0x84,        // USAGE_PAGE (Power Device)
// //         0x09, 0x16,        // USAGE (PowerConverter) - 正确
// //         0xA1, 0x00,        // COLLECTION (Physical)

// //         // Report ID 8 - Power Converter配置报告
// //         0x85, 0x08,        // REPORT_ID (8) - 固定为8
        
// //             // PowerConverterID字段 (4位) - 固定值1
// //             0x09, 0x17,        // USAGE (PowerConverterID) - 正确
// //             0x75, 0x04,        // REPORT_SIZE (4)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //             0x65, 0x00,        // UNIT (None)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
            
// //             // Pad字段 (4位) - 填充位  //待确认
// //             0x75, 0x04,        // REPORT_SIZE (4)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //             0x65, 0x00,        // UNIT (None)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)


// //             //==================== [6.11] 交流输入物理集合 ====================
// //             // 电源转换器交流输入包含输入 ID (2)、连接流 ID (1) 以及两个状态项：已使用 (Used) 和良好 (Good)。
// //             // 功能报告 ID 8 和输入报告 ID 8 继续。
// //             0x09, 0x1A,        // USAGE (Input) - 修正: 0x1A (原错误: 0x00)
// //             0xA1, 0x00,        // COLLECTION (Physical)
   
// //                 // InputID和FlowID字段 (各4位，共1字节)
// //                 0x09, 0x1B,        // USAGE (InputID) - 修正: 0x1B (原错误: 0x73)
// //                 0x09, 0x1F,        // USAGE (FlowID) - 修正: 0x1F (原错误: 0x71)
// //                 0x75, 0x04,        // REPORT_SIZE (4)
// //                 0x95, 0x02,        // REPORT_COUNT (2)
// //                 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                 0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //                 0x65, 0x00,        // UNIT (None)
// //                 0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)

// //                 // ==================== Present Status集合 ====================
// //                 0x09, 0x02,        // USAGE (PresentStatus)
// //                 0xA1, 0x02,        // COLLECTION (Logical)
                
// //                     // Used和Good状态位 (各1位，共2位)
// //                     0x09, 0x6D,        // USAGE (Used) - 修正: 0x6D (原错误: 0x74)
// //                     0x09, 0x61,        // USAGE (Good) - 修正: 0x61 (原错误: 0x75)
// //                     0x75, 0x01,        // REPORT_SIZE (1)
// //                     0x95, 0x02,        // REPORT_COUNT (2)
// //                     0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                     0x25, 0x01,        // LOGICAL_MAXIMUM (1)
// //                     // 0x65, 0x00,        // UNIT (None)
// //                     0xB1, 0x83,        // FEATURE (Const,Var,Abs,Vol)  - 特性(常量, 变量, 绝对值, 易失性)
                
// //                 0xC0,              // END_COLLECTION

// //                 // ==================== Changed Status集合 ====================
// //                 0x09, 0x03,        // USAGE (ChangedStatus)
// //                 0xA1, 0x02,        // COLLECTION (Logical)
                
// //                     // Used和Good变化状态 (各2位)
// //                     0x09, 0x6D,        // USAGE (Used) - 修正: 0x6D
// //                     0x09, 0x61,        // USAGE (Good) - 修正: 0x61
// //                     0x75, 0x02,        // REPORT_SIZE (2)
// //                     0x95, 0x02,        // REPORT_COUNT (2)
// //                     0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                     0x25, 0x01,        // LOGICAL_MAXIMUM (1)
// //                     0x65, 0x00,        // UNIT (None)
// //                     0x81, 0x82,        // INPUT (Data,Var,Abs,Vol)  - 输入(数据, 变量, 绝对值, 易失性)
                
// //                 0xC0,              // END_COLLECTION

// //             0xC0,              // END_COLLECTION (AC Input)

// //             //==================== [6.12] 交流输出物理集合 ====================
// //             // 电源转换器交流输出包含输出 ID (2)、连接流 ID (3) 以及四个状态项：
// //             // 已使用 (Used)、良好 (Good)、过载 (Overload) 和即将关断 (ShutdownImminent)。
// //             // 功能报告 ID 9 和输入报告 ID 9 开始。
// //             0x09, 0x1C,        // USAGE (Output) - 修正: 0x1C (原错误: 0x01)
// //             0xA1, 0x00,        // COLLECTION (Physical)

// //                 // Report ID 9 - AC Output配置报告
// //                 0x85, 0x09,        // REPORT_ID (9) - 报告ID固定为9

// //                 // OutputID和FlowID字段 (各4位，共8位 = 1字节) - Figure 10 Byte1
// //                 0x09, 0x1D,        // USAGE (OutputID) - 修正: 0x1D (原错误: 0x09)
// //                 0x09, 0x1F,        // USAGE (FlowID) - 修正: 0x1F (原错误: 0x04)
// //                 0x75, 0x04,        // REPORT_SIZE (4)
// //                 0x95, 0x02,        // REPORT_COUNT (2)
// //                 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                 0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //                 0x65, 0x00,        // UNIT (None)
// //                 0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)

// //                 // PercentLoad字段 (8位) - Figure 10 Byte2
// //                 0x09, 0x35,        // USAGE (PercentLoad) - 修正: 0x35 (原错误: 0x0A)
// //                 0x75, 0x08,        // REPORT_SIZE (8)
// //                 0x95, 0x01,        // REPORT_COUNT (1)
// //                 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                 0x25, 0xFF,        // LOGICAL_MAXIMUM (255)
// //                 // 0x65, 0x00,        // UNIT (None)
// //                 0x81, 0x83,        // INPUT (Const,Var,Abs,Vol)  - 输入(常量, 变量, 绝对值, 易失性)

// //                 // 当前状态集合 (Present Status) - 逻辑集合 - Figure 10 Byte3的低4位
// //                 0x09, 0x02,        // USAGE (PresentStatus) - 修正: 0x02 (原错误: 0x05)
// //                 0xA1, 0x02,        // COLLECTION (Logical)

// //                     // 四个状态位 (各1位)
// //                     0x09, 0x6D,        // USAGE (Used) - 修正: 0x6D (原错误: 0x06)
// //                     0x09, 0x61,        // USAGE (Good) - 修正: 0x61 (原错误: 0x07)
// //                     0x09, 0x65,        // USAGE (Overload) - 修正: 0x65 (原错误: 0x0B)
// //                     0x09, 0x69,        // USAGE (ShutdownImminent) - 修正: 0x69 (原错误: 0x0C)
// //                     0x75, 0x01,        // REPORT_SIZE (1)
// //                     0x95, 0x04,        // REPORT_COUNT (4)
// //                     0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                     0x25, 0x01,        // LOGICAL_MAXIMUM (1)
// //                     // 0x65, 0x00,        // UNIT (None)
// //                     0xB1, 0x83,        // FEATURE (Const,Var,Abs,Vol)  - 特性(常量, 变量, 绝对值, 易失性)

// //                 0xC0,               // END_COLLECTION - 结束当前状态逻辑集合

// //                 // Figure 10 Byte3的高4位Pad字段
// //                 0x75, 0x04,        // REPORT_SIZE (4)
// //                 0x95, 0x01,        // REPORT_COUNT (1)
// //                 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                 0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //                 0x65, 0x00,        // UNIT (None)
// //                 0xB1, 0x83,        // FEATURE (Const,Var,Abs,Vol)  - 特性(常量, 变量, 绝对值, 易失性)


// //                 // 变化状态集合 (Changed Status) - 逻辑集合 - Figure 11
// //                 0x09, 0x03,        // USAGE (ChangedStatus) - 修正: 0x03 (原错误: 0x08)
// //                 0xA1, 0x02,        // COLLECTION (Logical)

// //                     // // PercentLoad字段 (8位) - Figure 11 Byte0
// //                     // 0x09, 0x35,        // USAGE (PercentLoad) - 修正: 0x35
// //                     // 0x75, 0x08,        // REPORT_SIZE (8)
// //                     // 0x95, 0x01,        // REPORT_COUNT (1)
// //                     // 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                     // 0x25, 0xFF,        // LOGICAL_MAXIMUM (255)
// //                     // 0x65, 0x00,        // UNIT (None)
// //                     // 0x81, 0x82,        // INPUT (Data,Var,Abs,Vol) - 数据、变量、绝对值、易失性
                
// //                 // // Report ID 9 - AC Output配置报告   待定
// //                 // 0x85, 0x09,        // REPORT_ID (9) - 报告ID固定为9
                
// //                     // 四个变化状态位 (各2位) - Figure 11 Byte1
// //                     0x09, 0x6D,        // USAGE (Used) - 修正: 0x6D
// //                     0x09, 0x61,        // USAGE (Good) - 修正: 0x61
// //                     0x09, 0x65,        // USAGE (Overload) - 修正: 0x65
// //                     0x09, 0x69,        // USAGE (ShutdownImminent) - 修正: 0x69
// //                     0x75, 0x02,        // REPORT_SIZE (2) - 2位
// //                     0x95, 0x04,        // REPORT_COUNT (4) - 4个字段
// //                     0x15, 0x00,        // LOGICAL_MINIMUM (0) - 逻辑最小值0
// //                     0x25, 0x01,        // LOGICAL_MAXIMUM (1) - 逻辑最大值1
// //                     0x81, 0x82,        // INPUT (Data,Var,Abs,Vol)  - 输入(数据, 变量, 绝对值, 易失性)

// //                 0xC0,               // END_COLLECTION - 结束变化状态逻辑集合

// //             0xC0,               // END_COLLECTION - 结束AC输出物理集合   
             
// //             //==================== [6.13] 直流输入物理集合 ====================
// //             // 电源转换器直流输入包含输入 ID (2)、连接流 ID (2) 以及两个状态项：
// //             // 已使用 (Used) 和良好 (Good)。
// //             // 功能报告 ID 10 和输入报告 ID 10 开始。
// //             0x09, 0x1A,        // USAGE (Input) - 修正: 0x1A (原错误: 0x00)
// //             0xA1, 0x00,        // COLLECTION (Physical)

// //                 // Report ID 10 - DC Input配置报告
// //                 0x85, 0x0A,        // REPORT_ID (10) - 报告ID固定为10

// //                 // InputID和FlowID字段 (各4位，共8位 = 1字节)
// //                 0x09, 0x1B,        // USAGE (InputID) - 修正: 0x1B (原错误: 0x03)
// //                 0x09, 0x1F,        // USAGE (FlowID) - 修正: 0x1F (原错误: 0x04)
// //                 0x75, 0x04,        // REPORT_SIZE (4)
// //                 0x95, 0x02,        // REPORT_COUNT (2)
// //                 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                 0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //                 0x65, 0x00,        // UNIT (None)
// //                 0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)

// //                 // 当前状态集合 (Present Status) - 逻辑集合
// //                 0x09, 0x02,        // USAGE (PresentStatus) - 修正: 0x02 (原错误: 0x05)
// //                 0xA1, 0x02,        // COLLECTION (Logical)

// //                     // 两个状态位 (各1位)
// //                     0x09, 0x06,        // USAGE (Used) - 使用状态 (0x06)
// //                     0x09, 0x07,        // USAGE (Good) - 良好状态 (0x07)
// //                     0x75, 0x01,        // REPORT_SIZE (1) - 1位
// //                     0x95, 0x02,        // REPORT_COUNT (2) - 2个字段
// //                     0x15, 0x00,        // LOGICAL_MINIMUM (0) - 逻辑最小值0
// //                     0x25, 0x01,        // LOGICAL_MAXIMUM (1) - 逻辑最大值1
// //                     0xB1, 0x83,        // FEATURE (Const,Var,Abs,Vol)  - 特性(常量, 变量, 绝对值, 易失性)

// //                     // Pad字段 (6位) - 填充剩余位   待定
// //                     0x75, 0x06,        // REPORT_SIZE (6)
// //                     0x95, 0x01,        // REPORT_COUNT (1)
// //                     0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                     0x25, 0x3F,        // LOGICAL_MAXIMUM (63)
// //                     0x65, 0x00,        // UNIT (None)
// //                     0xB1, 0x83,        // FEATURE (Const,Var,Abs,Vol)  - 特性(常量, 变量, 绝对值, 易失性)

// //                 0xC0,               // END_COLLECTION - 结束当前状态逻辑集合

// //                 // 变化状态集合 (Changed Status) - 逻辑集合
// //                 0x09, 0x03,        // USAGE (ChangedStatus) - 修正: 0x03 (原错误: 0x08)
// //                 0xA1, 0x02,        // COLLECTION (Logical)

// //                     // 两个变化状态位 (各2位)
// //                     0x09, 0x6D,        // USAGE (Used) - 修正: 0x6D
// //                     0x09, 0x61,        // USAGE (Good) - 修正: 0x61
// //                     0x75, 0x02,        // REPORT_SIZE (2)
// //                     0x95, 0x02,        // REPORT_COUNT (2)
// //                     0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                     0x25, 0x01,        // LOGICAL_MAXIMUM (1)
// //                     0x81, 0x82,        // INPUT (Data,Var,Abs,Vol)  - 输入(数据, 变量, 绝对值, 易失性)

// //                     // Pad字段 (4位) - 填充剩余位   待定
// //                     0x75, 0x04,        // REPORT_SIZE (4)
// //                     0x95, 0x01,        // REPORT_COUNT (1)
// //                     0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                     0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //                     0x65, 0x00,        // UNIT (None)
// //                     0x81, 0x82,        // INPUT (Data,Var,Abs,Vol)  - 输入(数据, 变量, 绝对值, 易失性)

// //                 0xC0,               // END_COLLECTION - 结束变化状态逻辑集合

// //             0xC0,               // END_COLLECTION - 结束DC输入物理集合

// //         0xC0,               //END_COLLECTION (End Power Converter collection) 
// // */

// //         // ==================== [6.14] 电源摘要物理集合 ====================
// //         //作为静态数据，电量摘要集合包含
// //         // 电量摘要 ID (1)、已连接流 ID (3)、电源名称、电池存在指示、容量模式（电池容量单位）、电池可充电性指示、
// //         // 电池设计容量、电池设计电压、警告容量限值、电池粒度 1 和 2、产品名称、序列号、电池化学成分和制造商名称。

// //         // 作为动态数据，电量摘要集合包含
// //         // 上次满充电容量、电池当前电压、电池放电电流、当前剩余容量、电池耗尽前的当前运行时间、UPS 输出负载百分比以及六个状态项：
// //         // 对于交流输入：存在；对于电池：低于剩余容量限值、充电中和放电中；对于输出：过载和即将关机。
// //         // 功能报告 ID 11 和输入报告 ID 11 开始。
// //         0x05, 0x84,        // USAGE_PAGE (Power Device)
// //         0x09, 0x24,        // USAGE (PowerSummary) - 修正: 0x24 (原错误: 0x02)
// //         0xA1, 0x00,        // COLLECTION (Physical)

// //         // Report ID 11 - Power Summary配置报告
// //         0x85, 0x0B,        // REPORT_ID (11) - 报告ID固定为11

// //             // Byte1: 高4位FlowID + 低4位PowerSummaryID
// //             0x09, 0x25,        // USAGE (PowerSummaryID) - 修正: 0x25 (原错误: 0x18)
// //             0x09, 0x1F,        // USAGE (FlowID) - 修正: 0x1F (原错误: 0x04)
// //             0x75, 0x04,        // REPORT_SIZE (4)
// //             0x95, 0x02,        // REPORT_COUNT (2)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0x0F,        // LOGICAL_MAXIMUM (15)
// //             0x65, 0x00,        // UNIT (None)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)

// //             // Byte2: iName (8位)
// //             0x09, 0x01,        // USAGE (iName) - 修正: 0x01 (原错误: 0x2C)
// //             0x75, 0x08,        // REPORT_SIZE (8)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0xFF,        // LOGICAL_MAXIMUM (255)
// //             // 0x65, 0x00,        // UNIT (None)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)

// //             // Byte3: 3个状态位 + 5位Pad
// //             0x05, 0x85,        // USAGE_PAGE (Battery System)
// //             0x09, 0x4A,        // USAGE (BatteryPresent) - 正确
// //             0x09, 0x52,        // USAGE (CapacityMode) - 正确
// //             0x09, 0x53,        // USAGE (Rechargable) - 正确
// //             0x75, 0x01,        // REPORT_SIZE (1)
// //             0x95, 0x03,        // REPORT_COUNT (3)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0x01,        // LOGICAL_MAXIMUM (1)
// //             0x65, 0x00,        // UNIT (None)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)

// //             // 5位Pad
// //             0x75, 0x05,        // REPORT_SIZE (5)
// //             // 0x95, 0x01,        // REPORT_COUNT (1)
// //             // 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             // 0x25, 0x1F,        // LOGICAL_MAXIMUM (31)
// //             // 0x65, 0x00,        // UNIT (None)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)

// //             // Bytes 4-15: 4个24位容量值 (DesignCapacity, WarningCapacityLimit, CapacityGranularity1, CapacityGranularity2)
// //             0x05, 0x85,        // USAGE_PAGE (Battery System)
// //             0x09, 0x4B,        // USAGE (DesignCapacity) - 修正: 0x4B (原错误: 0x44)
// //             0x09, 0x47,        // USAGE (WarningCapacityLimit) - 正确
// //             0x09, 0x48,        // USAGE (CapacityGranularity1) - 正确
// //             0x09, 0x49,        // USAGE (CapacityGranularity2) - 正确
// //             0x75, 0x18,        // REPORT_SIZE (24)
// //             0x95, 0x04,        // REPORT_COUNT (4)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x26, 0xFE, 0xFF, 0xFF, // LOGICAL_MAXIMUM (0xFFFFFE)
// //             0x65, 0x10,        // UNIT (AmpSec)
// //             0x55, 0x00,        // UNIT_EXPONENT (0)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)

// //             // Bytes 16-17: ConfigVoltage (16位)
// //             0x05, 0x84,        // USAGE_PAGE (Power Device)
// //             0x09, 0x40,        // USAGE (ConfigVoltage) - 修正: 0x40 (原错误: 0x2A)
// //             0x75, 0x10,        // REPORT_SIZE (16)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x26, 0xFE, 0xFF,  // LOGICAL_MAXIMUM (0xFFFE)
// //             0x65, 0x24,        // UNIT (Volt)
// //             0x55, 0x05,        // UNIT_EXPONENT (5)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)

// //             // Bytes 18-21: 4个8位字符串索引
// //             0x05, 0x84,        // USAGE_PAGE (Power Device) - 电力设备页面
// //             0x09, 0xFE,        // USAGE (iProduct) - 修正: 0xFE (原错误: 0x2E)
// //             0x09, 0xFF,        // USAGE (iSerialNumber) - 修正: 0xFF (原错误: 0x2F)
// //             0x05, 0x85,        // USAGE_PAGE (Battery System)
// //             0x09, 0x5A,        // USAGE (iDeviceChemistry) - 正确
// //             0x09, 0x59,        // USAGE (iManufacturerName) - 正确
// //             0x75, 0x08,        // REPORT_SIZE (8)
// //             0x95, 0x04,        // REPORT_COUNT (4)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0xFF,        // LOGICAL_MAXIMUM (255)
// //             0x65, 0x00,        // UNIT (None)
// //             0xB1, 0x82,        // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)

// //             // Byte1: PercentLoad (8位)
// //             0x05, 0x84,        // USAGE_PAGE (Power Device)
// //             0x09, 0x35,        // USAGE (PercentLoad) - 修正: 0x35 (原错误: 0x0A)
// //             0x75, 0x08,        // REPORT_SIZE (8)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0xFE,        // LOGICAL_MAXIMUM (254)
// //             0x65, 0x00,        // UNIT (None)
// //             0x81, 0x83,        // INPUT (Const,Var,Abs,Vol)  - 输入(常量, 变量, 绝对值, 易失性)
            
// //             // Input Report部分 - Byte1: PercentLoad (8位)

// //             // Bytes 22-23: Voltage (16位)
// //             0x05, 0x84,        // USAGE_PAGE (Power Device)
// //             0x09, 0x30,        // USAGE (Voltage) - 正确
// //             0x75, 0x10,        // REPORT_SIZE (16)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x26, 0xFE, 0xFF,  // LOGICAL_MAXIMUM (0xFFFE)
// //             0x65, 0x24,        // UNIT (Volt)
// //             0x55, 0x05,        // UNIT_EXPONENT (5)
// //             0xB1, 0x83,        // FEATURE (Const,Var,Abs,Vol)  - 特性(常量, 变量, 绝对值, 易失性)

// //             // Bytes 24-25: Current (16位)  for Figure 12
// //             0x09, 0x31,        // USAGE (Current) - 正确
// //             0x75, 0x10,        // REPORT_SIZE (16)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x65, 0x21,        // UNIT (Amp)
// //             0x55, 0xFE,        // UNIT_EXPONENT (-2)
// //             0xB1, 0x83,        // FEATURE (Const,Var,Abs,Vol)  - 特性(常量, 变量, 绝对值, 易失性)

// //             // Bytes 26-28: FullChargeCapacity (24位)
// //             0x05, 0x85,        // USAGE_PAGE (Battery System)
// //             0x09, 0x42,        // USAGE (FullChargeCapacity) - 正确
// //             0x75, 0x18,        // REPORT_SIZE (24)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x26, 0xFE, 0xFF, 0xFF, // LOGICAL_MAXIMUM (0xFFFFFE)
// //             0x65, 0x10,        // UNIT (AmpSec)
// //             0x55, 0x00,        // UNIT_EXPONENT (0)
// //             0xB1, 0x83,        // FEATURE (Const,Var,Abs,Vol)  - 特性(常量, 变量, 绝对值, 易失性)

// //             // Input Report部分 - Figure 13

// //             // Byte1: PercentLoad (8位)
// //             0x05, 0x84,        // USAGE_PAGE (Power Device)
// //             0x09, 0x35,        // USAGE (PercentLoad) - 修正: 0x35 (原错误: 0x0A)
// //             0x75, 0x08,        // REPORT_SIZE (8)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0xFE,        // LOGICAL_MAXIMUM (254)
// //             0x65, 0x00,        // UNIT (None)
// //             0x81, 0x83,        // INPUT (Const,Var,Abs,Vol)  - 输入(常量, 变量, 绝对值, 易失性)

// //             // // Bytes 2-4: RemainingCapacity (24位)   For Figure 13 Byte 2-4
// //             // 0x05, 0x85,        // USAGE_PAGE (Battery System) - 电池系统页面
// //             // 0x09, 0x66,        // USAGE (RemainingCapacity) - 正确
// //             // 0x75, 0x18,        // REPORT_SIZE (24) - 24位
// //             // 0x95, 0x01,        // REPORT_COUNT (1) - 1个字段
// //             // 0x15, 0x00,        // LOGICAL_MINIMUM (0) - 逻辑最小值0
// //             // 0x26, 0xFE, 0xFF, 0xFF, // LOGICAL_MAXIMUM (0xFFFFFE) - 逻辑最大值
// //             // 0x65, 0x10,        // UNIT (AmpSec) - 安培秒单位
// //             // 0x55, 0x00,        // UNIT_EXPONENT (0) - 单位指数0
// //             // 0x82, 0x02, 0x01,  // INPUT (Const,Var,Abs,Vol) - 常量、变量、绝对值、易失性

// //             // RemainingCapacity字段 (8位) - 百分比剩余容量  For 6.14 Present
// //             0x05, 0x85,        // USAGE_PAGE (Battery System)
// //             0x09, 0x66,        // USAGE (RemainingCapacity) - 正确
// //             0x75, 0x08,        // REPORT_SIZE (8)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x25, 0x64,        // LOGICAL_MAXIMUM (100)
// //             // 0x65, 0x00,        // UNIT (None)
// //             0x81, 0x83,        // INPUT (Const,Var,Abs,Vol)  - 输入(常量, 变量, 绝对值, 易失性)

// //             // Bytes 5-6: RunTimeToEmpty (16位)
// //             0x09, 0x45,        // USAGE (RunTimeToEmpty) - 正确
// //             0x75, 0x10,        // REPORT_SIZE (16)
// //             0x95, 0x01,        // REPORT_COUNT (1)
// //             0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //             0x26, 0xFE, 0xFF,  // LOGICAL_MAXIMUM (0xFFFE)
// //             0x65, 0xE3,        // UNIT (second)
// //             0x55, 0x00,        // UNIT_EXPONENT (0)
// //             0x81, 0x83,        // INPUT (Const,Var,Abs,Vol)  - 输入(常量, 变量, 绝对值, 易失性)

// //             // Byte7: 6个状态位 + 2位Pad
// //             0x09, 0x02,        // USAGE (PresentStatus) - 修正: 0x02 (原错误: 0x05)
// //             0xA1, 0x02,        // COLLECTION (Logical)

// //                 0x05, 0x85,        // USAGE_PAGE (Battery System)
// //                 0x09, 0x4D,        // USAGE (ACPresent) - 正确
// //                 0x09, 0x44,        // USAGE (Charging) - 正确
// //                 0x09, 0x4B,        // USAGE (Discharging) - 正确
// //                 0x09, 0x4C,        // USAGE (BelowRemainingcapacityLimit) - 正确
// //                 0x05, 0x84,        // USAGE_PAGE (Power Device)
// //                 0x09, 0x69,        // USAGE (ShutdownImminent) - 修正: 0x69 (原错误: 0x0C)
// //                 0x09, 0x65,        // USAGE (Overload) - 修正: 0x65 (原错误: 0x0B)
// //                 0x75, 0x01,        // REPORT_SIZE (1)
// //                 0x95, 0x06,        // REPORT_COUNT (6)
// //                 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                 0x25, 0x01,        // LOGICAL_MAXIMUM (1)
// //                 0x81, 0x83,        // INPUT (Const,Var,Abs,Vol)  - 输入(常量, 变量, 绝对值, 易失性)

// //                 // 2位Pad  填充  待定
// //                 0x75, 0x02,        // REPORT_SIZE (2)
// //                 0x95, 0x01,        // REPORT_COUNT (1)
// //                 0x15, 0x00,        // LOGICAL_MINIMUM (0)
// //                 0x25, 0x03,        // LOGICAL_MAXIMUM (3)
// //                 0x65, 0x00,        // UNIT (None)
// //                 0x81, 0x83,        // INPUT (Const,Var,Abs,Vol)  - 输入(常量, 变量, 绝对值, 易失性)

// //             0xC0,               // END_COLLECTION - 结束当前状态逻辑集合

// //         0xC0,               // END_COLLECTION - 结束电源概要物理集合

// //     0xC0               // END_COLLECTION (Application)

// //     // //用途: 通常用于只读的状态标志，这些标志是固定的常量值，但可能随时发生变化。
// //     // 0x81, 0x83,  // INPUT (Const,Var,Abs,Vol)  - 输入(常量, 变量, 绝对值, 易失性)
// //     // //用途: 用于动态变化的测量值，如电压、电流等实时数据。
// //     // 0x81, 0x82,  // INPUT (Data,Var,Abs,Vol)  - 输入(数据, 变量, 绝对值, 易失性)
// //     // //用途: 用于只读的配置或状态信息，这些信息是固定的但可能变化。
// //     // 0xB1, 0x83,  // FEATURE (Const,Var,Abs,Vol)  - 特性(常量, 变量, 绝对值, 易失性)
// //     // //用途: 用于相对稳定的配置信息，如设备ID、固定参数等。
// //     // 0xB1, 0x82,  // FEATURE (Const,Var,Abs)  - 特性(常量, 变量, 绝对值)
// // };

// // A.1 Device Descriptor  USB设备描述符 
// tusb_desc_device_t descriptor_dev = {
//     .bLength = sizeof(tusb_desc_device_t),  //Numeric expression specifying the size of this descriptor.
//     .bDescriptorType = TUSB_DESC_DEVICE,   //Device descriptor type (assigned by USB).
//     .bcdUSB = 0x0200,       //USB HID Specification Release 1.0
//     .bDeviceClass = 0x00,   //Class code (assigned by USB). Note that the HID class is defined in the Interface descriptor.
//     .bDeviceSubClass = 0x00,  //Subclass code (assigned by USB). These codes are qualified by the value of the bDeviceClass field.
//     .bDeviceProtocol = 0x00,  //Protocol code. These codes are qualified by the value of the bDeviceSubclass field
//     .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,  //Maximum packet size for endpoint zero (only 8, 16, 32, or 64 are valid).
//     .idVendor = 0x04d8,    //Vendor ID (assigned by USB).
//     .idProduct = 0xd005,   //Product ID (assigned by manufacturer).
//     .bcdDevice = 0x0100,   //Device release number (assigned by manufacturer
//     .iManufacturer = IMANUFACTURER,  //Index of String descriptor describing manufacturer.
//     .iProduct = IPRODUCT,       //Index of String descriptor describing product.
//     .iSerialNumber = ISERIAL,   //Index of String descriptor describing the device’s serial number.
//     .bNumConfigurations = 0x01  //Number of possible configurations
// };

// // A.3 + A.4 + A.5 HID描述符
// const uint8_t hid_interface_desc[] = {
    
//     // A.3 Interface Descriptor 
//     // 核心作用：告诉主机“这里有一个接口，它是HID类的”。
//     0x09,                    // bLength, Size of this descriptor in bytes.
//     0x04,                    // bDescriptorType, Interface descriptor type (assigned by USB).
//     0x00,                    // bInterfaceNumber
//     0x00,                    // bAlternateSetting
//     0x01,                    // bNumEndpoints, Number of endpoints used by this interface
//     0x03,                    // bInterfaceClass (HID), Class code (HID code assigned by USB).
//     0x00,                    // bInterfaceSubClass, 0 No subclass; 1 Boot Interface subclass
//     0x00,                    // bInterfaceProtocol, 0 None
//     0x00,                    // iInterface, Index of string descriptor describing this interface
    
//     // A.5 HID Descriptor
//     0x09,                    // bLength, Size of this descriptor in bytes
//     0x21,                    // bDescriptorType (HID), HID descriptor type (assigned by USB).
//     0x11, 0x01,              // bcdHID (1.11), HID Class Specification release number in binarycoded decimal.
//     0x00,                    // bCountryCode, Hardware target country.
//     0x01,                    // bNumDescriptors, Number of HID class descriptors to follow
//     0x22,                    // bDescriptorType (Report),Report descriptor type.
//     LO8(sizeof(hid_report_descriptor)),  // wDescriptorLength (low), Total length of Report descriptor.
//     HI8(sizeof(hid_report_descriptor)),  // wDescriptorLength (high)
    
//     // A.4 Endpoint Descriptor
//     0x07,                    // bLength, Size of this descriptor in bytes.
//     0x05,                    // bDescriptorType, Endpoint descriptor type (assigned by USB).
//     0x81,                    // bEndpointAddress (IN endpoint 1)
//     // The address of the endpoint on the USB device described by this descriptor
//     // Bit 0..3 The endpoint number
//     // Bit 4..6 Reserved, reset to zero
//     // Bit 7    Direction, ignored for Control endpoints:
//     // 0        OUT endpoint
//     // 1        IN endpoint
//     0x03,                    // bmAttributes (Interrupt)
//     // This field describes the endpoint’s attributes when it is configured using the bConfigurationValue
//     // Bit 0..1 Transfer type:
//     // 00 Control
//     // 01 Isochronous
//     // 10 Bulk
//     // 11 Interrupt
//     LO8(CFG_TUD_HID_EP_BUFSIZE),  // wMaxPacketSize (low)
//     HI8(CFG_TUD_HID_EP_BUFSIZE),  // wMaxPacketSize (high)
//     0x0A                     // bInterval (10ms) , Interval for polling endpoint for data transfers, expressed in milliseconds.
// };

// // A.2 Configuration Descriptor  配置描述符
// #define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + sizeof(hid_interface_desc))
// uint8_t const desc_configuration[] = {

//     // 配置描述符，具体配置顺序由宏自行调配
//     TUD_CONFIG_DESCRIPTOR(
//         // bLength          由宏自动生成
//         // bDescriptorType  由宏自动生成
//         1,      // bConfigurationValue , Value to use as an argument to Set Configuration to select this configuration.
//         1,      // bNumInterfaces ,Number of interfaces supported by this configuration.
//         0,      // iConfiguration , Index of string descriptor describing this configuration.
//         TUSB_DESC_TOTAL_LEN, // wTotalLength 
//         // Total length of data returned for this configuration.
//         // Includes the combined length of all returned
//         // descriptors (configuration, interface, endpoint, and
//         // HID) returned for this configuration. This value
//         // includes the HID descriptor but none of the other
//         // HID class descriptors (report or designator).
//         TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, // bmAttributes ,Bus Powered + Self Powered
//         // 7 Bus Powered
//         // 6 Self Powered
//         // 5 Remote Wakeup
//         // 4..0 Reserved (reset to 0)
//         100     // bMaxPower 
//     ),
    
//     // HID接口描述符
//     hid_interface_desc[0], hid_interface_desc[1], hid_interface_desc[2],
//     hid_interface_desc[3], hid_interface_desc[4], hid_interface_desc[5],
//     hid_interface_desc[6], hid_interface_desc[7], hid_interface_desc[8],
//     hid_interface_desc[9], hid_interface_desc[10], hid_interface_desc[11],
//     hid_interface_desc[12], hid_interface_desc[13], hid_interface_desc[14],
//     hid_interface_desc[15], hid_interface_desc[16], hid_interface_desc[17],
//     hid_interface_desc[18], hid_interface_desc[19], hid_interface_desc[20],
//     hid_interface_desc[21], hid_interface_desc[22], hid_interface_desc[23],
//     hid_interface_desc[24]
// };

// // 字符串描述符
// const char* descriptor_str[] = {
//     [0] = "en",
//     [IMANUFACTURER] = "Dinghugang",
//     [IPRODUCT] = "My UPS",
//     [ISERIAL] = "383503417",
//     [IDEVICECHEMISTRY] = "Li-ion"
// };

// // TinyUSB回调函数
// uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {

//     ESP_LOGW(TAG, "return hid_report_descriptor !");

//     return hid_report_descriptor;
// }


// uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
//                                hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) 
// {
//     ESP_LOGI(TAG, "Get report: ID=0x%02X, Type=%d, ReqLen=%d", report_id, report_type, reqlen);

//     // 设置报告ID作为第一个字节
//     buffer[0] = report_id;

//     if (report_type == HID_REPORT_TYPE_FEATURE) {
//         switch(report_id) {
//             // ==================== Report ID 1: 主交流输入配置 ====================
//             case 0x01:
//                 if (reqlen >= 5) {
//                     buffer[1] = 0x01; // FlowID=1
//                     buffer[2] = 0x00; // iName
//                     buffer[3] = 220;  // ConfigVoltage
//                     buffer[4] = 50;   // ConfigFrequency
//                     return 5;
//                 }
//                 break;

//             // ==================== Report ID 2: 备份直流流配置 ====================
//             case 0x02:
//                 if (reqlen >= 6) {
//                     buffer[1] = 0x02; // FlowID=2
//                     buffer[2] = 0x00; // iName
//                     uint16_t dc_voltage = 480;
//                     buffer[3] = dc_voltage & 0xFF;
//                     buffer[4] = (dc_voltage >> 8) & 0xFF;
//                     buffer[5] = 0;    // ConfigFrequency
//                     return 6;
//                 }
//                 break;

//             // ==================== Report ID 3: 输出交流流配置 ====================
//             case 0x03:
//                 if (reqlen >= 7) {
//                     buffer[1] = 0x03; // FlowID=3
//                     buffer[2] = 0x00; // iName
//                     buffer[3] = 220;  // ConfigVoltage
//                     buffer[4] = 50;   // ConfigFrequency
//                     uint16_t apparent_power = 5000;
//                     buffer[5] = apparent_power & 0xFF;
//                     buffer[6] = (apparent_power >> 8) & 0xFF;
//                     return 7;
//                 }
//                 break;

//             // ==================== Report ID 4: 电池系统配置 ====================
//             case 0x04:
//                 if (reqlen >= 2) {
//                     buffer[1] = 0x01; // BatterySystemID=1
//                     return 2;
//                 }
//                 break;

//             // ==================== Report ID 5: 充电器配置 ====================
//             case 0x05:
//                 if (reqlen >= 2) {
//                     buffer[1] = 0x01; // ChargerID=1
//                     return 2;
//                 }
//                 break;

//             // ==================== Report ID 6: 电池配置 ====================
//             case 0x06:
//                 if (reqlen >= 12) {
//                     buffer[1] = 0x01; // BatteryID=1
//                     buffer[2] = 0x00; // CapacityMode Pad
                    
//                     uint32_t design_capacity = 360000;
//                     buffer[3] = design_capacity & 0xFF;
//                     buffer[4] = (design_capacity >> 8) & 0xFF;
//                     buffer[5] = (design_capacity >> 16) & 0xFF;
                    
//                     uint16_t config_voltage = 1500;
//                     buffer[6] = config_voltage & 0xFF;
//                     buffer[7] = (config_voltage >> 8) & 0xFF;
                    
//                     uint32_t remaining_capacity = 306000;
//                     buffer[8] = remaining_capacity & 0xFF;
//                     buffer[9] = (remaining_capacity >> 8) & 0xFF;
//                     buffer[10] = (remaining_capacity >> 16) & 0xFF;
                    
//                     buffer[11] = 0x09; // PresentStatus
//                     return 12;
//                 }
//                 break;

//             // // ==================== Report ID 8: 电源转换器配置 ====================
//             // case 0x08:
//             //     if (reqlen >= 2) {
//             //         buffer[1] = 0x01; // PowerConverterID=1
//             //         return 2;
//             //     }
//             //     break;

//             // ==================== Report ID 11: 电源摘要配置 ====================
//             case 0x0B:
//                 if (reqlen >= 29) {
//                     buffer[1] = 0x31; // PowerSummaryID=1 + FlowID=3
//                     buffer[2] = 0x00; // iName
//                     buffer[3] = 0x07; // Battery status
                    
//                     uint32_t design_capacity = 360000;
//                     buffer[4] = design_capacity & 0xFF;
//                     buffer[5] = (design_capacity >> 8) & 0xFF;
//                     buffer[6] = (design_capacity >> 16) & 0xFF;
                    
//                     uint32_t warning_capacity = 72000;
//                     buffer[7] = warning_capacity & 0xFF;
//                     buffer[8] = (warning_capacity >> 8) & 0xFF;
//                     buffer[9] = (warning_capacity >> 16) & 0xFF;
                    
//                     buffer[10] = 0x01; // CapacityGranularity1
//                     buffer[11] = 0x00;
//                     buffer[12] = 0x00;
                    
//                     buffer[13] = 0x0A; // CapacityGranularity2
//                     buffer[14] = 0x00;
//                     buffer[15] = 0x00;
                    
//                     uint16_t config_voltage = 1500;
//                     buffer[16] = config_voltage & 0xFF;
//                     buffer[17] = (config_voltage >> 8) & 0xFF;
                    
//                     buffer[18] = 0x01; // iProduct
//                     buffer[19] = 0x02; // iSerialNumber
//                     buffer[20] = 0x03; // iDeviceChemistry
//                     buffer[21] = 0x04; // iManufacturerName
                    
//                     uint16_t voltage = 1500;
//                     buffer[22] = voltage & 0xFF;
//                     buffer[23] = (voltage >> 8) & 0xFF;
                    
//                     int16_t current = 500;
//                     buffer[24] = current & 0xFF;
//                     buffer[25] = (current >> 8) & 0xFF;
                    
//                     buffer[26] = design_capacity & 0xFF;
//                     buffer[27] = (design_capacity >> 8) & 0xFF;
//                     buffer[28] = (design_capacity >> 16) & 0xFF;
                    
//                     return 29;
//                 }
//                 break;

//             default:
//                 ESP_LOGW(TAG, "Unknown feature report ID: 0x%02X", report_id);
//                 return 0;
//         }
//     }
//     else if (report_type == HID_REPORT_TYPE_INPUT) {
//         switch(report_id) {
//             // ==================== Report ID 4: 电池系统输入状态 ====================
//             case 0x04:
//                 if (reqlen >= 2) {
//                     buffer[1] = 0x00; // ChangedStatus
//                     return 2;
//                 }
//                 break;

//             // ==================== Report ID 6: 电池输入状态 ====================
//             case 0x06:
//                 if (reqlen >= 2) {
//                     buffer[1] = 0x09; // ChangedStatus
//                     return 2;
//                 }
//                 break;

//             // ==================== Report ID 11: 电源摘要输入数据 ====================
//             case 0x0B:
//                 if (reqlen >= 6) {
//                     buffer[1] = 30;   // PercentLoad
//                     buffer[2] = 85;   // RemainingCapacity
                    
//                     uint16_t runtime = 7200;
//                     buffer[3] = runtime & 0xFF;
//                     buffer[4] = (runtime >> 8) & 0xFF;
                    
//                     buffer[5] = 0x09; // StatusBits
//                     return 6;
//                 }
//                 break;

//             default:
//                 ESP_LOGW(TAG, "Unknown input report ID: 0x%02X", report_id);
//                 return 0;
//         }
//     }

//     return 0;
// }




// void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
//                            hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) 
// {
//     ESP_LOGI(TAG, "Set report: ID=0x%02X, Type=%u, Size=%u", report_id, report_type, bufsize);

//     if (report_type != HID_REPORT_TYPE_FEATURE || bufsize < 1) {
//         return;
//     }

//     uint8_t const* data = buffer + 1;
//     uint16_t data_size = bufsize - 1;

//     switch(report_id) {
//         // ==================== Report ID 1: 主交流输入配置 ====================
//         case 0x01: // 主交流输入配置
//             if (data_size >= 4) {
//                 uint8_t config_voltage = data[2];
//                 uint8_t config_frequency = data[3];
//                 ESP_LOGI(TAG, "AC Input Config - Voltage:%dV, Frequency:%dHz", 
//                         config_voltage * 128, config_frequency);
//                 // 硬件控制代码
//             }
//             break;

//         // ==================== Report ID 2: 备份直流流配置 ====================
//         case 0x02: // 备份直流流配置
//             if (data_size >= 5) {
//                 uint16_t config_voltage = (data[3] << 8) | data[2];
//                 ESP_LOGI(TAG, "DC Backup Config - Voltage:%dmV", config_voltage * 32);
//                 // 硬件控制代码
//             }
//             break;

//         case 0x03: // 输出交流流配置
//             if (data_size >= 6) {
//                 uint8_t config_voltage = data[2];
//                 uint8_t config_frequency = data[3];
//                 uint16_t config_apparent_power = (data[5] << 8) | data[4];
//                 ESP_LOGI(TAG, "AC Output Config - Voltage:%dV, Freq:%dHz, Power:%dVA", 
//                         config_voltage * 128, config_frequency, config_apparent_power * 128);
//                 // 硬件控制代码
//             }
//             break;

//         case 0x06: // 电池配置
//             if (data_size >= 11) {
//                 uint16_t config_voltage = (data[6] << 8) | data[5];
//                 ESP_LOGI(TAG, "Battery Config - Voltage:%dmV", config_voltage * 32);
//                 // 硬件控制代码
//             }
//             break;

//         case 0x0B: // 电源摘要配置
//             if (data_size >= 28) {
//                 uint16_t config_voltage = (data[16] << 8) | data[15];
//                 ESP_LOGI(TAG, "Power Summary Config - Voltage:%dmV", config_voltage * 32);
//                 // 硬件控制代码
//             }
//             break;

//         default:
//             ESP_LOGW(TAG, "Unknown report ID: 0x%02X", report_id);
//             break;
//     }
// }

// uint8_t tud_hid_get_protocol_cb(uint8_t instance) {
//     return HID_PROTOCOL_NONE;
// }

// void tud_hid_set_protocol_cb(uint8_t instance, uint8_t protocol) {
//     ESP_LOGI(TAG, "Protocol set to: %u", protocol);
// }

// // 更新UPS状态
// static void update_ups_state(void) {
//     // 模拟AC电源断开/连接（120秒切换一次）
//     static uint32_t ac_timer = 0;
//     if (xTaskGetTickCount() - ac_timer > pdMS_TO_TICKS(120000)) {
//         UPS.ACPresent ^= 1;
//         ac_timer = xTaskGetTickCount();
        
//         if (UPS.ACPresent) {
//             UPS.Charging = 1;
//             UPS.Discharging = 0;
//             ESP_LOGI(TAG, "AC connected - charging");
//         } else {
//             UPS.Charging = 0;
//             UPS.Discharging = 1;
//             ESP_LOGI(TAG, "AC disconnected - discharging");
//         }
//     }

//     // 模拟电池容量变化
//     if (UPS.Charging && battery_capacity < 100) {
//         battery_capacity++;
//         if (battery_capacity >= 100) {
//             UPS.FullyCharged = 1;
//             UPS.Charging = 0;
//         }
//     } else if (UPS.Discharging && battery_capacity > 0) {
//         battery_capacity--;
//         UPS.FullyCharged = 0;
//         if (battery_capacity <= 0) {
//             UPS.FullyDischarged = 1;
//         }
//     }

//     // 更新剩余时间
//     runtime_to_empty = (battery_capacity * 120) / 100;
// }

// // USB HID初始化
// static void usb_hid_init(void) {
//     const tinyusb_config_t tusb_cfg = {
//         .device_descriptor = &descriptor_dev,
//         .configuration_descriptor = desc_configuration,
//         .string_descriptor = descriptor_str,
//         .string_descriptor_count = sizeof(descriptor_str) / sizeof(descriptor_str[0]),
//         .external_phy = false,
//     };

//     ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
//     ESP_LOGI(TAG, "TinyUSB initialized");

//     while (!tud_mounted()) {
//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
//     ESP_LOGI(TAG, "USB connected");
// }

// // 主函数
// void app_main(void) {
//     ESP_LOGI(TAG, "DC UPS HID Device starting");

//     // 初始化NVS
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     // 初始化USB HID
//     usb_hid_init();
    
//     // 主循环
//     while (1) {
//         update_ups_state();
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }


// void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
//                            hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) 
// {
//     ESP_LOGI(TAG, "Set report: ID=0x%02X, Type=%u, Size=%u", report_id, report_type, bufsize);

//     if (report_type != HID_REPORT_TYPE_FEATURE || bufsize < 1) {
//         return;
//     }

//     // 跳过报告ID字节（buffer[0]），从buffer[1]开始是实际数据
//     uint8_t const* data = buffer + 1;
//     uint16_t data_size = bufsize - 1;

//     switch(report_id) {
//         // ==================== Report ID 1: 主交流输入配置 ====================
//         case 0x01:
//             if (data_size >= 4) {
//                 uint8_t flow_id = data[0] & 0x0F;
//                 uint8_t i_name = data[1];
//                 uint8_t config_voltage = data[2];
//                 uint8_t config_frequency = data[3];
                
//                 ESP_LOGI(TAG, "AC Input Config - FlowID:%d, iName:%d, Voltage:%dV, Frequency:%dHz", 
//                         flow_id, i_name, config_voltage * 128, config_frequency);
                
//                 // 直接应用配置到硬件
//                 float actual_voltage = config_voltage * 128.0f / 1000.0f;
//                 ESP_LOGI(TAG, "Applying AC Input Config: %.1fV, %dHz", actual_voltage, config_frequency);
//                 // 硬件控制代码: set_ac_input_voltage(actual_voltage);
//                 // 硬件控制代码: set_ac_input_frequency(config_frequency);
//             }
//             break;

//         // ==================== Report ID 2: 备份直流流配置 ====================
//         case 0x02:
//             if (data_size >= 5) {
//                 uint8_t flow_id = data[0] & 0x0F;
//                 uint8_t i_name = data[1];
//                 uint16_t config_voltage = (data[3] << 8) | data[2];
//                 uint8_t config_frequency = data[4];
                
//                 ESP_LOGI(TAG, "DC Backup Config - FlowID:%d, iName:%d, Voltage:%dmV, Frequency:%dHz", 
//                         flow_id, i_name, config_voltage * 32, config_frequency);
                
//                 // 直接应用配置到硬件
//                 float actual_voltage = config_voltage * 32.0f / 1000.0f;
//                 ESP_LOGI(TAG, "Applying DC Backup Config: %.2fV, %dHz", actual_voltage, config_frequency);
//                 // 硬件控制代码: set_dc_backup_voltage(actual_voltage);
//                 // 硬件控制代码: set_dc_backup_frequency(config_frequency);
//             }
//             break;

//         // ==================== Report ID 3: 输出交流流配置 ====================
//         case 0x03:
//             if (data_size >= 6) {
//                 uint8_t flow_id = data[0] & 0x0F;
//                 uint8_t i_name = data[1];
//                 uint8_t config_voltage = data[2];
//                 uint8_t config_frequency = data[3];
//                 uint16_t config_apparent_power = (data[5] << 8) | data[4];
                
//                 ESP_LOGI(TAG, "AC Output Config - FlowID:%d, Voltage:%dV, Freq:%dHz, Power:%dVA", 
//                         flow_id, config_voltage * 128, config_frequency, config_apparent_power * 128);
                
//                 // 直接应用配置到硬件
//                 float actual_voltage = config_voltage * 128.0f / 1000.0f;
//                 float actual_power = config_apparent_power * 128.0f;
//                 ESP_LOGI(TAG, "Applying AC Output Config: %.1fV, %dHz, %.0fVA", actual_voltage, config_frequency, actual_power);
//                 // 硬件控制代码: set_ac_output_voltage(actual_voltage);
//                 // 硬件控制代码: set_ac_output_frequency(config_frequency);
//                 // 硬件控制代码: set_ac_output_power_limit(actual_power);
//             }
//             break;

//         // ==================== Report ID 4: 电池系统配置 ====================
//         case 0x04:
//             if (data_size >= 1) {
//                 uint8_t battery_system_id = data[0] & 0x0F;
//                 ESP_LOGI(TAG, "Battery System ID set to: %d", battery_system_id);
//                 // 通常ID是只读的，这里只是日志记录
//             }
//             break;

//         // ==================== Report ID 5: 充电器配置 ====================
//         case 0x05:
//             if (data_size >= 1) {
//                 uint8_t charger_id = data[0] & 0x0F;
//                 ESP_LOGI(TAG, "Charger ID set to: %d", charger_id);
//                 // 通常ID是只读的
//             }
//             break;

//         // ==================== Report ID 6: 电池配置 ====================
//         case 0x06:
//             if (data_size >= 11) {
//                 uint8_t battery_id = data[0] & 0x0F;
//                 uint8_t capacity_mode = data[0] >> 4;
//                 uint8_t cap_mode_pad = data[1] >> 1;
                
//                 uint32_t design_capacity = (data[4] << 16) | (data[3] << 8) | data[2];
//                 uint16_t config_voltage = (data[6] << 8) | data[5];
//                 uint32_t remaining_capacity = (data[9] << 16) | (data[8] << 8) | data[7];
//                 uint8_t present_status = data[10];
                
//                 ESP_LOGI(TAG, "Battery Config - ID:%d, Design:%uAs, Voltage:%dmV, Remain:%umAh",
//                         battery_id, design_capacity, config_voltage * 32, remaining_capacity);
                
//                 // 直接应用配置到硬件
//                 float actual_voltage = config_voltage * 32.0f / 1000.0f;
//                 ESP_LOGI(TAG, "Applying Battery Config: %.2fV", actual_voltage);
//                 // 硬件控制代码: set_battery_voltage_limit(actual_voltage);
//                 // 硬件控制代码: update_battery_design_capacity(design_capacity);
//             }
//             break;

//         // ==================== Report ID 8: 电源转换器配置 ====================
//         case 0x08:
//             if (data_size >= 1) {
//                 uint8_t power_converter_id = data[0] & 0x0F;
//                 ESP_LOGI(TAG, "Power Converter ID set to: %d", power_converter_id);
//                 // 通常ID是只读的
//             }
//             break;

//         // ==================== Report ID 11: 电源摘要配置 ====================
//         case 0x0B:
//             if (data_size >= 28) {
//                 uint8_t summary_id = data[0] & 0x0F;
//                 uint8_t flow_id = data[0] >> 4;
//                 uint8_t i_name = data[1];
                
//                 uint8_t battery_status = data[2];
//                 uint32_t design_capacity = (data[5] << 16) | (data[4] << 8) | data[3];
//                 uint32_t warning_capacity = (data[8] << 16) | (data[7] << 8) | data[6];
//                 uint32_t capacity_gran1 = (data[11] << 16) | (data[10] << 8) | data[9];
//                 uint32_t capacity_gran2 = (data[14] << 16) | (data[13] << 8) | data[12];
//                 uint16_t config_voltage = (data[16] << 8) | data[15];
//                 uint8_t i_product = data[17];
//                 uint8_t i_serial = data[18];
//                 uint8_t i_chemistry = data[19];
//                 uint8_t i_manufacturer = data[20];
//                 uint16_t voltage = (data[22] << 8) | data[21];
//                 uint16_t current = (data[24] << 8) | data[23];
//                 uint32_t full_charge_capacity = (data[27] << 16) | (data[26] << 8) | data[25];
                
//                 ESP_LOGI(TAG, "Power Summary - SID:%d, FID:%d, Voltage:%dmV, Design:%uAs",
//                         summary_id, flow_id, config_voltage * 32, design_capacity);
                
//                 // 直接应用配置到硬件
//                 float actual_voltage = config_voltage * 32.0f / 1000.0f;
//                 ESP_LOGI(TAG, "Applying Power Summary Config: %.2fV", actual_voltage);
//                 // 硬件控制代码: set_system_voltage(actual_voltage);
//                 // 硬件控制代码: update_power_summary_capacity(design_capacity, warning_capacity);
//                 // 硬件控制代码: update_string_descriptors(i_product, i_serial, i_chemistry, i_manufacturer);
//             }
//             break;

//         default:
//             ESP_LOGW(TAG, "Unknown report ID: 0x%02X", report_id);
//             break;
//     }
// }























//当前版本 NAS能识别，但是状态异常，怀疑是配置问题
// #include <stdio.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "esp_err.h"
// #include "nvs_flash.h"
// #include "tusb.h"
// #include "class/hid/hid.h"
// #include "tinyusb.h"
// #include "device/usbd.h"  // 包含 tud_hid_report_received 的定义

// static const char *TAG = "HID_UPS_Device";

// // -------------------------- 1. 基础宏定义（解决编译错误） --------------------------
// #define LO8(x) ((x) & 0xFF)       // 提取16位数值的低字节
// #define HI8(x) ((x) >> 8)         // 提取16位数值的高字节

// // -------------------------- 2. HID 报告 ID 定义 --------------------------
// // 注：HID 报告 ID（Report ID）是 HID 协议中用于区分不同类型数据报告的标识，
// // 主机通过指定不同的 Report ID，可向 UPS 设备请求/读取对应的电池、设备信息

// #define HID_PD_IPRODUCT              0x01  // 产品名称报告ID：用于读取UPS设备的产品名称（如"ESP32 HID UPS"）
// #define HID_PD_SERIAL                0x02  // 序列号报告ID：用于读取UPS设备的序列号（如"ESP32-3417"）
// #define HID_PD_MANUFACTURER          0x03  // 制造商报告ID：用于读取UPS设备的制造商信息（如"ESP32 UPS"）
// #define HID_PD_IDEVICECHEMISTRY      0x04  // 设备化学类型报告ID：用于读取电池的化学类型（如"Li-ion"锂电池、"Pb"铅酸电池）
// #define HID_PD_RECHARGEABLE          0x06  // 充电属性报告ID：用于读取电池是否支持充电（如0x01=可充电，0x00=不可充电）
// #define HID_PD_PRESENTSTATUS         0x07  // 当前状态报告ID：用于读取UPS的实时工作状态（如市电供电/电池供电、充电中/放电中）
// #define HID_PD_REMAINTIMELIMIT       0x08  // 剩余时间限制报告ID：用于读取UPS设定的"剩余工作时间阈值"（如低电量时的预警时间）
// #define HID_PD_MANUFACTUREDATE       0x09  // 制造日期报告ID：用于读取UPS设备的生产制造日期（通常格式为年/月/日）
// #define HID_PD_CONFIGVOLTAGE         0x0A  // 配置电压报告ID：用于读取UPS的额定配置电压（即设计工作电压，如12V、24V）
// #define HID_PD_VOLTAGE               0x0B  // 实时电压报告ID：用于读取电池的当前实时电压（如3.7V、12.1V，需结合单位解析）
// #define HID_PD_REMAININGCAPACITY     0x0C  // 剩余容量报告ID：用于读取电池的当前剩余容量（如80%、5000mAh，需结合单位解析）
// #define HID_PD_RUNTIMETOEMPTY        0x0D  // 剩余放电时间报告ID：用于读取电池当前容量下可支持的剩余放电时间（如120分钟）
// #define HID_PD_FULLCHARGECAPACITY    0x0E  // 满电容量报告ID：用于读取电池完全充满后的总容量（即满电状态下的容量，如6000mAh）
// #define HID_PD_WARNCAPACITYLIMIT     0x0F  // 预警容量阈值报告ID：用于读取UPS设定的"低电量预警容量阈值"（如剩余20%时触发预警）
// #define HID_PD_CPCTYGRANULARITY1     0x10  // 容量粒度1报告ID：用于读取电池容量的"粗粒度精度"（如100mAh为一个单位，描述容量统计的精度等级）
// #define HID_PD_REMNCAPACITYLIMIT     0x11  // 剩余容量限制报告ID：用于读取UPS设定的"剩余容量保护阈值"（如剩余5%时触发关机保护）
// #define HID_PD_CAPACITYMODE          0x16  // 容量模式报告ID：用于读取UPS的容量计算模式（如"百分比模式"或"实际容量模式"）
// #define HID_PD_DELAYBE4SHUTDOWN      0x12  // 关机延迟报告ID：用于读取UPS触发关机前的延迟时间（如低电量时延迟30秒关机，给负载留缓冲）
// #define HID_PD_DELAYBE4REBOOT        0x13  // 重启延迟报告ID：用于读取UPS触发重启前的延迟时间（如市电恢复后延迟60秒重启负载）
// #define HID_PD_CPCTYGRANULARITY2     0x18  // 容量粒度2报告ID：用于读取电池容量的"细粒度精度"（如10mAh为一个单位，比粒度1精度更高）
// #define HID_PD_DESIGNCAPACITY        0x17  // 设计容量报告ID：用于读取电池的设计额定容量（即出厂时标注的容量，如6000mAh）
// #define HID_PD_AUDIBLEALARMCTRL      0x14  // 声音告警控制报告ID：用于控制UPS的蜂鸣器告警（如开启/关闭告警、调节告警音量）
// #define HID_PD_AVERAGETIME2FULL      0x1A  // 平均充满时间报告ID：用于读取电池当前状态下充满电所需的平均时间（如240分钟）
// #define HID_PD_AVERAGETIME2EMPTY     0x1C  // 平均放电时间报告ID：用于读取电池当前状态下完全放电的平均时间（如180分钟）
// #define HID_PD_IOEMINFORMATION       0x20  // OEM信息报告ID：用于读取UPS的厂商自定义信息（如硬件版本、固件版本、定制化参数）


// // 字符串索引定义
// // 注：字符串索引用于关联USB设备描述符中的"字符串描述符"，主机通过索引读取对应的文本信息
// #define IMANUFACTURER               0x01  // 制造商字符串索引：对应"制造商信息"的字符串描述符（如"ESP32 UPS"）
// #define IPRODUCT                    0x02  // 产品字符串索引：对应"产品名称"的字符串描述符（如"ESP32 HID UPS"）
// #define ISERIAL                     0x03  // 序列号字符串索引：对应"设备序列号"的字符串描述符（如"ESP32-3417"）
// #define IDEVICECHEMISTRY            0x04  // 设备化学类型字符串索引：对应"电池化学类型"的字符串描述符（如"Li-ion"）
// #define IOEMVENDOR                  0x05  // OEM厂商字符串索引：对应"OEM厂商信息"的字符串描述符（如自定义的厂商名称）


// // UPS 适配设备类型定义
// // 注：用于区分UPS适配的目标设备/场景，可根据不同类型调整UPS的参数或协议
// #define UPS_FOR_QNAP                0x01  // 适配QNAP（威联通）NAS设备的UPS模式
// #define UPS_FOR_THINKPAD            0x00  // 适配ThinkPad（联想笔记本）的UPS模式


// // HID 协议类型定义
// #define HID_PROTOCOL_NONE 0  // 无特定HID协议：表示当前HID设备不遵循标准HID子协议（如键盘0x01、鼠标0x02），
//                              // 仅使用自定义的HID报告交互（符合你的UPS设备"无特定协议"的属性）
  
// // -------------------------- 3. 报告结构体定义（当前未使用，屏蔽） --------------------------
// // typedef struct {
// //     uint8_t report_id;
// //     uint8_t capacity;  // 0-100%
// // } remaining_capacity_report_t;

// // typedef struct {
// //     uint8_t report_id;
// //     uint16_t voltage;  // 原始值（实际电压=原始值*0.01V）
// // } voltage_report_t;

// // typedef struct {
// //     uint8_t report_id;
// //     uint16_t seconds;  // 剩余时间（秒）
// // } time_report_t;


// // -------------------------- 4. 电源状态结构体 --------------------------
// // 描述：该结构体用于存储UPS设备的实时电源状态，采用"位域"设计（每个成员占1位），
// // 仅需2字节（16位）即可完整表示16种核心状态，节省HID报告的数据传输量（符合你的3字节Input Report长度限制）
// struct PresentStatus {
//     // 第1字节（低8位）：基础电源与电池状态
//     uint8_t Charging : 1;                       // 充电状态位：1=电池正在充电，0=未充电
//     uint8_t Discharging : 1;                    // 放电状态位：1=电池正在放电（如市电断开，UPS用电池供电），0=未放电
//     uint8_t ACPresent : 1;                      // 市电存在位：1=当前有市电输入（UPS正常供电），0=市电断开（切电池）
//     uint8_t BatteryPresent : 1;                 // 电池存在位：1=电池已连接且正常识别，0=电池未连接/故障
//     uint8_t BelowRemainingCapacityLimit : 1;    // 剩余容量低于阈值位：1=当前容量低于设定的保护阈值（如剩余5%），0=容量正常
//     uint8_t RemainingTimeLimitExpired : 1;      // 剩余时间超限位：1=剩余放电时间低于设定阈值（如不足5分钟），0=时间正常
//     uint8_t NeedReplacement : 1;                // 电池需更换位：1=电池老化/故障，需更换，0=电池状态正常
//     uint8_t VoltageNotRegulated : 1;            // 电压未稳压位：1=UPS输出电压超出稳压范围（异常），0=电压稳定（正常）
    
//     // 第2字节（高8位）：充电完成/故障与控制状态
//     uint8_t FullyCharged : 1;                   // 完全充电位：1=电池已充满电，0=未充满
//     uint8_t FullyDischarged : 1;                // 完全放电位：1=电池已放空（无法供电），0=仍有剩余容量
//     uint8_t ShutdownRequested : 1;              // 关机请求位：1=已收到主机的关机指令，0=无关机请求
//     uint8_t ShutdownImminent : 1;               // 即将关机位：1=UPS因低电量/故障即将自动关机（倒计时中），0=无关机计划
//     uint8_t CommunicationLost : 1;              // 通信丢失位：1=与主机的HID通信中断，0=通信正常
//     uint8_t Overload : 1;                       // 过载位：1=UPS负载超出额定功率（过载保护），0=负载正常
//     uint8_t unused1 : 1;                         // 预留位1：暂未使用，用于后续功能扩展（默认填0）
//     uint8_t unused2 : 1;                         // 预留位2：暂未使用，用于后续功能扩展（默认填0）
// };

// // 结构体转 uint16_t（二进制兼容）
// // 描述：将PresentStatus结构体（2字节）直接转换为uint16_t类型，用于HID报告的二进制数据传输
// // 原理：结构体总长度为2字节（16位），与uint16_t长度一致，通过指针强制转换实现二进制数据复用，
// // 避免手动拼接字节的繁琐操作，确保数据在传输中无格式偏差
// static inline uint16_t PresentStatus_to_uint16(const struct PresentStatus* ps) {
//     return *(const uint16_t*)(ps);
// }

// // -------------------------- 5. 全局状态变量 --------------------------
// static struct PresentStatus UPS = {
//     .ACPresent = 1,        // 初始接AC电源
//     .BatteryPresent = 1,   // 电池存在
//     .Charging = 1,         // 初始充电中
//     .FullyCharged = 0      // 未充满
// };
// static uint8_t battery_capacity = 50;  // 初始50%容量
// static uint16_t battery_voltage = 1200; // 12.00V (1200 * 0.01V)
// static uint16_t runtime_to_empty = 3600; // 1小时
// static uint16_t avg_time_to_empty = 3600;

// #if UPS_FOR_THINKPAD
// // 新增电池容量变量 (单位: mWh)   for windows
// static uint16_t remaining_capacity_mwh = 50000;       // 剩余容量: 30000 mWh (初始50%)
// static uint16_t design_capacity_mwh = 60000;          // 设计容量: 60000 mWh
// static uint16_t full_charged_capacity_mwh = 60000;    // 完全充电容量: 60000 mWh (初始等于设计容量)
// #endif


// // -------------------------- 6. USB 设备描述符 --------------------------
// tusb_desc_device_t descriptor_dev = {
//     .bLength = sizeof(tusb_desc_device_t),
//     .bDescriptorType = TUSB_DESC_DEVICE,
//     .bcdUSB = 0x0200,          // USB 2.0 兼容
//     .bDeviceClass = 0x00,      // 接口级定义设备类
//     .bDeviceSubClass = 0x00,
//     .bDeviceProtocol = 0x00,
//     .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
//     .idVendor = 0x04d8,         // 改为：QNAP 配置文件中的参数
//     .idProduct = 0xd005,        // 改为：QNAP 配置文件中的参数
//     .bcdDevice = 0x0100,       // 设备版本 1.0
//     .iManufacturer = IMANUFACTURER,
//     .iProduct = IPRODUCT,
//     .iSerialNumber = ISERIAL,
//     .bNumConfigurations = 0x01
// };


// // -------------------------- 7. HID 报告描述符（关键：Windows 识别为电源设备） --------------------------




// // const uint8_t hid_report_descriptor[] = {
// //     0x05, 0x84,        // USAGE_PAGE (Power Device)
// //     0x09, 0x04,        // USAGE (UPS)
// //     0xA1, 0x01,        // COLLECTION (Application)
// //     0x09, 0x24,        //   USAGE (Sink) // 恢复Sink
// //     //  0x09, 0x30,    //   USAGE (Battery)
// //     0xA1, 0x02,        //   COLLECTION (Logical)
    
// //     // ==================== 全局设置 ====================
// //     0x75, 0x08,        //     REPORT_SIZE (8)
// //     0x95, 0x01,        //     REPORT_COUNT (1)
// //     0x15, 0x00,        //     LOGICAL_MINIMUM (0)
// //     0x26, 0xFF, 0x00,  //     LOGICAL_MAXIMUM (255)

// //     // ==================== 添加设备能力报告（必需）====================
// //     0x85, 0x0F,        //     REPORT_ID (15) - 设备能力
// //     0x09, 0x01,        //     USAGE (iConfiguration)
// //     0xB1, 0x83,        //     FEATURE (Data,Var,Abs)

// //     // ==================== 字符串信息报告 ====================
// //     // 产品信息（ID=1）
// //     0x85, HID_PD_IPRODUCT,     //     REPORT_ID (1)
// //     0x09, 0xFE,                //     USAGE (iProduct)
// //     0x79, IPRODUCT,            //     STRING INDEX (2)
// //     0xB1, 0x83,                //     FEATURE (Const,Var,Abs,NonVol)

// //     // 序列号（ID=2）
// //     0x85, HID_PD_SERIAL,       //     REPORT_ID (2)
// //     0x09, 0xFF,                //     USAGE (iSerialNumber)
// //     0x79, ISERIAL,             //     STRING INDEX (3)
// //     0xB1, 0x83,                //     FEATURE (Const,Var,Abs,NonVol)

// //     // 制造商（ID=3）
// //     0x85, HID_PD_MANUFACTURER, //     REPORT_ID (3)
// //     0x09, 0xFD,                //     USAGE (iManufacturer)
// //     0x79, IMANUFACTURER,       //     STRING INDEX (1)
// //     0xB1, 0x83,                //     FEATURE (Const,Var,Abs,NonVol)

// //     // ==================== 电池系统报告 ====================
// //     0x05, 0x85,                //     USAGE_PAGE (Battery System)

// //     // 可充电属性（ID=6）
// //     0x85, HID_PD_RECHARGEABLE, //     REPORT_ID (6)
// //     0x09, 0x8B,                //     USAGE (Rechargable)
// //     0xB1, 0x83,                //     FEATURE (Const,Var,Abs,NonVol)

// //     // 电池化学类型（ID=4）
// //     0x85, 0x04,                //     REPORT_ID (4)
// //     0x09, 0x8D,                //     USAGE (Device Chemistry)
// //     0x79, IDEVICECHEMISTRY,    //     STRING INDEX (4)
// //     0xB1, 0x83,                //     FEATURE (Const,Var,Abs,NonVol)

// // #if UPS_FOR_THINKPAD
// //     // 剩余容量（ID=12）  for windows
// //     0x85, HID_PD_REMAININGCAPACITY, // REPORT_ID (12)
// //     0x09, 0x66,                // USAGE (RemainingCapacity)
// //     0x75, 0x10,                // REPORT_SIZE (16)  // 修改：改为16位
// //     0x95, 0x01,                // REPORT_COUNT (1)  // 修改：改为1个16位值
// //     0x15, 0x00,                // LOGICAL_MINIMUM (0)
// //     0x26, 0x60, 0xEA,          // LOGICAL_MAXIMUM (60000) // 与设计容量一致
// //     0xB1, 0x83,                // FEATURE (Const,Var,Abs,Vol)

// //     // 设计容量 (ID=23)   for windows
// //     0x85, HID_PD_DESIGNCAPACITY,     //     REPORT_ID (23)
// //     0x09, 0x83,                      //     USAGE (DesignCapacity)
// //     0x75, 0x10,                      //     REPORT_SIZE (16)
// //     0x95, 0x01,                      //     REPORT_COUNT (1)
// //     0x26, 0x60, 0xEA,                //     LOGICAL_MAXIMUM (60000)  // 示例值，单位可能是mWh或10mWh，根据需要调整
// //     0xB1, 0x82,                      //     FEATURE (Data,Var,Abs,Vol) // 注意属性

// //     // 完全充电容量 (ID=14)   for windows
// //     0x85, HID_PD_FULLCHARGECAPACITY, //     REPORT_ID (14)
// //     0x09, 0x67,                      //     USAGE (FullChargedCapacity)
// //     0x75, 0x10,                      //     REPORT_SIZE (16)
// //     0x95, 0x01,                      //     REPORT_COUNT (1)
// //     0x26, 0x60, 0xEA,                //     LOGICAL_MAXIMUM (60000)  // 示例值，应与设计容量单位一致
// //     0xB1, 0x83,                      //     FEATURE (Const,Var,Abs,Vol) // 注意属性
// // #endif
// //     // ==================== 电源设备报告 ====================
// //     0x05, 0x84,                //     USAGE_PAGE (Power Device)

// // #if UPS_FOR_QNAP
// //     // 剩余容量（ID=12）
// //     0x85, HID_PD_REMAININGCAPACITY, // REPORT_ID (12)
// //     0x09, 0x66,                //     USAGE (RemainingCapacity)
// //     0x75, 0x08,                //     REPORT_SIZE (8)
// //     0x95, 0x01,                //     REPORT_COUNT (1) // 修改：改为1字节
// //     0x25, 0x64,                //     LOGICAL_MAXIMUM (100)
// //     0xB1, 0x83,                //     FEATURE (Const,Var,Abs,Vol)
// // #endif

// //     // 电压（ID=11）
// //     0x85, HID_PD_VOLTAGE,      //     REPORT_ID (11)
// //     0x09, 0x30,                //     USAGE (Voltage)
// //     0x75, 0x10,                //     REPORT_SIZE (16)
// //     0x95, 0x01,                //     REPORT_COUNT (1)
// //     0x26, 0xFF, 0xFF,          //     LOGICAL_MAXIMUM (65535)
// //     0x66, 0x01, 0x10,          //     UNIT (Volts)
// //     0x55, 0x02,                //     UNIT_EXPONENT (-2)
// //     0xB1, 0x83,                //     FEATURE (Const,Var,Abs,Vol)


// //     // 剩余运行时间（ID=13）
// //     0x85, HID_PD_RUNTIMETOEMPTY, // REPORT_ID (13)
// //     0x09, 0x68,                //     USAGE (RunTimeToEmpty)
// //     0x75, 0x10,                //     REPORT_SIZE (16)
// //     0x95, 0x01,                //     REPORT_COUNT (1)
// //     0x26, 0xFE, 0xFF,          //     LOGICAL_MAXIMUM (65534)
// //     0x66, 0x01, 0x10,          //     UNIT (Seconds)
// //     0x55, 0x00,                //     UNIT_EXPONENT (0)
// //     0xB1, 0x83,                //     FEATURE (Const,Var,Abs,Vol)

// //     // 平均放电时间（ID=28）
// //     0x85, HID_PD_AVERAGETIME2EMPTY, // REPORT_ID (28)
// //     0x09, 0x69,                //     USAGE (AverageTimeToEmpty)
// //     0x75, 0x10,                //     REPORT_SIZE (16)
// //     0x95, 0x01,                //     REPORT_COUNT (1)
// //     0x26, 0xFE, 0xFF,          //     LOGICAL_MAXIMUM (65534)
// //     0x66, 0x01, 0x10,          //     UNIT (Seconds)
// //     0x55, 0x00,                //     UNIT_EXPONENT (0)
// //     0xB1, 0x83,                //     FEATURE (Const,Var,Abs,Vol)

// //     // ==================== 电源状态报告（保持16位打包格式）====================
// //     0x09, 0x02,                //     USAGE (PresentStatus)
// //     0xA1, 0x02,                //     COLLECTION (Logical)
    
// //     // 电源状态报告（ID=7） - 保持16位打包格式
// //     0x85, HID_PD_PRESENTSTATUS, //       REPORT_ID (7)
// //     0x05, 0x84,                //       USAGE_PAGE (Power Device)
// //     0x09, 0x02,                //       USAGE (PresentStatus)
// //     0x75, 0x10,                //       REPORT_SIZE (16) // 保持16位
// //     0x95, 0x01,                //       REPORT_COUNT (1)
// //     0x15, 0x00,                //       LOGICAL_MINIMUM (0)
// //     0x26, 0xFF, 0xFF,          //       LOGICAL_MAXIMUM (65535)
// //     0xB1, 0x83,                //       FEATURE (Const,Var,Abs,Vol)

// //     0xC0,                      //     END_COLLECTION (Logical)
// //     0xC0,                      //   END_COLLECTION (Logical)
// //     0xC0                       // END_COLLECTION (Application)
// // };

// // -------------------------- 8. 自定义 HID 接口描述符 --------------------------
// const uint8_t hid_interface_desc[] = {
//     // 接口描述符（9字节）
//     0x09,                    // bLength: 接口描述符长度
//     0x04,                    // bDescriptorType: 接口描述符类型
//     0x00,                    // bInterfaceNumber: 接口编号（0）
//     0x00,                    // bAlternateSetting: 备用接口（0）
//     0x01,                    // bNumEndpoints: 端点数量（1个输入端点）
//     0x03,                    // bInterfaceClass: HID 类（0x03）
//     0x00,                    // bInterfaceSubClass: 无子类
//     0x00,                    // bInterfaceProtocol: 无协议
//     0x00,                    // iInterface: 接口字符串索引（0=无）

//     // HID 描述符（9字节）
//     0x09,                    // bLength: HID 描述符长度
//     0x21,                    // bDescriptorType: HID 类型（0x21）
//     0x11, 0x01,              // bcdHID: HID 1.11 版本（小端序）
//     0x00,                    // bCountryCode: 无国家码
//     0x01,                    // bNumDescriptors: 报告描述符数量（1）
//     0x22,                    // bDescriptorType: 报告描述符类型（0x22）
//     LO8(sizeof(hid_report_descriptor)),  // wItemLength: 报告描述符长度（低字节）
//     HI8(sizeof(hid_report_descriptor)),  // wItemLength: 报告描述符长度（高字节）

//     // 输入端点描述符（7字节）
//     0x07,                    // bLength: 端点描述符长度
//     0x05,                    // bDescriptorType: 端点描述符类型
//     0x81,                    // bEndpointAddress: 输入端点（0x81，EP1 IN）
//     0x03,                    // bmAttributes: 中断传输（0x03）
//     LO8(CFG_TUD_HID_EP_BUFSIZE),  // wMaxPacketSize: 最大包大小（低字节）
//     HI8(CFG_TUD_HID_EP_BUFSIZE),  // wMaxPacketSize: 最大包大小（高字节）
//     0x0A                     // bInterval: 报告间隔（10ms）
// };

// // -------------------------- 9. USB 配置描述符（正确拼接接口描述符） --------------------------
// #define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + sizeof(hid_interface_desc))
// uint8_t const desc_configuration[] = {
//     // 配置描述符头部（9字节）
//     TUD_CONFIG_DESCRIPTOR(
//         1,                  // 配置编号
//         1,                  // 接口数量
//         0,                  // 配置字符串索引
//         TUSB_DESC_TOTAL_LEN, // 配置总长度
//         TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP,
//         100                 // 最大功耗（100mA）
//     ),

//     // 拼接自定义 HID 接口描述符（25字节：9+9+7）
//     // memcpy((uint8_t*)desc_configuration + TUD_CONFIG_DESC_LEN, 
//     //    hid_interface_desc, 
//     //    sizeof(hid_interface_desc));

//     hid_interface_desc[0],  hid_interface_desc[1],  hid_interface_desc[2],
//     hid_interface_desc[3],  hid_interface_desc[4],  hid_interface_desc[5],
//     hid_interface_desc[6],  hid_interface_desc[7],  hid_interface_desc[8],
//     hid_interface_desc[9],  hid_interface_desc[10], hid_interface_desc[11],
//     hid_interface_desc[12], hid_interface_desc[13], hid_interface_desc[14],
//     hid_interface_desc[15], hid_interface_desc[16], hid_interface_desc[17],
//     hid_interface_desc[18], hid_interface_desc[19], hid_interface_desc[20],
//     hid_interface_desc[21], hid_interface_desc[22], hid_interface_desc[23],
//     hid_interface_desc[24]
// };

// // -------------------------- 10. USB 字符串描述符 --------------------------
// const char* descriptor_str[] = {
//     [0] = "en",                    // 语言ID (英语)
//     [IMANUFACTURER] = "MicroChip Tech", // 制造商
//     [IPRODUCT] = "USB-HID UPS",// 产品名（Windows 显示）
//     [ISERIAL] = "383503417",    // 序列号
//     [IDEVICECHEMISTRY] = "Li-ion",  // 电池类型
// };

// // -------------------------- 11. USB 初始化 --------------------------
// static void usb_hid_init(void) {
//     const tinyusb_config_t tusb_cfg = {
//         .device_descriptor = &descriptor_dev,
//         .configuration_descriptor = desc_configuration,
//         .string_descriptor = descriptor_str,
//         .string_descriptor_count = sizeof(descriptor_str) / sizeof(descriptor_str[0]),
//         .external_phy = false,
//     };

//     ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
//     ESP_LOGI(TAG, "TinyUSB initialized");

//     while (!tud_mounted()) {
//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
//     ESP_LOGI(TAG, "USB connected");
// }

// // -------------------------- 12. TinyUSB 回调函数 --------------------------
// //返回 HID 报告描述符，告诉主机 “如何理解 UPS 发送的二进制数据”。
// uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
//     return hid_report_descriptor;
// }

// // 处理主机通过 Get_Report 指令请求特定报告
// uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
//                                hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
    
//     ESP_LOGI(TAG, "Host requested report: ID=0x%02X, Type=%d, ReqLen=%d", 
//         report_id, report_type, reqlen);
    
//     // 添加更详细的报告类型信息
//     const char* report_type_str = "Unknown";
//     switch(report_type) {
//         case HID_REPORT_TYPE_INPUT: report_type_str = "INPUT"; break;
//         case HID_REPORT_TYPE_OUTPUT: report_type_str = "OUTPUT"; break;
//         case HID_REPORT_TYPE_FEATURE: report_type_str = "FEATURE"; break;
//         default:
//         // 处理未知或无效的报告类型，包括 HID_REPORT_TYPE_INVALID
//         ESP_LOGE(TAG, "Unknown or invalid report type requested: %d", report_type);
//         return 0; // 返回 0 表示无法处理该请求
//     }
//     ESP_LOGI(TAG, "Report type: %s", report_type_str);

//     buffer[0] = report_id;

//     if (report_type == HID_REPORT_TYPE_FEATURE) {
//         switch(report_id) {
//             case 0x0F: // 设备能力报告（ID=15）
//                 buffer[1] = 0x07; // 支持UPS+电池+Sink
//                 return 2;

//             case HID_PD_IPRODUCT: // 产品信息（ID=1）
//                 buffer[1] = IPRODUCT;
//                 buffer[2] = 0x00;
//                 return 3;

//             case HID_PD_SERIAL: // 序列号（ID=2）
//                 buffer[1] = ISERIAL;
//                 buffer[2] = 0x00;
//                 return 3;

//             case HID_PD_MANUFACTURER: // 制造商（ID=3）
//                 buffer[1] = IMANUFACTURER;
//                 buffer[2] = 0x00;
//                 return 3;

//             case HID_PD_IDEVICECHEMISTRY: // 电池化学类型（ID=4）
//                 buffer[1] = IDEVICECHEMISTRY;
//                 buffer[2] = 0x00;
//                 return 3;

//             case HID_PD_RECHARGEABLE: // 可充电属性（ID=6）
//                 buffer[1] = 0x01; // 1=可充电
//                 buffer[2] = 0x00;
//                 return 3;
// #if UPS_FOR_QNAP
//             case HID_PD_PRESENTSTATUS: { //0x0C   设备状态报告（必填）
//                 // bit0: 0=市电模式, 1=电池模式; bit1: 0=正常, 1=低电量
//                 buffer[1] = 0x30;
//                 buffer[2] = 0x00;
//                 return 3;
//             }

//             case HID_PD_VOLTAGE: // 电压报告（ID=11，所有模式通用）
//                 buffer[1] = HI8(battery_voltage); // 低字节
//                 buffer[2] = LO8(battery_voltage); // 高字节
//                 ESP_LOGI(TAG, "反馈电压： %d mv\n", battery_voltage);
//                 return 3;


//             case HID_PD_REMAININGCAPACITY: // 0x0C - 剩余容量报告
//                 buffer[1] = battery_capacity; // 直接返回百分比
//                 return 2;

//             case HID_PD_RUNTIMETOEMPTY: // 0x0D - 剩余时间报告
//                 buffer[1] = LO8(runtime_to_empty);
//                 buffer[2] = HI8(runtime_to_empty);
//                 return 3;

// #elif UPS_FOR_THINKPAD
//             // // 1. 电池化学类型（ID=0x04）：返回实际字符串（而非索引）
//             // case HID_PD_IDEVICECHEMISTRY:
//             //     strncpy((char*)&buffer[1], descriptor_str[IDEVICECHEMISTRY], reqlen-1);
//             //     return reqlen; // 按主机请求长度返回（通常为字符串长度+1）

//             // 2. 电池电压（ID=0x0B）：返回16位电压值（实际电压=值*0.01V）
//             case HID_PD_VOLTAGE:
//                 buffer[1] = LO8(battery_voltage); // 低字节
//                 buffer[2] = HI8(battery_voltage); // 高字节
//                 return 3; // 1字节ID + 2字节电压值

//             // 3. 剩余容量（ID=0x0C）：返回16位容量值（单位：mWh）
//             case HID_PD_REMAININGCAPACITY:
//                 buffer[1] = LO8(remaining_capacity_mwh);
//                 buffer[2] = HI8(remaining_capacity_mwh);
//                 return 3;
            
//             // 4. 完全充电容量（ID=0x0E）：返回16位容量值（单位：mWh）
//             case HID_PD_FULLCHARGECAPACITY:
//                 buffer[1] = LO8(full_charged_capacity_mwh);
//                 buffer[2] = HI8(full_charged_capacity_mwh);
//                 return 3;
            
//             // 5. 设计容量（ID=0x17）：返回16位容量值（单位：mWh）
//             case HID_PD_DESIGNCAPACITY:
//                 buffer[1] = LO8(design_capacity_mwh);
//                 buffer[2] = HI8(design_capacity_mwh);
//                 return 3;

// #endif

//             default:
//                 // 对于其他未实现的报告，返回默认值
//                 buffer[1] = 0x00;
//                 buffer[2] = 0x00;
//                 return 3;
//         }
//     }

//     // 输入报告
//     switch(report_id) {
//         case HID_PD_PRESENTSTATUS: {
//             uint16_t status = PresentStatus_to_uint16(&UPS);
//             buffer[1] = (uint8_t)(status & 0xFF);
//             buffer[2] = (uint8_t)(status >> 8);
//             return 3;
//         }

// #if UPS_FOR_QNAP
//         case HID_PD_REMAININGCAPACITY:
//             buffer[1] = battery_capacity;
//             return 2; // 只有ID + 1字节数据

// #elif UPS_FOR_THINKPAD
//         case HID_PD_REMAININGCAPACITY:   //for windows
//             buffer[1] = LO8(remaining_capacity_mwh);  // 低字节
//             buffer[2] = HI8(remaining_capacity_mwh);  // 高字节
//             return 3;  // 修改：返回3字节（ID + 2字节数据）
// #endif
//         case HID_PD_VOLTAGE:
//             buffer[1] = (uint8_t)(battery_voltage & 0xFF);
//             buffer[2] = (uint8_t)(battery_voltage >> 8);
//             return 3;
//         case HID_PD_RUNTIMETOEMPTY:
//             buffer[1] = (uint8_t)(runtime_to_empty & 0xFF);
//             buffer[2] = (uint8_t)(runtime_to_empty >> 8);
//             return 3;
//         default:
//             return 0;
//     }
// }
// // 处理主机通过 Set_Report 指令向 UPS 发送控制指令
// void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
//                            hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
//     ESP_LOGI(TAG, "Set report: ID=0x%02X, Type=%u, Size=%u", report_id, report_type, bufsize);
    
//     // 打印接收到的数据内容（用于调试）
//     ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer, bufsize, ESP_LOG_INFO);

//     // 处理UPS控制命令
//     switch(report_id) {
//         case 0x10: // 关机命令
//             if (bufsize >= 2 && buffer[1] == 0x01) {
//                 ESP_LOGI(TAG, "Received shutdown command");
//                 // 执行关机逻辑
//             }
//             break;
            
//         case 0x11: // 自检命令
//             if (bufsize >= 2 && buffer[1] == 0x01) {
//                 ESP_LOGI(TAG, "Received self-test command");
//                 // 执行自检逻辑
//             }
//             break;
//     }

// }

// // 处理主机对 HID 协议的查询
// uint8_t tud_hid_get_protocol_cb(uint8_t instance) {
//     return HID_PROTOCOL_NONE;
// }

// // 处理主机对 HID 协议的设置
// void tud_hid_set_protocol_cb(uint8_t instance, uint8_t protocol) {
//     ESP_LOGI(TAG, "Protocol set to: %u", protocol);
// }


// // -------------------------- 13. 模拟 UPS 状态变化 --------------------------
// static void update_ups_state(void) {
//     // 模拟AC电源断开/连接（120秒切换一次）
//     static uint32_t ac_timer = 0;
//     if (xTaskGetTickCount() - ac_timer > pdMS_TO_TICKS(120000)) {
//         UPS.ACPresent ^= 1;
//         ac_timer = xTaskGetTickCount();
        
//         if (UPS.ACPresent) {
//             UPS.Charging = 1;
//             UPS.Discharging = 0;
//             ESP_LOGI(TAG, "\n*****AC connected - charging*****\n");
//         } else {
//             UPS.Charging = 0;
//             UPS.Discharging = 1;
//             ESP_LOGI(TAG, "\n*****AC disconnected - discharging*****\n");
//         }
//     }

// #if UPS_FOR_QNAP
//     // 模拟电池容量变化
//     if (UPS.Charging && battery_capacity < 100) {
//         battery_capacity++;
//         if (battery_capacity >= 100) {
//             UPS.FullyCharged = 1;
//             UPS.Charging = 0;
//         }
//     } else if (UPS.Discharging && battery_capacity > 0) {
//         battery_capacity--;
//         UPS.FullyCharged = 0;
//         if (battery_capacity <= 0) {
//             UPS.FullyDischarged = 1;
//         }
//     }

//         // 更新剩余时间（基于容量）
//     runtime_to_empty = battery_capacity * 60; // 1分钟/1%容量
//     avg_time_to_empty = runtime_to_empty;


// #elif UPS_FOR_THINKPAD
//     // // 模拟电池容量变化 - 修改：直接操作实际容量值  for windows
//     // if (UPS.Charging && remaining_capacity_mwh < full_charged_capacity_mwh) {
//     //     remaining_capacity_mwh += 100; // 每次充电增加100mWh
//     //     if (remaining_capacity_mwh >= full_charged_capacity_mwh) {
//     //         remaining_capacity_mwh = full_charged_capacity_mwh;
//     //         UPS.FullyCharged = 1;
//     //         UPS.Charging = 0;
//     //     }
//     // } else if (UPS.Discharging && remaining_capacity_mwh > 0) {
//     //     remaining_capacity_mwh -= 100; // 每次放电减少100mWh
//     //     UPS.FullyCharged = 0;
//     //     if (remaining_capacity_mwh <= 0) {
//     //         remaining_capacity_mwh = 0;
//     //         UPS.FullyDischarged = 1;
//     //     }
//     // }

//     // 更新剩余时间（基于剩余容量百分比）  for windows
//     uint8_t percent_remaining = (remaining_capacity_mwh * 100) / full_charged_capacity_mwh;
//     runtime_to_empty = percent_remaining * 60; // 1分钟/1%容量
//     avg_time_to_empty = runtime_to_empty;
// #endif
// }

// // -------------------------- 14. HID 报告发送任务 --------------------------
// // static void hid_send_task(void *pvParameters) {
// //     uint8_t report_buffer[64] = {0}; // 足够大的缓冲区
// //     uint8_t offset = 0;
    
// //     while (1) {
// //         if (tud_mounted() && tud_hid_ready()) {
// //             // 更新UPS状态
// //             update_ups_state();

// //             // 清空缓冲区
// //             memset(report_buffer, 0, sizeof(report_buffer));
// //             offset = 0;

// //             // 按照报告ID顺序填充数据（参考项目的顺序）
// // #if UPS_FOR_THINKPAD
// //             // 3. 设计容量报告（ID=23）  for windows
// //             report_buffer[offset++] = HID_PD_DESIGNCAPACITY;
// //             report_buffer[offset++] = LO8(design_capacity_mwh);
// //             report_buffer[offset++] = HI8(design_capacity_mwh);

// //             // 4. 完全充电容量报告（ID=14）  for windows
// //             report_buffer[offset++] = HID_PD_FULLCHARGECAPACITY;
// //             report_buffer[offset++] = LO8(full_charged_capacity_mwh);
// //             report_buffer[offset++] = HI8(full_charged_capacity_mwh);
// // #endif     
// //             // 1. 状态报告（ID=7）
// //             report_buffer[offset++] = HID_PD_PRESENTSTATUS;
// //             uint16_t status = PresentStatus_to_uint16(&UPS);
// //             report_buffer[offset++] = LO8(status);
// //             report_buffer[offset++] = HI8(status);

// // #if UPS_FOR_QNAP
// //             // 2. 剩余容量报告（ID=12）
// //             report_buffer[offset++] = HID_PD_REMAININGCAPACITY;
// //             report_buffer[offset++] = battery_capacity;
// // #elif UPS_FOR_THINKPAD
// //             // 2. 剩余容量报告（ID=12）- 修改：发送实际容量值，不是百分比  for windows
// //             report_buffer[offset++] = HID_PD_REMAININGCAPACITY;
// //             report_buffer[offset++] = LO8(remaining_capacity_mwh);  // 低字节
// //             report_buffer[offset++] = HI8(remaining_capacity_mwh);  // 高字节
// // #endif
// //             // 3. 电压报告（ID=11）
// //             report_buffer[offset++] = HID_PD_VOLTAGE;
// //             report_buffer[offset++] = LO8(battery_voltage);
// //             report_buffer[offset++] = HI8(battery_voltage);

// //             // 4. 剩余运行时间报告（ID=13）
// //             report_buffer[offset++] = HID_PD_RUNTIMETOEMPTY;
// //             report_buffer[offset++] = LO8(runtime_to_empty);
// //             report_buffer[offset++] = HI8(runtime_to_empty);


// //             // 发送完整报告
// //             // tud_hid_report(0, report_buffer, offset);

// // #if UPS_FOR_QNAP
// //             // ESP_LOGI(TAG, "Capacity: %d%%, Voltage: %.2fV, Runtime: %ds",
// //             //          battery_capacity, battery_voltage * 0.01f, runtime_to_empty);

// // #elif UPS_FOR_THINKPAD
// //             // 计算百分比用于显示  for windows
// //             uint8_t percent = (remaining_capacity_mwh * 100) / full_charged_capacity_mwh;
// //             ESP_LOGI(TAG, "Capacity: %dmWh (%d%%), Voltage: %.2fV, Runtime: %ds",
// //                      remaining_capacity_mwh, percent, battery_voltage * 0.01f, runtime_to_empty);
// // #endif
// //         } 
// //         // else {
// //         //     ESP_LOGW(TAG, "设备未挂载或HID未就绪");
// //         // }
// //         vTaskDelay(pdMS_TO_TICKS(5000)); // 建议2秒更新一次
// //     }
// // }

// // -------------------------- 15. 主函数 --------------------------
// void app_main(void) {
//     ESP_LOGI(TAG, "HID UPS Device (ESP32 + TinyUSB)");

//     // 初始化NVS
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     // 初始化USB HID
//     usb_hid_init();

//     // 创建HID发送任务
//     // xTaskCreate(hid_send_task, "hid_send_task", 8192, NULL, 5, NULL);
// }








/* This Report descriptor is a series of nested collections for all the different objects included in the UPS. Its skeleton is defined below. (PresentStatus and ChangedStatus collections are not shown here.)
UPS application collection
    Main AC flow physical collection 
    End Main AC flow collection 
    Backup DC flow physical collection 
    End Backup DC flow collection 
    Output AC flow physical collection 
    End Output AC flow collection 
    Battery System physical collection
        AC Input physical collection 
        End AC Input collection 
        Charger physical collection 
        End Charger collection 
        Battery physical collection 
        End Battery collection
        DC Output physical collection 
        End DC Output collection
    End Battery System collection 
    Power Converter physical collection
        AC Input physical collection 
        End AC Input collection
        AC Output physical collection 
        End AC Output collection
        DC Input physical collection 
        End DC Input collection
    End Power Converter collection 
    PowerSummary physical collection 
    End PowerSummary collection
End UPS collection */

/* UPS 应用集合
- 主交流电流量物理集合
- 主交流电流量结束集合

- 备用直流电流量物理集合
- 备用直流电流量结束集合

- 输出交流电流量物理集合
- 输出交流电流量结束集合

- 电池系统物理集合
    交流电输入物理集合
    交流电输入结束集合

    充电器物理集合
    充电器结束集合

    电池物理集合
    电池结束集合

    直流输出物理集合
    直流输出结束集合
- 电池系统结束集合

- 电源转换器物理集合
    交流电输入物理集合
    交流电输入结束集合

    交流电输出物理集合
    交流电输出结束集合

    直流输入物理集合
    直流电输入结束集合
- 电源结束转换器收集

-PowerSummary 物理收集
-PowerSummary 收集结束

UPS 收集结束 */




/* A.6.1 Header of UPS application collection
UsagePage(Power Device),
Usage(UPS), Collection(Application), ; UPS collection

A.6.2 Main AC flow physical collection
The main AC flow contains the flow ID (1), name, configuration voltage, and configuration frequency of AC. Feature report ID 1 begins.
UsagePage(Power Device),
Usage(Flow), Collection(Physical), ReportID (1), ; Main AC Flow
Usage(FlowID), Unit(none), ; Constant = 1 ReportSize(4), ReportCount(1), Logical Minimum (0), Logical Maximum (15), Unit(0), Feature(Constant, Variable, Absolute),
ReportSize(4), ReportCount(1),
Feature(Constant, Variable, Absolute), ; 4-bit pad Usage(iName),
ReportSize(8), ReportCount(1), Logical Minimum (0), Logical Maximum (255), Feature(Constant, Variable, Absolute),
Usage(ConfigVoltage),
ReportSize(8), ReportCount(1), Unit(Volt), UnitExponent(7), ; In Volts (110 or 220) Logical Minimum (0), Logical Maximum (250),
Feature(Constant, Variable, Absolute), Usage(ConfigFrequency),
ReportSize(8), ReportCount(1), Unit(Hertz), UnitExponent(0), ; In Hertz (50 or 60) Logical Minimum (0), Logical Maximum (60),
Feature(Constant, Variable, Absolute),
End Collection(), ; End of Main AC Flow collection

A.6.3 Backup DC flow physical collection
The backup DC flow contains the flow ID (2), name, configuration voltage, and configuration frequency of DC. Feature report ID 2 begins.
UsagePage(Power Device),
Usage(Flow), Collection(Physical), ReportID (2), ; Backup DC
Usage(FlowID), Unit(none), ; Constant = 2 ReportSize(4), ReportCount(1), Logical Minimum (0), Logical Maximum (15), Unit(0), Feature(Constant, Variable, Absolute),
ReportSize(4), ReportCount(1),
Feature(Constant, Variable, Absolute), ; 4-bit pad Usage(iName),
ReportSize(8), ReportCount(1), Logical Minimum (0), Logical Maximum (255), Feature(Constant, Variable, Absolute),
Usage(ConfigVoltage),
ReportSize(16), ReportCount(1), Unit(Volt), UnitExponent(5), ; In cVolts Logical Minimum (0), Logical Maximum (0xFFFE),
Feature(Constant, Variable, Absolute), Usage(ConfigFrequency),
ReportSize(8), ReportCount(1), Unit(Herz), UnitExponent(0), ; In Hertz (0) Logical Minimum (0), Logical Maximum (60),
Feature(Constant, Variable, Absolute),
End Collection(), ; End of DC Flow collection

A.6.4 Output AC flow physical collection
The output AC flow contains the flow ID (3), name, configuration voltage, configuration frequency, and configuration power of AC. Feature report ID 3 begins.
UsagePage(Power Device),
Usage(Flow), Collection(Physical), ReportID (3), ; UPS Output
Usage(FlowID), Unit(none), ; Constant=3, connected to flow 3 ReportSize(4), ReportCount(1), Logical Minimum (0), Logical Maximum (15), Unit(0), Feature(Constant, Variable, Absolute),
ReportSize(4), ReportCount(1),
Feature(Constant, Variable, Absolute), ; 4-bit pad Usage(iName),
ReportSize(8), ReportCount(1), Logical Minimum (0), Logical Maximum (255), Feature(Constant, Variable, Absolute),
Usage(ConfigVoltage),
ReportSize(8), ReportCount(1), Unit(Volt), UnitExponent(7), ; In Volts (110 or 220) Logical Minimum (0), Logical Maximum (250),
Feature(Constant, Variable, Absolute), Usage(ConfigFrequency),
ReportSize(8), ReportCount(1), Unit(Hertz), UnitExponent(0), ; in Hertz (50 or 60) Logical Minimum (0), Logical Maximum (60),
Feature(Constant, Variable, Absolute), Usage(ConfigApparentPower),
ReportSize(16), ReportCount(1), Unit(Watt), UnitExponent (7), ; In VA Logical Minimum (0), Logical Maximum (0xFFFE),
Feature(Constant, Variable, Absolute),
End Collection(), ; End of Output AC Flow collection

A.6.5 Header of Battery System Physical Collection
The header of the Battery System contains the battery system ID (1), followed by the collection corresponding to its sub-modules. Feature Report ID 4 begins.
UsagePage(Power Device),
Usage(BatterySystem), Collection(Physical), ReportID (4), ; Battery System Usage(BatterySystemID), Unit(none), ; Constant = 1 ReportSize(4), ReportCount(1), Logical Minimum (0), Logical Maximum (15), Unit(0), Feature(Constant, Variable, Absolute),
ReportSize(4), ReportCount(1),
Feature(Constant, Variable, Absolute), ; 4 bit pad

A.6.6 Battery System: AC Input Physical Collection
The Battery System AC input contains the input ID (1), the ID of the connected flow (1), and two status items: Used and Good. Feature report ID 4 continues. Input report ID 4 begins.
Usage(Input),
Collection(Physical), ; Battery System: AC Input
Usage(InputID), ; Constant=1
Usage(FlowID), ; Constant=1, connected to flow 1 ReportSize(4), ReportCount(2), Logical Minimum (0), Logical Maximum (15), Unit(0), Feature(Constant, Variable, Absolute),
Usage(CurrentStatus), Collection(Logical), ; Present status collection Usage(Used), Usage(Good),
ReportSize(1), ReportCount(2), Logical Minimum (0), Logical Maximum (1), Feature(Constant, Variable, Absolute, Volatile),
End Collection(), ; End of Present Status collection Usage(ChangedStatus), Collection(Logical), ; Changed Status collection
Usage(Used), Usage(Good),
ReportSize(2), ReportCount(2), Logical Minimum (0), Logical Maximum (1), Input(Data, Variable, Absolute, Volatile),
End Collection(), ; End of Changed Status collection
End Collection(), ; End of AC Input collection

A.6.7 Battery System: Charger Physical Collection
The Battery System Charger contains the charger ID (1). Feature report ID 5 begins.
Usage(Charger), Collection(Physical), ReportID (5), ; Battery system: Charger Usage(ChargerID), ; Constant = 1
ReportSize(4), ReportCount(1), Logical Minimum (0), Logical Maximum (15), Unit(0), Feature(Constant, Variable, Absolute),
End Collection(), ; End of Charger collection

A.6.8 Battery System: DC Output Physical Collection
The Battery System DC output contains the output ID (1) and the ID of the connected flow (2). Feature report ID 5 continues.
Usage(Output), Collection(Physical), ; Battery System: DC Output
Usage(OutputID), ; Constant = 1
Usage(FlowID), ; Constant = 2, Connected to flow 2 ReportSize(4), ReportCount(2), Logical Minimum (0), Logical Maximum (15), Unit(0), Feature(Constant, Variable, Absolute),
End Collection(), ; End of DC Output collection

A.6.9 Battery System: Battery Physical Collection
The Battery System battery contains the battery ID (1), the capacity mode, the design capacity, the configuration voltage, the remaining capacity, and four status items: Good, BelowRemainingCapacityLimit, Charging, and Discharging. Feature report ID 6 and Input report ID 6 begin.
Usage(Battery), Collection(Physical), ReportID (6), ; Battery System: Battery Usage(BatteryID), ; Constant = 1
ReportSize(4), ReportCount(1), Logical Minimum (0), Logical Maximum (15), Unit(0), Feature(Constant, Variable, Absolute),
UsagePage(Battery System), Usage(CapacityMode),
ReportSize(1), ReportCount(1), Logical Minimum (0), Logical Maximum (1), Feature(Constant, Variable, Absolute),
ReportSize(3), ReportCount(1),
Feature(Constant, Variable, Absolute), ; 3-bit pad Usage(DesignCapacity),
ReportSize(24), ReportCount(1), Unit(Amp.s), UnitExponent(0), ; In Amp.secs Logical Minimum (0), Logical Maximum (0xFFFFFE),
Feature(Constant, Variable, Absolute), UsagePage(Power Device), Usage(ConfigVoltage),
ReportSize(16), ReportCount(1), Unit(Volt), UnitExponent(5), ; In c.Volts Logical Minimum (0), Logical Maximum (0xFFFE),
Feature(Constant, Variable, Absolute), UsagePage(Battery System), Usage(RemainingCapacity),
ReportSize(24), ReportCount(1), Unit(mAh),UnitExponent(0), ; In Amp.secs Logical Minimum (0), Logical Maximum (0xFFFFFE),
Feature(Constant, Variable, Absolute, Volatile),
UsagePage(Power Device), Unit(none),
Usage(PresentStatus), Collection(Logical), ; Present status collection Usage(Good),
UsagePage(Battery System), Usage(BelowRemainingCapacityLimit), Usage(Charging), Usage(Discharging),
ReportSize(1), ReportCount(4), Logical Minimum (0), Logical Maximum (1), Unit(0), Feature(Constant, Variable, Absolute, Volatile),
End Collection(), ; End of Present Status collection UsagePage(Power Device),
Usage(ChangedStatus), Collection(Logical), ; Changed Status collection Usage(Good),
UsagePage(Battery System), Usage(BelowRemainingCapacityLimit), Usage(Charging), Usage(Discharging),
ReportSize(2), ReportCount(4), Logical Minimum (0), Logical Maximum (1), Input(Data, Variable, Absolute, Volatile),
End Collection(), ; End of Changed Status collection
End Collection(), ; End of Battery collection
End Collection(), ; End of Battery System collection

A.6.10 Header of Power Converter Physical Collection
The header of the Power Converter contains the power converter ID followed by the collection corresponding to its sub-modules. Feature report ID 8 and Input report ID 8 begin.
UsagePage(Power Device),
Usage(PowerConverter), Collection(Physical), ReportID (8), ; Power Converter Usage(PowerConverterID), ; Constant = 1 ReportSize(4), ReportCount(1), Logical Minimum (0), Logical Maximum (15), Unit(0), Feature(Constant, Variable, Absolute),

A.6.11 Power Converter: AC Input Physical Collection
The Power Converter AC input contains the input ID (2), the ID of the connected flow (1), and two status items: Used and Good. Feature report ID 8 and Input report ID 8 continue.
Usage(Input), Collection(Physical), ; Power Converter: AC Input
Usage(InputID), ; Constant = 2
Usage(FlowID), ; Constant = 1, Connected to flow 1 ReportSize(4), ReportCount(2), Logical Minimum (0), Logical Maximum (15), Unit(0), Feature(Constant, Variable, Absolute),
Usage(PresentStatus), Collection(Logical), ; Present status collection Usage(Used), Usage(Good),
ReportSize(1), ReportCount(2), Logical Minimum (0), Logical Maximum (1), Feature(Constant, Variable, Absolute, Volatile),
End Collection(), ; End of Present Status collection Usage(ChangedStatus), Collection(Logical), ; Changed Status collection
Usage(Used), Usage(Good),
ReportSize(2), ReportCount(2), Logical Minimum (0), Logical Maximum (1), Input(Data, Variable, Absolute, Volatile),
End Collection(), ; End of Changed Status collection
End Collection(), ; End of AC Input collection

A.6.12 Power Converter: AC Output Physical Collection
The Power Converter AC output contains the output ID (2), the ID of the connected flow (3), and four status items: Used, Good, Overload, and ShutdownImminent. Feature Report ID 9 and Input Report ID 9 begin.
Usage(Output), Collection(Physical), ; Power Converter: AC Output ReportID (9),
Usage(OutputID), ; Constant = 2
Usage(FlowID), ; Constant = 3, Connected to flow 3 ReportSize(4), ReportCount(2), Logical Minimum (0), Logical Maximum (15), Unit(0), Feature(Constant, Variable, Absolute),
Usage(PercentLoad),
ReportSize(8), ReportCount(1), Logical Minimum (0), Logical Maximum (255), Input(Constant, Variable, Absolute, Volatile),
Usage(PresentStatus), Collection(Logical), ; Present status collection Usage(Used), Usage(Good),
Usage(Overload), Usage(ShutdownImminent),
ReportSize(1), ReportCount(4), Logical Minimum (0), Logical Maximum (1), Feature(Constant, Variable, Absolute, Volatile),
End Collection(), ; End of Present Status collection Usage(ChangedStatus), Collection(Logical), ; Changed Status collection
Usage(Used), Usage(Good), Usage(Overload), Usage(ShutdownImminent),
ReportSize(2), ReportCount(4), Logical Minimum (0), Logical Maximum (1), Input(Data, Variable, Absolute, Volatile),
End Collection(), ; End of Changed Status collection
End Collection(), ; End of AC Output collection

A.6.13 Power Converter: DC Input Physical Collection
The Power Converter DC input contains the input ID (2), the ID of the connected flow (2), and two status items: Used and Good. Feature report ID 10 and Input report ID 10 begin.
Usage(Input), Collection(Physical),
ReportID (10), ; Power Converter: DC Input
Usage(InputID), ; Constant = 3
Usage(FlowID), ; Constant = 2, Connected to flow 2 ReportSize(4), ReportCount(2), Logical Minimum (0), Logical Maximum (15), Unit(0), Feature(Constant, Variable, Absolute),
Usage(PresentStatus), Collection(Logical), ; Present status collection Usage(Used),Usage(Good),
ReportSize(1), ReportCount(2), Logical Minimum (0), Logical Maximum (1), Feature(Constant, Variable, Absolute, Volatile),
End Collection(), ; End of Present Status collection Usage(ChangedStatus), Collection(Logical), ; Changed Status collection
Usage(Used),Usage(Good),
ReportSize(2), ReportCount(2), Logical Minimum (0), Logical Maximum (1), Input(Data, Variable, Absolute, Volatile),
End Collection(), ; End of Changed Status collection
End Collection(), ; End of DC Input collection
End Collection(), ; End of PowerConverter collection

A.6.14 Power Summary Physical Collection
As static data, the Power Summary collection contains the power summary ID (1), the ID of the connected flow (3), the name of the power source, a battery presence indicator, the capacity mode (unit of battery capacity), a battery rechargeability indicator, the battery design capacity, the battery design voltage, the warning capacity limit, battery granularities 1 and 2, the product name, the serial number, the battery chemistry, and the manufacturer name.
As dynamic data, the Power Summary collection contains the last full charge capacity, the battery present voltage, the battery discharge current, the present remaining capacity, the present run time before the battery is empty, the UPS output percent of load, and six status items: for AC input: Present; for battery: BelowRemainingCapacityLimit, Charging, and Discharging; and for output: Overload and ShutdownImminent.
Feature report ID 11 and Input report ID 11 begin.
UsagePage(Power Device), Usage(PowerSummary), Collection(Physical) ; Power Summary ReportID (11),
Usage(PowerSummaryID), ; Constant = 1
Usage(FlowID), ; Constant = 3, connected to flow 3 ReportSize(4), ReportCount(2), Logical Minimum(0), Logical Maximum (15), Unit(0), Feature(Constant, Variable, Absolute),
Usage(iName), ; Constant = pointer to "UPS Power Output Source" ReportSize(8), ReportCount(1), Logical Maximum(255, Unit(0),
Feature(Constant, Variable, Absolute), UsagePage(Battery System),
Usage(BatteryPresent) ; Constant = 1 (battery present)
Usage(CapacityMode), ; Constant = 0 (As)
Usage(Rechargable), ; Constant = 1 (rechargeable) ReportSize(1), ReportCount(3), Logical Maximum (1),
Feature(Constant, Variable, Absolute),
ReportCount(5), ; 5 bits padding
Feature(Constant, Variable, Absolute),
UsagePage(Battery System), Usage(DesignCapacity), ; Value = Battery Design Capacity Usage(WarningCapacityLimit), ; Value = x% of DesignCapacity
Usage(CapacityGranularity1), ; Value = y% of DesignCapacity
Usage(CapacityGranularity2), ; Value = z% of DesignCapacity ReportSize(24), ReportCount(4), Unit(AmpSec), UnitExponent(0), Logical Maximum (0xFFFFFE), Feature(Constant, Variable, Absolute),
UsagePage(Power Device), Usage(ConfigVoltage), ; Value = Battery Design Voltage ReportSize(16), ReportCount(1), Unit(Volt), UnitExponent(5), Logical Maximum (0xFFFE), Feature(Constant, Variable, Absolute),
UsagePage(Power Device), Usage(iProduct), ; Value = pointer to "ACME UPS 1000 " Usage(iSerialNumber), ; Value = pointer to "1000-234" UsagePage(Battery System), Usage(iDeviceChemistry), ; Value = pointer to "PbAc" Usage(iManufacturerName), ; Value = pointer to "Battery in ACME UPS" ReportSize(8), ReportCount(4), Logical Maximum (0xFF), Unit(0),
Feature(Constant, Variable, Absolute),
UsagePage(Power Device), Usage(PercentLoad), ; Value = Present UPS output percent load ReportSize(8), ReportCount(1), Logical Maximum (254), Unit(0),
Input(Constant, Variable, Absolute, Volatile),
UsagePage(Power Device), Usage(Voltage), ; Value = Battery present voltage ReportSize(16), ReportCount(1), Unit(Volt), UnitExponent(5), Logical Maximum (0xFFFE), Feature(Constant, Variable, Absolute, Volatile),
Usage(Current), ; Value = Battery present discharge current ReportCount(1), Unit(Amp), UnitExponent(-2),
Feature(Constant, Variable, Absolute, Volatile),
UsagePage(Battery System), Usage(FullChargeCapacity), ; Value = 100% of Design Capacity ReportSize(24), ReportCount(1), Unit(AmpSec), UnitExponent(0), Logical Maximum (0xFFFFFE) Feature(Constant, Variable, Absolute, Volatile),
Usage(RemainingCapacity), ; Value = Present remaining capacity ReportCount(1),
Input(Constant, Variable, Absolute, Volatile),
Usage(RunTimeToEmpty), ; Value = Present run time before battery empty ReportSize(16), ReportCount(1), Unit(second), UnitExponent(0), Logical Maximum (0xFFFE), Input(Constant, Variable, Absolute, Volatile)
Usage(PresentStatus), Collection(Logical) ; PresentStatus collection UsagePage(Battery System), Usage(ACPresent) ; AC Input status Usage(Charging), ; Battery status
Usage(Discharging), ; Battery status
Usage(BelowRemainingcapacityLimit), ; Battery status UsagePage(Power Device), Usage(ShutdownImminent), ; UPS output status Usage(Overload), ; UPS output status ReportSize(1), ReportCount(6), Logical Maximum (1),
Input(Constant, Variable, Absolute, Volatile),
End Collection, ; End of Present Status collection
End Collection, ; End of Power Summary collection
End Collection() ; End of UPS collection */


// https://networkupstools.org/docs/man/usbhid-ups.html
// https://github.com/networkupstools/nut/blob/master/drivers/usbhid-ups.c

// version=4
// 0x1d6b,,NOT_UPS
// 0x5e3,0x608,NOT_UPS
// 0x46d,0x826,NOT_UPS
// 0x1005,0xb155,NOT_UPS
// 0xb05,0x17ba,NOT_UPS
// 0x1c05,0x3074,NOT_UPS
// 0x1c05,0x2074,NOT_UPS
// 0x3f0,0xe207,NOT_UPS
// 0x51d,,usbhid-ups
// 0x463,,usbhid-ups
// 0x764,,usbhid-ups
// 0x9ae,,usbhid-ups
// 0x50d,,usbhid-ups
// 0xd9f,0x4,usbhid-ups
// 0x665,0x5161,blazer_usb
// 0x590,0x43,omron_usb
// 0x590,0x44,omron_usb
// 0x590,0x4b,omron_usb
// 0x590,0x4f,omron_usb
// 0x590,0x50,omron_usb
// 0x590,0x54,omron_usb
// 0x590,0x57,omron_usb
// 0x590,0x58,omron_usb
// 0x590,0x65,omron_usb
// 0x590,0x66,omron_usb
// 0x590,0x6b,omron_usb
// 0x590,0x6c,omron_usb
// 0x590,0x6e,omron_usb
// 0x590,0x6f,omron_usb
// 0x590,0x70,omron_usb
// 0x590,0x71,omron_usb
// 0x590,0x72,omron_usb
// 0x590,0x73,omron_usb
// 0x590,0x75,omron_usb
// 0x590,0x76,omron_usb
// 0x590,0x77,omron_usb
// 0x590,0x80,omron_usb
// 0x590,0x81,omron_usb
// 0x590,0x83,omron_usb
// 0x590,0x86,omron_usb
// 0x590,0x87,omron_usb
// 0x590,0xa1,omron_usb
// 0x590,0xa3,omron_usb
// 0x590,0xad,omron_usb
// 0x590,0xae,omron_usb
// 0x590,0xaf,omron_usb
// 0x590,0xb4,omron_usb
// 0x590,0xb5,omron_usb
// 0x590,0xb6,omron_usb
// 0x590,0xb7,omron_usb
// 0x590,0xb8,omron_usb
// 0x590,0xb9,omron_usb




































// //2025-09-10-14-38  备份
// #include <stdio.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "esp_err.h"
// #include "nvs_flash.h"
// #include "tusb.h"
// #include "class/hid/hid.h"
// #include "tinyusb.h"

// static const char *TAG = "HID_UPS_Device";

// // -------------------------- 1. 基础宏定义（解决编译错误） --------------------------
// #define LO8(x) ((x) & 0xFF)       // 提取16位数值的低字节
// #define HI8(x) ((x) >> 8)         // 提取16位数值的高字节

// // -------------------------- 2. HID 报告 ID 定义 --------------------------
// #define HID_PD_IPRODUCT              0x01
// #define HID_PD_SERIAL                0x02
// #define HID_PD_MANUFACTURER          0x03
// #define HID_PD_RECHARGEABLE          0x06
// #define HID_PD_PRESENTSTATUS         0x07
// #define HID_PD_REMAININGCAPACITY     0x0C
// #define HID_PD_VOLTAGE               0x0B
// #define HID_PD_RUNTIMETOEMPTY        0x0D
// #define HID_PD_AVERAGETIME2EMPTY     0x1C

// // 字符串索引
// #define IMANUFACTURER 0x01
// #define IPRODUCT      0x02
// #define ISERIAL       0x03
// #define IDEVICECHEMISTRY 0x04

// #define HID_PROTOCOL_NONE 0

// // -------------------------- 3. 电源状态结构体 --------------------------
// struct PresentStatus {
//     uint8_t Charging : 1;
//     uint8_t Discharging : 1;
//     uint8_t ACPresent : 1;
//     uint8_t BatteryPresent : 1;
//     uint8_t BelowRemainingCapacityLimit : 1;
//     uint8_t RemainingTimeLimitExpired : 1;
//     uint8_t NeedReplacement : 1;
//     uint8_t VoltageNotRegulated : 1;
    
//     uint8_t FullyCharged : 1;
//     uint8_t FullyDischarged : 1;
//     uint8_t ShutdownRequested : 1;
//     uint8_t ShutdownImminent : 1;
//     uint8_t CommunicationLost : 1;
//     uint8_t Overload : 1;
//     uint8_t unused1 : 1;
//     uint8_t unused2 : 1;
// };

// // 结构体转 uint16_t（二进制兼容）
// static inline uint16_t PresentStatus_to_uint16(const struct PresentStatus* ps) {
//     return *(const uint16_t*)(ps);
// }

// // -------------------------- 4. HID 报告描述符（关键：Windows 识别为电源设备） --------------------------
// const uint8_t hid_report_descriptor[] = {
//     0x05, 0x84,        // USAGE_PAGE (Power Device)
//     0x09, 0x04,        // USAGE (UPS) → 明确声明为UPS设备
//     0xA1, 0x01,        // COLLECTION (Application)
//         // 0x09, 0x24,    //   USAGE (Sink)   //删除SINK，兼容QNAP
//         0x09, 0x30,    //   USAGE (Battery)
//         0xA1, 0x02,    //   COLLECTION (Logical)

//             // 全局设置：统一报告格式（1字节ID + N字节数据）
//             0x75, 0x08,                    //     REPORT_SIZE (8)
//             0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
//             0x26, 0xFF, 0x00,              //     LOGICAL_MAXIMUM (255)

//             // -------------------------- 强制：设备能力报告（ID=0x0F） --------------------------
//             0x05, 0x84,                    //     USAGE_PAGE (Power Device)
//             0x85, 0x0F,                    //     REPORT_ID (15) → Windows强制ID
//             0x09, 0x01,                    //     USAGE (Device Capabilities)
//             0x95, 0x02,                    //     REPORT_COUNT (2) → 总长度3字节（1+2）
//             0x26, 0xFF, 0x00,              //     LOGICAL_MAXIMUM (255) → 替换原PHYSICAL_MAXIMUM
//             0xB1, 0x23,                    //     FEATURE (Constant, Nonvolatile) → 固定不可变

//             // -------------------------- 强制：电池化学类型报告（ID=0x04） --------------------------
//             0x05, 0x85,                    //     USAGE_PAGE (Battery System)
//             0x85, 0x04,                    //     REPORT_ID (4) → Windows强制ID
//             0x09, 0x8D,                    //     USAGE (Device Chemistry)
//             0x95, 0x02,                    //     REPORT_COUNT (2) → 总长度3字节
//             0x79, IDEVICECHEMISTRY,        //     STRING INDEX (4) → 关联Li-ion字符串
//             0xB1, 0x23,                    //     FEATURE (Constant, Nonvolatile)

//             // -------------------------- 原有报告修正（确保ID和长度匹配） --------------------------
//             // 产品信息（ID=1）
//             0x05, 0x84,                    //     USAGE_PAGE (Power Device)
//             0x85, HID_PD_IPRODUCT,         //     REPORT_ID (1)
//             0x09, 0xFE,                    //     USAGE (iProduct)
//             0x95, 0x02,                    //     REPORT_COUNT (2) → 总长度3
//             0x79, IPRODUCT,                //     STRING INDEX (2)
//             0xB1, 0x23,                    //     FEATURE (Constant, Nonvolatile)
            
//             // 序列号（ID=2）
//             0x85, HID_PD_SERIAL,           //     REPORT_ID (2)
//             0x09, 0xFF,                    //     USAGE (iSerialNumber)
//             0x95, 0x02,                    //     REPORT_COUNT (2)
//             0x79, ISERIAL,                 //     STRING INDEX (3)
//             0xB1, 0x23,                    //     FEATURE (Constant, Nonvolatile)
            
//             // 制造商（ID=3）
//             0x85, HID_PD_MANUFACTURER,     //     REPORT_ID (3)
//             0x09, 0xFD,                    //     USAGE (iManufacturer)
//             0x95, 0x02,                    //     REPORT_COUNT (2)
//             0x79, IMANUFACTURER,           //     STRING INDEX (1)
//             0xB1, 0x23,                    //     FEATURE (Constant, Nonvolatile)
            
//             // 可充电属性（ID=6）
//             0x05, 0x85,                    //     USAGE_PAGE (Battery System)
//             0x85, HID_PD_RECHARGEABLE,     //     REPORT_ID (6)
//             0x09, 0x8B,                    //     USAGE (Rechargable)
//             0x95, 0x02,                    //     REPORT_COUNT (2)
//             0x25, 0x01,                    //     LOGICAL_MAXIMUM (1) → 明确0=不可充，1=可充
//             0xB1, 0x23,                    //     FEATURE (Constant, Nonvolatile)
            
//             // 电源状态（ID=7，16位数据）
//             0x85, HID_PD_PRESENTSTATUS,    //     REPORT_ID (7)
//             0x09, 0x44,                    //     USAGE (Charging)
//             0x75, 0x10,                    //     REPORT_SIZE (16)
//             0x95, 0x01,                    //     REPORT_COUNT (1) → 2字节数据，总长度3
//             0x26, 0xFF, 0xFF,              //     LOGICAL_MAXIMUM (65535)
//             0x81, 0xA3,                    //     INPUT (Variable, Absolute, Volatile)
            
//             // 剩余容量（ID=12，8位数据+填充）
//             0x85, HID_PD_REMAININGCAPACITY,//     REPORT_ID (12)
//             0x09, 0x66,                    //     USAGE (RemainingCapacity)
//             0x75, 0x08,                    //     REPORT_SIZE (8)
//             0x95, 0x02,                    //     REPORT_COUNT (2) → 1字节数据+1填充，总长度3
//             0x25, 0x64,                    //     LOGICAL_MAXIMUM (100) → 0-100%，符合Windows预期
//             0x81, 0xA3,                    //     INPUT (Variable, Absolute, Volatile)
            
//             // 电压（ID=11，16位数据，单位修正）
//             0x85, HID_PD_VOLTAGE,          //     REPORT_ID (11)
//             0x09, 0x30,                    //     USAGE (Voltage)
//             0x75, 0x10,                    //     REPORT_SIZE (16)
//             0x95, 0x01,                    //     REPORT_COUNT (1) → 2字节数据，总长度3
//             0x26, 0xFF, 0xFF,              //     LOGICAL_MAXIMUM (65535) QNAP 要求支持最大时间值/Windows不接受65535
//             0x66, 0x01, 0x10,              //     UNIT (Volts)
//             0x55, 0x02,                    //     UNIT_EXPONENT (-2) → 实际电压=数据*0.01V，正确
//             0x81, 0xA3,                    //     INPUT (Variable, Absolute, Volatile)
            
//             // 剩余运行时间（ID=13）
//             0x85, HID_PD_RUNTIMETOEMPTY,   //     REPORT_ID (13)
//             0x09, 0x68,                    //     USAGE (RunTimeToEmpty)
//             0x75, 0x10,                    //     REPORT_SIZE (16)
//             0x95, 0x01,                    //     REPORT_COUNT (1)
//             0x26, 0xFE, 0xFF,              //     LOGICAL_MAXIMUM (65534) → Windows不接受65535（无效值）
//             0x66, 0x01, 0x10,              //     UNIT (Seconds)
//             0x55, 0x00,                    //     UNIT_EXPONENT (0)
//             0x81, 0xA3,                    //     INPUT (Variable, Absolute, Volatile)
            
//             // 平均放电时间（ID=28）
//             0x85, HID_PD_AVERAGETIME2EMPTY,//     REPORT_ID (28)
//             0x09, 0x69,                    //     USAGE (AverageTimeToEmpty)
//             0x75, 0x10,                    //     REPORT_SIZE (16)
//             0x95, 0x01,                    //     REPORT_COUNT (1)
//             0x26, 0xFE, 0xFF,              //     LOGICAL_MAXIMUM (65534)
//             0x66, 0x01, 0x10,              //     UNIT (Seconds)
//             0x55, 0x00,                    //     UNIT_EXPONENT (0)
//             0x81, 0xA3,                    //     INPUT (Variable, Absolute, Volatile)
//         0xC0,                              //   END_COLLECTION
//     0xC0                                   // END_COLLECTION
// };

// // -------------------------- 5. 报告结构体定义 --------------------------
// typedef struct {
//     uint8_t report_id;
//     uint8_t capacity;  // 0-100%
// } remaining_capacity_report_t;

// typedef struct {
//     uint8_t report_id;
//     uint16_t voltage;  // 原始值（实际电压=原始值*0.01V）
// } voltage_report_t;

// typedef struct {
//     uint8_t report_id;
//     uint16_t seconds;  // 剩余时间（秒）
// } time_report_t;


// // -------------------------- 6. USB 设备描述符 --------------------------
// tusb_desc_device_t descriptor_dev = {
//     .bLength = sizeof(tusb_desc_device_t),
//     .bDescriptorType = TUSB_DESC_DEVICE,
//     .bcdUSB = 0x0200,          // USB 2.0 兼容
//     .bDeviceClass = 0x00,      // 接口级定义设备类
//     .bDeviceSubClass = 0x00,
//     .bDeviceProtocol = 0x00,
//     .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
//     .idVendor = 0x0665,         // 改为：QNAP 官方兼容的 UPS 厂商 ID（CyberPower，QNAP 原生支持）
//     .idProduct = 0x5161,        // 改为：QNAP 验证通过的产品 ID（CyberPower CP1500PFCLCD 型号）
//     .bcdDevice = 0x0100,       // 设备版本 1.0
//     .iManufacturer = IMANUFACTURER,
//     .iProduct = IPRODUCT,
//     .iSerialNumber = ISERIAL,
//     .bNumConfigurations = 0x01
// };

// // -------------------------- 7. 自定义 HID 接口描述符（修复编译错误） --------------------------
// const uint8_t hid_interface_desc[] = {
//     // 接口描述符（9字节）
//     0x09,                    // bLength: 接口描述符长度
//     0x04,                    // bDescriptorType: 接口描述符类型
//     0x00,                    // bInterfaceNumber: 接口编号（0）
//     0x00,                    // bAlternateSetting: 备用接口（0）
//     0x01,                    // bNumEndpoints: 端点数量（1个输入端点）
//     0x03,                    // bInterfaceClass: HID 类（0x03）
//     0x00,                    // bInterfaceSubClass: 无子类
//     0x00,                    // bInterfaceProtocol: 无协议
//     0x00,                    // iInterface: 接口字符串索引（0=无）

//     // HID 描述符（9字节）
//     0x09,                    // bLength: HID 描述符长度
//     0x21,                    // bDescriptorType: HID 类型（0x21）
//     0x11, 0x01,              // bcdHID: HID 1.11 版本（小端序）
//     0x00,                    // bCountryCode: 无国家码
//     0x01,                    // bNumDescriptors: 报告描述符数量（1）
//     0x22,                    // bDescriptorType: 报告描述符类型（0x22）
//     LO8(sizeof(hid_report_descriptor)),  // wItemLength: 报告描述符长度（低字节）
//     HI8(sizeof(hid_report_descriptor)),  // wItemLength: 报告描述符长度（高字节）

//     // 输入端点描述符（7字节）
//     0x07,                    // bLength: 端点描述符长度
//     0x05,                    // bDescriptorType: 端点描述符类型
//     0x81,                    // bEndpointAddress: 输入端点（0x81，EP1 IN）
//     0x03,                    // bmAttributes: 中断传输（0x03）
//     LO8(CFG_TUD_HID_EP_BUFSIZE),  // wMaxPacketSize: 最大包大小（低字节）
//     HI8(CFG_TUD_HID_EP_BUFSIZE),  // wMaxPacketSize: 最大包大小（高字节）
//     0x0A                     // bInterval: 报告间隔（10ms）
// };

// #define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + sizeof(hid_interface_desc))

// // -------------------------- 8. USB 配置描述符（正确拼接接口描述符） --------------------------
// #define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + sizeof(hid_interface_desc))
// uint8_t const desc_configuration[] = {
//     // 配置描述符头部（9字节）
//     TUD_CONFIG_DESCRIPTOR(
//         1,                  // 配置编号
//         1,                  // 接口数量
//         0,                  // 配置字符串索引
//         TUSB_DESC_TOTAL_LEN, // 配置总长度
//         TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP,
//         100                 // 最大功耗（100mA）
//     ),

//     // 拼接自定义 HID 接口描述符（25字节：9+9+7）
//     hid_interface_desc[0],  hid_interface_desc[1],  hid_interface_desc[2],
//     hid_interface_desc[3],  hid_interface_desc[4],  hid_interface_desc[5],
//     hid_interface_desc[6],  hid_interface_desc[7],  hid_interface_desc[8],
//     hid_interface_desc[9],  hid_interface_desc[10], hid_interface_desc[11],
//     hid_interface_desc[12], hid_interface_desc[13], hid_interface_desc[14],
//     hid_interface_desc[15], hid_interface_desc[16], hid_interface_desc[17],
//     hid_interface_desc[18], hid_interface_desc[19], hid_interface_desc[20],
//     hid_interface_desc[21], hid_interface_desc[22], hid_interface_desc[23],
//     hid_interface_desc[24]
// };

// // -------------------------- 9. USB 字符串描述符 --------------------------
// const char* descriptor_str[] = {
//     [0] = "en",                    // 语言ID (英语)
//     [IMANUFACTURER] = "ESP32 UPS", // 制造商
//     [IPRODUCT] = "ESP32 HID UPS",// 产品名（Windows 显示）
//     [ISERIAL] = "ESP32-123456",    // 序列号
//     [IDEVICECHEMISTRY] = "Li-ion"  // 电池类型
// };

// // -------------------------- 10. 全局状态变量 --------------------------
// static struct PresentStatus UPS = {
//     .ACPresent = 1,        // 初始接AC电源
//     .BatteryPresent = 1,   // 电池存在
//     .Charging = 1,         // 初始充电中
//     .FullyCharged = 0      // 未充满
// };
// static uint8_t battery_capacity = 50;  // 初始50%容量
// static uint16_t battery_voltage = 1200; // 12.00V (1200 * 0.01V)
// static uint16_t runtime_to_empty = 3600; // 1小时
// static uint16_t avg_time_to_empty = 3600;

// // -------------------------- 11. TinyUSB 回调函数 --------------------------
// uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
//     return hid_report_descriptor;
// }

// uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
//                                hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
//     // 所有功能报告均返回3字节（1字节ID + 2字节数据）
//     buffer[0] = report_id; // 第1字节固定为报告ID

//     if (report_type == HID_REPORT_TYPE_FEATURE) {
//         switch(report_id) {
//             case 0x0F: // 设备能力报告（ID=15）
//                 buffer[1] = 0x07; // 低字节：支持UPS+电池+Sink（0b0111）
//                 buffer[2] = 0x00; // 高字节填充
//                 return 3;

//             case HID_PD_IPRODUCT: // 产品信息（ID=1）
//                 buffer[1] = IPRODUCT;    // 产品字符串索引（2）
//                 buffer[2] = 0x00;        // 填充字节
//                 return 3;

//             case HID_PD_SERIAL: // 序列号（ID=2）
//                 buffer[1] = ISERIAL;     // 序列号字符串索引（3）
//                 buffer[2] = 0x00;        // 填充字节
//                 return 3;

//             case HID_PD_MANUFACTURER: // 制造商（ID=3）
//                 buffer[1] = IMANUFACTURER; // 制造商字符串索引（1）
//                 buffer[2] = 0x00;          // 填充字节
//                 return 3;

//             case HID_PD_RECHARGEABLE: // 可充电属性（ID=6）
//                 buffer[1] = 0x01; // 1=可充电，0=不可充电
//                 buffer[2] = 0x00; // 填充字节
//                 return 3;

//             default:
//                 // 未知报告ID，返回3字节默认数据
//                 buffer[1] = 0x00;
//                 buffer[2] = 0x00;
//                 return 3;
//         }
//     }

//     // 输入报告（主机主动读取时）
//     switch(report_id) {
//         case HID_PD_PRESENTSTATUS: {
//             uint16_t status = PresentStatus_to_uint16(&UPS);
//             buffer[1] = (uint8_t)(status & 0xFF);
//             buffer[2] = (uint8_t)(status >> 8);
//             return 3;
//         }
//         case HID_PD_REMAININGCAPACITY:
//             buffer[1] = battery_capacity;
//             buffer[2] = 0x00; // 填充
//             return 3;
//         case HID_PD_VOLTAGE:
//             buffer[1] = (uint8_t)(battery_voltage & 0xFF);
//             buffer[2] = (uint8_t)(battery_voltage >> 8);
//             return 3;
//         default:
//             return 3;
//     }
// }

// // 处理主机发送的输出报告
// void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
//                            hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
//     ESP_LOGI(TAG, "Set report: ID=%u, Type=%u, Size=%u", report_id, report_type, bufsize);
    
//     // Windows可能会发送控制命令，需要正确响应
//     if (report_type == HID_REPORT_TYPE_OUTPUT) {
//         // 处理输出报告
//         tud_hid_report(report_id, buffer, bufsize);
//     }
// }

// // 处理GET_PROTOCOL请求
// uint8_t tud_hid_get_protocol_cb(uint8_t instance) {
//     return HID_PROTOCOL_NONE;
// }

// // 处理SET_PROTOCOL请求
// void tud_hid_set_protocol_cb(uint8_t instance, uint8_t protocol) {
//     ESP_LOGI(TAG, "Protocol set to: %u", protocol);
// }

// // void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
// //                            hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
// //     ESP_LOGI(TAG, "Host set report: ID=%d, Size=%d", report_id, bufsize);
// // }

// // -------------------------- 12. USB 初始化 --------------------------
// static void usb_hid_init(void) {
//     const tinyusb_config_t tusb_cfg = {
//         .device_descriptor = &descriptor_dev,
//         .configuration_descriptor = desc_configuration,
//         .string_descriptor = descriptor_str,
//         .string_descriptor_count = sizeof(descriptor_str) / sizeof(descriptor_str[0]),
//         .external_phy = false,
//     };

//     ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
//     ESP_LOGI(TAG, "TinyUSB initialized");

//     while (!tud_mounted()) {
//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
//     ESP_LOGI(TAG, "USB connected");
// }

// // -------------------------- 13. 模拟 UPS 状态变化 --------------------------
// static void update_ups_state(void) {
//     // 模拟AC电源断开/连接（30秒切换一次）
//     static uint32_t ac_timer = 0;
//     if (xTaskGetTickCount() - ac_timer > pdMS_TO_TICKS(30000)) {
//         UPS.ACPresent ^= 1;
//         ac_timer = xTaskGetTickCount();
        
//         if (UPS.ACPresent) {
//             UPS.Charging = 1;
//             UPS.Discharging = 0;
//             ESP_LOGI(TAG, "AC connected - charging");
//         } else {
//             UPS.Charging = 0;
//             UPS.Discharging = 1;
//             ESP_LOGI(TAG, "AC disconnected - discharging");
//         }
//     }

//     // 模拟电池容量变化
//     if (UPS.Charging && battery_capacity < 100) {
//         battery_capacity++;
//         if (battery_capacity >= 100) {
//             UPS.FullyCharged = 1;
//             UPS.Charging = 0;
//         }
//     } else if (UPS.Discharging && battery_capacity > 0) {
//         battery_capacity--;
//         UPS.FullyCharged = 0;
//         if (battery_capacity <= 0) {
//             UPS.FullyDischarged = 1;
//         }
//     }

//     // 更新剩余时间（基于容量）
//     runtime_to_empty = battery_capacity * 60; // 1分钟/1%容量
//     avg_time_to_empty = runtime_to_empty;
// }

// // -------------------------- 14. HID 报告发送任务 --------------------------
// static void hid_send_task(void *pvParameters) {
//     while (1) {
//         if (tud_mounted() && tud_hid_ready()) {

//             ESP_LOGI(TAG, "设备已挂载，准备发送报告");  // 新增日志

//             // 更新UPS状态
//             update_ups_state();

//             // 1. 发送状态报告（3字节：ID + 2字节状态数据）
//             uint16_t status = PresentStatus_to_uint16(&UPS);
//             uint8_t status_buf[3] = {
//                 HID_PD_PRESENTSTATUS,       // 报告ID（1字节）
//                 (uint8_t)(status & 0xFF),   // 状态低字节（2字节数据）
//                 (uint8_t)(status >> 8)      // 状态高字节
//             };
//             tud_hid_report(HID_PD_PRESENTSTATUS, status_buf, 3);


//             // 2. 发送剩余容量报告（3字节：ID + 容量 + 填充字节）
//             uint8_t cap_buf[3] = {
//                 HID_PD_REMAININGCAPACITY,   // 报告ID（1字节）
//                 battery_capacity,           // 容量数据（1字节）
//                 0x00                        // 填充字节（确保总长度3字节）
//             };
//             tud_hid_report(HID_PD_REMAININGCAPACITY, cap_buf, 3);


//             // 3. 发送电压报告（3字节：ID + 2字节电压数据）
//             uint8_t volt_buf[3] = {
//                 HID_PD_VOLTAGE,             // 报告ID（1字节）
//                 (uint8_t)(battery_voltage & 0xFF),  // 电压低字节
//                 (uint8_t)(battery_voltage >> 8)     // 电压高字节
//             };
//             tud_hid_report(HID_PD_VOLTAGE, volt_buf, 3);


//             // 4. 发送剩余运行时间报告（3字节：ID + 2字节时间数据）
//             uint8_t time_buf[3] = {
//                 HID_PD_RUNTIMETOEMPTY,      // 报告ID（1字节）
//                 (uint8_t)(runtime_to_empty & 0xFF),  // 时间低字节
//                 (uint8_t)(runtime_to_empty >> 8)     // 时间高字节
//             };
//             tud_hid_report(HID_PD_RUNTIMETOEMPTY, time_buf, 3);


//             ESP_LOGI(TAG, "Capacity: %d%%, Voltage: %.2fV, Runtime: %ds",
//                      battery_capacity, battery_voltage * 0.01f, runtime_to_empty);

//         }else {
//             ESP_LOGW(TAG, "设备未挂载或HID未就绪");  // 新增：判断设备是否连接
//         }

//         vTaskDelay(pdMS_TO_TICKS(5000)); // 1秒更新一次
//     }
// }

// // -------------------------- 15. 主函数 --------------------------
// void app_main(void) {
//     ESP_LOGI(TAG, "HID UPS Device (ESP32 + TinyUSB)");

//     // 初始化NVS
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     // 初始化USB HID
//     usb_hid_init();

//     // 创建HID发送任务
//     xTaskCreate(hid_send_task, "hid_send_task", 8192, NULL, 5, NULL);
// }





















// //ESP32-S3 可以运行，但是感觉版本不对
// #include <stdio.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "esp_err.h"
// #include "nvs_flash.h"
// #include "tusb.h"                  // ESP-IDF 5.4.2 TinyUSB主头文件
// #include "class/hid/hid.h"         // HID核心头文件
// #include "tinyusb.h"               // ESP-IDF TinyUSB配置头文件

// static const char *TAG = "HID_Power_Device";

// // -------------------------- 1. 手动定义缺失的宏 --------------------------
// #ifndef HID_PROTOCOL_NONE
// #define HID_PROTOCOL_NONE 0
// #endif
// #ifndef LOWBYTE
// #define LOWBYTE(x) ((uint8_t)((x) & 0xFF))
// #endif
// #ifndef HIGHBYTE
// #define HIGHBYTE(x) ((uint8_t)(((x) >> 8) & 0xFF))
// #endif

// // -------------------------- 2. HID报告描述符（简化版） --------------------------
// const uint8_t hid_report_descriptor[] = {
//     0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
//     0x09, 0x00,        // USAGE (Undefined)
//     0xA1, 0x01,        // COLLECTION (Application)
//         0x85, 0x01,    //   REPORT_ID (1)
//         0x09, 0x00,    //   USAGE (Undefined)
//         0x15, 0x00,    //   LOGICAL_MINIMUM (0)
//         0x25, 0xFF,    //   LOGICAL_MAXIMUM (255)
//         0x75, 0x08,    //   REPORT_SIZE (8)
//         0x95, 0x01,    //   REPORT_COUNT (1)
//         0x81, 0x02,    //   INPUT (Data,Var,Abs)
//     0xC0,              // END_COLLECTION
// };

// // HID报告结构体
// typedef struct __attribute__((packed)) {
//     uint8_t report_id;  // 报告ID=1
//     uint8_t power_data; // 电源状态（0=低电，255=满电）
// } hid_power_report_t;

// // -------------------------- 3. USB设备描述符 --------------------------
// tusb_desc_device_t descriptor_dev = {
//     .bLength = sizeof(tusb_desc_device_t),
//     .bDescriptorType = TUSB_DESC_DEVICE,
//     .bcdUSB = 0x0200,          // USB 2.0（避免BOS描述符要求）
//     .bDeviceClass = TUSB_CLASS_HID,       // 明确为HID类
//     .bDeviceSubClass = 0x00,
//     .bDeviceProtocol = 0x00,
//     .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,  // 由menuconfig定义（默认64）
//     .idVendor = 0x1234,        // 自定义Vendor ID
//     .idProduct = 0x5678,       // 自定义Product ID
//     .bcdDevice = 0x0100,       // 设备版本
//     .iManufacturer = 0x01,     // 制造商字符串索引
//     .iProduct = 0x02,          // 产品名字符串索引
//     .iSerialNumber = 0x03,     // 序列号字符串索引
//     .bNumConfigurations = 0x01 // 1个配置
// };

// // -------------------------- 4. USB配置描述符（关键：必须提供） --------------------------
// // 配置描述符总长度 = 配置描述符长度 + HID接口描述符长度
// #define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)

// // 配置描述符（包含HID接口，端点地址0x82）
// uint8_t const desc_configuration[] = {
//     // 配置描述符头部（TUD_CONFIG_DESCRIPTOR宏自动生成标准格式）
//     TUD_CONFIG_DESCRIPTOR(
//         1,                  // 配置编号
//         1,                  // 接口数量
//         0,                  // 配置字符串索引
//         TUSB_DESC_TOTAL_LEN, // 配置总长度
//         TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, // 支持远程唤醒
//         100                 // 最大功耗（100mA）
//     ),
//     // HID接口描述符（TUD_HID_DESCRIPTOR宏自动生成，避免手动拼接错误）
//     TUD_HID_DESCRIPTOR(
//         0,                  // 接口编号
//         0,                  // 接口字符串索引
//         HID_PROTOCOL_NONE,  // 无HID协议
//         sizeof(hid_report_descriptor), // HID报告描述符长度
//         0x82,               // 输入端点地址（0x82 = 端点2，IN方向）
//         64,                 // 端点最大包长
//         10                  // 报告间隔（10ms）
//     )
// };

// // -------------------------- 5. USB字符串描述符 --------------------------
// const char* descriptor_str[] = {
//     [0] = "en",                // 语言描述符（英语）
//     [1] = "ESP32 Manufacturer",// 制造商
//     [2] = "HID Power Device",  // 产品名（Windows显示的设备名）
//     [3] = "ESP32S3-123456"     // 序列号
// };

// // -------------------------- 6. TinyUSB回调函数 --------------------------
// // 提供HID报告描述符
// uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
//     ESP_LOGI(TAG, "HID Report Descriptor requested");
//     return hid_report_descriptor;
// }

// // 处理主机获取HID报告的请求
// uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, 
//                                hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
//     if (report_id == 0x01 && reqlen >= sizeof(hid_power_report_t)) {
//         hid_power_report_t report = {
//             .report_id = 0x01,
//             .power_data = 255  // 初始为满电
//         };
//         memcpy(buffer, &report, sizeof(report));
//         return sizeof(report);
//     }
//     return 0;
// }

// // 处理主机设置HID报告的请求（空实现）
// void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, 
//                            hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
//     ESP_LOGI(TAG, "Set Report: ID=%d, Size=%d", report_id, bufsize);
// }

// // -------------------------- 7. USB初始化（修复：新增configuration_descriptor） --------------------------
// static void usb_hid_init(void) {
//     // TinyUSB配置（关键：新增configuration_descriptor成员！）
//     const tinyusb_config_t tusb_cfg = {
//         .device_descriptor = &descriptor_dev,          // 设备描述符
//         .configuration_descriptor = desc_configuration, // 配置描述符（之前遗漏，导致错误）
//         .string_descriptor = descriptor_str,           // 字符串描述符
//         .string_descriptor_count = sizeof(descriptor_str) / sizeof(descriptor_str[0]), // 字符串数量
//         .external_phy = false,  // ESP32S3使用内置PHY
//     };

//     // 安装TinyUSB驱动（现在参数完整，不会报错）
//     ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
//     ESP_LOGI(TAG, "TinyUSB driver installed successfully");

//     // 等待USB主机（Windows）连接
//     while (!tud_mounted()) {
//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
//     ESP_LOGI(TAG, "USB connected to Windows (HID device ready)");
// }

// // -------------------------- 8. HID报告发送任务 --------------------------
// static void hid_send_task(void *pvParameters) {
//     hid_power_report_t report = {
//         .report_id = 0x01,
//         .power_data = 255  // 初始满电
//     };
//     int8_t data_dir = -1;  // 数据变化方向（-1=递减，1=递增）

//     while (1) {
//         // 模拟电源状态循环（255→0→255）
//         ESP_LOGI(TAG, "power_data: %d\n", report.power_data);
//         report.power_data += data_dir;
//         if (report.power_data <= 0) {
//             report.power_data = 0;
//             data_dir = 1;
//             ESP_LOGI(TAG, "Power status: Low (Need charge)");
//         } else if (report.power_data >= 255) {
//             report.power_data = 255;
//             data_dir = -1;
//             ESP_LOGI(TAG, "Power status: Full (Charged)");
//         }

//         // 向Windows发送HID报告（确保USB已连接且HID就绪）
//         if (tud_mounted() && tud_hid_ready()) {
//             bool ret = tud_hid_report(0, &report, sizeof(report));
//             if (ret) {
//                 ESP_LOGD(TAG, "Sent HID Report: ID=%d, Power=%d", 
//                          report.report_id, report.power_data);
//             }
//         }

//         vTaskDelay(pdMS_TO_TICKS(1000));  // 1秒发送一次
//     }
// }

// // -------------------------- 9. 主函数 --------------------------
// void app_main(void) {
//     ESP_LOGI(TAG, "HID Power Device (ESP32S3 + IDF 5.4.2)");

//     // 初始化NVS（ESP-IDF必要步骤）
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     // 初始化USB HID（现在参数完整，不会报错）
//     usb_hid_init();

//     // 创建HID报告发送任务
//     xTaskCreate(hid_send_task, "hid_send_task", 4096, NULL, 5, NULL);

//     ESP_LOGI(TAG, "All modules initialized (Task started)");
// }





////ESP32-P4-DEVICE
// #include <stdio.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "esp_err.h"
// #include "nvs_flash.h"
// #include "tusb.h"  // 包含 TinyUSB 主头文件
// #include "class/hid/hid.h"
// #include "tinyusb.h"


// static const char *TAG = "HID_Power_Device";

// // // 计算描述符总长度：配置描述符(9) + 接口描述符(9) + HID描述符(9) + 端点描述符(7) + 端点描述符(7)
// // #define TUSB_DESC_TOTAL_LEN (9 + 9 + 9 + 7 + 7)

// // 计算描述符总长度  //高速
// #define TUSB_DESC_TOTAL_LEN (9 + 9 + 9 + 7)  // 配置+接口+HID+输入端点


// // 如果编译器仍然找不到 HID_PROTOCOL_NONE，手动定义它
// #ifndef HID_PROTOCOL_NONE
// #define HID_PROTOCOL_NONE 0
// #define HID_PROTOCOL_KEYBOARD 1
// #define HID_PROTOCOL_MOUSE 2
// #endif

// // HID 报告描述符 - Power Device (Usage Page 0x84)
// const uint8_t hid_report_descriptor[] = {
//     // Usage Page (Power Device)
//     0x05, 0x84,        // USAGE_PAGE (Power Device)
//     // Usage (Uninterruptible Power Supply)
//     0x09, 0x01,        // USAGE (Uninterruptible Power Supply)
    
//     // Collection (Application)
//     0xA1, 0x01,        // COLLECTION (Application)
    
//     // Report ID 1
//     0x85, 0x01,        //   REPORT_ID (1)
    
//     // Battery Capacity Mode (0=maH, 1=percentage)
//     0x09, 0x22,        //   USAGE (Battery Capacity Mode)
//     0x15, 0x00,        //   LOGICAL_MINIMUM (0)
//     0x25, 0x01,        //   LOGICAL_MAXIMUM (1)
//     0x75, 0x08,        //   REPORT_SIZE (8)
//     0x95, 0x01,        //   REPORT_COUNT (1)
//     0x81, 0x02,        //   INPUT (Data,Var,Abs)
    
//     // Battery Strength (0-100%)
//     0x09, 0x21,        //   USAGE (Battery Strength)
//     // ... 其他描述符内容
//     0xC0,              // END_COLLECTION
// };

// // HID 报告结构体
// typedef struct __attribute__((packed)) {
//     uint8_t report_id;
//     uint8_t capacity_mode;
//     uint8_t battery_strength;
//     uint8_t present_status;
//     uint8_t capacity_limit;
// } hid_power_report_t;

// // USB 设备描述符
// tusb_desc_device_t descriptor_dev = {
//     .bLength = sizeof(tusb_desc_device_t),
//     .bDescriptorType = TUSB_DESC_DEVICE,
//     .bcdUSB = 0x0210,
//     .bDeviceClass = TUSB_CLASS_MISC,
//     .bDeviceSubClass = MISC_SUBCLASS_COMMON,
//     .bDeviceProtocol = MISC_PROTOCOL_IAD,
//     .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
//     .idVendor = 0x1234,
//     .idProduct = 0x5678,
//     .bcdDevice = 0x0100,
//     .iManufacturer = 0x01,
//     .iProduct = 0x02,
//     .iSerialNumber = 0x03,
//     .bNumConfigurations = 0x01
// };

// // 设备资质描述符 - 告诉主机设备支持的速度能力
// tusb_desc_device_qualifier_t descriptor_dev_qualifier = {
//     .bLength = sizeof(tusb_desc_device_qualifier_t),
//     .bDescriptorType = TUSB_DESC_DEVICE_QUALIFIER,
//     .bcdUSB = 0x0200,
//     .bDeviceClass = TUSB_CLASS_MISC,
//     .bDeviceSubClass = MISC_SUBCLASS_COMMON,
//     .bDeviceProtocol = MISC_PROTOCOL_IAD,
//     .bMaxPacketSize0 = 64,
//     .bNumConfigurations = 1,
//     .bReserved = 0
// };

// // 字符串描述符
// const char* descriptor_str[] = {
//     [0] = "ESP32",              // 语言
//     [1] = "ESP32 Manufacturer", // 制造商
//     [2] = "HID Power Device",   // 产品
//     [3] = "123456"              // 序列号
// };



// // 配置描述符 - 同时支持全速和高速
// uint8_t const desc_configuration[] = {
//     // 全速配置描述符
//     TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    
//     // HID接口描述符
//     TUD_HID_DESCRIPTOR(0, 0, HID_PROTOCOL_NONE, sizeof(hid_report_descriptor), 0x81, 64, 10),
    
//     // // 高速配置描述符（需要添加）
//     // TUD_CONFIG_DESCRIPTOR(2, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    
//     // // 高速HID接口描述符
//     // TUD_HID_DESCRIPTOR(0, 0, HID_PROTOCOL_NONE, sizeof(hid_report_descriptor), 0x81, 512, 10)
// };


// // TinyUSB HID 回调函数实现
// uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance)
// {
//     ESP_LOGI(TAG, "HID Report Descriptor callback for instance %d", instance);
//     return hid_report_descriptor;
// }


// uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
// {
//     ESP_LOGI(TAG, "Get Report request: ID=%d, Type=%d, ReqLen=%d", report_id, report_type, reqlen);
    
//     if (report_id == 0x01 && reqlen >= sizeof(hid_power_report_t)) {
//         hid_power_report_t current_report = {
//             .report_id = 0x01,
//             .capacity_mode = 1,
//             .battery_strength = 95,
//             .present_status = 0,
//             .capacity_limit = 20
//         };
//         memcpy(buffer, &current_report, sizeof(current_report));
//         return sizeof(current_report);
//     }
//     return 0;
// }

// void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
// {
//     ESP_LOGI(TAG, "Set Report: ID=%d, Type=%d, Size=%d", report_id, report_type, bufsize);
// }

// void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len)
// {
//     ESP_LOGD(TAG, "Report sent complete, len=%d", len);
// }


// // 初始化 USB HID 设备
// static void usb_hid_init(void)
// {
//     // 配置 TinyUSB - 提供所有必需的描述符
//     const tinyusb_config_t tusb_cfg = {
//         .device_descriptor = &descriptor_dev,
//         .string_descriptor = descriptor_str,
//         .string_descriptor_count = sizeof(descriptor_str) / sizeof(descriptor_str[0]),
//         .external_phy = false,
//         .configuration_descriptor = desc_configuration, // 让 TinyUSB 自动生成配置描述符
//         .qualifier_descriptor = NULL, // 确保设备资质描述符为 NULL，这是关键！
//     };
    
//     // 安装 TinyUSB 驱动
//     ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
//     ESP_LOGI(TAG, "TinyUSB driver installed");

//     // 等待 USB 连接
//     ESP_LOGI(TAG, "Waiting for USB connection...");
//     while (!tud_mounted()) {
//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
//     ESP_LOGI(TAG, "USB connected");
// }

// // HID Power Device 任务
// static void hid_power_device_task(void *pvParameters)
// {
//     hid_power_report_t power_report = {
//         .report_id = 0x01,
//         .capacity_mode = 1,
//         .battery_strength = 100,
//         .present_status = 0,
//         .capacity_limit = 20
//     };

//     int8_t battery_direction = -1;

//     while (1) {
//         // 模拟电池电量变化
//         power_report.battery_strength += battery_direction;

//         // 更新电源状态
//         if (power_report.battery_strength <= 20) {
//             power_report.present_status = 2;
//             battery_direction = 1;
//         } else if (power_report.battery_strength >= 100) {
//             power_report.present_status = 0;
//             battery_direction = -1;
//         } else if (power_report.battery_strength > 20 && battery_direction > 0) {
//             power_report.present_status = 3;
//         } else if (power_report.battery_strength > 20 && battery_direction < 0) {
//             power_report.present_status = 1;
//         }

//         // 发送 HID 报告
//         if (tud_mounted() && tud_hid_ready()) {
//             bool result = tud_hid_report(0, &power_report, sizeof(power_report));
//             if (result) {
//                 ESP_LOGI(TAG, "Power Report: Battery=%d%%, Status=%d", 
//                         power_report.battery_strength, power_report.present_status);
//             }
//         }

//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }

// void app_main(void)
// {
//     ESP_LOGI(TAG, "Starting HID Power Device (UPS) example");

//     // 初始化 NVS
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     // 初始化 USB HID
//     usb_hid_init();

//     // 创建 HID Power Device 任务
//     xTaskCreate(hid_power_device_task, "hid_power_task", 4096, NULL, 5, NULL);

//     ESP_LOGI(TAG, "HID Power Device started successfully");
// }




















/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

// #include <stdlib.h>
// #include "esp_log.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "tinyusb.h"
// #include "class/hid/hid_device.h"
// #include "driver/gpio.h"

// #define APP_BUTTON (GPIO_NUM_0) // Use BOOT signal by default
// static const char *TAG = "example";

// /************* TinyUSB descriptors ****************/

// #define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

// /**
//  * @brief HID report descriptor
//  *
//  * In this example we implement Keyboard + Mouse HID device,
//  * so we must define both report descriptors
//  */
// const uint8_t hid_report_descriptor[] = {
//     TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)),
//     TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE))
// };

// /**
//  * @brief String descriptor
//  */
// const char* hid_string_descriptor[5] = {
//     // array of pointer to string descriptors
//     (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
//     "TinyUSB",             // 1: Manufacturer
//     "TinyUSB Device",      // 2: Product
//     "123456",              // 3: Serials, should use chip ID
//     "Example HID interface",  // 4: HID
// };

// /**
//  * @brief Configuration descriptor
//  *
//  * This is a simple configuration descriptor that defines 1 configuration and 1 HID interface
//  */
// static const uint8_t hid_configuration_descriptor[] = {
//     // Configuration number, interface count, string index, total length, attribute, power in mA
//     TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

//     // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
//     TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
// };

// /********* TinyUSB HID callbacks ***************/

// // Invoked when received GET HID REPORT DESCRIPTOR request
// // Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
// uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
// {
//     // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
//     return hid_report_descriptor;
// }

// // Invoked when received GET_REPORT control request
// // Application must fill buffer report's content and return its length.
// // Return zero will cause the stack to STALL request
// uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
// {
//     (void) instance;
//     (void) report_id;
//     (void) report_type;
//     (void) buffer;
//     (void) reqlen;

//     return 0;
// }

// // Invoked when received SET_REPORT control request or
// // received data on OUT endpoint ( Report ID = 0, Type = 0 )
// void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
// {
// }

// /********* Application ***************/

// typedef enum {
//     MOUSE_DIR_RIGHT,
//     MOUSE_DIR_DOWN,
//     MOUSE_DIR_LEFT,
//     MOUSE_DIR_UP,
//     MOUSE_DIR_MAX,
// } mouse_dir_t;

// #define DISTANCE_MAX        125
// #define DELTA_SCALAR        5

// static void mouse_draw_square_next_delta(int8_t *delta_x_ret, int8_t *delta_y_ret)
// {
//     static mouse_dir_t cur_dir = MOUSE_DIR_RIGHT;
//     static uint32_t distance = 0;

//     // Calculate next delta
//     if (cur_dir == MOUSE_DIR_RIGHT) {
//         *delta_x_ret = DELTA_SCALAR;
//         *delta_y_ret = 0;
//     } else if (cur_dir == MOUSE_DIR_DOWN) {
//         *delta_x_ret = 0;
//         *delta_y_ret = DELTA_SCALAR;
//     } else if (cur_dir == MOUSE_DIR_LEFT) {
//         *delta_x_ret = -DELTA_SCALAR;
//         *delta_y_ret = 0;
//     } else if (cur_dir == MOUSE_DIR_UP) {
//         *delta_x_ret = 0;
//         *delta_y_ret = -DELTA_SCALAR;
//     }

//     // Update cumulative distance for current direction
//     distance += DELTA_SCALAR;
//     // Check if we need to change direction
//     if (distance >= DISTANCE_MAX) {
//         distance = 0;
//         cur_dir++;
//         if (cur_dir == MOUSE_DIR_MAX) {
//             cur_dir = 0;
//         }
//     }
// }

// static void app_send_hid_demo(void)
// {
//     // Keyboard output: Send key 'a/A' pressed and released
//     ESP_LOGI(TAG, "Sending Keyboard report");
//     uint8_t keycode[6] = {HID_KEY_A};
//     tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode);
//     vTaskDelay(pdMS_TO_TICKS(50));
//     tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);

//     // Mouse output: Move mouse cursor in square trajectory
//     ESP_LOGI(TAG, "Sending Mouse report");
//     int8_t delta_x;
//     int8_t delta_y;
//     for (int i = 0; i < (DISTANCE_MAX / DELTA_SCALAR) * 4; i++) {
//         // Get the next x and y delta in the draw square pattern
//         mouse_draw_square_next_delta(&delta_x, &delta_y);
//         tud_hid_mouse_report(HID_ITF_PROTOCOL_MOUSE, 0x00, delta_x, delta_y, 0, 0);
//         vTaskDelay(pdMS_TO_TICKS(20));
//     }
// }

// void app_main(void)
// {
//     // Initialize button that will trigger HID reports
//     const gpio_config_t boot_button_config = {
//         .pin_bit_mask = BIT64(APP_BUTTON),
//         .mode = GPIO_MODE_INPUT,
//         .intr_type = GPIO_INTR_DISABLE,
//         .pull_up_en = true,
//         .pull_down_en = false,
//     };
//     ESP_ERROR_CHECK(gpio_config(&boot_button_config));

//     ESP_LOGI(TAG, "USB initialization");
//     const tinyusb_config_t tusb_cfg = {
//         .device_descriptor = NULL,
//         .string_descriptor = hid_string_descriptor,
//         .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
//         .external_phy = false,
// #if (TUD_OPT_HIGH_SPEED)
//         .fs_configuration_descriptor = hid_configuration_descriptor, // HID configuration descriptor for full-speed and high-speed are the same
//         .hs_configuration_descriptor = hid_configuration_descriptor,
//         .qualifier_descriptor = NULL,
// #else
//         .configuration_descriptor = hid_configuration_descriptor,
// #endif // TUD_OPT_HIGH_SPEED
//     };

//     ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
//     ESP_LOGI(TAG, "USB initialization DONE");

//     while (1) {
//         if (tud_mounted()) {
//             static bool send_hid_data = true;
//             if (send_hid_data) {
//                 app_send_hid_demo();
//             }
//             send_hid_data = !gpio_get_level(APP_BUTTON);
//         }
//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
// }
