#ifndef _COMMON_H_
#define _COMMON_H_
#include "arduino.h"
#include <avr/wdt.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
//#include <SD.h>
#define DE_BUG
#define GSM_UART Serial1
//#define RFID_UART Serial2
//#define WIFI_PROBE Serial3

#ifdef DE_BUG    //   在COMMON.h 中启用
  #define mySerial Serial  
#endif

#define DEV_ID "P1234567890"  //设备ID号


const int led = 2;        //白色led引脚定义D2
const int sw = 3;         //门禁关闭检测引脚D3
const int led_red = 6;    //红色led未上线亮、上线熄灭、刷卡时灭一下 D6
const int led_blue = 7;   //蓝色led未上线熄灭、上线亮、刷卡时灭一下 D7

const int gsm_reset = 38;  //GSM模块复位引脚定义 D38
const int rfid_reset = 4; //RFID刷卡模块复位引脚定义 D4

extern unsigned char locked_flag;

#endif
