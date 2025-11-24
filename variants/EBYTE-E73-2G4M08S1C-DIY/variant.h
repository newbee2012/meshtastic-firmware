/*
 Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
 Copyright (c) 2016 Sandeep Mistry All right reserved.
 Copyright (c) 2018, Adafruit Industries (adafruit.com)

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details.
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_EBYTE_E73_2G4M08S1C_DIY
#define _VARIANT_EBYTE_E73_2G4M08S1C_DIY

#define MESHTASTIC_EXCLUDE_WIFI 1
#define MESHTASTIC_EXCLUDE_MQTT 1
//#define MESHTASTIC_EXCLUDE_AUDIO 1             // 排除音频模块（语音、蜂鸣器等）
#define MESHTASTIC_EXCLUDE_DETECTIONSENSOR 1   // 排除动作/检测传感器模块
#define MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR 1 // 排除环境传感器（温湿度、气压、光照等）
#define MESHTASTIC_EXCLUDE_HEALTH_TELEMETRY 1  // 排除健康传感器数据采集（心率、血氧等）
//#define MESHTASTIC_EXCLUDE_EXTERNALNOTIFICATION 1 // 排除外部通知模块（外部事件触发消息）
#define MESHTASTIC_EXCLUDE_PAXCOUNTER 1        // 排除 PaxCounter 功能（统计人员流量）
#define MESHTASTIC_EXCLUDE_POWER_TELEMETRY 1   // 排除电源监控/电流电压采集模块
#define MESHTASTIC_EXCLUDE_RANGETEST 1         // 排除射程测试模块
//#define MESHTASTIC_EXCLUDE_REMOTEHARDWARE 1    // 排除远程硬件控制模块
//#define MESHTASTIC_EXCLUDE_STOREFORWARD 1      // 排除消息存储转发模块
//#define MESHTASTIC_EXCLUDE_TEXTMESSAGE 1       // 排除文本消息模块
#define MESHTASTIC_EXCLUDE_ATAK 1              // 排除 ATAK（Android Tactical Assault Kit）集成
#define MESHTASTIC_EXCLUDE_CANNEDMESSAGES 1    // 排除预设快速消息模块
#define MESHTASTIC_EXCLUDE_NEIGHBORINFO 1      // 排除邻居信息收集模块
//#define MESHTASTIC_EXCLUDE_TRACEROUTE 1        // 排除 Traceroute 路径测试模块
#define MESHTASTIC_EXCLUDE_WAYPOINT 1          // 排除地理标记/航点模块
#define MESHTASTIC_EXCLUDE_INPUTBROKER 1       // 排除输入代理模块（事件转发）
#define MESHTASTIC_EXCLUDE_SERIAL 1            // 排除串口接口模块
#define MESHTASTIC_EXCLUDE_POWERSTRESS 1       // 排除功率压力测试模块
//#define MESHTASTIC_EXCLUDE_ADMIN 1             // 排除管理员/管理功能模块



/** Master clock frequency */
#define VARIANT_MCK (64000000ul)

#define USE_LFXO // Board uses 32khz crystal for LF
//#define USE_LFRC    // Board uses RC for LF
/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define EBYTE_E73_2G4M08S1C_DIY

// Number of pins defined in PinDescription array
#define PINS_COUNT (48)
#define NUM_DIGITAL_PINS (48)
#define NUM_ANALOG_INPUTS (6)
#define NUM_ANALOG_OUTPUTS (0)

// // LEDs
#define PIN_LED1 (32 + 13)
#define LED_BLUE PIN_LED1
// #define LED_GREEN PIN_LED1
#define LED_BUILTIN PIN_LED1
#define LED_STATE_ON 1 // State when LED is lit

// // QSPI Pins
// #define PIN_QSPI_SCK (32 + 14)
// #define PIN_QSPI_CS (32 + 15)
// #define PIN_QSPI_IO0 (32 + 12) // MOSI if using two bit interface
// #define PIN_QSPI_IO1 (32 + 1) // MISO if using two bit interface
// #define PIN_QSPI_IO2 (0 + 7)   // WP if using two bit interface (i.e. not used)
// #define PIN_QSPI_IO3 (0 + 5)   // HOLD if using two bit interface (i.e. not used)

// // On-board QSPI Flash
// #define EXTERNAL_FLASH_DEVICES MX25R1635F
// #define EXTERNAL_FLASH_USE_QSPI

/*
 * Lora radio
 */
#define RADIOLIB_DEBUG_BASIC 1
#define SPI_INTERFACES_COUNT 1
#define PIN_SPI_NSS (26)
#define PIN_SPI_SCK (6)
#define PIN_SPI_MOSI (0 + 5)
#define PIN_SPI_MISO (32 + 9)
#define USE_SX1268 // E22-400M30S, E22-400M33S, E22-400M22S, and E22-400MM22S use SX1268
#define SX126X_DIO3_TCXO_VOLTAGE 1.8
#define TCXO_OPTIONAL
#define SX126X_MAX_POWER PIN_SPI_NSS // SX126xInterface.cpp defaults to 22 if not defined, but here we define it for good practice
static const uint8_t SS = PIN_SPI_NSS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK = PIN_SPI_SCK;
#define SX126X_CS PIN_SPI_NSS   // EBYTE module's NSS pin
#define SX126X_RESET (0 + 8) // EBYTE module's NRST pin
#define SX126X_BUSY (0 + 17)  // EBYTE module's BUSY pin
#define SX126X_DIO1 (0 + 15)  // EBYTE module's DIO1 pin
#define SX126X_DIO2 (0 + 7)  // EBYTE module's DIO2 pin
#define SX126X_TXEN (0 + 12)
#define SX126X_RXEN (0 + 4)


#define LORA_CS PIN_SPI_NSS     // Compatibility with variant file configuration structure
#define LORA_SCK PIN_SPI_SCK   // Compatibility with variant file configuration structure
#define LORA_RESET SX126X_RESET
#define LORA_MOSI PIN_SPI_MOSI // Compatibility with variant file configuration structure
#define LORA_MISO PIN_SPI_MISO // Compatibility with variant file configuration structure
 // Compatibility with variant file configuration structure
#define LORA_DIO0 -1        // a No connect on the SX1262/SX1268 module
#define LORA_DIO1 SX126X_DIO1
#define LORA_DIO2 SX126X_DIO2 // Compatibility with variant file configuration structure
#define LORA_DIO3 


// //SCREEN
#define WIRE_INTERFACES_COUNT 1
#define PIN_WIRE_SCL 31 // SCL     P0.29
#define PIN_WIRE_SDA 30 // SDA     P0.31
#define HAS_SCREEN 1 // Assume no screen present by default to prevent crash...
#define USE_SSD1306

// GPS
#define PIN_SERIAL1_RX (0 + 20)
#define PIN_SERIAL1_TX (0 + 22) 
#define PIN_SERIAL2_RX (-1)
#define PIN_SERIAL2_TX (-1)
#define USE_GPS_E108GN03D
// //#define USE_GPS_E108GN04D
#define HAS_GPS 1 // Don't need to set this to 0 to prevent a crash as it doesn't crash if GPS not found, will probe by default
#define PIN_GPS_EN (0 + 24)
#define PIN_GPS_PPS (0 + 13)
#define GPS_EN_ACTIVE 1
#define GPS_TX_PIN PIN_SERIAL1_RX
#define GPS_RX_PIN PIN_SERIAL1_TX
#define GPS_THREAD_INTERVAL 50

// Buttons
#define BUTTON_PIN (32 + 4) // Use the BOOT button as the user button

#ifdef USE_GPS_E108GN03D
    #define GPS_BAUDRATE 9600      //E108-GN03D
#elif defined(USE_GPS_E108GN04D)
    #define GPS_BAUDRATE 38400   //E108-GN04D
#endif


// // ratio of voltage divider = 3.33 (R1=100k, R2=220k)
// #define ADC_MULTIPLIER 3.33
// #define BATTERY_PIN (32 + 6) // A battery voltage measurement pin, voltage divider connected here to measure battery voltage
// #define BATTERY_SENSE_RESOLUTION_BITS 12
// #define BATTERY_SENSE_RESOLUTION 4096.0
// #define ADC_ATTEN ADC_ATTEN_DB_11

/*
 * Analog pins
 */
#define PIN_A4 (0 + 28)
static const uint8_t A4 = PIN_A4;
#define ADC_RESOLUTION 14
#define BATTERY_PIN PIN_A4
// and has 12 bit resolution
#define BATTERY_SENSE_RESOLUTION_BITS 12
#define BATTERY_SENSE_RESOLUTION 4096.0
#undef AREF_VOLTAGE
#define AREF_VOLTAGE 3.0
#define VBAT_AR_INTERNAL AR_INTERNAL_3_0
#define ADC_MULTIPLIER (3.2006F)

//Buzzer
#define PIN_BUZZER (0 + 2)

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
