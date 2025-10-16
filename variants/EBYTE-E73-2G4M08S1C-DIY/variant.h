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
/** Master clock frequency */
#define VARIANT_MCK (64000000ul)

#define USE_LFXO // Board uses 32khz crystal for LF

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
#define NUM_ANALOG_INPUTS (1)
#define NUM_ANALOG_OUTPUTS (0)

// // LEDs
#define PIN_LED1 (32 + 9) // green (confirmed on 1.0 board)
#define LED_BLUE PIN_LED1 // fake for bluefruit library
#define LED_GREEN PIN_LED1
#define LED_BUILTIN LED_GREEN
#define LED_STATE_ON 0 // State when LED is lit

// // QSPI Pins
// #define PIN_QSPI_SCK (32 + 14)
// #define PIN_QSPI_CS (32 + 15)
// #define PIN_QSPI_IO0 (32 + 12) // MOSI if using two bit interface
// #define PIN_QSPI_IO1 (32 + 13) // MISO if using two bit interface
// #define PIN_QSPI_IO2 (0 + 7)   // WP if using two bit interface (i.e. not used)
// #define PIN_QSPI_IO3 (0 + 5)   // HOLD if using two bit interface (i.e. not used)

// // On-board QSPI Flash
// #define EXTERNAL_FLASH_DEVICES MX25R1635F
// #define EXTERNAL_FLASH_USE_QSPI

/*
 * Lora radio
 */
#define RADIOLIB_DEBUG 1
#define RADIOLIB_DEBUG_BASIC 1
#define SPI_INTERFACES_COUNT 1
#define PIN_SPI_NSS (0 + 26)
#define PIN_SPI_SCK 22
#define PIN_SPI_MOSI (0 + 24)
#define PIN_SPI_MISO (0 + 13)
#define USE_SX1268 // E22-400M30S, E22-400M33S, E22-400M22S, and E22-400MM22S use SX1268
#define SX126X_DIO3_TCXO_VOLTAGE 2.2
#define TCXO_OPTIONAL
#define SX126X_MAX_POWER 22 // SX126xInterface.cpp defaults to 22 if not defined, but here we define it for good practice
static const uint8_t SS = PIN_SPI_NSS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK = PIN_SPI_SCK;
#define SX126X_CS PIN_SPI_NSS   // EBYTE module's NSS pin
#define SX126X_SCK PIN_SPI_SCK                                                                                                                                         // EBYTE module's SCK pin
#define SX126X_MOSI PIN_SPI_MOSI // EBYTE module's MOSI pin
#define SX126X_MISO PIN_SPI_MISO  // EBYTE module's MISO pin
#define SX126X_RESET (32 + 4) // EBYTE module's NRST pin
#define SX126X_BUSY (32 + 6)  // EBYTE module's BUSY pin
#define SX126X_DIO1 (32 + 13)  // EBYTE module's DIO1 pin
#define SX126X_DIO2 (32 + 10)  // EBYTE module's DIO2 pin
// #define SX126X_TXEN 24
// #define SX126X_RXEN 13
#define SPI_FREQUENCY 40000
#define SPI_READ_FREQUENCY 16000

#define LORA_CS PIN_SPI_NSS     // Compatibility with variant file configuration structure
#define LORA_SCK PIN_SPI_SCK   // Compatibility with variant file configuration structure
#define LORA_RESET SX126X_RESET
#define LORA_MOSI PIN_SPI_MOSI // Compatibility with variant file configuration structure
#define LORA_MISO PIN_SPI_MISO // Compatibility with variant file configuration structure
#define LORA_DIO1 SX126X_DIO1
 // Compatibility with variant file configuration structure
//#define LORA_DIO0 -1        // a No connect on the SX1262/SX1268 module
#define LORA_DIO1 SX126X_DIO1
#define LORA_DIO2 SX126X_DIO2 // Compatibility with variant file configuration structure
#define LORA_DIO3
// #define E22_TXEN SX126X_TXEN
// #define E22_RXEN SX126X_RXEN


// //SCREEN
#define WIRE_INTERFACES_COUNT 1
#define PIN_WIRE_SCL 29 // SCL     P0.29
#define PIN_WIRE_SDA 31 // SDA     P0.31
// #define HAS_SCREEN 0 // Assume no screen present by default to prevent crash...
// #define USE_SSD1306
// #define SSD1306_SCL PIN_WIRE_SCL
// #define SSD1306_SDA PIN_WIRE_SDA

// GPS
// #define PIN_SERIAL1_RX 3 // P0.01
// #define PIN_SERIAL1_TX (32 + 10) // P1.10

#define PIN_SERIAL1_RX (-1)
#define PIN_SERIAL1_TX (-1) 
#define PIN_SERIAL2_RX (-1)
#define PIN_SERIAL2_TX (-1)
// #define USE_GPS_E108GN03D
// // //#define USE_GPS_E108GN04D
// #define HAS_GPS 1 // Don't need to set this to 0 to prevent a crash as it doesn't crash if GPS not found, will probe by default
// #define PIN_GPS_EN 28
// #define GPS_EN_ACTIVE 1
// #define GPS_TX_PIN PIN_SERIAL1_RX
// #define GPS_RX_PIN PIN_SERIAL1_TX
// #define GPS_THREAD_INTERVAL 50

// Buttons
#define BUTTON_PIN 20 // Use the BOOT button as the user button

#ifdef USE_GPS_E108GN03D
    #define GPS_BAUDRATE 9600      //E108-GN03D
#elif defined(USE_GPS_E108GN04D)
    #define GPS_BAUDRATE 38400   //E108-GN04D
#endif

// // ratio of voltage divider = 3.33 (R1=100k, R2=220k)
// #define ADC_MULTIPLIER 3.33
// #define BATTERY_PIN 1 // A battery voltage measurement pin, voltage divider connected here to measure battery voltage
// #define ADC_CHANNEL ADC1_GPIO1_CHANNEL
// #define BATTERY_SENSE_RESOLUTION_BITS 12
// #define BATTERY_SENSE_RESOLUTION 4096.0
// #define ADC_ATTEN ADC_ATTEN_DB_11

// Buzzer
// #define PIN_BUZZER 12

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
