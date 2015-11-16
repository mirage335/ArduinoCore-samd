/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

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

#pragma once

#include "WVariant.h"

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

// Frequency of the board main oscillator
#define VARIANT_MAINOSC (32768ul)

// Master clock frequency
#define VARIANT_MCK     (48000000ul)

//#ifdef __cplusplus
//extern "C"
//{
//#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (26u)
#define NUM_DIGITAL_PINS     (14u)
#define NUM_ANALOG_INPUTS    (6u)
#define NUM_ANALOG_OUTPUTS   (1u)

#define digitalPinToPort(P)        (&(PORT->Group[g_APinDescription[P].ulPort]))
#define digitalPinToBitMask(P)     (1 << g_APinDescription[P].ulPin)
//#define analogInPinToBit(P)      ()
#define portOutputRegister(port)   (&(port->OUT.reg))
#define portInputRegister(port)    (&(port->IN.reg))
#define portModeRegister(port)     (&(port->DIR.reg))
#define digitalPinHasPWM(P)        (g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER)
#define digitalPinToInterrupt(P)   (g_APinDescription[P].ulExtInt)

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)


// LEDs
// ----
#define PIN_LED              (6u)
#define PIN_LED2             (7u)
#define LED_BUILTIN          PIN_LED

// Analog pins
// -----------
#define PIN_A0               (8u)
#define PIN_A1               (9u)
#define PIN_A2               (10u)
#define PIN_A3               (11u)
#define PIN_A4               (12u)
#define PIN_A5               (13u)
#define PIN_A6               (14u)
static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A5;
static const uint8_t A6  = PIN_A6;
#define ADC_RESOLUTION        12

// SPI Interfaces
// --------------
#define SPI_INTERFACES_COUNT 2

// SPI
#define PIN_SPI_MISO         (17u)
#define PIN_SPI_MOSI         (15u)
#define PIN_SPI_SCK          (16u)
#define PIN_SPI_SS           (32u)
#define PERIPH_SPI           sercom1
#define PAD_SPI_TX           SPI_PAD_0_SCK_1
#define PAD_SPI_RX           SERCOM_RX_PAD_3
static const uint8_t SS   = PIN_SPI_SS;   // SPI Slave SS not used. Set here only for reference.
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

// SPI1: Connected to WINC1501B
#define PIN_SPI1_MISO         (25u)
#define PIN_SPI1_MOSI         (22u)
#define PIN_SPI1_SCK          (23u)
#define PIN_SPI1_SS           (24u)
#define PERIPH_SPI1           sercom2
#define PAD_SPI1_TX           SPI_PAD_0_SCK_1
#define PAD_SPI1_RX           SERCOM_RX_PAD_3
static const uint8_t SS1   = PIN_SPI1_SS;   // SPI Slave SS not used. Set here only for reference.
static const uint8_t MOSI1 = PIN_SPI1_MOSI;
static const uint8_t MISO1 = PIN_SPI1_MISO;
static const uint8_t SCK1  = PIN_SPI1_SCK;

// Wire Interfaces
// ---------------
#define WIRE_INTERFACES_COUNT 1

// Wire
#define PIN_WIRE_SDA        (18u)
#define PIN_WIRE_SCL        (19u)
#define PERIPH_WIRE         sercom0
#define WIRE_IT_HANDLER     SERCOM0_Handler

// USB
// ---
#define PIN_USB_HOST_ENABLE (32ul)
#define PIN_USB_DM          (30ul)
#define PIN_USB_DP          (31ul)

//#ifdef __cplusplus
//}
//#endif

// Needed for WINC1501B (WiFi101) library
// --------------------------------------
#define WINC1501_RESET_PIN  (26u)
#define WINC1501_INTN_PIN   (29u)
#define WINC1501_SPI        SPI1
#define WINC1501_SPI_CS_PIN PIN_SPI1_SS

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"

// Instances of SERCOM
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

// Serial1
extern Uart Serial1;
#define PIN_SERIAL1_RX       (20ul)
#define PIN_SERIAL1_TX       (21ul)
#define PAD_SERIAL1_TX       (UART_TX_PAD_2)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_3)
#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_MONITOR         SerialUSB
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

// Alisa Serial to SerialUSB, it's the most used serial port
#define Serial                      SerialUSB
