# Platform Peripheral Support List

- ✅：已支持（Supported）
- ⚙️：仅编译（Compile Only）
- 🔄：开发中（In Progress/Working）
- ❌：未支持（Not Supported）
- 🚫：硬件不支持/不可用（Unavailable）

## Linux Support

| `Peripheral` | Ubuntu/Debian |
| ------------ | ------------- |
| POWER | ✅ |
| GPIO | ✅ |
| FLASH | ✅ |
| UART | ✅ |
| SPI | ❌ |
| I2C | ❌ |
| CAN | ❌ |
| CANFD | ❌ |
| ADC | ❌ |
| DAC | ❌ |
| PWM | ❌ |
| USB-DEVICE | ❌ |
| WDG | ❌ |

| `Network` | Ubuntu/Debian |
| ----------- | ------------- |
| WIFI Client | ✅ |
| SmartConfig | ❌ |
| Bluetooth | ❌ |

## STM32 Support

| `Peripheral` | STM32F0/F1/F4/G0/G4/H7 | STM32L0/L1/L4 |
| ------------ | ---------------------- | ------------- |
| POWER | ✅ | ⚙️ |
| GPIO | ✅ | ⚙️ |
| FLASH | ✅ | ⚙️ |
| UART | ✅ | ⚙️ |
| SPI | ✅ | ⚙️ |
| I2C | ✅ | ⚙️ |
| CAN | ✅ | ⚙️ |
| CANFD | ✅ | ⚙️ |
| ADC | ✅ | ⚙️ |
| DAC | ✅ | ⚙️ |
| PWM | ✅ | ⚙️ |
| USB-DEVICE | ✅ | ⚙️ |
| WDG | ✅ | ⚙️ |

| `Network` | STM32 |
| ----------- | ----- |
| WIFI Client | ❌ |
| SmartConfig | ❌ |
| Bluetooth | ❌ |

## ESP32 Support

| `Peripheral` | ESP32 | ESP32-S3 | ESP32-C3 | ESP32-C6 |
| ------------ | ----- | -------- | -------- | -------- |
| POWER | ✅ | ✅ | ✅ | ✅ |
| GPIO | ✅ | ✅ | ✅ | ✅ |
| FLASH | ✅ | ✅ | ✅ | ✅ |
| UART | ✅ | ✅ | ✅ | ✅ |
| SPI | ✅ | ✅ | ✅ | ✅ |
| I2C | ✅ | ✅ | ✅ | ✅ |
| CAN | ❌ | ❌ | ❌ | ❌ |
| CANFD | 🚫 | 🚫 | 🚫 | 🚫 |
| ADC | ✅ | ✅ | ✅ | ✅ |
| DAC | ✅ | 🚫 | 🚫 | 🚫 |
| PWM | ✅ | ✅ | ✅ | ✅ |
| USB-DEVICE | 🚫 | ✅ | 🚫 | 🚫 |
| CDC-JTAG | 🚫 | 🚫 | ✅ | ✅ |
| WDG | ✅ | ✅ | ✅ | ✅ |

| `Network` | ESP32 | ESP32-S3 | ESP32-C3 | ESP32-C6 |
| ----------- | ----- | -------- | -------- | -------- |
| WIFI Client | ✅ | ✅ | ✅ | ✅ |
| SmartConfig | ❌ | ❌ | ❌ | ❌ |
| Bluetooth | ❌ | ❌ | ❌ | ❌ |

## CH32 Support

| `Peripheral` | CH32V305/CH32V307/CH32V203 |
| ------------ | -------------------------- |
| POWER | ✅ |
| GPIO | ✅ |
| FLASH | ✅ |
| UART | ✅ |
| SPI | ✅ |
| I2C | ✅ |
| CAN | ✅ |
| CANFD | 🚫 |
| ADC | ❌ |
| DAC | ❌ |
| PWM | ✅ |
| USB-DEVICE | ✅ |
| WDG | ❌ |

| `Network` | CH32V307 |
| ----------- | -------- |
| WIFI Client | ❌ |
| SmartConfig | ❌ |
| Bluetooth | ❌ |

## HPM Support

| `Peripheral` | HPM5301/HPM5361 |
| ------------ | --------------- |
| POWER | ❌ |
| GPIO | ✅ |
| FLASH | ❌ |
| UART | 🔄 |
| SPI | 🔄 |
| I2C | ✅ |
| CAN | 🔄 |
| CANFD | 🔄 |
| ADC | ❌ |
| DAC | ❌ |
| PWM | ✅ |
| USB-DEVICE | ❌ |
| WDG | ❌ |

## MSPM0 Support

| `Peripheral` | MSPM0G3507 |
| ------------ | ---------- |
| POWER | ❌ |
| GPIO | ✅ |
| FLASH | ❌ |
| UART | ✅ |
| SPI | ✅ |
| I2C | ✅ |
| CAN | ❌ |
| CANFD | ❌ |
| ADC | ❌ |
| DAC | ❌ |
| PWM | ✅ |
| USB-DEVICE | 🚫 |
| WDG | ❌ |

## Webots and WebAssembly Support

Webots（`LIBXR_DRIVER=webots`）与 WebAssembly（`LIBXR_DRIVER=webasm`）的驱动层只提供 `Timebase`，上表中的外设都没有驱动。

The driver layers of Webots (`LIBXR_DRIVER=webots`) and WebAssembly (`LIBXR_DRIVER=webasm`) provide only `Timebase`; none of the peripherals in the tables above has a driver there.
