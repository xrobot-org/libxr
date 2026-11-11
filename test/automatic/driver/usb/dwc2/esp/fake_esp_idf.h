// Fake ESP-IDF for host tests of the ESP32-S2/S3 USB backend: the declarations the
// backend uses from FreeRTOS.h, sdkconfig.h, soc/soc_caps.h, esp_err.h, esp_attr.h,
// esp_heap_caps.h, esp_dma_utils.h, esp_memory_utils.h, hal/cache_hal.h,
// esp_intr_alloc.h, esp_private/usb_phy.h, esp_rom_sys.h, soc/interrupts.h and the ROM
// USB headers, written against the ESP-IDF 5.3 to 5.5 API (esp_dma_mem_info_t in
// esp_dma_utils.h). The headers with those names in this directory include this file. The
// target is an ESP32-S3; the calls are recorded in fake_esp so that the test can run the
// registered interrupt handler and check what the backend asked for.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// sdkconfig.h and soc/soc_caps.h
#define CONFIG_IDF_TARGET_ESP32S3 1
#define SOC_USB_OTG_SUPPORTED 1
#define SOC_PSRAM_DMA_CAPABLE 1

// Internal data RAM of the ESP32-S3, the memory the USB DMA can reach.
#define SOC_DMA_LOW 0x3FC88000UL
#define SOC_DMA_HIGH 0x3FD00000UL

// esp_err.h
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL (-1)

// esp_attr.h
#define IRAM_ATTR
#define DRAM_ATTR

// esp_heap_caps.h
#define MALLOC_CAP_8BIT (1U << 2)
#define MALLOC_CAP_DMA (1U << 3)
#define MALLOC_CAP_INTERNAL (1U << 11)

// soc/interrupts.h
#define ETS_USB_INTR_SOURCE 38

// esp_intr_alloc.h
#define ESP_INTR_FLAG_INTRDISABLED (1 << 11)
typedef void (*intr_handler_t)(void* arg);
typedef struct intr_handle_data_t* intr_handle_t;

// esp_private/usb_phy.h
typedef enum
{
  USB_PHY_CTRL_OTG,
  USB_PHY_CTRL_SERIAL_JTAG
} usb_phy_controller_t;
typedef enum
{
  USB_PHY_TARGET_INT,
  USB_PHY_TARGET_EXT
} usb_phy_target_t;
typedef enum
{
  USB_PHY_MODE_DEFAULT,
  USB_OTG_MODE_HOST,
  USB_OTG_MODE_DEVICE
} usb_otg_mode_t;
typedef enum
{
  USB_PHY_SPEED_UNDEFINED,
  USB_PHY_SPEED_LOW,
  USB_PHY_SPEED_FULL,
  USB_PHY_SPEED_HIGH
} usb_phy_speed_t;
typedef struct
{
  usb_phy_controller_t controller;
  usb_phy_target_t target;
  usb_otg_mode_t otg_mode;
  usb_phy_speed_t otg_speed;
  const void* ext_io_conf;
  const void* otg_io_conf;
} usb_phy_config_t;
typedef struct phy_context_t* usb_phy_handle_t;

// esp_dma_utils.h
typedef struct
{
  uint32_t extra_heap_caps;
  size_t dma_alignment_bytes;
} esp_dma_mem_info_t;

// freertos/FreeRTOS.h
typedef struct
{
  uint32_t owner;
  uint32_t count;
} portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0U, 0U}

#ifdef __cplusplus
extern "C"
{
#endif

  /// What the backend asked the fake IDF for; the test reads and resets it.
  typedef struct
  {
    intr_handler_t handler;  ///< Registered by esp_intr_alloc().
    void* handler_arg;
    int source;
    bool interrupt_enabled;
    unsigned phy_created;
    unsigned rom_deinit;
    unsigned critical_depth;
    unsigned cache_syncs;
    unsigned delay_us;  ///< Sum of the esp_rom_delay_us() arguments.
  } FakeEspState;
  extern FakeEspState fake_esp;

  esp_err_t esp_intr_alloc(int source, int flags, intr_handler_t handler, void* arg,
                           intr_handle_t* handle);
  esp_err_t esp_intr_enable(intr_handle_t handle);
  esp_err_t esp_intr_disable(intr_handle_t handle);
  esp_err_t usb_new_phy(const usb_phy_config_t* config, usb_phy_handle_t* handle);
  void esp_rom_delay_us(uint32_t us);
  void usb_dev_deinit(void);
  void usb_dw_ctrl_deinit(void);
  extern void* uart_acm_dev;
  bool cache_hal_vaddr_to_cache_level_id(uint32_t vaddr, uint32_t length,
                                         uint32_t* cache_level, uint32_t* cache_id);
  esp_err_t esp_cache_msync(void* addr, size_t size, int flags);
  bool esp_dma_is_buffer_alignment_satisfied(const void* ptr, size_t size,
                                             esp_dma_mem_info_t info);
  void fake_esp_critical_enter(portMUX_TYPE* mux);
  void fake_esp_critical_exit(portMUX_TYPE* mux);

#ifdef __cplusplus
}
#endif

#define portENTER_CRITICAL_SAFE(mux) fake_esp_critical_enter(mux)
#define portEXIT_CRITICAL_SAFE(mux) fake_esp_critical_exit(mux)

// esp_memory_utils.h
static inline bool esp_ptr_dma_capable(const void* pointer)
{
  return (uintptr_t)pointer >= SOC_DMA_LOW && (uintptr_t)pointer < SOC_DMA_HIGH;
}
static inline bool esp_ptr_word_aligned(const void* pointer)
{
  return ((uintptr_t)pointer % 4U) == 0U;
}
