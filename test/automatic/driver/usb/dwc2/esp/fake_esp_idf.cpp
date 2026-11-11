// The ESP-IDF functions the ESP32-S2/S3 USB backend calls, for host tests: interrupt
// allocation keeps the handler for the test to run, the PHY and ROM calls are counted,
// the critical section tracks its nesting, and internal RAM has no cache to synchronize.
#include "fake_esp_idf.h"

FakeEspState fake_esp = {};
void* uart_acm_dev = nullptr;

extern "C"
{
  esp_err_t esp_intr_alloc(int source, int flags, intr_handler_t handler, void* arg,
                           intr_handle_t* handle)
  {
    fake_esp.handler = handler;
    fake_esp.handler_arg = arg;
    fake_esp.source = source;
    fake_esp.interrupt_enabled = (flags & ESP_INTR_FLAG_INTRDISABLED) == 0;
    if (handle != nullptr) *handle = reinterpret_cast<intr_handle_t>(&fake_esp);
    return ESP_OK;
  }

  esp_err_t esp_intr_enable(intr_handle_t)
  {
    fake_esp.interrupt_enabled = true;
    return ESP_OK;
  }

  esp_err_t esp_intr_disable(intr_handle_t)
  {
    fake_esp.interrupt_enabled = false;
    return ESP_OK;
  }

  esp_err_t usb_new_phy(const usb_phy_config_t* config, usb_phy_handle_t* handle)
  {
    if (config == nullptr || config->controller != USB_PHY_CTRL_OTG ||
        config->otg_mode != USB_OTG_MODE_DEVICE)
      return ESP_FAIL;
    ++fake_esp.phy_created;
    if (handle != nullptr) *handle = reinterpret_cast<usb_phy_handle_t>(&fake_esp);
    return ESP_OK;
  }

  // The model has no clock: the delay is only recorded.
  void esp_rom_delay_us(uint32_t us) { fake_esp.delay_us += us; }

  void usb_dev_deinit(void) { ++fake_esp.rom_deinit; }

  void usb_dw_ctrl_deinit(void) { ++fake_esp.rom_deinit; }

  // Internal RAM is not behind a cache on this target.
  bool cache_hal_vaddr_to_cache_level_id(uint32_t, uint32_t, uint32_t*, uint32_t*)
  {
    return false;
  }

  esp_err_t esp_cache_msync(void*, size_t, int)
  {
    ++fake_esp.cache_syncs;
    return ESP_OK;
  }

  bool esp_dma_is_buffer_alignment_satisfied(const void* ptr, size_t size,
                                             esp_dma_mem_info_t info)
  {
    const size_t align = info.dma_alignment_bytes == 0U ? 4U : info.dma_alignment_bytes;
    return reinterpret_cast<uintptr_t>(ptr) % align == 0U && size % align == 0U;
  }

  void fake_esp_critical_enter(portMUX_TYPE* mux)
  {
    ++mux->count;
    ++fake_esp.critical_depth;
  }

  void fake_esp_critical_exit(portMUX_TYPE* mux)
  {
    --mux->count;
    --fake_esp.critical_depth;
  }

}  // extern "C"
