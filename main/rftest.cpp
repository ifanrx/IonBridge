#include "rftest.h"

#include "sdkconfig.h"

#ifdef CONFIG_ENABLE_RFTEST
#warning "Ensure `sdkconfig.rftest` is used before compiling with ENABLE_RFTEST"

#include <cinttypes>
#include <cstdint>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_phy_cert_test.h"
#include "nvs_namespace.h"
#include "utils.h"

static const char *TAG = "RFTEST";
#endif

void cert_test() {
#ifdef CONFIG_ENABLE_RFTEST
  uint32_t mode_args[6] = {};
  esp_err_t err = TestModeNVSData::GetAndEraseTestModeAArgs(mode_args);
  if (err != ESP_OK) {
    return;
  }
  ESP_LOGW(TAG, "Starting certification test");
  esp_phy_wifi_rate_t rate = static_cast<esp_phy_wifi_rate_t>(mode_args[1]);
  int8_t backoff = static_cast<int8_t>(mode_args[2]);
  ESP_LOGI(TAG,
           "Test mode A parameters - mode: %" PRIu32
           ", rate: %d, backoff: %d, "
           "param3: %" PRIu32 ", param4: %" PRIu32 ", param5: %" PRIu32,
           mode_args[0], rate, (int)backoff, mode_args[3], mode_args[4],
           mode_args[5]);
  esp_wifi_power_domain_on();
  esp_phy_rftest_config(1);
  esp_phy_rftest_init();
  esp_phy_tx_contin_en(true);
  esp_phy_test_start_stop(3);
  esp_phy_wifi_tx(mode_args[0], rate, backoff, mode_args[3], mode_args[4],
                  mode_args[5]);
  while (true) {
    DELAY_MS(1000);
  }
#endif
}
