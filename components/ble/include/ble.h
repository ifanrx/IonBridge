#ifndef H_NIMBLE_
#define H_NIMBLE_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ble_init();
esp_err_t ble_deinit();
const char *get_client_address();

bool ble_start_advertising(bool force = false);
bool ble_stop_advertising();
bool ble_is_advertising();
bool ble_is_connected();
esp_err_t get_ble_conn_rssi(int8_t *rssi);
uint16_t get_ble_mtu();

esp_err_t start_ble_adv_task();
void ble_adv_start();
void ble_adv_stop();
void ble_adv_delay_start();

void ble_notify(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif  // !H_NIMBLE_
