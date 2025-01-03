#include <inttypes.h>
#include <string.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>

#include "animation.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "ionbridge.h"
#include "rpc.h"
#include "storage.h"
#include "sw3566_data_types.h"
#include "uart.h"
#include "utils.h"

static const char *TAG = "SW3566";

#define HASH_SIZE 5
#define PROGRAM_PAGE_SIZE 0x80
#define USERAPP_BOOT_MS 100
#define RESET_DELAY_MS 50
#define SW3566_COUNT CONFIG_SW3566_COUNT

typedef enum : uint8_t {
  AT_BOOTLOADER,
  AT_USERAPP,
  BOOT_TO_USERAPP,
  OPERATIONAL,
} SW3566BootStage;

uint8_t generate_nonce();

esp_err_t rpc::mcu::send_command(uint8_t mcu, uint16_t command,
                                 const uint8_t *request, uint8_t request_size,
                                 uint8_t *response, uint8_t *response_size) {
  // Every command requires at least one byte of request data
  if (request_size == 0) {
    ESP_LOGE(TAG, "SW3566 %d: Invalid request size: %d", mcu, request_size);
    return ESP_ERR_INVALID_ARG;
  }
  ESP_LOGI(TAG, "Sending command 0x%04X to SW3566 %d, request size: %d",
           command, mcu, request_size);
  SEND_UART_MSG(mcu, command, request, request_size, response, response_size,
                "send_command 0x%04X", command);
  return ESP_OK;
}

esp_err_t rpc::mcu::keep_alive(uint8_t mcu, KeepAliveStatus *status,
                               uint32_t *uptime_ms, uint32_t *reboot_reason) {
  KeepAliveRequest req{.nonce = generate_nonce()};
  KeepAliveResponse res{};
  uint8_t res_size = UART_MESSAGE_VARIABLE_LENGTH;

  SEND_UART_MSG(mcu, SW3566Command::KEEP_ALIVE, (const uint8_t *)&req,
                sizeof(req), (uint8_t *)&res, &res_size, "keep_alive");
  if (status != nullptr) {
    memcpy(status, &res.status, sizeof(res.status));
  }
  if (res_size > 5) {
    // if the response has more than 5 bytes, it contains uptime and reboot
    // reason
    ESP_LOGD(TAG, "Port %d: Uptime: %" PRIu32 "ms, RebootReason: 0x%" PRIx32,
             mcu, res.uptimeMS, res.rebootReason);
    if (uptime_ms != nullptr) {
      memcpy(uptime_ms, &res.uptimeMS, sizeof(res.uptimeMS));
    }
    if (reboot_reason != nullptr) {
      memcpy(reboot_reason, &res.rebootReason, sizeof(res.rebootReason));
    }
  }
  return ESP_OK;
}

esp_err_t rpc::mcu::boot(uint8_t mcu) {
  BootRequest req{.nonce = generate_nonce()};
  BootResponse res{};
  uint8_t res_size = sizeof(res);

  SEND_UART_MSG(mcu, SW3566Command::BOOT, (const uint8_t *)&req, sizeof(req),
                (uint8_t *)&res, &res_size, "boot");
  if (res.status != GenericStatus::OK) {
    ESP_LOGE(TAG, "SW3566 %d: Boot error %d", mcu, res.status);
    return ESP_ERR_INVALID_RESPONSE;
  }
  return ESP_OK;
}

esp_err_t rpc::mcu::bringup(uint8_t mcu, bool skip_upgrade) {
  ESP_LOGI(TAG, "Bringing up SW3566[%d]", mcu);
  KeepAliveStatus keep_alive_status;
  esp_err_t __attribute__((unused)) err;
  SW3566BootStage stage = SW3566BootStage::AT_BOOTLOADER;

  while (true) {
    switch (stage) {
      case SW3566BootStage::AT_BOOTLOADER:
        ESP_LOGI(TAG, "SW3566[%d] BOOTLOADER", mcu);
        ESP_RETURN_ON_ERROR(keep_alive(mcu, &keep_alive_status), TAG,
                            "keep_alive failed for MCU: %d", mcu);
        if (keep_alive_status != KeepAliveStatus::BOOTLOADER_ALIVE) {
          ESP_LOGE(TAG, "SW3566[%d]: Not in bootloader mode", mcu);
          return ESP_ERR_INVALID_STATE;
        }
        stage = SW3566BootStage::BOOT_TO_USERAPP;
        break;

      case SW3566BootStage::AT_USERAPP:
        ESP_LOGI(TAG, "SW3566[%d] USERAPP", mcu);
        ESP_RETURN_ON_ERROR(keep_alive(mcu, &keep_alive_status), TAG,
                            "keep_alive failed for MCU: %d", mcu);
        if (keep_alive_status != KeepAliveStatus::USER_APPLICATION_ALIVE) {
          ESP_LOGE(TAG, "SW3566[%d]: Not in user application. Status: %d", mcu,
                   keep_alive_status);
          return ESP_ERR_INVALID_STATE;
        }
        stage = SW3566BootStage::OPERATIONAL;
        break;

      case SW3566BootStage::BOOT_TO_USERAPP:
        ESP_LOGI(TAG, "SW3566[%d] BOOT_TO_USERAPP", mcu);
        ESP_RETURN_ON_ERROR(boot(mcu), TAG, "boot failed for MCU: %d", mcu);
        // Delay to allow the user application to boot
        DELAY_MS(USERAPP_BOOT_MS);
        stage = SW3566BootStage::AT_USERAPP;
        break;

      case SW3566BootStage::OPERATIONAL:
        ESP_LOGI(TAG, "SW3566[%d] OPERATIONAL", mcu);
        return ESP_OK;

      default:
        ESP_LOGE(TAG, "SW3566[%d] Invalid boot stage %d", mcu, stage);
        return ESP_ERR_INVALID_STATE;
    }
  }
}

esp_err_t rpc::mcu::boot_all(bool *booted, uint8_t count, bool skip_upgrade) {
  if (booted == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  for (uint8_t i = 0; i < count; i++) {
    if (!booted[i]) {
      continue;
    }
    esp_err_t err = bringup(i, skip_upgrade);
    booted[i] = (err == ESP_OK);
    if (!booted[i]) {
      ESP_ERROR_COMPLAIN(booted[i], "Failed to boot SW3566 %d", i);
    }
  }
  return ESP_OK;
}

esp_err_t rpc::mcu::set_subscription(uint8_t mcu, Subscriptions subs) {
  SetSubscriptionsRequest req{
      .nonce = generate_nonce(),
      .subscriptions = subs,
      .host_uptime_ms = (uint32_t)(esp_timer_get_time() / 1000),
  };
  SetSubscriptionsResponse res;
  uint8_t res_size = sizeof(res);

  SEND_UART_MSG(mcu, SW3566Command::SET_SUBSCRIPTIONS, (const uint8_t *)&req,
                sizeof(req), (uint8_t *)&res, &res_size, "set_subscriptions");
  if (res.status != GenericStatus::OK) {
    ESP_LOGE(TAG, "SW3566 %d set_subscriptions: %d", mcu, res.status);
    return ESP_ERR_INVALID_RESPONSE;
  }
  return ESP_OK;
}

esp_err_t rpc::mcu::set_port_state(uint8_t mcu, bool shutdown) {
  SetPortStateRequest req{.shutdown = shutdown};
  SetPortStateResponse res;
  uint8_t res_size = sizeof(res);

  SEND_UART_MSG(mcu, SW3566Command::SET_PORT_STATE, (const uint8_t *)&req,
                sizeof(req), (uint8_t *)&res, &res_size, "set_port_state");
  if (res.status != GenericStatus::OK) {
    ESP_LOGE(TAG, "SW3566 %d set_port_state: %d", mcu, res.status);
    return ESP_ERR_INVALID_RESPONSE;
  }
  return ESP_OK;
}

esp_err_t rpc::mcu::get_pd_status(uint8_t mcu, ClientPDStatus *status) {
  PDStatusRequest req{.nonce = generate_nonce()};
  PDStatusResponse res;
  memset(&res, 0, sizeof(res));
  uint8_t res_size = UART_MESSAGE_VARIABLE_LENGTH;

  SEND_UART_MSG(mcu, SW3566Command::GET_PD_STATUS, (const uint8_t *)&req,
                sizeof(req), (uint8_t *)&res, &res_size, "get_pd_status");
  if (res.status == GenericStatus::OK && status != nullptr) {
    memcpy(status, &res.data, sizeof(res.data));
  }
  return ESP_OK;
}

esp_err_t rpc::mcu::get_port_details(uint8_t mcu, PortDetails *details) {
  PortDetailsRequest req{.nonce = generate_nonce()};
  PortDetailsResponse res;
  uint8_t res_size = sizeof(res);

  SEND_UART_MSG(mcu, SW3566Command::GET_PORT_DETAILS, (const uint8_t *)&req,
                sizeof(req), (uint8_t *)&res, &res_size, "get_port_details");
  if (res.status != GenericStatus::OK) {
    ESP_LOGE(TAG, "sw3566::get_port_details(%d): %d", mcu, res.status);
    return ESP_ERR_INVALID_RESPONSE;
  }
  if (details != nullptr) {
    memcpy(details, &res.details, sizeof(res.details));
  }
  return ESP_OK;
}

uint8_t generate_nonce() {
  // Get a 32-bit random number from the hardware RNG
  uint32_t random_value = esp_random();

  // Extract a uint8_t from the 32-bit random number
  uint8_t random_uint8 = (uint8_t)(random_value & 0xFF);

  return random_uint8;
}

uint8_t round_to_nearest_multiple(uint8_t val, uint8_t multiple) {
  uint8_t rounded_val = ((val + multiple - 1) / multiple) * multiple;
  return rounded_val;
}

esp_err_t rpc::mcu::set_power_features(uint8_t mcu, PowerFeatures features) {
  SetPowerFeaturesRequest req = {.nonce = generate_nonce(),
                                 .features = features};
  SetPowerFeaturesResponse res;
  uint8_t res_size = sizeof(res);

  SEND_UART_MSG(mcu, SW3566Command::SET_POWER_FEATURES, (const uint8_t *)&req,
                sizeof(req), (uint8_t *)&res, &res_size, "set_power_features");
  if (res.status != GenericStatus::OK) {
    ESP_LOGE(TAG, "SW3566 %d set_power_features: %d", mcu, res.status);
    return ESP_ERR_INVALID_RESPONSE;
  }

  return ESP_OK;
}

esp_err_t rpc::mcu::get_power_features(uint8_t mcu, PowerFeatures *features) {
  GetPowerFeaturesRequest req = {.nonce = generate_nonce()};
  GetPowerFeaturesResponse res;
  uint8_t res_size = sizeof(res);

  SEND_UART_MSG(mcu, SW3566Command::GET_POWER_FEATURES, (const uint8_t *)&req,
                sizeof(req), (uint8_t *)&res, &res_size, "get_power_features");
  if (res.status != GenericStatus::OK) {
    ESP_LOGE(TAG, "SW3566 %d get_power_features: %d", mcu, res.status);
    return ESP_ERR_INVALID_RESPONSE;
  }

  if (features != nullptr) {
    memcpy(features, &res.features, sizeof(PowerFeatures));
  }

  return ESP_OK;
}

esp_err_t rpc::mcu::set_max_power_budget(uint8_t mcu, uint8_t budget,
                                         uint8_t watermark,
                                         bool force_rebroadcast,
                                         uint8_t *set_budget) {
  SetMaxPowerBudgetRequest req = {
      .max_power_budget = budget,
      .watermark = watermark,
      .force_rebroadcast = force_rebroadcast,
      .unused = 0,
  };
  SetMaxPowerBudgetResponse res;
  uint8_t res_size = sizeof(res);

  SEND_UART_MSG(mcu, SW3566Command::SET_MAX_POWER_BUDGET, (const uint8_t *)&req,
                sizeof(req), (uint8_t *)&res, &res_size,
                "set_max_power_budget");
  if (set_budget != nullptr) {
    *set_budget = res.set_budget;
    ESP_LOGI(TAG,
             "SW3566 %d set_max_power_budget: %d, watermark: %d, "
             "force_rebroadcast: %d, return: %d",
             mcu, budget, watermark, force_rebroadcast, res.set_budget);
  }
  return ESP_OK;
}

esp_err_t rpc::mcu::get_max_power_budget(uint8_t mcu, uint8_t *budget) {
  GetMaxPowerBudgetRequest req = {.nonce = generate_nonce()};
  GetMaxPowerBudgetResponse res;
  uint8_t res_size = sizeof(res);

  SEND_UART_MSG(mcu, SW3566Command::GET_MAX_POWER_BUDGET, (const uint8_t *)&req,
                sizeof(req), (uint8_t *)&res, &res_size,
                "get_max_power_budget");
  if (res.status != GenericStatus::OK) {
    ESP_LOGE(TAG, "SW3566 %d get_max_power_budget: %d", mcu, res.status);
    return ESP_ERR_INVALID_RESPONSE;
  }
  if (budget != nullptr) {
    *budget = res.max_power_budget;
  }

  return ESP_OK;
}
