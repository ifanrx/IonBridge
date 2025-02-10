#include "ble.h"

#include <cstdint>
#include <cstring>
#include <queue>
#include <string>
#include <vector>

#include "NimBLEAdvertising.h"
#include "NimBLECharacteristic.h"
#include "NimBLEDevice.h"
#include "animation.h"
#include "ble_callbacks.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "host/ble_gap.h"
#include "machine_info.h"
#include "message.h"
#include "portmacro.h"
#include "protocol.h"
#include "sdkconfig.h"
#include "utils.h"
#include "wifi.h"
#include "wifi_state.h"

#define BLE_ATT_MTU CONFIG_BLE_ATT_MTU
#define SERVICE_UUID CONFIG_SERVICE_UUID
#define CHARACTERISTIC_UUID_TX CONFIG_CHARACTERISTIC_UUID_TX
#define CHARACTERISTIC_UUID_RX CONFIG_CHARACTERISTIC_UUID_RX
#define BLE_ADV_INTERVAL_MIN_MS BLE_GAP_CONN_ITVL_MS(CONFIG_BLE_ADV_INTERVAL_MS)
#define BLE_ADV_INTERVAL_MAX_MS BLE_ADV_INTERVAL_MIN_MS
#define BLE_ADV_TIMER_INTERVAL_MS CONFIG_BLE_ADV_TIMER_INTERVAL * 1000
#define BLE_ADV_DURATION_MS CONFIG_BLE_ADV_DURATION_MS
#define BLE_DEFAULT_ADV_INTERVAL_MS 0

using namespace std;

static const char *TAG = "BLE";

typedef std::queue<std::vector<uint8_t>> BLEMessagesQueue;

static NimBLEUUID serviceUUID(SERVICE_UUID);
static NimBLEUUID rxCharUUID(CHARACTERISTIC_UUID_RX);
static NimBLEUUID txCharUUID(CHARACTERISTIC_UUID_TX);
static NimBLEServer *pServer;
static NimBLECharacteristic *pCharacteristicTX;
static NimBLECharacteristic *pCharacteristicRX;
static std::string clientAddress = "";
static bool deviceConnected = false;
static uint16_t peerMTU = 23, connTimeout = 120;
static BLEMessagesQueue bleMessagesQueue;
static uint16_t deviceConnHandle = 0;
static CharacteristicCallbacks *callbacks = new CharacteristicCallbacks();

typedef enum {
  BLE_ADV_START,
  BLE_ADV_STOP,
  BLE_ADV_DELAY_START,
  BLE_ADV_DELAY_START_LIMITED_DURATION,
} BLE_ADV_STATE;

static TaskHandle_t bleAdvTaskHandle = NULL;
static TimerHandle_t bleAdvTimer;
static BLE_ADV_STATE bleAdvState = BLE_ADV_STOP;
static void start_ble_adv_timer(int interval_ms, int adv_duration_ms);
static void stop_ble_adv_timer();
static void ble_adv_timer_callback(TimerHandle_t xTimer);
static void ble_adv_task(void *arg);

void ServerCallbacks::onConnect(NimBLEServer *pServer,
                                NimBLEConnInfo &connInfo) {
  clientAddress = connInfo.getAddress().toString();
  ESP_LOGI(TAG, "Connected, client address: %s", clientAddress.c_str());

  /** We can use the connection handle here to ask for different connection
   * parameters.
   *
   * Args:
   *   - connection handle
   *   - min connection interval
   *   - max connection interval
   *   - latency
   *   - supervision timeout
   * Units:
   *   - Min/Max Intervals: 1.25 millisecond increments.
   *   - Latency: number of intervals allowed to skip.
   *   - Timeout: 10 millisecond increments, try for 5x interval
   *              time for best results.
   */
  uint16_t connHandle = connInfo.getConnHandle();
  pServer->updateConnParams(connHandle, 0x20, 0x40, 0, connTimeout);
  deviceConnHandle = connHandle;
  deviceConnected = true;

  peerMTU = pServer->getPeerMTU(connHandle);
  ESP_LOGI(TAG, "Peer MTU: %d", peerMTU);
  AnimationController::GetInstance().StartAnimation(AnimationId::BLE_CONNECTED,
                                                    true);
}

void ServerCallbacks::onDisconnect(NimBLEServer *pServer,
                                   NimBLEConnInfo &connInfo, int reason) {
  ESP_LOGI(TAG, "Disconnected with reason: %s(0x%04X)",
           NimBLEUtils::returnCodeToString(reason), reason);
  ESP_LOGD(TAG, "Connection Interval: %d ms", connInfo.getConnInterval());
  ESP_LOGD(TAG, "Connection Timeout: %d ms", connInfo.getConnTimeout());
  ESP_LOGD(TAG, "Connection Latency: %d", connInfo.getConnLatency());
  ESP_LOGD(TAG, "MTU size: %d bytes", connInfo.getMTU());
  deviceConnected = false;
  clientAddress.clear();
  AnimationController::GetInstance().StopAnimation(AnimationId::BLE_CONNECTED);
}

void ServerCallbacks::onMTUChange(uint16_t mtu, NimBLEConnInfo &connInfo) {
  NimBLEServer *pServer = NimBLEDevice::getServer();
  uint16_t connHandle = connInfo.getConnHandle();
  ESP_LOGI(TAG, "MTU updated: %u for connection ID: %u", mtu, connHandle);
  peerMTU = mtu;
  pServer->updateConnParams(connHandle, 0x20, 0x40, 0, connTimeout);
}

void CharacteristicCallbacks::onWrite(NimBLECharacteristic *pCharacteristic,
                                      NimBLEConnInfo &connInfo) {
  NimBLEAttValue value = pCharacteristic->getValue();
  const uint8_t *pData = value.data();
  uint16_t length = value.length();
  ESP_LOGD(TAG, "Write callback for characteristic %s with data length %d",
           pCharacteristic->getUUID().toString().c_str(), length);
  if (length <= 0) {
    return;
  }

  ESP_LOGD(TAG, "Received: %s", to_hex_string(pData, length).c_str());
  bleMessagesQueue.push(std::vector<uint8_t>(pData, pData + length));
}

void CharacteristicCallbacks::onRead(NimBLECharacteristic *pCharacteristic,
                                     NimBLEConnInfo &connInfo) {
  logValue(pCharacteristic, "onRead");
}

void CharacteristicCallbacks::onStatus(NimBLECharacteristic *pCharacteristic,
                                       int code) {
  ESP_LOGD(TAG, "Notification/Indication return code: %d, %s", code,
           NimBLEUtils::returnCodeToString(code));
}

void CharacteristicCallbacks::onSubscribe(NimBLECharacteristic *pCharacteristic,
                                          NimBLEConnInfo &connInfo,
                                          uint16_t subValue) {
#ifdef BLE_VERBOSE_LOGGING
  std::string str = "Client ID: ";
  str += std::to_string(connInfo.getConnHandle());
  str += ", Address: ";
  str += connInfo.getAddress().toString();
  str += ", Subscription: ";

  switch (subValue) {
    case 0:
      str += "Unsubscribed from ";
      break;
    case 1:
      str += "Subscribed to notifications for ";
      break;
    case 2:
      str += "Subscribed to indications for ";
      break;
    case 3:
      str += "Subscribed to notifications and indications for ";
      break;
    default:
      str += "Unknown subscription status for ";
      break;
  }

  str += pCharacteristic->getUUID().toString();
  ESP_LOGD(TAG, "%s", str.c_str());
#endif
}

void CharacteristicCallbacks::logValue(NimBLECharacteristic *pCharacteristic,
                                       const char *name) {
  NimBLEAttValue value = pCharacteristic->getValue();
  uint16_t length = value.length();
  const uint8_t *pData = value.data();
  ESP_LOGD(TAG, "Characteristic UUID: %s, Function: %s, Value: %s, Length: %d",
           pCharacteristic->getUUID().toString().c_str(), name,
           to_hex_string(pData, length).c_str(), length);
}

static void ble_update_adv_interval() {
  if (pServer == nullptr) {
    return;
  }
  NimBLEAdvertising *pAdvertising = pServer->getAdvertising();
  if (pAdvertising == nullptr) {
    return;
  }
  if (wifi_is_connected()) {
    pAdvertising->setMinInterval(BLE_ADV_INTERVAL_MIN_MS);
    pAdvertising->setMaxInterval(BLE_ADV_INTERVAL_MAX_MS);
  } else {
    pAdvertising->setMinInterval(BLE_DEFAULT_ADV_INTERVAL_MS);
    pAdvertising->setMaxInterval(BLE_DEFAULT_ADV_INTERVAL_MS);
  }
}

esp_err_t ble_init() {
  if (NimBLEDevice::isInitialized()) {
    return ESP_OK;
  }

  esp_err_t __attribute__((unused)) ret;
  deviceConnected = false;
  NimBLEService *service;
  uint8_t manufacturer_data[8] = {
      0xE9, 0x36, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  };
  uint8_t deviceAddr[6];

  std::string deviceName = MachineInfo::GetInstance().GetDeviceName();

  ESP_RETURN_ON_FALSE(NimBLEDevice::init(deviceName), ESP_FAIL, TAG,
                      "Error NimBLEDevice::init");
  ESP_RETURN_ON_FALSE(NimBLEDevice::isInitialized(), ESP_FAIL, TAG,
                      "Failed to initialize BLE Device");
  ESP_LOGI(TAG, "BLE Device Name: %s", deviceName.c_str());
  NimBLEDevice::setMTU(BLE_ATT_MTU);
  NimBLEDevice::setPowerLevel(
      static_cast<esp_power_level_t>(CONFIG_BLE_PWR_TYPE_ADV),
      ESP_BLE_PWR_TYPE_ADV);
  NimBLEDevice::setPowerLevel(
      static_cast<esp_power_level_t>(CONFIG_BLE_PWR_TYPE_SCAN),
      ESP_BLE_PWR_TYPE_SCAN);
  NimBLEDevice::setPowerLevel(
      static_cast<esp_power_level_t>(CONFIG_BLE_PWR_TYPE_DEFAULT),
      ESP_BLE_PWR_TYPE_DEFAULT);
  NimBLEDevice::setSecurityAuth(false, true, true);

  pServer = NimBLEDevice::createServer();
  if (!pServer) {
    ESP_LOGE(TAG, "Failed to create the BLE server");
    return ESP_ERR_INVALID_STATE;
  }
  pServer->advertiseOnDisconnect(true);
  // callback class will be deleted when server is destructed.
  pServer->setCallbacks(new ServerCallbacks());

  service = pServer->createService(serviceUUID);
  if (!service) {
    ESP_LOGE(TAG, "Failed to create the BLE service for %s",
             serviceUUID.toString().c_str());
    return ESP_ERR_INVALID_VERSION;
  }

  pCharacteristicTX = service->createCharacteristic(
      txCharUUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pCharacteristicRX = service->createCharacteristic(
      rxCharUUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  if (!pCharacteristicRX || !pCharacteristicTX) {
    ESP_LOGE(TAG, "Failed to create the BLE characteristics");
    return ESP_ERR_INVALID_MAC;
  }
  pCharacteristicTX->setCallbacks(callbacks);
  pCharacteristicRX->setCallbacks(callbacks);

  if (!service->start()) {
    ESP_LOGE(TAG, "Failed to start the BLE service");
    return ESP_ERR_INVALID_RESPONSE;
  }
  pServer->start();

  NimBLEAdvertising *pAdvertising = pServer->getAdvertising();
  if (pAdvertising == nullptr) {
    ESP_LOGE(TAG, "pServer->getAdvertising failed");
    return ESP_ERR_INVALID_ARG;
  }

  get_ble_mac_address(deviceAddr);
  // copy last 3 bytes of MAC to fit in adv data
  memcpy(manufacturer_data + 2, deviceAddr + 3, 3);
  manufacturer_data[5] = MachineInfo::GetInstance().GetProductFamilyEnumVal();
  manufacturer_data[6] = MachineInfo::GetInstance().GetDeviceModelEnumVal();
  manufacturer_data[7] = MachineInfo::GetInstance().GetProductColorEnumVal();

  pAdvertising->setManufacturerData(std::vector<uint8_t>(
      manufacturer_data, manufacturer_data + sizeof(manufacturer_data)));
  pAdvertising->setName(deviceName);
  pAdvertising->enableScanResponse(true);
  pAdvertising->addServiceUUID(service->getUUID());
  ble_update_adv_interval();
  ESP_LOGI(TAG, "BLE initialized");
  return ESP_OK;
}

esp_err_t ble_deinit() {
  if (!NimBLEDevice::isInitialized()) {
    return ESP_OK;
  }

  ESP_RETURN_ON_FALSE(pServer->stopAdvertising(), ESP_FAIL, TAG,
                      "Failed to stop Advertising");
  ESP_RETURN_ON_FALSE(NimBLEDevice::deinit(true), ESP_FAIL, TAG,
                      "Failed to deinit BLE Device");
  ESP_LOGI(TAG, "BLE deinitialized");
  deviceConnected = false;
  return ESP_OK;
}

void handle_ble_messages_task(void *arg) {
  MessageHandler *msgHandler = new MessageHandler();
  esp_err_t __attribute__((unused)) ret;
  while (true) {
    if (!bleMessagesQueue.empty()) {
      std::vector<uint8_t> data = bleMessagesQueue.front();
      bleMessagesQueue.pop();
      ESP_LOG_BUFFER_HEX_LEVEL(TAG, data.data(), data.size(), ESP_LOG_DEBUG);

      Message *msg = nullptr;
      ESP_GOTO_ON_ERROR(
          MessageFactory::createMessage(data.data(), data.size(), &msg), NEXT,
          TAG, "MessageFactory::createMessage: invalid data");
      // Attribute Opcode(1) + Handle(2) = 3
      msgHandler->setPeerMTU(peerMTU - 3);
      process_message(*msg, *msgHandler);
      delete msg;
    }
  NEXT:
    DELAY_MS(50);
  }

  delete msgHandler;
}

const char *get_client_address() { return clientAddress.c_str(); }

void get_ble_mac_address(uint8_t *ble_address) {
  if (ble_address == nullptr) {
    return;
  }
  memcpy(ble_address,
         NimBLEDevice::getAddress().reverseByteOrder().getBase()->val, 6);
}

bool ble_start_advertising(bool force) {
  if (ble_is_advertising() && !force) {
    ESP_LOGD(TAG, "Advertising is already started");
    return true;
  }
  if (deviceConnected && force) {
    ESP_RETURN_ON_FALSE(pServer && pServer->disconnect(deviceConnHandle), false,
                        TAG,
                        "Failed to disconnect from previous device before "
                        "starting advertising");
  }
  ble_adv_start();
  return true;
}

bool ble_stop_advertising() {
  if (!ble_is_advertising()) {
    return true;
  }
  ble_adv_stop();
  return true;
}

esp_err_t get_ble_conn_rssi(int8_t *rssi) {
  int ret = ble_gap_conn_rssi(deviceConnHandle, rssi);
  ESP_RETURN_ON_FALSE(ret == 0, ESP_FAIL, TAG, "ble_gap_conn_rssi: %d", ret);
  return ESP_OK;
}

uint16_t get_ble_mtu() { return peerMTU; }

bool ble_is_advertising() { return bleAdvState != BLE_ADV_STOP; }

bool ble_is_connected() { return deviceConnected; }

void ble_adv_task(void *arg) {
  BaseType_t xResult;
  uint32_t state;
  BLE_ADV_STATE new_state = BLE_ADV_STOP;
  while (true) {
    // Wait to be notified from Wi-Fi/MQTT task
    xResult = xTaskNotifyWait(pdTRUE, pdTRUE, &state, portMAX_DELAY);
    if (xResult != pdPASS) {
      ESP_LOGE(TAG, "Failed to wait for notification");
      continue;
    }
    new_state = (BLE_ADV_STATE)state;
    if (bleAdvState == new_state) {
      continue;
    }
    stop_ble_adv_timer();
    switch (new_state) {
      case BLE_ADV_START: {
        // start advertising indefinitely
        wifi_controller.Notify(WiFiEventType::STOP_WEB_SERVER);
        if (ble_init() != ESP_OK || !pServer->startAdvertising()) {
          ESP_LOGE(TAG, "Failed to start advertising");
          continue;
        }
        AnimationController::GetInstance().StartAnimation(
            AnimationId::BLE_ADVERTISING, true);
      } break;
      case BLE_ADV_STOP: {
        // stop advertising immediately
        if (NimBLEDevice::isInitialized()) {
          if (deviceConnected && !pServer->disconnect(deviceConnHandle)) {
            ESP_LOGE(TAG,
                     "Failed to disconnect from previous device before "
                     "stopping advertising");
            continue;
          }
#ifdef CONFIG_DEINIT_BLE_ON_STOP_ADVERTISING
          if (ble_deinit() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to deinit BLE");
            continue;
          }
#else
          if (!pServer->stopAdvertising()) {
            ESP_LOGE(TAG, "Failed to stop advertising");
            continue;
          }
#endif
        }
        wifi_controller.Notify(WiFiEventType::START_WEB_SERVER);
        AnimationController::GetInstance().StopAnimation(
            AnimationId::BLE_ADVERTISING);
      } break;
      case BLE_ADV_DELAY_START: {
        // start a timer, and enter "BLE_ADV_START" state after timer triggers
        if (bleAdvState == BLE_ADV_START) {
          continue;
        }
        start_ble_adv_timer(BLE_ADV_TIMER_INTERVAL_MS, 0);
      } break;
      case BLE_ADV_DELAY_START_LIMITED_DURATION: {
        // same as BLE_ADV_DELAY_START, but only advertise for X seconds, then
        // enter BLE_ADV_STOP state
        start_ble_adv_timer(BLE_ADV_TIMER_INTERVAL_MS, BLE_ADV_DURATION_MS);
      } break;
      default: {
        ESP_LOGW(TAG, "Invalid BLE adv task state: %" PRIx32, state);
        continue;
      } break;
    }
    // State changed, cancel all pending actions and perform the new action
    ESP_LOGI(TAG, "BLE adv task state changed: %d => %d", bleAdvState,
             new_state);
    bleAdvState = new_state;
  }
}

esp_err_t start_ble_adv_task() {
  BaseType_t res = xTaskCreate(ble_adv_task, "ble_adv", 1024 * 2, nullptr, 5,
                               &bleAdvTaskHandle);
  if (res != pdPASS) {
    ESP_LOGE(TAG, "Failed to create BLE adv task");
    return ESP_ERR_NO_MEM;
  }
  return ESP_OK;
}

void start_ble_adv_timer(int interval_ms, int adv_duration_ms) {
  if (bleAdvTimer != nullptr) {
    ESP_LOGW(TAG, "BLE adv timer is already running");
    return;
  }
  bleAdvTimer = xTimerCreate("bleAdvTimer", pdMS_TO_TICKS(interval_ms), pdFALSE,
                             (void *)adv_duration_ms, ble_adv_timer_callback);
  if (bleAdvTimer == nullptr) {
    ESP_LOGE(TAG, "Failed to create BLE adv timer");
    return;
  }
  xTimerStart(bleAdvTimer, 0);
}

void stop_ble_adv_timer() {
  if (bleAdvTimer == nullptr) {
    ESP_LOGD(TAG, "BLE adv timer is not running");
    return;
  }
  xTimerStop(bleAdvTimer, 0);
  xTimerDelete(bleAdvTimer, 0);
  bleAdvTimer = nullptr;
}

void ble_adv_timer_callback(TimerHandle_t xTimer) {
  wifi_controller.Notify(WiFiEventType::STOP_WEB_SERVER);
  if (!NimBLEDevice::isInitialized()) {
    ESP_RETURN_VOID_ON_ERROR(ble_init(), TAG, "Failed to initialize BLE");
  }

  int adv_duration_ms = (int)pvTimerGetTimerID(xTimer);
  if (!pServer->startAdvertising(adv_duration_ms)) {
    ESP_LOGE(TAG, "Failed to start BLE advertising, duration: %d",
             adv_duration_ms);
  }
  ESP_LOGI(TAG, "BLE advertising started, duration: %d", adv_duration_ms);
  if (adv_duration_ms == 0) {
    ble_adv_start();
  } else {
    DELAY_MS(adv_duration_ms);
    if (bleAdvState == BLE_ADV_DELAY_START_LIMITED_DURATION) {
      // stop advertising after duration
      ble_adv_stop();
    }
  }
}

void ble_adv_start() {
  xTaskNotify(bleAdvTaskHandle, BLE_ADV_START, eSetValueWithOverwrite);
}

void ble_adv_stop() {
  xTaskNotify(bleAdvTaskHandle, BLE_ADV_STOP, eSetValueWithOverwrite);
}

void ble_adv_delay_start() {
  xTaskNotify(bleAdvTaskHandle, BLE_ADV_DELAY_START, eSetValueWithOverwrite);
}

void ble_adv_delay_start_limited_duration() {
  xTaskNotify(bleAdvTaskHandle, BLE_ADV_DELAY_START_LIMITED_DURATION,
              eSetValueWithOverwrite);
}

void ble_notify(const uint8_t *data, size_t length) {
  if (NimBLEDevice::isInitialized() && pCharacteristicTX && deviceConnected) {
    pCharacteristicTX->notify(data, length);
  }
}
