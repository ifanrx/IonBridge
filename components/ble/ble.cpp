#include "ble.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <queue>
#include <string>
#include <vector>

#include "NimBLEAdvertising.h"
#include "NimBLEAttValue.h"
#include "NimBLECharacteristic.h"
#include "NimBLEConnInfo.h"
#include "NimBLEDevice.h"
#include "NimBLELocalValueAttribute.h"
#include "NimBLEServer.h"
#include "NimBLEService.h"
#include "NimBLEUUID.h"
#include "NimBLEUtils.h"
#include "ble_callbacks.h"
#include "display_animation.h"
#include "display_manager.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
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

#ifdef CONFIG_ENABLE_BLE_AND_MQTT_MUTUAL_EXCLUSION
#include "wifi.h"
#endif

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
static NimBLEServer *pServer = nullptr;
static NimBLECharacteristic *pCharacteristicTX = nullptr;
static NimBLECharacteristic *pCharacteristicRX = nullptr;
static CharacteristicCallbacks *callbacks = nullptr;

static std::string clientAddress = "";
static bool deviceConnected = false;
static uint16_t peerMTU = 23, connTimeout = 120;
static BLEMessagesQueue bleMessagesQueue;
static uint16_t deviceConnHandle = 0;

enum BLE_ADV_STATE : uint8_t {
  BLE_ADV_START,
  BLE_ADV_STOP,
  BLE_ADV_DELAY_START,
};

static TaskHandle_t bleAdvTaskHandle = NULL, bleMsgTaskHandle = NULL;
static BLE_ADV_STATE bleAdvCurrState = BLE_ADV_STOP;
static void ble_adv_task(void *arg);
static void ble_msg_task(void *arg);

static BLE_ADV_STATE handle_ble_adv_start();
static BLE_ADV_STATE handle_ble_adv_stop();
static BLE_ADV_STATE handle_ble_adv_delay_start(int interval_ms);

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
  DisplayManager::GetInstance().SetAnimation(AnimationType::BLE_CONNECTED,
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
  DisplayManager::GetInstance().SetAnimation(AnimationType::IDLE_ANIMATION,
                                             true);
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

#ifdef CONFIG_ENABLE_BLE_AND_MQTT_MUTUAL_EXCLUSION
  if (wifi_controller.GetStateType() == WiFiStateType::IDLE) {
    wifi_controller.Notify(WiFiEventType::ABORT);
    // WiFi abort handling will initialize BLE separately,
    // so return error to avoid duplicate initialization
    return ESP_ERR_INVALID_STATE;
  }
#endif

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
  pServer->setCallbacks(new ServerCallbacks(), true);

  // service would be deleted when server is destructed.
  service = pServer->createService(serviceUUID);
  if (!service) {
    ESP_LOGE(TAG, "Failed to create the BLE service for %s",
             serviceUUID.toString().c_str());
    return ESP_ERR_INVALID_VERSION;
  }

  // characteristic would be deleted when service is destructed.
  pCharacteristicTX = service->createCharacteristic(
      txCharUUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pCharacteristicRX = service->createCharacteristic(
      rxCharUUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  if (!pCharacteristicRX || !pCharacteristicTX) {
    ESP_LOGE(TAG, "Failed to create the BLE characteristics");
    return ESP_ERR_INVALID_MAC;
  }

  callbacks = new CharacteristicCallbacks();
  pCharacteristicTX->setCallbacks(callbacks);
  pCharacteristicRX->setCallbacks(callbacks);

  if (!service->start()) {
    ESP_LOGE(TAG, "Failed to start the BLE service");
    return ESP_ERR_INVALID_RESPONSE;
  }
  pServer->start();

  // pAdvertising would be deleted after calling NimBLEDevice::deinit()
  NimBLEAdvertising *pAdvertising = pServer->getAdvertising();
  if (pAdvertising == nullptr) {
    ESP_LOGE(TAG, "pServer->getAdvertising failed");
    return ESP_ERR_INVALID_ARG;
  }

  memcpy(deviceAddr,
         NimBLEDevice::getAddress().reverseByteOrder().getBase()->val, 6);
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

  if (bleMsgTaskHandle == NULL) {
    ret = xTaskCreate(ble_msg_task, "ble_msg", CONFIG_BLE_TASK_STACK_SIZE,
                      nullptr, 1, &bleMsgTaskHandle);
    ESP_RETURN_ON_FALSE(ret == pdPASS, ESP_FAIL, TAG,
                        "Failed to create BLE message task");
  }

  return ESP_OK;
}

esp_err_t ble_deinit() {
  if (!NimBLEDevice::isInitialized()) {
    return ESP_OK;
  }

  if (bleMsgTaskHandle != nullptr) {
    // Store handle in temporary variable
    TaskHandle_t tempHandle = bleMsgTaskHandle;

    // Signal task to exit
    xTaskNotify(tempHandle, 0, eNoAction);

    // Wait briefly to allow task to start cleanup
    vTaskDelay(pdMS_TO_TICKS(100));

    // Only now clear the global handle
    bleMsgTaskHandle = nullptr;
  }
  while (!bleMessagesQueue.empty()) {
    bleMessagesQueue.pop();
  }

  if (pServer) {
    ESP_RETURN_ON_FALSE(pServer->stopAdvertising(), ESP_FAIL, TAG,
                        "Failed to stop advertising");
  }

  ESP_RETURN_ON_FALSE(NimBLEDevice::deinit(true), ESP_FAIL, TAG,
                      "Failed to deinit BLE Device");
  ESP_LOGI(TAG, "BLE deinitialized");

  if (callbacks) {
    delete callbacks;
    callbacks = nullptr;
  }
  pServer = nullptr;
  pCharacteristicTX = nullptr;
  pCharacteristicRX = nullptr;
  deviceConnected = false;
  return ESP_OK;
}

void ble_msg_task(void *arg) {
  ESP_LOGI(TAG, "BLE message task running");
  std::unique_ptr<MessageHandler> msgHandler =
      std::make_unique<MessageHandler>();
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
    if (xTaskNotifyWait(pdTRUE, pdTRUE, nullptr, pdMS_TO_TICKS(50)) == pdTRUE) {
      // Received a notification, stop the current loop and exit the task
      break;
    }
  }

  // Clean up any resources before exiting
  msgHandler.reset();  // Explicitly clean up the unique_ptr
  ESP_LOGI(TAG, "BLE message task is exiting");
  vTaskDelete(NULL);
}

const char *get_client_address() { return clientAddress.c_str(); }

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

bool ble_is_advertising() { return bleAdvCurrState != BLE_ADV_STOP; }

bool ble_is_connected() { return deviceConnected; }

void ble_adv_task(void *arg) {
  BaseType_t xResult;
  uint32_t state;
  BLE_ADV_STATE new_state = BLE_ADV_STOP, next_state = new_state;

  while (true) {
    // Wait to be notified from Wi-Fi/MQTT task
    xResult = xTaskNotifyWait(pdTRUE, pdTRUE, &state, pdMS_TO_TICKS(10));
    if (xResult == pdPASS) {
      new_state = (BLE_ADV_STATE)state;
    }

    switch (new_state) {
      case BLE_ADV_START: {
        next_state = handle_ble_adv_start();
      } break;
      case BLE_ADV_STOP: {
        next_state = handle_ble_adv_stop();
      } break;
      case BLE_ADV_DELAY_START: {
        // start a timer, and enter "BLE_ADV_START" state after timer triggers
        next_state = handle_ble_adv_delay_start(BLE_ADV_TIMER_INTERVAL_MS);
      } break;
      default: {
        ESP_LOGW(TAG, "Invalid BLE adv task state: %" PRIx32, state);
        continue;
      } break;
    }
    if (next_state != bleAdvCurrState) {
      ESP_LOGI(TAG, "BLE adv task state changed: %d => %d", bleAdvCurrState,
               next_state);
      bleAdvCurrState = next_state;
    }
    new_state = bleAdvCurrState;
  }
}

esp_err_t start_ble_adv_task() {
  BaseType_t res = xTaskCreate(ble_adv_task, "ble_adv", 1024 * 3, nullptr, 5,
                               &bleAdvTaskHandle);
  if (res != pdPASS) {
    ESP_LOGE(TAG, "Failed to create BLE adv task");
    return ESP_ERR_NO_MEM;
  }
  return ESP_OK;
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

void ble_notify(const uint8_t *data, size_t length) {
  if (NimBLEDevice::isInitialized() && pCharacteristicTX && deviceConnected) {
    pCharacteristicTX->notify(data, length);
  }
}

// Handle current state transmit to BLE_ADV_START state
BLE_ADV_STATE handle_ble_adv_start() {
  if (bleAdvCurrState == BLE_ADV_START) {
    return bleAdvCurrState;
  }

  // start advertising indefinitely
  wifi_controller.Notify(WiFiEventType::STOP_WEB_SERVER);
  if (ble_init() != ESP_OK || !pServer->startAdvertising()) {
    ESP_LOGE(TAG, "Failed to start advertising");
    wifi_controller.Notify(WiFiEventType::START_WEB_SERVER);
    return bleAdvCurrState;
  }

  DisplayManager::GetInstance().SetAnimation(AnimationType::BLE_ADVERTISING,
                                             true);
  return BLE_ADV_START;
}

// Handle current state transmit to BLE_ADV_STOP state
BLE_ADV_STATE handle_ble_adv_stop() {
  if (bleAdvCurrState == BLE_ADV_STOP) {
    return bleAdvCurrState;
  }

  // stop advertising immediately
  if (NimBLEDevice::isInitialized()) {
    if (deviceConnected && !pServer->disconnect(deviceConnHandle)) {
      ESP_LOGE(TAG,
               "Failed to disconnect from previous device before "
               "stopping advertising");
      return bleAdvCurrState;
    }
#ifdef CONFIG_DEINIT_BLE_ON_STOP_ADVERTISING
    if (ble_deinit() != ESP_OK) {
      ESP_LOGE(TAG, "Failed to deinit BLE");
      return bleAdvCurrState;
    }
#else
    if (!pServer->stopAdvertising()) {
      ESP_LOGE(TAG, "Failed to stop advertising");
      return bleAdvCurrState;
    }
#endif
  }

  wifi_controller.Notify(WiFiEventType::START_WEB_SERVER);
  DisplayManager::GetInstance().SetAnimation(AnimationType::IDLE_ANIMATION,
                                             true);
  return BLE_ADV_STOP;
}

// Handle current state transmit to BLE_ADV_DELAY_START state
BLE_ADV_STATE handle_ble_adv_delay_start(int interval_ms) {
  static int64_t adv_start_at = 0;
  BLE_ADV_STATE new_state = BLE_ADV_DELAY_START;
  int64_t now = esp_timer_get_time();

  if (bleAdvCurrState == BLE_ADV_START) {
    // BLE advertising is already started, so do nothing
    return BLE_ADV_START;
  }
  if (bleAdvCurrState == BLE_ADV_STOP) {
    // Calculate the time to start advertising
    adv_start_at = now + interval_ms * 1000;
  }

  if (now < adv_start_at) {
    // Stay in the current state until the time to start advertising
    return new_state;
  }

  wifi_controller.Notify(WiFiEventType::STOP_WEB_SERVER);
  if (!(ble_init() == ESP_OK && pServer->startAdvertising())) {
    ESP_LOGE(TAG, "Failed to start advertising");
    wifi_controller.Notify(WiFiEventType::START_WEB_SERVER);
    return new_state;
  }

  // Keep advertising indefinitely, so return to BLE_ADV_START state
  return BLE_ADV_START;
}
