#ifndef TASK_MAP_H_
#define TASK_MAP_H_

#include <stdint.h>

#include <unordered_map>

#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "portmacro.h"
#include "sdkconfig.h"

class TaskMap {
 public:
  TaskMap() {
    capacity = CONFIG_TASK_MAP_CAPACITY;
    map_mutex = xSemaphoreCreateMutex();
    mutex_timeout = pdMS_TO_TICKS(CONFIG_TASK_MAP_MUTEX_TIMEOUT_MS);
  }
  TaskMap(uint8_t cap, uint32_t timeout_ms = 100) : capacity(cap) {
    map_mutex = xSemaphoreCreateMutex();
    mutex_timeout = pdMS_TO_TICKS(timeout_ms);
  }
  ~TaskMap() {
    if (map_mutex) {
      vSemaphoreDelete(map_mutex);
    }
  }

  bool add_task(uint8_t addr, uint16_t command, TaskHandle_t handle);
  bool remove_task(uint8_t addr, uint16_t command);
  TaskHandle_t get_task(uint8_t addr, uint16_t command);

 private:
  uint8_t capacity;

  std::unordered_map<uint32_t, TaskHandle_t> tasks;
  SemaphoreHandle_t map_mutex;  // Mutex to protect the map
  TickType_t mutex_timeout;
};

#endif
