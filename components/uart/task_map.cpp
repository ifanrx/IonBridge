#include "task_map.h"

#define COMPOSITE_KEY(addr, command) \
  ((uint32_t)(((uint32_t)(addr) << 16) | (uint32_t)(command)))

bool TaskMap::add_task(uint8_t addr, uint16_t command, TaskHandle_t handle) {
  if (tasks.size() >= capacity) {
    return false;
  }
  if (xSemaphoreTake(map_mutex, mutex_timeout) == pdTRUE) {  // Take the mutex
    uint32_t key = COMPOSITE_KEY(addr, command);
    tasks[key] = handle;
    xSemaphoreGive(map_mutex);  // Release the mutex
    return true;
  }
  return false;
}

bool TaskMap::remove_task(uint8_t addr, uint16_t command) {
  if (xSemaphoreTake(map_mutex, mutex_timeout) == pdTRUE) {  // Take the mutex
    uint32_t key = COMPOSITE_KEY(addr, command);
    tasks.erase(key);
    xSemaphoreGive(map_mutex);  // Release the mutex
    return true;
  }
  return false;
}

TaskHandle_t TaskMap::get_task(uint8_t addr, uint16_t command) {
  uint32_t key = COMPOSITE_KEY(addr, command);
  if (xSemaphoreTake(map_mutex, mutex_timeout) == pdTRUE) {  // Take the mutex
    auto it = tasks.find(key);
    if (it != tasks.end()) {
      xSemaphoreGive(map_mutex);  // Release the mutex
      return it->second;
    }
    xSemaphoreGive(map_mutex);  // Release the mutex
  }
  return nullptr;
}
