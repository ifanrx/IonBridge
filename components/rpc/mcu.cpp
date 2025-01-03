#include "rpc.h"

esp_err_t rpc::mcu::port_connect(uint8_t mcu) {
  return set_port_state(mcu, false);
}

esp_err_t rpc::mcu::port_disconnect(uint8_t mcu) {
  return set_port_state(mcu, true);
}
