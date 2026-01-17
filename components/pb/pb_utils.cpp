#include "pb_utils.h"

#include "esp_log.h"
#include "pb_encode.h"

const static char *TAG = "pb_utils";

std::vector<uint8_t> serialize_pb(const pb_msgdesc_t *fields,
                                  const void *data) {
  size_t required_size = 0;
  if (!pb_get_encoded_size(&required_size, fields, data)) {
    ESP_LOGE(TAG, "Failed to get encoded size");
    return {};
  }
  std::vector<uint8_t> payload(required_size);
  pb_ostream_t stream = pb_ostream_from_buffer(payload.data(), payload.size());
  if (!pb_encode(&stream, fields, data)) {
    ESP_LOGE(TAG, "Failed to encode data %s", PB_GET_ERROR(&stream));
    return {};
  }
  return payload;
}