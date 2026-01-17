#include "fpga.h"

#include <cstring>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/semphr.h"    // IWYU pragma: keep
#include "freertos/task.h"      // IWYU pragma: keep
#include "storage.h"
#include "utils.h"

static const char *TAG = "FPGA";

SemaphoreHandle_t fpga_config_semaphore = NULL;
static bool spi_initialized = false;
static spi_device_handle_t spidev = NULL;

void fpga_task(void *pvParameters);

void fpga_init(const char *variant) {
  fpga_config_semaphore = xSemaphoreCreateBinary();
  xTaskCreate(fpga_task, "fpga", 8192, (void *)variant, 1, NULL);
  xSemaphoreTake(fpga_config_semaphore, portMAX_DELAY);
  vSemaphoreDelete(fpga_config_semaphore);
}

void fpga_task(void *pvParameters) {
  esp_err_t __attribute__((unused)) ret;
  const char *variant = (const char *)pvParameters;
  uint8_t data[FPGA_BITSTREAM_BUFSZ];
  size_t length, total_length, remaining_length;

  // Initialize SPI bus only once
  if (!spi_initialized) {
    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.mode = 0;
    devcfg.clock_speed_hz = SPI_MASTER_FREQ_20M;
    devcfg.spics_io_num = -1;
    devcfg.queue_size = 1;
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_10),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.mosi_io_num = GPIO_NUM_1;
    buscfg.miso_io_num = -1;
    buscfg.sclk_io_num = GPIO_NUM_0;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = FPGA_BITSTREAM_BUFSZ;

    ESP_GOTO_ON_ERROR(gpio_config(&io_conf), RETURN, TAG, "gpio_config");

    ESP_GOTO_ON_ERROR(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO),
                      RETURN, TAG, "spi_bus_initialize");

    ESP_GOTO_ON_ERROR(spi_bus_add_device(SPI2_HOST, &devcfg, &spidev), RETURN,
                      TAG, "spi_bus_add_device");

    spi_initialized = true;
  }

  // FPGA reset sequence
  gpio_set_level(GPIO_NUM_10, 0);
  vTaskDelay(pdMS_TO_TICKS(100));
  gpio_set_level(GPIO_NUM_10, 1);
  vTaskDelay(pdMS_TO_TICKS(100));

  total_length = Storage::GetFPGAFirmwareSize(variant);
  remaining_length = total_length;

  ESP_LOGI(TAG, "Loading FPGA firmware%s%s (%d bytes)",
           variant ? " variant: " : "", variant ? variant : "",
           (unsigned int)total_length);

  while (remaining_length > 0) {
    length = remaining_length > FPGA_BITSTREAM_BUFSZ ? FPGA_BITSTREAM_BUFSZ
                                                     : remaining_length;
    ESP_GOTO_ON_ERROR(
        Storage::GetFPGAFirmware(data, length, total_length - remaining_length,
                                 variant),
        RETURN, TAG, "Storage::GetFPGAFirmware");

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * length;
    t.tx_buffer = data;
    ESP_GOTO_ON_ERROR(spi_device_transmit(spidev, &t), RETURN, TAG,
                      "spi_device_transmit");

    remaining_length -= length;
  }

  DELAY_MS(100);

  if (variant) {
    ESP_LOGI(TAG, "FPGA is configured (variant: %s)", variant);
  } else {
    ESP_LOGI(TAG, "FPGA is configured");
  }

RETURN:
  // Release streaming decompression cache to free memory
  Storage::ReleaseDecompressionCache();
  xSemaphoreGive(fpga_config_semaphore);
  vTaskDelete(NULL);
}
