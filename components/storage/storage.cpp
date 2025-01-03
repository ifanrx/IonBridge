#include "storage.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <fstream>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "ionbridge.h"
#include "sdkconfig.h"

#define FILE_SYSTEM_BASE_PATH CONFIG_FILE_SYSTEM_BASE_PATH
#define TEMP_FOLDER_PATH CONFIG_TEMP_FOLDER_PATH "/"
#define FIRMWARE_PATH_LEN 32
#define FIRMWARE_PATH_TPL FILE_SYSTEM_BASE_PATH "/%s_%s.bin"

#ifdef CONFIG_MCU_MODEL_SW3566
#define SW3566_FIRMWARE_SEMVER CONFIG_SW3566_FIRMWARE_SEMVER
#define SW3566_CHECKSUM_ADDR 0x1dfa0
#define FPGA_FIRMWARE_SEMVER CONFIG_FPGA_FIRMWARE_SEMVER
#endif

static const char *TAG = "Storage";

esp_err_t Storage::Init() {
  esp_vfs_littlefs_conf_t conf = {
      .base_path = FILE_SYSTEM_BASE_PATH,  // Mount path
      .partition_label =
          "littlefs",  // Partition label, matching the name in partition table
      .partition = NULL,  // Select partition automatically based on the label
      .format_if_mount_failed = true,  // Format file system if mounting fails
      .read_only = false,              // Read-only mode flag
      .dont_mount = false,
      .grow_on_mount = true,
  };

  // Initialize and mount the LittleFS file system
  ESP_RETURN_ON_ERROR(esp_vfs_littlefs_register(&conf), "TAG",
                      "Failed to register esp_vfs_littlefs");

  int ret = mkdir(CONFIG_TEMP_FOLDER_PATH, 0777);
  if (ret != 0 && errno != EEXIST) {
    ESP_LOGI(TAG, "Cannot create directory %s: %s", CONFIG_TEMP_FOLDER_PATH,
             strerror(errno));
    return ESP_ERR_INVALID_STATE;
  }
  return ESP_OK;
}

const char *Storage::GetBasePath() { return FILE_SYSTEM_BASE_PATH; }

void Storage::GetTempFilePath(const char *filename, char *path) {
  strcpy(path, TEMP_FOLDER_PATH);
  strcat(path, filename);
}

std::streamsize Storage::GetFileSize(const char *path) {
  std::ifstream file(path, std::ifstream::ate | std::ifstream::binary);
  if (!(file && file.good())) {
    // Return -1 or another suitable error value if the file fails to open
    return -1;
  }
  // tellg() returns the current position of the 'get' pointer.
  // Since we used std::ifstream::ate, the pointer is at the end of the file
  // when opened
  return file.tellg();
}

esp_err_t Storage::ReadFilePart(const char *path, uint8_t *part,
                                size_t partSize, int offset) {
  std::ifstream file(path, std::ios::binary);
  if (!(file && file.good())) {
    // Error handling if the file cannot be opened
    ESP_LOGE(TAG, "%s not found", path);
    return ESP_FAIL;
  }

  // Seek to the specified location in the file
  file.seekg(offset);
  if (!file) {
    // Error handling if the seek operation failed
    ESP_LOGE(TAG, "Error seeking to %d", offset);
    return ESP_FAIL;
  }

  if (file.readsome(reinterpret_cast<char *>(part), partSize) != partSize) {
    // Handle the error if the file does not contain enough data
    ESP_LOGE(TAG, "Error reading part of the file");
    return ESP_FAIL;
  }

  return ESP_OK;
}

esp_err_t Storage::MoveFile(const char *src, const char *dst, bool overwrite) {
  ESP_LOGI(TAG, "Moving file from %s to %s", src, dst);
  // Check if the destination file already exists
  struct stat st;
  if (stat(dst, &st) == 0) {
    ESP_LOGW(TAG, "Destination file %s already exists", dst);
    if (!overwrite) {
      return ESP_FAIL;
    }
    // Remove the destination file if it already exists
    if (unlink(dst) != 0) {
      ESP_LOGE(TAG, "Failed to remove destination file %s", dst);
      return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Successfully removed destination file %s", dst);
  }

  if (rename(src, dst) != 0) {
    ESP_LOGE(TAG, "Failed to move file from %s to %s", src, dst);
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t Storage::DeleteFile(const char *path) {
  ESP_LOGI(TAG, "Deleting file: %s", path);
  if (unlink(path) != 0) {
    ESP_LOGE(TAG, "Failed to delete file: %s", path);
    return ESP_FAIL;
  }
  return ESP_OK;
}

size_t Storage::GetUserAppSize() {
#ifdef CONFIG_MCU_MODEL_SW3566
  esp_err_t ret __attribute__((unused));
  char path[FIRMWARE_PATH_LEN];
  ESP_GOTO_ON_ERROR(GetUserAppPath(SW3566_FIRMWARE_SEMVER, path, sizeof(path)),
                    FAIL, TAG, "GetUserAppPath");
  return GetFileSize(path);
FAIL:
#endif
  return 0;
}

esp_err_t Storage::GetUserAppPath(const char *version, char *path,
                                  size_t pathSize) {
  return GetFirmwarePath("SW3566", version, path, pathSize);
}

bool Storage::GetUserAppChecksum(uint32_t *checksum) {
  *checksum = 0;
#ifdef CONFIG_MCU_MODEL_SW3566
  // Define the offset in the file for the checksum
  int checksumOffset = SW3566_CHECKSUM_ADDR;
  // Read the 5 uint32_t data for the SHA-1 checksum

  ESP_RETURN_FALSE_ON_ERROR(
      GetUserAppPart((uint8_t *)checksum, sizeof(uint32_t) * 5, checksumOffset),
      "GetUserAppPart checksum");
#endif
  return true;
}

esp_err_t Storage::GetUserAppPart(uint8_t *part, size_t partSize, int offset) {
#ifdef CONFIG_MCU_MODEL_SW3566
  // Open the file in binary mode
  char path[FIRMWARE_PATH_LEN];
  ESP_RETURN_ON_ERROR(
      GetUserAppPath(SW3566_FIRMWARE_SEMVER, path, sizeof(path)), TAG,
      "GetUserAppPath");
  return ReadFilePart(path, part, partSize, offset);
#else
  return ESP_FAIL;
#endif
}

esp_err_t Storage::GetFPGAFirmware(uint8_t *part, size_t partSize, int offset) {
#ifdef CONFIG_MCU_MODEL_SW3566
  char path[FIRMWARE_PATH_LEN];
  ESP_RETURN_ON_ERROR(
      GetFPGAFirmwarePath(FPGA_FIRMWARE_SEMVER, path, sizeof(path)), TAG,
      "GetFPGAFirmwarePath");

  return ReadFilePart(path, part, partSize, offset);
#else
  return ESP_FAIL;
#endif
}

size_t Storage::GetFPGAFirmwareSize() {
#ifdef CONFIG_MCU_MODEL_SW3566
  esp_err_t ret __attribute__((unused));
  char path[FIRMWARE_PATH_LEN];
  ESP_GOTO_ON_ERROR(
      GetFPGAFirmwarePath(FPGA_FIRMWARE_SEMVER, path, sizeof(path)), FAIL, TAG,
      "GetFPGAFirmwarePath");
  return GetFileSize(path);
FAIL:
#endif
  return 0;
}

esp_err_t Storage::GetFPGAFirmwarePath(const char *version, char *path,
                                       size_t pathSize) {
  return GetFirmwarePath("FPGA", version, path, pathSize);
}

esp_err_t Storage::GetFirmwarePath(const char *prefix, const char *version,
                                   char *path, size_t pathSize) {
  int res = snprintf(path, pathSize, FIRMWARE_PATH_TPL, prefix, version);
  if (res < 0 || res >= pathSize) {
    return ESP_ERR_INVALID_SIZE;
  }
  return ESP_OK;
}

esp_err_t Storage::RemoveOldVersionFile(const char *currentVersion,
                                        const char *prefix) {
  DIR *directory = opendir(FILE_SYSTEM_BASE_PATH);
  if (!directory) {
    ESP_LOGE(TAG, "Failed to open directory %s", FILE_SYSTEM_BASE_PATH);
    return ESP_FAIL;
  }

  struct dirent *entry;
  char filepath[320];
  while ((entry = readdir(directory)) != NULL) {
    if (entry->d_type != DT_REG) {  // Skip if it's not a regular file
      continue;
    }

    char *filename_copy = strdup(entry->d_name);
    if (!filename_copy) {
      closedir(directory);  // Ensure directory is closed before returning
      return ESP_ERR_NO_MEM;
    }

    // Split the filename by "_"
    char *file_prefix = strtok(filename_copy, "_");
    char *file_version_with_ext = strtok(NULL, "_");

    // Check if the file prefix matches the given prefix
    if (!(file_prefix && file_version_with_ext &&
          strcmp(file_prefix, prefix) == 0)) {
      free(filename_copy);  // Free the allocated memory for the filename copy
      continue;
    }

    // Remove the ".bin" extension from the version part
    char *dot_position = strrchr(file_version_with_ext, '.');
    if (dot_position) {
      *dot_position = '\0';  // Terminate the string to remove the extension
    }

    // Check if the file is the current version and skip deletion if it is
    if (strcmp(file_version_with_ext, currentVersion) == 0) {
      free(filename_copy);  // Free the allocated memory for the filename copy
      continue;
    }

    // Construct the full path for the file to be deleted
    snprintf(filepath, sizeof(filepath), "%s/%s", FILE_SYSTEM_BASE_PATH,
             entry->d_name);

    // Delete the file at the constructed path
    DeleteFile(filepath);

    // Free the allocated memory for the filename copy
    free(filename_copy);
  }

  // Close the directory stream to release resources
  closedir(directory);

  return ESP_OK;
}
