#include "storage.h"

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>

#include "decompress.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "mbedtls/md5.h"   // IWYU pragma: keep
#include "mbedtls/sha1.h"  // IWYU pragma: keep
#include "sdkconfig.h"
#include "version.h"

#define FILE_SYSTEM_BASE_PATH CONFIG_FILE_SYSTEM_BASE_PATH
#define TEMP_FOLDER_PATH CONFIG_TEMP_FOLDER_PATH "/"
#define FIRMWARE_PATH_LEN 32
#define VERSION_LEN 32
#define FIRMWARE_PATH_TPL FILE_SYSTEM_BASE_PATH "/%s_%s.bin"

#ifdef CONFIG_MCU_MODEL_SW3566
#define SW3566_FIRMWARE_SEMVER CONFIG_SW3566_FIRMWARE_SEMVER
#define FPGA_FIRMWARE_SEMVER CONFIG_FPGA_FIRMWARE_SEMVER
#endif

// Define compression file extension based on Kconfig
#ifdef CONFIG_STORAGE_COMPRESSION_XZ
#define COMPRESSED_EXT ".xz"
#elif defined(CONFIG_STORAGE_COMPRESSION_GZIP)
#define COMPRESSED_EXT ".gz"
#else
#define COMPRESSED_EXT ""
#endif

static const char *TAG = "Storage";

// Lightweight stat wrapper to avoid duplicated error-prone boilerplate
static bool file_exists(const char *path) {
  struct stat st;
  return stat(path, &st) == 0;
}

static const char *get_compression_ext(Storage::CompressionType compress) {
  switch (compress) {
    case Storage::COMPRESSION_XZ:
      return ".xz";
    case Storage::COMPRESSION_GZIP:
      return ".gz";
    default:
      return nullptr;
  }
}

// Normalize caller-provided paths to avoid double-appending extensions and to
// allow fallback to uncompressed files. The caller may already include the
// compression suffix.
struct CompressionPaths {
  char compressed[256];
  char uncompressed[256];
  bool has_compressed_ext;
};

static CompressionPaths BuildCompressionPaths(
    const char *path, Storage::CompressionType compress) {
  CompressionPaths paths = {{0}, {0}, false};

  // No compression configured; only the raw path is relevant.
  const char *ext = get_compression_ext(compress);
  if (!ext) {
    snprintf(paths.uncompressed, sizeof(paths.uncompressed), "%s", path);
    return paths;
  }

  size_t ext_len = strlen(ext);
  size_t path_len = strlen(path);
  paths.has_compressed_ext =
      path_len >= ext_len &&
      strncmp(path + path_len - ext_len, ext, ext_len) == 0;

  // Always keep the caller path as the compressed candidate
  snprintf(paths.compressed, sizeof(paths.compressed), "%s", path);
  if (!paths.has_compressed_ext) {
    snprintf(paths.compressed, sizeof(paths.compressed), "%s%s", path, ext);
  }

  // Base path without compression extension for fallback
  snprintf(paths.uncompressed, sizeof(paths.uncompressed), "%s", path);
  if (paths.has_compressed_ext) {
    paths.uncompressed[path_len - ext_len] = '\0';
  }

  return paths;
}

// Forward declaration for decompression dispatcher
static esp_err_t ReadCompressedFilePart(const char *compressed_path,
                                        uint8_t *part, size_t partSize,
                                        int offset);

// Simple raw read helper for uncompressed files.
static esp_err_t ReadUncompressedFilePart(const char *path, uint8_t *part,
                                          size_t partSize, int offset) {
  FILE *file = fopen(path, "rb");
  if (file == NULL) {
    ESP_LOGE(TAG, "Failed to open file: %s", path);
    return ESP_FAIL;
  }

  if (fseek(file, offset, SEEK_SET) != 0) {
    ESP_LOGE(TAG, "Error seeking to %d in %s", offset, path);
    fclose(file);
    return ESP_FAIL;
  }

  size_t bytes_read = fread(part, 1, partSize, file);
  fclose(file);

  if (bytes_read != partSize) {
    ESP_LOGE(TAG, "Error reading part of file %s: expected %d, got %d", path,
             (int)partSize, (int)bytes_read);
    return ESP_FAIL;
  }
  return ESP_OK;
}

// Note: Materialization was removed in favor of streaming-only decompression.
// All compressed files now use streaming decompression without temp files.
// Exception: Multi-chip upgrade uses temporary decompression to avoid
// re-decompressing the same firmware for each chip.

// Temporary firmware paths for multi-chip upgrade optimization
static char s_upgrade_userapp_temp[64] = {0};
static char s_upgrade_programmer_temp[64] = {0};

// Cleanup all .tmp files in temp folder to free space before decompression.
// This handles stale temp files from previous failed/interrupted upgrades.
static void cleanup_temp_folder() {
  DIR *dir = opendir(CONFIG_TEMP_FOLDER_PATH);
  if (!dir) {
    return;
  }

  struct dirent *entry;
  char filepath[320];  // Match other directory iteration buffers
  while ((entry = readdir(dir)) != NULL) {
    if (entry->d_type != DT_REG) {
      continue;
    }
    // Only remove .tmp files (upgrade temp files)
    size_t len = strlen(entry->d_name);
    if (len > 4 && strcmp(entry->d_name + len - 4, ".tmp") == 0) {
      snprintf(filepath, sizeof(filepath), "%s/%s", CONFIG_TEMP_FOLDER_PATH,
               entry->d_name);
      if (remove(filepath) == 0) {
        ESP_LOGI(TAG, "Cleaned up stale temp file: %s", filepath);
      }
    }
  }
  closedir(dir);
}

void FreeDecompressionCache(DecompressionCache *cache) {
  if (!cache) {
    return;
  }
#ifdef CONFIG_STORAGE_COMPRESSION_GZIP
  FreeGzipCache(cache);
#endif
#ifdef CONFIG_STORAGE_COMPRESSION_XZ
  FreeXzCache(cache);
#endif
  if (cache->file) {
    fclose(cache->file);
  }
  free(cache->in_buf);
  free(cache->out_buf);
  *cache = DecompressionCache{};
}

// File-scope cache for streaming decompression, accessible for external cleanup
static DecompressionCache s_decompression_cache;

void Storage::ReleaseDecompressionCache() {
  FreeDecompressionCache(&s_decompression_cache);
  ESP_LOGD(TAG, "Decompression cache released");
}

esp_err_t InitDecompressionCache(DecompressionCache *cache, const char *path,
                                 Storage::CompressionType type,
                                 size_t chunk_size) {
  FreeDecompressionCache(cache);
  cache->type = type;
  snprintf(cache->path, sizeof(cache->path), "%s", path);
  cache->offset = 0;

  cache->file = fopen(path, "rb");
  if (!cache->file) {
    ESP_LOGE(TAG, "%s not found", path);
    FreeDecompressionCache(cache);
    return ESP_FAIL;
  }

  cache->in_buf = (uint8_t *)malloc(chunk_size);
  cache->out_buf = (uint8_t *)malloc(chunk_size);
  if (!cache->in_buf || !cache->out_buf) {
    ESP_LOGE(TAG, "Failed to allocate %d byte buffers", (int)chunk_size);
    FreeDecompressionCache(cache);
    return ESP_FAIL;
  }

#ifdef CONFIG_STORAGE_COMPRESSION_GZIP
  if (type == Storage::COMPRESSION_GZIP) {
    esp_err_t ret = InitGzipCache(cache);
    if (ret != ESP_OK) {
      FreeDecompressionCache(cache);
      return ret;
    }
  }
#endif

#ifdef CONFIG_STORAGE_COMPRESSION_XZ
  if (type == Storage::COMPRESSION_XZ) {
    esp_err_t ret = InitXzCache(cache, chunk_size);
    if (ret != ESP_OK) {
      FreeDecompressionCache(cache);
      return ret;
    }
  }
#endif

  return ESP_OK;
}

// Get compression type from build configuration
Storage::CompressionType Storage::GetCompressionType() {
#if defined(CONFIG_STORAGE_COMPRESSION_XZ)
  return COMPRESSION_XZ;
#elif defined(CONFIG_STORAGE_COMPRESSION_GZIP)
  return COMPRESSION_GZIP;
#else
  return COMPRESSION_NONE;
#endif
}

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

void Storage::GetTempFilePath(const char *filename, char *path,
                              bool use_compression) {
  strcpy(path, TEMP_FOLDER_PATH);
  strcat(path, filename);

  // Add compression extension if requested
  if (use_compression) {
    CompressionType compress = GetCompressionType();
    if (compress == COMPRESSION_XZ) {
      strcat(path, ".xz");
    } else if (compress == COMPRESSION_GZIP) {
      strcat(path, ".gz");
    }
  }
}

// XZ Variable-Length Integer decoder
static size_t read_xz_vli(FILE *file) {
  size_t value = 0;
  size_t shift = 0;
  uint8_t byte;

  do {
    if (fread(&byte, 1, 1, file) != 1) {
      return 0;  // Read error
    }
    value |= (size_t)(byte & 0x7F) << shift;
    shift += 7;
    if (shift > 63) {
      return 0;  // Overflow protection
    }
  } while (byte & 0x80);

  return value;
}

// Parse XZ file to extract uncompressed size from Index
static size_t get_xz_uncompressed_size(const char *xz_path) {
  FILE *file = fopen(xz_path, "rb");
  if (!file) {
    ESP_LOGE(TAG, "Failed to open XZ file: %s", xz_path);
    return 0;
  }

  // Read Stream Footer (last 12 bytes)
  if (fseek(file, -12, SEEK_END) != 0) {
    ESP_LOGE(TAG, "Failed to seek to XZ footer");
    fclose(file);
    return 0;
  }

  uint8_t footer[12];
  if (fread(footer, 1, 12, file) != 12) {
    ESP_LOGE(TAG, "Failed to read XZ footer");
    fclose(file);
    return 0;
  }

  // Verify footer magic bytes (0x00, "YZ" = 0x59, 0x5A)
  if (footer[10] != 0x59 || footer[11] != 0x5A) {
    ESP_LOGE(TAG, "Invalid XZ footer magic");
    fclose(file);
    return 0;
  }

  // Extract Backward Size (4 bytes at offset 4, little-endian)
  uint32_t backward_size =
      footer[4] | (footer[5] << 8) | (footer[6] << 16) | (footer[7] << 24);

  // Backward Size is stored as (actual_size / 4) - 1
  size_t index_size = ((size_t)backward_size + 1) * 4;

  // Seek to start of Index (before footer, after padding)
  // Index position = -(footer_size + index_size)
  if (fseek(file, -(12 + (long)index_size), SEEK_END) != 0) {
    ESP_LOGE(TAG, "Failed to seek to XZ Index");
    fclose(file);
    return 0;
  }

  // Read Index Indicator (should be 0x00)
  uint8_t index_indicator;
  if (fread(&index_indicator, 1, 1, file) != 1 || index_indicator != 0x00) {
    ESP_LOGE(TAG, "Invalid XZ Index indicator: 0x%02X", index_indicator);
    fclose(file);
    return 0;
  }

  // Read Number of Records (VLI)
  size_t num_records = read_xz_vli(file);
  if (num_records == 0) {
    ESP_LOGE(TAG, "Failed to read XZ number of records");
    fclose(file);
    return 0;
  }

  // Sum uncompressed sizes from all Block records
  size_t total_uncompressed = 0;
  for (size_t i = 0; i < num_records; i++) {
    // Read Unpadded Size (VLI) - we don't need this value
    size_t unpadded_size = read_xz_vli(file);
    if (unpadded_size == 0) {
      ESP_LOGE(TAG, "Failed to read unpadded size for record %d", (int)i);
      fclose(file);
      return 0;
    }

    // Read Uncompressed Size (VLI) - this is what we want
    size_t uncompressed_size = read_xz_vli(file);
    if (uncompressed_size == 0) {
      ESP_LOGE(TAG, "Failed to read uncompressed size for record %d", (int)i);
      fclose(file);
      return 0;
    }

    total_uncompressed += uncompressed_size;
  }

  fclose(file);
  ESP_LOGD(TAG, "XZ file %s: %d records, total uncompressed size: %d", xz_path,
           (int)num_records, (int)total_uncompressed);
  return total_uncompressed;
}

size_t Storage::GetFileSize(const char *path) {
  // FAST PATH: Build-time constants for bundled firmware (O(1) lookup)
  // Check if this is SW3566 or FPGA firmware that's bundled at build time
#if defined(CONFIG_SW3566_FIRMWARE_SIZE)
  if (strstr(path, "SW3566") != NULL) {
    ESP_LOGD(TAG, "Using build-time size constant for SW3566: %d",
             CONFIG_SW3566_FIRMWARE_SIZE);
    return CONFIG_SW3566_FIRMWARE_SIZE;
  }
#endif

#if defined(CONFIG_FPGA_FIRMWARE_SIZE)
  if (strstr(path, "FPGA") != NULL) {
    ESP_LOGD(TAG, "Using build-time size constant for FPGA: %d",
             CONFIG_FPGA_FIRMWARE_SIZE);
    return CONFIG_FPGA_FIRMWARE_SIZE;
  }
#endif

  // RUNTIME PATH: Parse compressed files (programmer firmware, user-specified)
  // Check build configuration for compression
  CompressionType compress = GetCompressionType();

  // If compression configured, check for compressed file
  if (compress == COMPRESSION_GZIP) {
    char gz_path[256];
    snprintf(gz_path, sizeof(gz_path), "%s.gz", path);

    struct stat st;
    if (stat(gz_path, &st) == 0) {
      // Compressed file exists, read uncompressed size from gzip footer
      FILE *file = fopen(gz_path, "rb");
      if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open compressed file: %s", gz_path);
        // Fallback to uncompressed
      } else {
        // Seek to last 4 bytes (ISIZE field in gzip format)
        if (fseek(file, -4, SEEK_END) == 0) {
          // Read uncompressed size (little-endian uint32_t)
          uint32_t uncompressed_size = 0;
          if (fread(&uncompressed_size, 1, 4, file) == 4) {
            fclose(file);
            ESP_LOGD(TAG, "Compressed file %s, uncompressed size: %lu", gz_path,
                     (unsigned long)uncompressed_size);
            return uncompressed_size;
          }
        }
        fclose(file);
      }
    }
    // 404: Compressed file not found, fallback to uncompressed
    ESP_LOGD(TAG, "GZIP file not found, falling back to uncompressed: %s",
             path);
  } else if (compress == COMPRESSION_XZ) {
    char xz_path[256];
    snprintf(xz_path, sizeof(xz_path), "%s.xz", path);

    struct stat st;
    if (stat(xz_path, &st) == 0) {
      // Parse XZ file directly to get uncompressed size
      size_t uncompressed_size = get_xz_uncompressed_size(xz_path);
      if (uncompressed_size > 0) {
        ESP_LOGD(TAG, "XZ compressed file %s, uncompressed size: %d", xz_path,
                 (int)uncompressed_size);
        return uncompressed_size;
      }
      ESP_LOGW(TAG, "Failed to parse XZ file size for %s", xz_path);
      // Continue to try uncompressed fallback
    }
    // 404: Compressed file not found, fallback to uncompressed
    ESP_LOGD(TAG, "XZ file not found, falling back to uncompressed: %s", path);
  }

  // Fall back to uncompressed file
  FILE *file = fopen(path, "rb");
  if (file == NULL) {
    ESP_LOGE(TAG, "Failed to open file: %s", path);
    return 0;
  }

  // Set position to end of file
  if (fseek(file, 0, SEEK_END) != 0) {
    fclose(file);
    return 0;
  }

  // Get position which is the file size
  size_t size = ftell(file);

  fclose(file);
  return size;
}

esp_err_t Storage::ReadFilePart(const char *path, uint8_t *part,
                                size_t partSize, int offset) {
  // Check build configuration for compression
  CompressionType compress = GetCompressionType();

  CompressionPaths paths = BuildCompressionPaths(path, compress);
  bool has_compressed = false;
  bool has_uncompressed = false;

  // Check for compressed file if compression configured
  if (compress != COMPRESSION_NONE) {
    has_compressed = file_exists(paths.compressed);

    if (has_compressed) {
      // All compressed files use streaming decompression (no temp files)
      ESP_LOGD(TAG, "Using compressed file: %s", paths.compressed);
      return ReadCompressedFilePart(paths.compressed, part, partSize, offset);
    } else {
      // 404: Compressed file not found, fallback to uncompressed
      ESP_LOGD(TAG, "Compressed file not found, falling back to: %s", path);
    }
  }

  // Fall back to uncompressed file
  has_uncompressed = file_exists(paths.uncompressed);

  if (!has_uncompressed) {
    ESP_LOGE(TAG, "File not found: %s (neither compressed nor uncompressed)",
             path);
    return ESP_ERR_NOT_FOUND;
  }

  return ReadUncompressedFilePart(paths.uncompressed, part, partSize, offset);
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

size_t Storage::GetProgrammerAppSize(const Version *version) {
#ifdef CONFIG_MCU_MODEL_SW3566
  esp_err_t ret __attribute__((unused));
  char path[FIRMWARE_PATH_LEN];
  if (version != nullptr) {
    ESP_GOTO_ON_ERROR(GetProgrammerPath((char *)version->ToString().c_str(),
                                        path, sizeof(path)),
                      FAIL, TAG, "GetUserAppPath");
    return GetFileSize(path);
  FAIL:
    return 0;
  }
  return 0;
#else
  return 0;
#endif
}

// Resolve firmware path to the file that ACTUALLY EXISTS.
// Tries compressed first, falls back to uncompressed.
static esp_err_t ResolveFirmwarePath(const char *prefix, const char *version,
                                     char *path, size_t path_size,
                                     bool *is_compressed) {
  Storage::GetFirmwarePath(prefix, version, path, path_size, true);
  if (file_exists(path)) {
    if (is_compressed) *is_compressed = true;
    return ESP_OK;
  }

  Storage::GetFirmwarePath(prefix, version, path, path_size, false);
  if (is_compressed) *is_compressed = false;
  return file_exists(path) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t Storage::GetUserAppPath(const char *version, char *path,
                                  size_t path_size, bool *is_compressed) {
  return ResolveFirmwarePath("SW3566", version, path, path_size, is_compressed);
}

esp_err_t Storage::GetProgrammerPath(const char *version, char *path,
                                     size_t pathSize, bool *is_compressed) {
  return ResolveFirmwarePath("PROG", version, path, pathSize, is_compressed);
}

esp_err_t Storage::GetUserAppPart(uint8_t *part, size_t partSize, int offset) {
#ifdef CONFIG_MCU_MODEL_SW3566
  // Check for pre-decompressed temp file (multi-chip upgrade optimization)
  const char *temp_path = GetUpgradeTempPath(FIRMWARE_USERAPP);
  if (temp_path) {
    return ReadUncompressedFilePart(temp_path, part, partSize, offset);
  }

  // Fall back to streaming decompression
  char path[FIRMWARE_PATH_LEN];
  ESP_RETURN_ON_ERROR(
      GetUserAppPath(SW3566_FIRMWARE_SEMVER, path, sizeof(path)), TAG,
      "GetUserAppPath");
  return ReadFilePart(path, part, partSize, offset);
#else
  return ESP_FAIL;
#endif
}

esp_err_t Storage::GetProgrammerPart(uint8_t *part, size_t partSize, int offset,
                                     const Version *version) {
#ifdef CONFIG_MCU_MODEL_SW3566
  // Open the file in binary mode
  char path[FIRMWARE_PATH_LEN];
  char version_str[VERSION_LEN];
  if (version != nullptr) {
    strcpy(version_str, version->ToString().c_str());
  } else {
    return ESP_FAIL;
  }
  ESP_RETURN_ON_ERROR(GetProgrammerPath(version_str, path, sizeof(path)), TAG,
                      "GetProgrammerPath");
  return ReadFilePart(path, part, partSize, offset);
#else
  return ESP_FAIL;
#endif
}

esp_err_t Storage::GetFPGAFirmware(uint8_t *part, size_t partSize, int offset,
                                   const char *variant) {
#ifdef CONFIG_MCU_MODEL_SW3566
  char path[FIRMWARE_PATH_LEN];
  ESP_RETURN_ON_ERROR(
      GetFPGAFirmwarePath(FPGA_FIRMWARE_SEMVER, path, sizeof(path), variant),
      TAG, "GetFPGAFirmwarePath");

  return ReadFilePart(path, part, partSize, offset);
#else
  return ESP_FAIL;
#endif
}

size_t Storage::GetFPGAFirmwareSize(const char *variant) {
#ifdef CONFIG_MCU_MODEL_SW3566
  esp_err_t ret __attribute__((unused));
  char path[FIRMWARE_PATH_LEN];
  ESP_GOTO_ON_ERROR(
      GetFPGAFirmwarePath(FPGA_FIRMWARE_SEMVER, path, sizeof(path), variant),
      FAIL, TAG, "GetFPGAFirmwarePath");
  return GetFileSize(path);
FAIL:
#endif
  return 0;
}

esp_err_t Storage::GetFPGAFirmwarePath(const char *version, char *path,
                                       size_t path_size, const char *variant) {
  if (variant && strlen(variant) > 0) {
    int res = snprintf(path, path_size, "%s/FPGA_%s_%s.bin",
                       FILE_SYSTEM_BASE_PATH, variant, version);
    if (res < 0 || res >= path_size) {
      return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
  }
  // Don't add compression extension here - ReadFilePart handles it
  // automatically
  return GetFirmwarePath("FPGA", version, path, path_size, false);
}

esp_err_t Storage::GetFirmwarePath(const char *prefix, const char *version,
                                   char *path, size_t path_size,
                                   bool use_compression) {
  int res = snprintf(path, path_size, FIRMWARE_PATH_TPL, prefix, version);
  if (res < 0 || res >= path_size) {
    return ESP_ERR_INVALID_SIZE;
  }

  // Add compression extension if requested
  if (use_compression) {
    CompressionType compress = GetCompressionType();
    if (compress == COMPRESSION_XZ) {
      strncat(path, ".xz", path_size - strlen(path) - 1);
    } else if (compress == COMPRESSION_GZIP) {
      strncat(path, ".gz", path_size - strlen(path) - 1);
    }
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

    // Remove extensions to extract version
    // Handle compound extensions: .bin, .bin.xz, .bin.gz
    size_t len = strlen(file_version_with_ext);

    // Remove compression extension if present (.xz or .gz)
    if (len > 3 && strcmp(file_version_with_ext + len - 3, ".xz") == 0) {
      file_version_with_ext[len - 3] = '\0';
      len -= 3;
    } else if (len > 3 && strcmp(file_version_with_ext + len - 3, ".gz") == 0) {
      file_version_with_ext[len - 3] = '\0';
      len -= 3;
    }

    // Remove .bin extension
    if (len > 4 && strcmp(file_version_with_ext + len - 4, ".bin") == 0) {
      file_version_with_ext[len - 4] = '\0';
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

esp_err_t Storage::ComputeMD5(const char *path, char *hexdigest) {
#define MD5_MAX_LEN 16
  const size_t buf_size = CONFIG_STORAGE_DECOMPRESSION_CHUNK_SIZE;

  // Check build configuration for compression
  CompressionType compress = GetCompressionType();

  CompressionPaths paths = BuildCompressionPaths(path, compress);
  const char *file_to_hash = path;

  if (compress != COMPRESSION_NONE && paths.compressed[0] != '\0' &&
      file_exists(paths.compressed)) {
    file_to_hash = paths.compressed;
    ESP_LOGD(TAG, "Computing MD5 of compressed file: %s", paths.compressed);
  } else {
    file_to_hash = paths.uncompressed;
    if (compress != COMPRESSION_NONE) {
      ESP_LOGD(TAG,
               "Compressed file not found, computing MD5 of uncompressed: %s",
               paths.uncompressed);
    }
  }

  // Open file for MD5 computation (no decompression, direct hash)
  FILE *file = fopen(file_to_hash, "rb");
  if (file == NULL) {
    ESP_LOGE(TAG, "Failed to open file for MD5: %s", file_to_hash);
    return ESP_FAIL;
  }

  // Allocate buffer on heap to avoid stack overflow
  unsigned char *buf = (unsigned char *)malloc(buf_size);
  if (!buf) {
    ESP_LOGE(TAG, "Failed to allocate %d byte MD5 buffer", (int)buf_size);
    fclose(file);
    return ESP_FAIL;
  }

  uint8_t md5[MD5_MAX_LEN];
  mbedtls_md5_context ctx;

  mbedtls_md5_init(&ctx);
  mbedtls_md5_starts(&ctx);

  // Read file and compute MD5 (no decompression)
  size_t read;
  while ((read = fread(buf, 1, buf_size, file)) > 0) {
    mbedtls_md5_update(&ctx, buf, read);
  }

  if (ferror(file)) {
    ESP_LOGE(TAG, "Error reading file: %s", file_to_hash);
    free(buf);
    mbedtls_md5_free(&ctx);
    fclose(file);
    return ESP_FAIL;
  }

  mbedtls_md5_finish(&ctx, md5);
  mbedtls_md5_free(&ctx);
  fclose(file);
  free(buf);

  // Convert MD5 to hexadecimal string
  for (int i = 0; i < MD5_MAX_LEN; i++) {
    sprintf(hexdigest + i * 2, "%02x", md5[i]);
  }

  ESP_LOGD(TAG, "MD5 of %s: %s", file_to_hash, hexdigest);
  return ESP_OK;

#undef MD5_MAX_LEN
}

esp_err_t Storage::ComputeRawFileSHA1(const char *path, uint8_t *sha1_out) {
#define SHA1_LEN 20
  const size_t buf_size = CONFIG_STORAGE_DECOMPRESSION_CHUNK_SIZE;

  // Check build configuration for compression
  CompressionType compress = GetCompressionType();

  CompressionPaths paths = BuildCompressionPaths(path, compress);
  const char *file_to_hash = path;

  // Prefer compressed file if it exists
  if (compress != COMPRESSION_NONE && paths.compressed[0] != '\0' &&
      file_exists(paths.compressed)) {
    file_to_hash = paths.compressed;
  } else {
    file_to_hash = paths.uncompressed;
  }

  // Open file for SHA-1 computation (no decompression, direct hash)
  FILE *file = fopen(file_to_hash, "rb");
  if (file == NULL) {
    ESP_LOGE(TAG, "Failed to open file for SHA-1: %s", file_to_hash);
    return ESP_FAIL;
  }

  // Allocate buffer on heap
  uint8_t *buf = (uint8_t *)malloc(buf_size);
  if (!buf) {
    ESP_LOGE(TAG, "Failed to allocate %d byte SHA-1 buffer", (int)buf_size);
    fclose(file);
    return ESP_ERR_NO_MEM;
  }

  mbedtls_sha1_context ctx;
  mbedtls_sha1_init(&ctx);
  mbedtls_sha1_starts(&ctx);

  // Read file and compute SHA-1 (no decompression)
  size_t read;
  while ((read = fread(buf, 1, buf_size, file)) > 0) {
    mbedtls_sha1_update(&ctx, buf, read);
  }

  esp_err_t ret = ESP_OK;
  if (ferror(file)) {
    ESP_LOGE(TAG, "Error reading file: %s", file_to_hash);
    ret = ESP_FAIL;
  } else {
    mbedtls_sha1_finish(&ctx, sha1_out);
    ESP_LOGD(TAG, "SHA-1 computed for raw file: %s", file_to_hash);
  }

  mbedtls_sha1_free(&ctx);
  fclose(file);
  free(buf);

  return ret;
#undef SHA1_LEN
}

esp_err_t Storage::ComputeCompressedFileSHA1(const char *path,
                                             uint8_t *sha1_out,
                                             size_t uncompressed_size) {
#define SHA1_LEN 20
  const size_t chunk_size = CONFIG_STORAGE_DECOMPRESSION_CHUNK_SIZE;

  // Use provided uncompressed size (compressed file size != decompressed size)
  size_t file_size = uncompressed_size;
  if (file_size == 0) {
    ESP_LOGE(TAG, "Invalid uncompressed size for SHA-1: %s", path);
    return ESP_FAIL;
  }

  // Allocate buffer for streaming read
  uint8_t *buffer = (uint8_t *)malloc(chunk_size);
  if (!buffer) {
    ESP_LOGE(TAG, "Failed to allocate %d byte SHA-1 buffer", (int)chunk_size);
    return ESP_ERR_NO_MEM;
  }

  mbedtls_sha1_context ctx;
  mbedtls_sha1_init(&ctx);
  mbedtls_sha1_starts(&ctx);

  // Stream through file, computing SHA-1 of decompressed content
  size_t offset = 0;
  esp_err_t ret = ESP_OK;
  while (offset < file_size) {
    size_t to_read =
        (file_size - offset) > chunk_size ? chunk_size : (file_size - offset);
    ret = ReadFilePart(path, buffer, to_read, offset);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Error reading file for SHA-1 at offset %d", (int)offset);
      break;
    }
    mbedtls_sha1_update(&ctx, buffer, to_read);
    offset += to_read;
  }

  if (ret == ESP_OK) {
    mbedtls_sha1_finish(&ctx, sha1_out);
  }

  mbedtls_sha1_free(&ctx);
  free(buffer);

  // Release decompression cache to free memory
  ReleaseDecompressionCache();

  if (ret == ESP_OK) {
    ESP_LOGD(TAG, "SHA-1 computed for %s (%d bytes)", path, (int)file_size);
  }
  return ret;
#undef SHA1_LEN
}

bool Storage::Exists(const char *path) {
  if (file_exists(path)) {
    return true;
  }

  CompressionType compress = GetCompressionType();
  if (compress == COMPRESSION_NONE) {
    return false;
  }

  CompressionPaths paths = BuildCompressionPaths(path, compress);

  // Caller may already include the compression extension; if so, also check the
  // uncompressed fallback. Otherwise, look for the compressed companion file.
  if (paths.has_compressed_ext) {
    return file_exists(paths.uncompressed);
  }
  return file_exists(paths.compressed);
}
static esp_err_t ReadCompressedFilePart(const char *compressed_path,
                                        uint8_t *part, size_t partSize,
                                        int offset) {
  DecompressionCache &cache = s_decompression_cache;
  const size_t chunk_size = CONFIG_STORAGE_DECOMPRESSION_CHUNK_SIZE;
  Storage::CompressionType type = Storage::GetCompressionType();

  if (type == Storage::COMPRESSION_NONE) {
    ESP_LOGE(TAG, "Compression not enabled");
    return ESP_FAIL;
  }

  bool path_changed =
      strncmp(cache.path, compressed_path, sizeof(cache.path)) != 0 ||
      cache.type != type;

  // Reset if switching files or if caller seeks backward.
  if (path_changed || offset < (int)cache.offset) {
    ESP_LOGD(TAG, "Resetting decompression cache for %s", compressed_path);
    ESP_RETURN_ON_ERROR(
        InitDecompressionCache(&cache, compressed_path, type, chunk_size), TAG,
        "InitDecompressionCache");
  }

  size_t target_end = offset + partSize;
  size_t bytes_copied = 0;
  bool success = false;

#ifdef CONFIG_STORAGE_COMPRESSION_GZIP
  if (type == Storage::COMPRESSION_GZIP) {
    success = GzipDecompressChunk(&cache, part, partSize, offset, target_end,
                                  &bytes_copied, chunk_size);
  }
#endif

#ifdef CONFIG_STORAGE_COMPRESSION_XZ
  if (type == Storage::COMPRESSION_XZ) {
    success = XzDecompressChunk(&cache, part, partSize, offset, target_end,
                                &bytes_copied, chunk_size);
  }
#endif

  if (!success || bytes_copied != partSize) {
    ESP_LOGE(TAG, "Failed to read requested bytes (got %d, expected %d)",
             (int)bytes_copied, (int)partSize);
    FreeDecompressionCache(&cache);
    return ESP_FAIL;
  }

  return ESP_OK;
}

// Decompress firmware to temp file for multi-chip upgrade optimization.
// This avoids re-decompressing the same firmware for each chip.
esp_err_t Storage::PrepareFirmwareForUpgrade(FirmwareType type) {
#ifdef CONFIG_MCU_MODEL_SW3566
  char src_path[FIRMWARE_PATH_LEN];
  char *temp_path;
  const char *temp_name;
  size_t firmware_size;
  bool is_compressed = false;

  if (type == FIRMWARE_USERAPP) {
    ESP_RETURN_ON_ERROR(GetUserAppPath(SW3566_FIRMWARE_SEMVER, src_path,
                                       sizeof(src_path), &is_compressed),
                        TAG, "GetUserAppPath");
    temp_path = s_upgrade_userapp_temp;
    temp_name = "userapp.tmp";
    firmware_size = CONFIG_SW3566_FIRMWARE_SIZE;
  } else {
    // FIRMWARE_PROGRAMMER not commonly used for multi-chip upgrade
    return ESP_ERR_NOT_SUPPORTED;
  }

  // No decompression needed for uncompressed files
  if (!is_compressed) {
    temp_path[0] = '\0';
    return ESP_OK;
  }

  // Check if temp file already exists (reuse for multi-chip upgrade)
  snprintf(temp_path, 64, "%s%s", TEMP_FOLDER_PATH, temp_name);
  if (file_exists(temp_path)) {
    ESP_LOGI(TAG, "Upgrade temp file already exists: %s", temp_path);
    return ESP_OK;
  }

  // Cleanup stale temp files to free space before decompression
  cleanup_temp_folder();

  ESP_LOGI(TAG, "Preparing firmware for upgrade: %s -> %s", src_path,
           temp_path);

  // Allocate buffer for streaming decompression
  const size_t chunk_size = CONFIG_STORAGE_DECOMPRESSION_CHUNK_SIZE;
  uint8_t *buffer = (uint8_t *)malloc(chunk_size);
  if (!buffer) {
    ESP_LOGE(TAG, "Failed to allocate decompression buffer");
    return ESP_ERR_NO_MEM;
  }

  // Open temp file for writing
  FILE *out_file = fopen(temp_path, "wb");
  if (!out_file) {
    ESP_LOGE(TAG, "Failed to create temp file: %s", temp_path);
    free(buffer);
    temp_path[0] = '\0';
    return ESP_FAIL;
  }

  // Stream decompress to temp file
  size_t offset = 0;
  esp_err_t ret = ESP_OK;
  while (offset < firmware_size) {
    size_t to_read = (firmware_size - offset) > chunk_size
                         ? chunk_size
                         : (firmware_size - offset);
    ret = ReadFilePart(src_path, buffer, to_read, offset);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read firmware at offset %d", (int)offset);
      break;
    }

    size_t written = fwrite(buffer, 1, to_read, out_file);
    if (written != to_read) {
      ESP_LOGE(TAG, "Failed to write temp file at offset %d", (int)offset);
      ret = ESP_FAIL;
      break;
    }

    offset += to_read;
  }

  fclose(out_file);
  free(buffer);
  ReleaseDecompressionCache();

  if (ret != ESP_OK) {
    remove(temp_path);
    temp_path[0] = '\0';
    return ret;
  }

  ESP_LOGI(TAG, "Firmware prepared for upgrade: %d bytes", (int)offset);
  return ESP_OK;
#else
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

// Cleanup temp firmware file after upgrade completes
void Storage::CleanupUpgradeFirmware() {
  if (s_upgrade_userapp_temp[0] != '\0') {
    if (file_exists(s_upgrade_userapp_temp)) {
      remove(s_upgrade_userapp_temp);
      ESP_LOGI(TAG, "Cleaned up upgrade temp file: %s", s_upgrade_userapp_temp);
    }
    s_upgrade_userapp_temp[0] = '\0';
  }

  if (s_upgrade_programmer_temp[0] != '\0') {
    if (file_exists(s_upgrade_programmer_temp)) {
      remove(s_upgrade_programmer_temp);
      ESP_LOGI(TAG, "Cleaned up upgrade temp file: %s",
               s_upgrade_programmer_temp);
    }
    s_upgrade_programmer_temp[0] = '\0';
  }
}

// Check if upgrade temp file is available for faster reads
const char *Storage::GetUpgradeTempPath(FirmwareType type) {
  const char *temp_path = (type == FIRMWARE_USERAPP)
                              ? s_upgrade_userapp_temp
                              : s_upgrade_programmer_temp;
  if (temp_path[0] != '\0' && file_exists(temp_path)) {
    return temp_path;
  }
  return nullptr;
}
