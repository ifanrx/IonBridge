#ifndef STORAGE_HELPER_H_
#define STORAGE_HELPER_H_

#include <sys/types.h>

#include <cstddef>

#include "esp_err.h"
#include "version.h"

namespace Storage {
// Compression types supported by storage layer
enum CompressionType {
  COMPRESSION_NONE = 0,
  COMPRESSION_XZ = 1,
  COMPRESSION_GZIP = 2
};

// Firmware types for upgrade preparation
enum FirmwareType { FIRMWARE_USERAPP = 0, FIRMWARE_PROGRAMMER = 1 };

// Get compression type from build configuration
CompressionType GetCompressionType();

esp_err_t Init();
const char *GetBasePath();
void GetTempFilePath(const char *filename, char *path, bool use_compression);
size_t GetFileSize(const char *path);
esp_err_t ReadFilePart(const char *path, uint8_t *part, size_t partSize,
                       int offset);
esp_err_t MoveFile(const char *src, const char *dst, bool overwrite = false);
esp_err_t DeleteFile(const char *path);

esp_err_t GetFirmwarePath(const char *prefix, const char *version, char *path,
                          size_t pathSize, bool use_compression);

size_t GetUserAppSize();
size_t GetProgrammerAppSize(const Version *version = nullptr);

// Resolve firmware path - returns the path that ACTUALLY EXISTS.
// Tries compressed first, falls back to uncompressed.
// is_compressed: output parameter, true if resolved path is compressed file.
// Returns ESP_ERR_NOT_FOUND if neither compressed nor uncompressed file exists.
esp_err_t GetUserAppPath(const char *version, char *path, size_t pathSize,
                         bool *is_compressed = nullptr);
esp_err_t GetProgrammerPath(const char *version, char *path, size_t pathSize,
                            bool *is_compressed = nullptr);
esp_err_t GetUserAppPart(uint8_t *part, size_t partSize, int offset);
esp_err_t GetProgrammerPart(uint8_t *part, size_t partSize, int offset,
                            const Version *version = nullptr);
esp_err_t GetFPGAFirmware(uint8_t *part, size_t partSize, int offset,
                          const char *variant = nullptr);
size_t GetFPGAFirmwareSize(const char *variant = nullptr);
esp_err_t GetFPGAFirmwarePath(const char *version, char *path, size_t pathSize,
                              const char *variant = nullptr);
esp_err_t RemoveOldVersionFile(const char *currentVersion, const char *prefix);
esp_err_t ComputeMD5(const char *path, char *hexdigest);
bool Exists(const char *path);
// Release streaming decompression cache (call after sequential firmware load)
void ReleaseDecompressionCache();
// Compute SHA-1 of raw file bytes (no decompression)
// Use this to hash compressed .xz files directly for integrity verification
esp_err_t ComputeRawFileSHA1(const char *path, uint8_t *sha1_out);
// Compute SHA-1 of a (possibly compressed) file using streaming decompression
// uncompressed_size: expected size after decompression (required for compressed
// files)
esp_err_t ComputeCompressedFileSHA1(const char *path, uint8_t *sha1_out,
                                    size_t uncompressed_size);

// Multi-chip upgrade optimization: decompress firmware once to temp file
// Call before upgrading multiple chips, cleanup after all done
esp_err_t PrepareFirmwareForUpgrade(FirmwareType type);
void CleanupUpgradeFirmware();
const char *GetUpgradeTempPath(FirmwareType type);
};  // namespace Storage

#endif
