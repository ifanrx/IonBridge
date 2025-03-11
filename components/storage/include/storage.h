#ifndef STORAGE_HELPER_H_
#define STORAGE_HELPER_H_

#include <sys/types.h>

#include <cstddef>
#include <ios>

#include "esp_err.h"

namespace Storage {
esp_err_t Init();
const char *GetBasePath();
void GetTempFilePath(const char *filename, char *path);
std::streamsize GetFileSize(const char *path);
esp_err_t ReadFilePart(const char *path, uint8_t *part, size_t partSize,
                       int offset);
esp_err_t MoveFile(const char *src, const char *dst, bool overwrite = false);
esp_err_t DeleteFile(const char *path);

esp_err_t GetFirmwarePath(const char *prefix, const char *version, char *path,
                          size_t pathSize);

size_t GetUserAppSize();
esp_err_t GetUserAppPath(const char *version, char *path, size_t pathSize);
esp_err_t GetUserAppPart(uint8_t *part, size_t partSize, int offset);
bool GetUserAppChecksum(uint32_t *checksum);
esp_err_t GetFPGAFirmware(uint8_t *part, size_t partSize, int offset);
size_t GetFPGAFirmwareSize();
esp_err_t GetFPGAFirmwarePath(const char *version, char *path, size_t pathSize);
esp_err_t RemoveOldVersionFile(const char *currentVersion, const char *prefix);
esp_err_t ComputeMD5(const char *path, char *hexdigest);
bool Exists(const char *path);
};  // namespace Storage

#endif
