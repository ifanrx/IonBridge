#ifndef DECOMPRESS_H_
#define DECOMPRESS_H_

#include <cstddef>
#include <cstdio>

#include "esp_err.h"
#include "sdkconfig.h"
#include "storage.h"

#ifdef CONFIG_STORAGE_COMPRESSION_GZIP
#include "zlib.h"
#endif

#ifdef CONFIG_STORAGE_COMPRESSION_XZ
#include "xz.h"
#endif

// Cache to avoid re-initializing and re-reading compressed streams on each
// chunked read (e.g., FPGA bitstream). Maintains streaming decoder state across
// sequential reads without buffering the whole file.
struct DecompressionCache {
  Storage::CompressionType type = Storage::COMPRESSION_NONE;
  char path[256] = {0};
  FILE *file = nullptr;
  size_t offset = 0;  // Total bytes produced so far
  uint8_t *in_buf = nullptr;
  uint8_t *out_buf = nullptr;
#ifdef CONFIG_STORAGE_COMPRESSION_GZIP
  z_stream gz_stream{};
#endif
#ifdef CONFIG_STORAGE_COMPRESSION_XZ
  struct xz_dec *xz_dec = nullptr;
  struct xz_buf xz_buf{};
#endif
};

// Initialize decompression cache for streaming reads
esp_err_t InitDecompressionCache(DecompressionCache *cache, const char *path,
                                 Storage::CompressionType type,
                                 size_t chunk_size);

// Free all resources held by decompression cache
void FreeDecompressionCache(DecompressionCache *cache);

#ifdef CONFIG_STORAGE_COMPRESSION_GZIP
// Initialize GZIP decoder in cache
esp_err_t InitGzipCache(DecompressionCache *cache);
// Free GZIP decoder resources
void FreeGzipCache(DecompressionCache *cache);
// GZIP streaming decompression - produce bytes into output buffer
// Returns true on success, false on error
// Sets bytes_copied to number of bytes actually copied to output
bool GzipDecompressChunk(DecompressionCache *cache, uint8_t *part,
                         size_t partSize, int offset, size_t target_end,
                         size_t *bytes_copied, size_t chunk_size);
#endif

#ifdef CONFIG_STORAGE_COMPRESSION_XZ
// Initialize XZ decoder in cache
esp_err_t InitXzCache(DecompressionCache *cache, size_t chunk_size);
// Free XZ decoder resources
void FreeXzCache(DecompressionCache *cache);
// XZ streaming decompression - produce bytes into output buffer
// Returns true on success, false on error
// Sets bytes_copied to number of bytes actually copied to output
bool XzDecompressChunk(DecompressionCache *cache, uint8_t *part,
                       size_t partSize, int offset, size_t target_end,
                       size_t *bytes_copied, size_t chunk_size);
#endif

#endif  // DECOMPRESS_H_
