#include "decompress.h"

#ifdef CONFIG_STORAGE_COMPRESSION_XZ

#include <cstring>

#include "esp_log.h"
#include "sdkconfig.h"
#include "xz.h"

static const char *TAG = "XZ";

esp_err_t InitXzCache(DecompressionCache *cache, size_t chunk_size) {
  uint32_t dict_size = CONFIG_STORAGE_COMPRESSION_XZ_DICT_SIZE * 1024;
  // Use XZ_PREALLOC to reduce memory fragmentation:
  // - Allocates dictionary upfront with decoder state
  // - All memory allocated before decompression starts
  // - Freed together, leaving larger contiguous blocks
  cache->xz_dec = xz_dec_init(XZ_PREALLOC, dict_size);
  if (!cache->xz_dec) {
    ESP_LOGE(TAG, "Failed to initialize XZ decoder (dict=%uKB)",
             CONFIG_STORAGE_COMPRESSION_XZ_DICT_SIZE);
    return ESP_FAIL;
  }
  cache->xz_buf.in = cache->in_buf;
  cache->xz_buf.in_pos = 0;
  cache->xz_buf.in_size = 0;
  cache->xz_buf.out = cache->out_buf;
  cache->xz_buf.out_pos = 0;
  cache->xz_buf.out_size = chunk_size;
  return ESP_OK;
}

void FreeXzCache(DecompressionCache *cache) {
  if (cache->type == Storage::COMPRESSION_XZ && cache->xz_dec) {
    xz_dec_end(cache->xz_dec);
    cache->xz_dec = nullptr;
  }
}

bool XzDecompressChunk(DecompressionCache *cache, uint8_t *part,
                       size_t partSize, int offset, size_t target_end,
                       size_t *bytes_copied, size_t chunk_size) {
  struct xz_buf *b = &cache->xz_buf;
  *bytes_copied = 0;

  while (cache->offset < target_end) {
    if (b->in_pos == b->in_size) {
      b->in_size = fread(cache->in_buf, 1, chunk_size, cache->file);
      b->in_pos = 0;
      if (ferror(cache->file)) {
        ESP_LOGE(TAG, "Error reading XZ file");
        return false;
      }
      if (b->in_size == 0) {
        break;  // EOF
      }
    }

    enum xz_ret ret = xz_dec_run(cache->xz_dec, b);
    if (ret != XZ_OK && ret != XZ_STREAM_END) {
      ESP_LOGE(TAG, "XZ decompression error: %d", ret);
      return false;
    }

    size_t produced = b->out_pos;
    size_t copy_start = cache->offset < (size_t)offset ? offset : cache->offset;
    size_t copy_end = (cache->offset + produced) > target_end
                          ? target_end
                          : (cache->offset + produced);
    if (copy_end > copy_start) {
      size_t copy_offset = copy_start - cache->offset;
      size_t copy_len = copy_end - copy_start;
      memcpy(part + *bytes_copied, cache->out_buf + copy_offset, copy_len);
      *bytes_copied += copy_len;
    }

    cache->offset += produced;
    b->out_pos = 0;

    if (ret == XZ_STREAM_END) {
      break;
    }
  }

  return true;
}

#endif  // CONFIG_STORAGE_COMPRESSION_XZ
