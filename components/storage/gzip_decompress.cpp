#include "decompress.h"

#ifdef CONFIG_STORAGE_COMPRESSION_GZIP

#include <cstring>

#include "esp_log.h"
#include "zlib.h"

static const char *TAG = "GZIP";

esp_err_t InitGzipCache(DecompressionCache *cache) {
  memset(&cache->gz_stream, 0, sizeof(cache->gz_stream));
  int ret = inflateInit2(&cache->gz_stream, 16 + MAX_WBITS);
  if (ret != Z_OK) {
    ESP_LOGE(TAG, "Failed to initialize zlib: %d", ret);
    return ESP_FAIL;
  }
  return ESP_OK;
}

void FreeGzipCache(DecompressionCache *cache) {
  if (cache->type == Storage::COMPRESSION_GZIP) {
    inflateEnd(&cache->gz_stream);
  }
}

bool GzipDecompressChunk(DecompressionCache *cache, uint8_t *part,
                         size_t partSize, int offset, size_t target_end,
                         size_t *bytes_copied, size_t chunk_size) {
  z_stream *strm = &cache->gz_stream;
  *bytes_copied = 0;

  while (cache->offset < target_end) {
    if (strm->avail_in == 0) {
      strm->avail_in = fread(cache->in_buf, 1, chunk_size, cache->file);
      if (ferror(cache->file)) {
        ESP_LOGE(TAG, "Error reading compressed file");
        return false;
      }
      strm->next_in = cache->in_buf;
      if (strm->avail_in == 0) {
        break;  // EOF
      }
    }

    strm->avail_out = chunk_size;
    strm->next_out = cache->out_buf;
    int ret = inflate(strm, Z_NO_FLUSH);
    if (ret != Z_OK && ret != Z_STREAM_END) {
      ESP_LOGE(TAG, "Decompression error: %d", ret);
      return false;
    }

    size_t produced = chunk_size - strm->avail_out;
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

    if (ret == Z_STREAM_END) {
      break;
    }
  }

  return true;
}

#endif  // CONFIG_STORAGE_COMPRESSION_GZIP
