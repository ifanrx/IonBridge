#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#include <cstddef>
#include <vector>

template <typename T>
class RingBuffer {
  std::vector<T> buffer{};
  size_t head = 0;
  size_t tail = 0;
  size_t count = 0;
  size_t capacity;

 public:
  RingBuffer(size_t capacity) : capacity(capacity) { buffer.reserve(capacity); }

  void Push(const T &item) {
    buffer[head] = item;
    head = (head + 1) % capacity;
    if (count < capacity) {
      count++;
    } else {
      tail = (tail + 1) % capacity;
    }
  }

  size_t Size() const { return count; }

  std::vector<T> GetLastNPoints(size_t n) const {
    std::vector<T> result;
    if (n > count) {
      n = count;  // Adjust N if it is greater than the number of elements in
                  // the buffer
    }
    result.reserve(n);
    for (size_t i = 0; i < n; i++) {
      size_t index = (head + capacity - n + i) % capacity;
      result.push_back(buffer[index]);
    }
    return result;
  }

  std::vector<T> GetPointsInRange(size_t &start, size_t end) const {
    std::vector<T> result;
    if (end == 0 || end > count) {
      end = count;
    }
    if (start >= count || start > end) {
      return result;
    }
    size_t n = end - start;
    result.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      size_t index = (tail + start + i) % capacity;
      result.push_back(buffer[index]);
    }
    if (end == count) {
      // back to 0 indicates no more data
      start = 0;
    } else {
      start = end;
    }
    return result;
  }
};

#endif
