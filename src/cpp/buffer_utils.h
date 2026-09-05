#pragma once

#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace hesim3d {

struct FrameBuffer {
  uint32_t width{0};
  uint32_t height{0};
  uint32_t channels{3};  // 3 = RGB, 4 = RGBA
  uint64_t timestamp_us{0};
  std::vector<uint8_t> data;
};

class RingBuffer {
 public:
  RingBuffer(size_t capacity = 16) : capacity_(capacity), head_(0), tail_(0), count_(0) {
    buffers_.resize(capacity_);
  }

  void allocate_slots(uint32_t width, uint32_t height, uint32_t channels) {
    std::lock_guard<std::mutex> lock(mutex_);
    width_ = width;
    height_ = height;
    channels_ = channels;
    size_t frame_size = width * height * channels;
    for (auto& buf : buffers_) {
      buf.width = width;
      buf.height = height;
      buf.channels = channels;
      buf.data.resize(frame_size, 0);
    }
    head_ = 0;
    tail_ = 0;
    count_ = 0;
  }

  void push(const uint8_t* src_data, size_t size, uint64_t timestamp_us) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (buffers_.empty()) return;

    auto& slot = buffers_[head_];
    if (slot.data.size() == size) {
      std::memcpy(slot.data.data(), src_data, size);
      slot.timestamp_us = timestamp_us;
      head_ = (head_ + 1) % capacity_;
      if (count_ < capacity_) {
        count_++;
      } else {
        tail_ = (tail_ + 1) % capacity_;  // Overwrite oldest
      }
    }
  }

  bool pop(FrameBuffer& out_frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (count_ == 0) return false;

    out_frame = buffers_[tail_];
    tail_ = (tail_ + 1) % capacity_;
    count_--;
    return true;
  }

  bool peek_latest(FrameBuffer& out_frame) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (count_ == 0) return false;

    size_t latest_idx = (head_ + capacity_ - 1) % capacity_;
    out_frame = buffers_[latest_idx];
    return true;
  }

  size_t count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return count_;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    head_ = 0;
    tail_ = 0;
    count_ = 0;
  }

 private:
  size_t capacity_;
  size_t head_;
  size_t tail_;
  size_t count_;
  uint32_t width_{0};
  uint32_t height_{0};
  uint32_t channels_{3};
  std::vector<FrameBuffer> buffers_;
  mutable std::mutex mutex_;
};

}  // namespace hesim3d
