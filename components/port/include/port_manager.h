#ifndef PORT_MANAGER_H
#define PORT_MANAGER_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <memory>

#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "port.h"
#include "sdkconfig.h"

#ifdef CONFIG_INTERFACE_TYPE_UART
#include "uart.h"
#endif

using PortPredicate = std::function<bool(const Port&)>;
using ForEachCallback = std::function<void(Port&)>;
using ForEachConstCallback = std::function<void(const Port&)>;

#ifdef CONFIG_MCU_MODEL_SW3566
constexpr uint8_t NUM_PORTS = CONFIG_SW3566_COUNT;
#elifdef CONFIG_MCU_MODEL_FAKE_SW3566
constexpr uint8_t NUM_PORTS = CONFIG_FAKE_SW3566_COUNT;
#endif

enum class PortManagerEventType {
#ifdef CONFIG_INTERFACE_TYPE_UART
  UART_MESSAGE = (1 << 0),
#endif
  TERMINATE_EVENT = 0xFF,
};

struct PortManagerEvent {
  PortManagerEventType event_type;
  uint8_t port_id;
  union {
#ifdef CONFIG_INTERFACE_TYPE_UART
    uart_message_t uart_msg;
#endif
  };
};

template <typename Predicate>
class FilteredRange {
 public:
  FilteredRange(std::array<std::unique_ptr<Port>, NUM_PORTS>& ports,
                Predicate pred)
      : ports_(ports), pred_(pred) {}

  class Iterator {
   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = Port&;
    using difference_type = std::ptrdiff_t;
    using pointer = Port*;
    using reference = Port&;

    Iterator(std::array<std::unique_ptr<Port>, NUM_PORTS>::iterator current,
             std::array<std::unique_ptr<Port>, NUM_PORTS>::iterator end,
             Predicate pred)
        : current_(current), end_(end), pred_(pred) {
      advanceToNextValid();
    }

    reference operator*() const { return *(current_->get()); }
    pointer operator->() const { return current_->get(); }

    Iterator& operator++() {
      ++current_;
      advanceToNextValid();
      return *this;
    }

    Iterator operator++(int) {
      Iterator tmp = *this;
      ++(*this);
      return tmp;
    }

    bool operator==(const Iterator& other) const {
      return current_ == other.current_;
    }
    bool operator!=(const Iterator& other) const { return !(*this == other); }

   private:
    void advanceToNextValid() {
      while (current_ != end_ && !pred_(*current_->get())) {
        ++current_;
      }
    }

    std::array<std::unique_ptr<Port>, NUM_PORTS>::iterator current_;
    std::array<std::unique_ptr<Port>, NUM_PORTS>::iterator end_;
    Predicate pred_;
  };

  Iterator begin() { return Iterator(ports_.begin(), ports_.end(), pred_); }
  Iterator end() { return Iterator(ports_.end(), ports_.end(), pred_); }

 private:
  std::array<std::unique_ptr<Port>, NUM_PORTS>& ports_;
  Predicate pred_;
};

class PortManager {
 public:
  static PortManager& GetInstance() {
    static PortManager instance;
    return instance;
  }

  // Delete copy constructor and assignment operator to enforce singleton
  // pattern
  PortManager(const PortManager&) = delete;
  PortManager& operator=(const PortManager&) = delete;

  size_t Size() const { return ports_.size(); }

  void StartTask();

#ifdef CONFIG_INTERFACE_TYPE_UART
  void EnqueueUARTMessage(const uart_message_t& uart_msg, uint8_t port_id);
#endif

  bool InitializePort(uint8_t port_id, bool active,
                      uint8_t initial_power_budget);
  Port* GetPort(uint8_t port_id);

  void ForEach(ForEachCallback cb);
  void ForEach(ForEachConstCallback cb) const;

  // All ports data functions
  uint16_t GetPortsPowerUsage() const;
  uint8_t GetPortsBitMask(PortPredicate predicate) const;
  uint8_t GetOpenPortsBitMask() const;
  uint8_t GetAttachedPortsBitMask() const;
  uint8_t GetAttachedPortCount() const;
  uint8_t GetActivePortCount() const;
  uint8_t CountPorts(PortPredicate predicate) const;

  uint32_t GetChargingDurationSeconds() const;
  uint32_t GetChargingAt() const {
    if (charging_at_ == -1) {
      return 0;
    }
    return charging_at_;
  }

  std::array<uint8_t, NUM_PORTS> GetAllPortsPowerUsage();

  // Iterator classes
  class Iterator {
   public:
    // Iterator traits
    using iterator_category = std::forward_iterator_tag;
    using value_type = Port&;
    using difference_type = std::ptrdiff_t;
    using pointer = Port*;
    using reference = Port&;

    // Constructor
    Iterator(std::array<std::unique_ptr<Port>, NUM_PORTS>::iterator it)
        : it_(it) {}

    // Dereference operator
    Port& operator*() const { return *(it_->get()); }

    // Arrow operator
    Port* operator->() const { return it_->get(); }

    // Pre-increment
    Iterator& operator++() {
      ++it_;
      return *this;
    }

    // Post-increment
    Iterator operator++(int) {
      Iterator tmp = *this;
      ++it_;
      return tmp;
    }

    // Equality comparison
    bool operator==(const Iterator& other) const { return it_ == other.it_; }

    bool operator!=(const Iterator& other) const { return it_ != other.it_; }

   private:
    std::array<std::unique_ptr<Port>, NUM_PORTS>::iterator it_;
  };

  class ConstIterator {
   public:
    // Iterator traits
    using iterator_category = std::forward_iterator_tag;
    using value_type = const Port&;
    using difference_type = std::ptrdiff_t;
    using pointer = const Port*;
    using reference = const Port&;

    // Constructor
    ConstIterator(
        std::array<std::unique_ptr<Port>, NUM_PORTS>::const_iterator it)
        : it_(it) {}

    // Dereference operator
    const Port& operator*() const { return *(it_->get()); }

    // Arrow operator
    const Port* operator->() const { return it_->get(); }

    // Pre-increment
    ConstIterator& operator++() {
      ++it_;
      return *this;
    }

    // Post-increment
    ConstIterator operator++(int) {
      ConstIterator tmp = *this;
      ++it_;
      return tmp;
    }

    // Equality comparison
    bool operator==(const ConstIterator& other) const {
      return it_ == other.it_;
    }

    bool operator!=(const ConstIterator& other) const {
      return it_ != other.it_;
    }

   private:
    std::array<std::unique_ptr<Port>, NUM_PORTS>::const_iterator it_;
  };

  // Iterator functions
  Iterator begin() { return Iterator(ports_.begin()); }
  Iterator end() { return Iterator(ports_.end()); }
  ConstIterator begin() const { return ConstIterator(ports_.cbegin()); }
  ConstIterator end() const { return ConstIterator(ports_.cend()); }
  ConstIterator cbegin() const { return ConstIterator(ports_.cbegin()); }
  ConstIterator cend() const { return ConstIterator(ports_.cend()); }

  // Method that accepts a predicate and returns a filtered iterator range
  template <typename Predicate>
  auto FilteredPorts(Predicate pred) {
    return FilteredRange<Predicate>(ports_, pred);
  }
  auto GetAlivePorts() {
    return FilteredPorts([](const Port& port) {
      return !(port.Abnormal() || port.IsPowerLimiting());
    });
  }

 private:
  PortManager();
  ~PortManager();

  // Thread safety
  mutable SemaphoreHandle_t mutex_;

  // Task handle
  static void TaskLoopWrapper(void* pvParameters);
  void TaskLoop();
  bool taskRunning_ = false;
  TaskHandle_t taskHandle_;
  QueueHandle_t eventQueue_;

  // Ports managed by PortManager
  std::array<std::unique_ptr<Port>, NUM_PORTS> ports_;

  uint32_t charging_at_ = -1;
  uint8_t attaced_ports_ = 0;
  uint16_t pd_rx_hard_reset_count_ = 0;
  uint16_t pd_rx_error_count_ = 0;
  uint16_t pd_rx_cable_reset_count_ = 0;

  // Helper function
  void CheckAttachedPort(const Port& port);

  // Handler functions
#ifdef CONFIG_INTERFACE_TYPE_UART
  void HandleUARTMessage(const PortManagerEvent& event);
  void HandleBroadcastMessage(const uart_message_t& uart_msg);
  void HandlePortDetailsMessage(Port& port, const uart_message_t& uart_msg);
  void HandlePDStatusMessage(Port& port, const uart_message_t& uart_msg);
  void HandleInvalidCommandError(Port& port, const uart_message_t& uart_msg);
  void HandleUncorrectableError(Port& port, const uart_message_t& uart_msg);
  void HandleChargingAlert(Port& port, const uart_message_t& uart_msg);
  void HandleKeepAliveMessage(Port& port, const uart_message_t& uart_msg);
  void HandlePDPCapDataMessage(Port& port, const uart_message_t& uart_msg);
#endif
};

#endif
