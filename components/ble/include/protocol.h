#ifndef H_PROTOCOL_
#define H_PROTOCOL_

#include <cstdint>
#include <cstring>  // for memcpy
#include <vector>

#include "esp_err.h"

// Flag Definitions
#define BLE_NONE 0x0
#define BLE_SYN 0x1
#define BLE_ACK 0x2
#define BLE_FIN 0x3
#define BLE_RST 0x4
#define BLE_SYN_ACK 0x5

#define FRAG_NONE 0x0
#define FRAG_FIRST 0x1
#define FRAG_MORE 0x2
#define FRAG_LAST 0x3
#define IS_FIRST_FRAG(flags) \
  ((flags & 0x3) == FRAG_FIRST || (flags & 0x3) == FRAG_NONE)
#define IS_LAST_FRAG(flags) \
  ((flags & 0x3) == FRAG_LAST || (flags & 0x3) == FRAG_NONE)

// ATU_MAX: 512, ATU_HEADER: 3, BLEHeader: 9, so the data length: 500
#define BUFSIZE 500

#ifdef __cplusplus
extern "C" {
#endif  // DEBUG

typedef struct {
  uint8_t bytes[3];
} ble_size_t;  // Convert into uint32_t later

// Convert ble_size_t to uint32_t
uint32_t get_ble_size_t(ble_size_t b, uint8_t version);
// Convert uint32_t to ble_size_t
ble_size_t set_ble_size_t(uint32_t value, uint8_t version);

typedef struct {
  uint8_t version;   // Version: 0x0 for now
  uint8_t id;        // Each message has a unique ID, increases by 1
  int8_t service;    // Service ID -- positive is request, negative is response
  uint8_t sequence;  // Sequence number for fragmentation, increases by 1 for
                     // next fragment
  uint8_t flags;  // Flags for controlling the transmissions (fragmentation and
                  // TCP flags)
  ble_size_t size;   // Total size of message payload
  uint8_t checksum;  // Checksum for integrity check
} BLEHeader;

#ifdef __cplusplus
}
#endif

class Message {
  uint8_t *payload_;
  BLEHeader header_;

 public:
  Message(uint8_t version, uint8_t id, int8_t service, uint8_t sequence,
          uint8_t flags, const uint8_t *data, size_t dataLength);
  Message(BLEHeader header, const uint8_t *payload, size_t payloadSize);
  ~Message();

  // Copying and assignment are non-trivial due to dynamic allocation.
  // For safety, either implement them correctly or delete them to prevent
  // default shallow copying.
  Message(const Message &) = delete;  // Delete the copy constructor
  Message &operator=(const Message &) =
      delete;  // Delete the copy assignment operator

  // Move constructor
  Message(Message &&other) noexcept;
  // Move Assignment Operator
  Message &operator=(Message &&other) noexcept;

  std::vector<uint8_t> serialize() const;

  bool validate();
  uint8_t getVersion() const;
  uint8_t getID() const;
  int8_t getService() const;
  uint8_t getSequence() const;
  uint8_t getFlags() const;
  uint32_t getSize() const;
  uint8_t getChecksum() const;
  uint8_t *getPayload();
};

class IMessageHandler {
 public:
  IMessageHandler() = default;
  virtual ~IMessageHandler() = default;

  virtual void handle_syn(Message &msg) = 0;
  virtual void handle_syn_ack(Message &msg) = 0;
  virtual void handle_ack(Message &msg) = 0;
  virtual void handle_fin(Message &msg) = 0;
};

class MessageFactory {
 public:
  static esp_err_t createMessage(const uint8_t *data, size_t dataLength,
                                 Message **msg);
};

void process_message(Message &msg, IMessageHandler &handler);
bool construct_message(const uint8_t *bytes, const size_t length, Message &msg);

#endif  // H_PROTOCOL_
