#ifndef H_MESSAGE_
#define H_MESSAGE_

#include <vector>

#include "app.h"
#include "protocol.h"

// 4 handlers * 1024 bytes = 4KB
#define MAX_CONCURRENT_CONN 4
#define MAX_MESSAGE_SIZE 1024

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t id;  // This represents the message id
               // for which this handler is being used
  bool free;   // Indicates if this slot is free or not
  uint8_t *data;
  uint32_t size;
  uint8_t expected_sequence;  // Tracks the expected sequence number for the
                              // next fragment
} MessageFragHandler;

class MessageHandler : public IMessageHandler {
 private:
  // Private member variables here
  MessageFragHandler *handlers_;
  App *app_;
  uint16_t peerMTU_;

  std::vector<uint8_t> requestBuffer_;

  void sendResponse(Message &msg, const uint8_t *data, size_t dataLength,
                    uint8_t tcpFlag) const;

  // Returns a pointer to the handler for the given message ID or NULL if none
  // exist
  MessageFragHandler *getHandlerById(uint8_t id);
  // Returns a pointer to an available handler or NULL if none exist
  MessageFragHandler *getFreeHandler();

 public:
  // Constructor
  explicit MessageHandler(App *app);
  ~MessageHandler();

  void handle_syn(Message &msg) override;

  void handle_syn_ack(Message &msg) override;

  void handle_fin(Message &msg) override;

  void handle_ack(Message &msg) override;

  void setPeerMTU(uint16_t peerMTU);
};

#ifdef __cplusplus
}
#endif

#endif  // H_MESSAGE_
