#ifndef PORT_STATE_H
#define PORT_STATE_H

#include <string>

#include "esp_timer.h"
#include "singleton.h"

enum PortStateType {
  UNKNOWN = -1,
  ACTIVE,
  INACTIVE,
  ATTACHED,
  OPENING,
  CLOSING,
  OVER_TEMP_WARNING,
  OVER_TEMP_ALERT,
  COOLING,
  CHECKING,
  RECOVERING,
  DEAD,
  POWER_LIMITING,
};

class Port;  // forward declaration

class PortState {
 protected:
  PortStateType type_;

 public:
  virtual ~PortState() = default;
  virtual void Handle(Port &port) = 0;
  virtual void Entering(Port &port) {}
  virtual void Exiting(Port &port) {}

  explicit PortState(PortStateType type) : type_(type) {}
  PortStateType Type() const { return type_; }
  std::string ToString() const {
    switch (type_) {
      case UNKNOWN:
        return "UNKNOWN";
      case ACTIVE:
        return "ACTIVE";
      case INACTIVE:
        return "INACTIVE";
      case ATTACHED:
        return "ATTACHED";
      case OPENING:
        return "OPENING";
      case CLOSING:
        return "CLOSING";
      case OVER_TEMP_WARNING:
        return "OVER_TEMP_WARNING";
      case OVER_TEMP_ALERT:
        return "OVER_TEMP_ALERT";
      case COOLING:
        return "COOLING";
      case CHECKING:
        return "CHECKING";
      case RECOVERING:
        return "RECOVERING";
      case DEAD:
        return "DEAD";
      case POWER_LIMITING:
        return "LIMITED_POWER";
      default:
        return "";
    }
  }
};

class PortActiveState : public PortState, public Singleton<PortActiveState> {
 public:
  void Handle(Port &port) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<PortActiveState>;
  // Private constructor to prevent external instantiation
  PortActiveState() : PortState(PortStateType::ACTIVE) {}
};

class PortInactiveState : public PortState,
                          public Singleton<PortInactiveState> {
 public:
  void Handle(Port &port) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<PortInactiveState>;
  // Private constructor to prevent external instantiation
  PortInactiveState() : PortState(PortStateType::INACTIVE) {}
};

class PortAttachedState : public PortState,
                          public Singleton<PortAttachedState> {
 public:
  void Handle(Port &port) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<PortAttachedState>;
  // Private constructor to prevent external instantiation
  PortAttachedState() : PortState(PortStateType::ATTACHED) {}
};

class PortOpeningState : public PortState, public Singleton<PortOpeningState> {
 public:
  void Handle(Port &port) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<PortOpeningState>;
  // Private constructor to prevent external instantiation
  PortOpeningState() : PortState(PortStateType::OPENING) {}
};

class PortClosingState : public PortState, public Singleton<PortClosingState> {
 public:
  void Handle(Port &port) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<PortClosingState>;
  // Private constructor to prevent external instantiation
  PortClosingState() : PortState(PortStateType::CLOSING) {}
};

class PortOverTempWarningState : public PortState,
                                 public Singleton<PortOverTempWarningState> {
 public:
  void Handle(Port &port) override;

 private:
  uint8_t exit_offset_ = 5;
  uint8_t exit_temperature_ = CONFIG_PORT_TEMP_WARNING_THRESHOLD - exit_offset_;
  uint8_t alert_temperature_ = CONFIG_PORT_TEMP_ALERT_THRESHOLD;
  uint8_t max_temperature_ = CONFIG_PORT_TEMP_MAX_THRESHOLD;

  // Allow Singleton to access the constructor
  friend class Singleton<PortOverTempWarningState>;
  // Private constructor to prevent external instantiation
  PortOverTempWarningState() : PortState(PortStateType::OVER_TEMP_WARNING) {}
};

class PortOverTempAlertState : public PortState,
                               public Singleton<PortOverTempAlertState> {
 public:
  void Handle(Port &port) override;

 private:
  uint8_t exit_offset_ = 5;
  uint8_t exit_temperature_ = CONFIG_PORT_TEMP_ALERT_THRESHOLD - exit_offset_;
  uint8_t max_temperature_ = CONFIG_PORT_TEMP_MAX_THRESHOLD;

  // Allow Singleton to access the constructor
  friend class Singleton<PortOverTempAlertState>;
  // Private constructor to prevent external instantiation
  PortOverTempAlertState() : PortState(PortStateType::OVER_TEMP_ALERT) {}
};

class PortCoolingState : public PortState, public Singleton<PortCoolingState> {
 public:
  void Handle(Port &port) override;

 private:
  uint8_t exit_offset_ = 5;
  uint8_t exit_temperature_ = CONFIG_PORT_TEMP_WARNING_THRESHOLD - exit_offset_;
  bool closed_ = false;

  // Allow Singleton to access the constructor
  friend class Singleton<PortCoolingState>;
  // Private constructor to prevent external instantiation
  PortCoolingState() : PortState(PortStateType::COOLING) {}
};

class PortCheckingState : public PortState,
                          public Singleton<PortCheckingState> {
 public:
  void Handle(Port &port) override;
  void Entering(Port &port) override;
  void Exiting(Port &port) override;

 private:
  int64_t enter_time_ = -1;

  // Allow Singleton to access the constructor
  friend class Singleton<PortCheckingState>;
  // Private constructor to prevent external instantiation
  PortCheckingState() : PortState(PortStateType::CHECKING) {}

  esp_timer_handle_t checking_timer_ = nullptr;
  void StartWatchdog(Port &port);
  void StopWatchdog();
  static void WatchdogCallback(void *arg);
};

class PortRecoveringState : public PortState,
                            public Singleton<PortRecoveringState> {
 public:
  void Handle(Port &port) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<PortRecoveringState>;
  // Private constructor to prevent external instantiation
  PortRecoveringState() : PortState(PortStateType::RECOVERING) {}
};

class PortDeadState : public PortState, public Singleton<PortDeadState> {
 public:
  void Handle(Port &port) override {}

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<PortDeadState>;
  // Private constructor to prevent external instantiation
  PortDeadState() : PortState(PortStateType::DEAD) {}
};

class PortPowerLimitingState : public PortState,
                               public Singleton<PortPowerLimitingState> {
 public:
  void Handle(Port &port) override {}

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<PortPowerLimitingState>;
  // Private constructor to prevent external instantiation
  PortPowerLimitingState() : PortState(PortStateType::POWER_LIMITING) {}
};

#endif  // !PORT_STATE_H
