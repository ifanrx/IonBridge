#ifndef TELEMETRY_TIMER_H_
#define TELEMETRY_TIMER_H_

#include <chrono>
#include <cstdint>
#include <functional>

using namespace std::chrono;

using TelemetryTimerFunc = std::function<void()>;

class TelemetryTimer {
  TelemetryTimerFunc func;
  milliseconds interval;
  steady_clock::time_point next_trigger;
  bool runnable = false;

 public:
  TelemetryTimer(TelemetryTimerFunc func, uint16_t interval_ms)
      : func(func), interval(interval_ms) {
    Reset();
  }
  TelemetryTimer() {
    func = nullptr;
    interval = milliseconds(0);
    Reset();
  }

  void Reset() {
    runnable = func != nullptr;
    if (runnable) {
      next_trigger = steady_clock::now() + interval;
    }
  }
  void Run(steady_clock::time_point now) {
    if (runnable && now >= next_trigger) {
      func();
      Reset();
    }
  }
};

#ifdef __cplusplus
extern "C" {
#endif

void StartTelemetryTimerTask();

#ifdef __cplusplus
}
#endif

#endif
