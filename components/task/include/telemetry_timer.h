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
  bool runnable = false, power_on_only = false;

 public:
  TelemetryTimer(TelemetryTimerFunc func, uint16_t interval_ms,
                 bool power_on_only = false)
      : func(func), interval(interval_ms), power_on_only(power_on_only) {
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
    if (ShouldRun(now)) {
      func();
      Reset();
    }
  }

 private:
  bool ShouldRun(steady_clock::time_point now) const;
};

#ifdef __cplusplus
extern "C" {
#endif

void StartTelemetryTimerTask();

#ifdef __cplusplus
}
#endif

#endif
