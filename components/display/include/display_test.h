#ifndef H_DISPLAY_TEST_
#define H_DISPLAY_TEST_

#include <cstdint>
#include <optional>
#include <vector>

#ifdef __cplusplus
extern "C" {
#endif

enum DisplayTestKind : uint8_t {
  TEST_IGNORE = 0,
  TEST_EXIT,
  TEST_ENTER,
  TEST_LIGHTNESS,
  TEST_AUTO,
  TEST_SCREEN,
  TEST_AUTO_ONCE,
};

enum DisplayTestScreen : uint8_t {
  TEST_SCREEN_DEFAULT = 0,
  TEST_SCREEN_RED,
  TEST_SCREEN_GREEN,
  TEST_SCREEN_BLUE,
  TEST_SCREEN_WHITE,
  TEST_SCREEN_BLACK,
  TEST_SCREEN_GREY,
  TEST_SCREEN_OFF,
  TEST_SCREEN_LOW_LIGHT,
  TEST_SCREEN_HIGH_LIGHT,
  TEST_SCREEN_1,
  TEST_SCREEN_2,
  TEST_SCREEN_3,
  // other screens can be added here
};

typedef union {
  uint8_t raw;
  uint8_t lightness;         // lightness value for display
  DisplayTestScreen screen;  // screen to display
  uint8_t duration_secs;     // duration in seconds for the test;
} DisplayTestParam;

#ifdef __cplusplus
}  // extern "C"
#endif

struct DisplayTestEvent {
  DisplayTestKind kind;                   // kind of test event
  std::optional<DisplayTestParam> param;  // parameters for the test event

  explicit DisplayTestEvent(const std::vector<uint8_t>& data) {
    if (data.empty()) {
      kind = TEST_IGNORE;
      return;
    }

    kind = static_cast<DisplayTestKind>(data[0]);
    switch (kind) {
      case TEST_LIGHTNESS:
      case TEST_SCREEN:
        if (data.size() > 1) {
          param = DisplayTestParam{data[1]};
        } else {
          kind = TEST_IGNORE;
        }
        break;
      case TEST_AUTO:
      case TEST_AUTO_ONCE:
        if (data.size() > 1) {
          param = DisplayTestParam{data[1]};
        }
        break;
      default:
        param = std::nullopt;
        break;
    }
  }
};

#endif  // H_DISPLAY_TEST_
