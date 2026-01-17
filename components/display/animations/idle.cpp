#include "display_animation.h"
#include "display_renderer.h"
#include "display_types.h"

// ============================================================================
// Idle Animation
// ============================================================================

void IdleAnimation::Entering() {
  auto* r = GetRenderer();
  r->ShowIdleState(data_source_, 0);
}

AnimationAction IdleAnimation::PlayNextFrame(int64_t dt_us) {
  auto* r = GetRenderer();
  return AnimationAction::Continue(r->ShowIdleState(data_source_, dt_us));
}

void IdleAnimation::Exiting() {
  auto* r = GetRenderer();
  r->Clear();
}

// ============================================================================
// Screen Off Animation
// ============================================================================

void ScreenOffAnimation::Entering() {
  auto* r = GetRenderer();
  r->GetDriver()->SetMode(DisplayMode::OFF);
}

void ScreenOffAnimation::Exiting() {
  // Don't set any mode here - let the next animation set its own mode
}
