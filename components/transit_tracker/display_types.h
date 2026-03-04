#pragma once

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <vector>

#include "schedule_state.h"

namespace esphome {
namespace transit_tracker {

// =====================================================================
// Pin modes
// =====================================================================

enum PinMode : uint8_t {
  PIN_NONE = 0,       // Not pinned (Off or On)
  PIN_GENERAL = 1,    // Always in pinned section, red pin
  PIN_LEAVING_SOON = 2, // Unpinned until departure < threshold, then green pin
  PIN_BOTH = 3,       // Always pinned; green pin when departure < threshold, else red
};

// =====================================================================
// Easing
// =====================================================================

/// Hermite smoothstep: 0→0, 0.5→0.5, 1→1 with zero-derivative endpoints.
inline float smoothstep(float t) {
  t = std::max(0.0f, std::min(1.0f, t));
  return t * t * (3.0f - 2.0f * t);
}

// =====================================================================
// Constants
// =====================================================================

/// Duration of a single row collapse/expand in the replace animation (ms).
static constexpr int kReplacePerRowMs = 500;
/// Stagger delay between consecutive row animations (ms).
static constexpr int kReplaceStaggerMs = 250;
/// Pause between collapse and expand phases (ms).
static constexpr int kReplaceMidPauseMs = 500;
/// Width reserved for pin icon (1px margin + 5px icon + 1px margin).
static constexpr int kPinIconWidth = 7;
/// Minimum row scale before a collapsing row is considered invisible.
static constexpr float kMinRowScale = 0.05f;
/// Pixel threshold: h-scroll "near zero" (used for cycle boundary detection).
static constexpr int kScrollNearZeroPx = 3;
/// Pixel gap between marquee repetitions.
static constexpr int kMarqueeGap = 20;
/// Dwell time at home position between scroll cycles (ms).
static constexpr int kScrollCycleDwellMs = 3000;

// =====================================================================
// Dedup helper
// =====================================================================

/// Deduplicate the pinned section of a trip vector in-place.
/// Keeps only the first (soonest) trip for each distinct composite_key.
inline void dedup_pinned_section(std::vector<Trip> &trips, int &pinned_count) {
  std::set<std::string> seen;
  int write = 0;
  for (int i = 0; i < pinned_count; i++) {
    std::string key = trips[i].composite_key();
    if (seen.find(key) == seen.end()) {
      seen.insert(key);
      if (write != i) trips[write] = trips[i];
      write++;
    }
  }
  if (write < pinned_count) {
    trips.erase(trips.begin() + write, trips.begin() + pinned_count);
    pinned_count = write;
  }
}

// =====================================================================
// Horizontal scroll state
// =====================================================================

struct HScrollState {
  unsigned long start_time{0};
  int scroll_speed{10};
  int pending_speed{0};
  int total_distance{0};
  int prev_total_distance{0};

  // Deferred start: when overflow newly appears while idle, dwell at the
  // home position for one cycle duration before scrolling begins.
  // This is symmetric with the wind-down logic (overflow disappearing
  // finishes the current cycle before stopping).
  bool deferred_start{false};
  unsigned long deferred_since{0};
  // Skips the next deferred-start trigger.  Set by reset() so that the
  // first update_distance() after a transition does not re-trigger deferral
  // (the transition already provided a visual break).  Defaults to false
  // so that the very first overflow on boot gets a 3s dwell before scrolling.
  bool skip_next_defer{false};

  // After a transition reset, the first scroll cycle skips the inter-cycle
  // dwell — the transition's own animation already provided a visual pause.
  bool skip_first_dwell{false};
  int base_cycle_count{0};

  // Updated each frame by compute()
  int offset{0};
  int cycle_count{0};
  bool idle{true};

  void reset(unsigned long now) {
    start_time = now;
    offset = 0;
    cycle_count = 0;
    idle = true;
    total_distance = 0;
    prev_total_distance = 0;
    deferred_start = false;
    deferred_since = 0;
    skip_next_defer = true;
    skip_first_dwell = false;
    base_cycle_count = 0;
  }

  void update_distance(int max_headsign_width, bool any_overflow) {
    if (any_overflow && max_headsign_width > 0) {
      int new_dist = max_headsign_width + kMarqueeGap;
      bool was_fully_idle = (total_distance <= 0 && prev_total_distance <= 0 && !deferred_start);
      total_distance = new_dist;
      if (was_fully_idle && !skip_next_defer) {
        deferred_start = true;
      }
      skip_next_defer = false;
    } else {
      skip_next_defer = false;
      if (deferred_start) {
        // Overflow disappeared during deferred dwell — cancel without wind-down
        deferred_start = false;
        deferred_since = 0;
        total_distance = 0;
      } else if (total_distance > 0) {
        prev_total_distance = total_distance;
        total_distance = 0;
      }
    }
  }

  void compute(unsigned long now) {
    if (start_time == 0) { start_time = now; idle = true; offset = 0; cycle_count = 0; return; }
    int dist = total_distance > 0 ? total_distance : prev_total_distance;
    if (dist <= 0) { idle = true; offset = 0; cycle_count = 0; return; }

    // Deferred start: dwell at home position for kScrollCycleDwellMs
    // before scrolling begins.  Uses the same fixed 3s dwell as the
    // inter-cycle pause, regardless of scroll speed or distance.
    if (deferred_start) {
      // Apply pending speed during deferred dwell so the dwell duration
      // uses the correct speed (not the default 10px/s).
      if (pending_speed != 0) {
        scroll_speed = pending_speed;
        pending_speed = 0;
      }
      if (deferred_since == 0) deferred_since = now;
      if (now - deferred_since >= kScrollCycleDwellMs) {
        // Dwell complete — begin actual scrolling
        deferred_start = false;
        deferred_since = 0;
        start_time = now;
      }
      offset = 0;
      cycle_count = 0;
      idle = true;
      return;
    }

    // Apply pending speed immediately when at home position (offset == 0).
    // This ensures the correct speed is used from the very first scroll
    // cycle after a reset — e.g. on reboot, the configured speed arrives
    // via MQTT after the component starts, so pending_speed must be applied
    // before the first scroll begins rather than waiting for a dwell.
    if (pending_speed != 0 && offset == 0) {
      scroll_speed = pending_speed;
      pending_speed = 0;
    }

    // Each cycle = scroll phase + dwell phase (paused at home position).
    unsigned long scroll_phase_ms = (unsigned long)dist * 1000 / scroll_speed;

    // --- Standard cycles with dwell ---
    unsigned long cycle_total_ms = scroll_phase_ms + kScrollCycleDwellMs;

    unsigned long elapsed = now - start_time;
    int full_cycles = (int)(elapsed / cycle_total_ms);
    unsigned long cycle_elapsed = elapsed % cycle_total_ms;
    bool in_dwell = (cycle_elapsed >= scroll_phase_ms);

    if (!in_dwell) {
      // Scrolling phase
      offset = (int)(cycle_elapsed * scroll_speed / 1000);
      idle = (offset < kScrollNearZeroPx);
    } else {
      // Dwell phase — paused at home position between cycles
      offset = 0;
      idle = true;
    }

    // cycle_count = completed scroll phases (increments when entering dwell)
    cycle_count = base_cycle_count + (in_dwell ? full_cycles + 1 : full_cycles);

    // Wind-down: if real overflow is gone, finish the current cycle then stop
    if (total_distance <= 0 && prev_total_distance > 0 && in_dwell) {
      prev_total_distance = 0;
      offset = 0;
    }

    // Apply pending speed change at cycle boundary (during dwell)
    if (pending_speed != 0 && in_dwell) {
      scroll_speed = pending_speed;
      pending_speed = 0;
      start_time = now;
      base_cycle_count = cycle_count;
      offset = 0;
      cycle_count = base_cycle_count;
    }
  }
};

// =====================================================================
// Scroll container (paging state for one section)
// =====================================================================

struct ScrollContainer {
  int page_offset{0};
  unsigned long page_timer{0};
  int last_cycle_count{0};

  int page_count(int pool_size, int slot_count) const {
    if (pool_size <= slot_count || slot_count <= 0) return 1;
    return pool_size - slot_count + 1; // rotate style
  }

  bool needs_paging(int pool_size, int slot_count) const {
    return pool_size > slot_count && slot_count > 0;
  }

  void advance_page(int pool_size, int slot_count) {
    int pages = page_count(pool_size, slot_count);
    page_offset = (page_offset + 1) % pages;
  }

  void clamp(int pool_size, int slot_count) {
    int pages = page_count(pool_size, slot_count);
    if (page_offset >= pages) page_offset = std::max(0, pages - 1);
  }

  int start_index(int pool_size, int slot_count) const {
    if (pool_size <= slot_count || slot_count <= 0) return 0;
    return std::min(page_offset, pool_size - slot_count);
  }

  int end_index(int pool_size, int slot_count) const {
    return std::min(start_index(pool_size, slot_count) + slot_count, pool_size);
  }

  void reset() { page_offset = 0; page_timer = 0; last_cycle_count = 0; }
};

// =====================================================================
// Display diff — detects changes between consecutive frames
// =====================================================================

struct DisplayDiff {
  // Previous frame state
  std::vector<std::string> prev_keys;
  std::vector<time_t> prev_departures;
  std::vector<Trip> prev_trips;
  std::vector<bool> prev_is_pinned;
  std::vector<bool> prev_is_leaving_soon;
  int prev_pinned_count{-1};
  std::set<std::string> prev_pinned_routes;

  // Diff results (computed each frame when idle)
  bool layout_changed{false};
  bool pinned_set_changed{false};
  bool data_changed{false};

  bool has_changes() const {
    return layout_changed || pinned_set_changed || data_changed;
  }

  void compute(const std::vector<std::string> &cur_keys,
               const std::vector<time_t> &cur_deps,
               int cur_pinned_count,
               const std::set<std::string> &cur_pinned_routes) {
    layout_changed = false;
    pinned_set_changed = false;
    data_changed = false;

    if (prev_pinned_count < 0) return;  // first frame — no diff

    if (cur_pinned_count != prev_pinned_count) { layout_changed = true; return; }
    if (cur_pinned_routes != prev_pinned_routes) { pinned_set_changed = true; return; }

    // Key or departure-time changes
    if (cur_keys.size() != prev_keys.size()) { data_changed = true; return; }
    for (size_t i = 0; i < cur_keys.size(); i++) {
      if (cur_keys[i] != prev_keys[i]) { data_changed = true; return; }
      if (i < cur_deps.size() && i < prev_departures.size()) {
        if (std::abs((long)(cur_deps[i] - prev_departures[i])) > 60) {
          data_changed = true; return;
        }
      }
    }
  }

  void commit(const std::vector<std::string> &keys,
              const std::vector<time_t> &deps,
              const std::vector<Trip> &trips,
              const std::vector<bool> &is_pinned,
              const std::vector<bool> &is_leaving_soon,
              int pinned_count,
              const std::set<std::string> &pinned_routes) {
    prev_keys = keys;
    prev_departures = deps;
    prev_trips = trips;
    prev_is_pinned = is_pinned;
    prev_is_leaving_soon = is_leaving_soon;
    prev_pinned_count = pinned_count;
    prev_pinned_routes = pinned_routes;
  }
};

// =====================================================================
// Transition — the single state machine for all animations
// =====================================================================

struct Transition {
  enum Phase {
    IDLE = 0,
    WAIT_SCROLL,   // waiting for h-scroll to reach ~0
    COLLAPSE,      // rows shrinking (replace) or section sliding (v-scroll)
    MID_PAUSE,     // blank pause between collapse and expand
    EXPAND,        // rows growing
    POST_PAUSE,    // dwell after expand before h-scroll resumes
  };

  Phase phase{IDLE};
  unsigned long phase_start{0};

  // Which sections animate (both = full layout change)
  bool animate_pinned{false};
  bool animate_unpinned{false};

  // Snapshot of display content at transition start (for WAIT_SCROLL + COLLAPSE)
  std::vector<Trip> old_trips;
  std::vector<bool> old_is_pinned;
  std::vector<bool> old_is_leaving_soon;
  int old_eff_pinned{0};
  bool old_respect_pin_inset{true};

  // For vertical scroll page changes
  bool is_vscroll{false};
  int vscroll_section_y{0};      // y of first row in scrolling section
  int vscroll_section_rows{0};   // how many rows scroll
  int vscroll_old_start{0};      // old page start index into pool
  int vscroll_new_start{0};      // new page start index into pool
  bool vscroll_is_pinned{false}; // which pool

  // Total rows in the stagger animation
  int stagger_rows{0};

  // ---- Timing helpers ----

  int cascade_ms() const { return kReplaceStaggerMs * std::max(0, stagger_rows - 1); }
  int collapse_duration_ms() const { return kReplacePerRowMs + cascade_ms(); }
  int expand_duration_ms() const { return kReplacePerRowMs + cascade_ms(); }
  int total_replace_ms() const {
    return collapse_duration_ms() + kReplaceMidPauseMs + expand_duration_ms();
  }

  /// Compute the scale (0–1) for a row during collapse or expand.
  /// @param anim_row  The row's index within the animated rows (0-based).
  /// @param elapsed   Time since the current phase started (ms).
  /// @param expanding True for expand phase, false for collapse phase.
  float row_scale(int anim_row, unsigned long elapsed, bool expanding) const {
    int row_start = anim_row * kReplaceStaggerMs;
    float el = std::max(0.0f, (float)((int)elapsed - row_start));
    float t = std::min(1.0f, el / (float)kReplacePerRowMs);
    t = smoothstep(t);
    return expanding ? t : (1.0f - t);
  }

  void reset() {
    phase = IDLE;
    phase_start = 0;
    animate_pinned = animate_unpinned = false;
    old_trips.clear();
    old_is_pinned.clear();
    old_is_leaving_soon.clear();
    old_eff_pinned = 0;
    old_respect_pin_inset = true;
    is_vscroll = false;
    stagger_rows = 0;
  }
};

}  // namespace transit_tracker
}  // namespace esphome
