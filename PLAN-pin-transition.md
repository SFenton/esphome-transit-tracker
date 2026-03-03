# Smooth Pin Transition — Steps 1 & 2

## Overview

When `pinned_routes_` changes (routes added/removed from pins), instead of instantly switching layouts, run a multi-phase transition animation. After steps 1–2 complete, snap to the final split/flat layout.

## Current Flow

1. `set_pinned_routes_from_text()` or MQTT handler updates `pinned_routes_` immediately
2. Next `draw_schedule()` call sees new `effective_pinned_count`, renders split layout instantly

## Proposed Flow

### New State: `pin_transition_phase_` (int, 0 = idle)

Stored in the header alongside existing split scroll state:

```cpp
// Pin transition animation state
int pin_transition_phase_{0};          // 0=idle, 1=wait-for-scroll, 2=collapse-rows
unsigned long pin_transition_start_{0}; // millis when current phase began
int pin_transition_old_pinned_count_{0}; // effective_pinned_count BEFORE change
std::set<std::string> pin_transition_old_pinned_routes_; // snapshot of old pinned set
std::set<std::string> pin_transition_new_pinned_routes_; // snapshot of new pinned set (target)
```

### Detection: When do we trigger?

In `draw_schedule()`, after computing `effective_pinned_count` and `actual_pinned_in_visible`:

```
if (pin_transition_phase_ == 0) {
  compare current effective_pinned_count vs last-rendered count
  if different → snapshot old state, enter phase 1
}
```

We need a `last_rendered_pinned_count_` (int) to detect changes frame-to-frame.

### Phase 1: Wait for scroll idle (step 1)

**Entry:** Pin count changed, `pin_transition_phase_` set to 1.

**Behavior during this phase:**
- Continue rendering the OLD layout (use `pin_transition_old_pinned_routes_` to partition trips as before)
- Check each frame:
  - If `total_scroll_distance > 0` (headsign marquee active): wait until `h_scroll_offset == 0` (start of cycle)
  - If `total_scroll_distance == 0`: wait 500ms from phase entry

**On completion:** Enter phase 2, record `pin_transition_start_ = millis()`.

### Phase 2: Collapse pinned rows out of flat list (step 2)

**Duration:** 500ms

**Which rows collapse?**
- Rows whose composite_key is in `pin_transition_new_pinned_routes_` but was NOT in `pin_transition_old_pinned_routes_` (newly pinned)
- Only animate rows currently visible on screen (within `start_index..end_index`); off-screen ones just disappear

**Animation:**
- Each collapsing row has its height lerp from `nominal_font_height` → 0 using smoothstep
- Rows below shift up to fill the gap
- Clip the collapsing row's content to its shrinking height
- Progress: `t = elapsed / 500ms`, `smoothstep(t)`

**Drawing during phase 2:**
- Use the OLD layout (flat list, old partition)
- For each visible trip in `start_index..end_index`:
  - If trip is "newly pinned": `row_height = font_height * (1 - smoothstep(t))`
  - Else: normal height
- Accumulate y positions based on dynamic heights
- Clip each collapsing row to its current height

**On completion:** Set `pin_transition_phase_ = 0`, update `last_rendered_pinned_count_`. Normal rendering takes over with the new pinned state → instant snap to split layout.

### Where in `draw_schedule()` does this go?

After computing `effective_pinned_count` and before the `has_split` branch:

```
[existing: compute effective_pinned_count]

// --- Pin transition detection & animation ---
if (pin_transition_phase_ == 0 && effective_pinned_count != last_rendered_pinned_count_) {
    // Snapshot & enter phase 1
}

if (pin_transition_phase_ > 0) {
    // Run transition state machine
    // Draw using old layout with animation
    // mutex.unlock() and return early (skip normal split/flat drawing)
}

last_rendered_pinned_count_ = effective_pinned_count;
// --- End pin transition ---

[existing: has_split branch]
```

### Edge Cases

1. **Pin change during active transition:** Reset to phase 1 with new snapshots
2. **All pins removed (split → flat):** Same animation in reverse — collapse the pinned rows in the split section, then snap to flat
3. **Paging active:** Freeze paging during transition (don't advance pages)
4. **No visible newly-pinned trips on screen:** Skip phase 2 (0ms), go straight to completion

## Files Modified

- `transit_tracker.h` — Add new member variables
- `transit_tracker.cpp` — Add transition logic in `draw_schedule()`

## Size Estimate

- ~5 new member variables in header
- ~80-120 lines of new logic in `draw_schedule()`
