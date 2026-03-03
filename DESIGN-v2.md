# Transit Tracker v2: Clean-Room Redesign

## 1. Analysis of the Current Attempt

### What was attempted

Starting from a clean 503-line `transit_tracker.cpp` that did exactly one thing — draw a flat list of trips with synchronized horizontal scroll — the following features were incrementally bolted on across 12+ commits:

1. **Uniform headsign alignment** — align headsign start/end columns across all rows
2. **Hidden routes** — filter trips by composite key via MQTT/HA
3. **Route paging (vertical scroll mode)** — cycle through more trips than fit on screen
4. **Pinning** — pin favorite routes to the top rows, creating a split layout
5. **Pin transition animations** — 14+ animation phases for pin/unpin
6. **Replace mode** — collapse-in-place as alternative to vertical scroll
7. **Data transitions** — animated collapse when trips expire or change
8. **Pinned data transitions** — same, but for the pinned section
9. **MQTT auto-discovery** — select entities (Off/On/Pinned), next-only switches, color lights
10. **Next-only routes** — show only the soonest trip per route
11. **Route color overrides** — MQTT-driven per-route color changes
12. **Configurable scroll speed** — slow/medium/fast with pending-at-cycle-boundary
13. **Post-replace pause** — pause h-scroll after replace animations

### Why it has failed

The code grew from **503 lines to 4,888 lines** — a **10x increase** — with almost all of the growth inside a single function: `draw_schedule()`. This function is now a ~3,800-line monolith containing:

- **14+ pin transition phases** (numbered 1–14), each with its own rendering logic
- **6 split-scroll phases** (0–6), each with its own rendering logic  
- **2 data transition phases** × 2 (unpinned + pinned) = 4 more state machines
- **~60 member variables** tracking animation state, timing, caching, pending changes
- A **combinatorial explosion of guards**: every new feature must check whether every other feature's animation is active, and each interaction creates a new edge case

The fundamental problem is **architectural**: there is no separation between:
- **State management** (what trips are visible, what's pinned, what page we're on)
- **Animation orchestration** (what transitions are in flight, what's waiting)  
- **Rendering** (drawing pixels to the display)

Every new feature touches all three concerns simultaneously inside one function, and every fix for one interaction creates new interactions with other features. This is the classic "whack-a-mole" pattern.

#### Specific failure modes observed in the code:

1. **Frozen state snapshots everywhere**: `pin_transition_old_pinned_routes_`, `data_transition_old_trips_`, `pinned_dt_old_trips_`, `pin_swap_old_unpinned_` — each transition caches its own snapshot, and these caches must be kept in sync with each other. When they get out of sync, you get snaps.

2. **Phase-dependent routing tables**: comments like "Phases 1-2, 5-9, 12 use old hidden routes; phases 3-4, 10-11, 13 use target" are a code smell indicating the state machine has grown beyond human ability to reason about.

3. **Guard condition proliferation**: data transition detection is gated on `!in_split_scroll && !in_split_pause && this->split_scroll_phase_ == 0 && this->pin_transition_phase_ == 0 && !this->data_transition_old_keys_.empty()` — and there are dozens of similar guards.

4. **Duplicate rendering paths**: the same trips are drawn in ~15 different code paths (normal split, phase 2 collapse, phase 3 push, phase 4 slide, phase 5 hide, phase 6 slide-left, phase 8 scroll-up, phase 9 push-down, phase 10 reveal, phase 11 dwell, phase 12 shrink, phase 14 replace, split-scroll phases, data-transition phases, pinned-data-transition phases).

5. **Timing races**: h-scroll must be "near zero" before certain transitions start, but the threshold (`kScrollNearZeroPx = 3`) and the various `seen_nonzero_offset_` flags create edge cases where transitions never trigger or trigger at the wrong moment.

---

## 2. New Design

### Core Principles

1. **One rendering path**. Every frame produces a list of `DrawCommand`s (trip + y position + h-scroll offset + clip rect + opacity). The renderer is dumb: it just executes the commands. All intelligence lives upstream.

2. **Slot-based layout**. The display has N slots (= `limit_`). Each slot has a target trip assignment and a current animated position. Transitions are expressed as slot reassignments with interpolation — not as phase machines.

3. **Animation queue, not phase machine**. Instead of numbered phases with hardcoded sequencing, transitions are expressed as composable `Animation` objects pushed onto a queue. Each animation knows how to interpolate a set of slots from state A to state B. When it completes, the next one starts.

4. **Immutable snapshots for rendering**. Each frame takes a single snapshot of the trip data at the top of `draw_schedule()`, and all animation/rendering for that frame works from that snapshot. No mid-frame mutation, no separate caches per animation.

### Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    draw_schedule()                       │
│                                                         │
│  1. Snapshot trips (mutex lock/unlock immediately)      │
│  2. Filter (hidden, next-only)                          │
│  3. Partition (pinned / unpinned)                       │
│  4. Paginate (pinned page, unpinned page)               │
│  5. Assign to slots                                     │
│  6. Detect changes → push animations                    │
│  7. Tick animations → compute slot positions            │
│  8. Compute h-scroll offset (shared, single calc)       │
│  9. Render slots                                        │
└─────────────────────────────────────────────────────────┘
```

### Data Model

```cpp
// A display slot — one row on the LED matrix
struct Slot {
    int index;             // 0..limit-1
    bool is_pinned_zone;   // true = top section, false = bottom section
    
    // Current assignment
    std::string trip_key;  // composite_key of assigned trip ("" = empty)
    
    // Animation state (for this slot individually)
    float y_offset;        // current animated y offset (0 = at rest)
    float opacity;         // 1.0 = fully visible, 0.0 = collapsed
    float x_inset;         // pin icon inset (animated 0→7 or 7→0)
};

// The entire display state, recomputed each frame
struct FrameState {
    // Inputs (snapshot)
    std::vector<Trip> all_trips;        // raw from server
    std::vector<Trip> visible_trips;    // after hidden/next-only filter
    std::vector<Trip> pinned_trips;     // partitioned, deduped
    std::vector<Trip> unpinned_trips;   // partitioned
    
    // Layout
    int pinned_slot_count;              // 0 if no pins, else min(pinned_rows, actual_pinned)
    int unpinned_slot_count;            // limit - pinned_slot_count
    
    // Paging
    int pinned_page_offset;             // 0-based index into pinned_trips
    int unpinned_page_offset;           // 0-based index into unpinned_trips
    
    // Horizontal scroll (shared across all visible slots)
    int h_scroll_offset;               // current pixel offset
    int h_scroll_total_distance;       // total marquee cycle distance
    int h_scroll_cycle_count;          // completed cycles since last reset
    
    // Slot assignments for this frame
    Slot slots[MAX_LIMIT];              // MAX_LIMIT = 6 or whatever max
};
```

### Animation System

Instead of 14 numbered phases, we have a small set of **composable animation primitives**:

```cpp
enum class AnimationType {
    NONE,
    COLLAPSE_ROW,      // opacity 1→0 (row shrinks vertically)
    EXPAND_ROW,        // opacity 0→1 (row grows vertically)  
    SLIDE_VERTICAL,    // y_offset animates from A to B
    SLIDE_HORIZONTAL,  // x_inset animates from A to B (pin icon)
    PAGE_SCROLL,       // entire section shifts vertically by N rows
};

struct SlotAnimation {
    AnimationType type;
    int slot_index;         // which slot
    float start_value;
    float end_value;
    unsigned long start_time;
    unsigned long duration_ms;
    
    // Returns current interpolated value, using smoothstep
    float value_at(unsigned long now) const;
    bool is_complete(unsigned long now) const;
};

class AnimationQueue {
    std::vector<SlotAnimation> active_;     // currently running (parallel)
    std::vector<SlotAnimation> pending_;    // start after all active complete
    
    void tick(unsigned long now);           // advance all active, promote pending
    bool is_idle() const;                   // nothing active or pending
    void push_parallel(SlotAnimation a);    // add to current batch
    void push_sequential(SlotAnimation a);  // add to next batch (after current completes)
    void cancel_all();                      // immediate stop (for emergency)
};
```

### Transition Recipes

Every user-visible transition is a **recipe** that pushes a sequence of animations:

#### Pin a route (flat → split)
```
1. Wait for h-scroll idle (animation system has built-in "wait for idle" support)
2. COLLAPSE_ROW on the slot that held the newly-pinned trip (in the unpinned section)
3. After collapse: reassign slots (pinned section = 1 row, unpinned = limit-1)
4. SLIDE_HORIZONTAL on all slots (x_inset 0→7, pin icon slides in)
5. EXPAND_ROW on pinned slot 0 (the new pin appears)
```

In replace mode, steps 2-5 become:
```
2. COLLAPSE_ROW on ALL slots (staggered by kReplaceStaggerMs)
3. After all collapsed: reassign slots to new layout
4. EXPAND_ROW on ALL slots (staggered)
```

#### Unpin a route (split → flat)
```
1. Wait for h-scroll idle
2. COLLAPSE_ROW on pinned slot (pin disappears)
3. SLIDE_HORIZONTAL on all slots (x_inset 7→0, pin icon slides out)
4. Reassign slots to flat layout
5. SLIDE_VERTICAL on unpinned trips (move up to fill gap)
```

#### Trip expires (data transition)
```
1. Wait for h-scroll idle
2. COLLAPSE_ROW on the slot of the departed trip
3. After collapse: reassign slot to next trip, EXPAND_ROW
```

#### Page change (vertical scroll)
```
1. Wait for h-scroll idle (or h-scroll cycle boundary)
2. PAGE_SCROLL on affected section (smooth vertical shift)
```

### Key Design Decisions

#### 1. Pinned and unpinned as two independent containers

```cpp
struct ScrollContainer {
    int start_slot;       // first slot index (0 for pinned, pinned_count for unpinned)
    int slot_count;       // number of slots in this container
    int pool_size;        // total trips available for this container
    int page_offset;      // current page offset into the pool
    
    // Each container has independent paging but SHARED h-scroll
    unsigned long page_timer;
    
    void advance_page();
    void compute_page_trips(const std::vector<Trip>& pool, 
                            std::vector<Trip>& out_trips) const;
};
```

When `pinned_count == 0`, there is one container spanning all slots. When `pinned_count > 0`, there are two containers. The rendering code doesn't care — it just iterates slots.

#### 2. Shared horizontal scroll

There is exactly ONE h-scroll computation per frame, affecting ALL visible slots:

```cpp
struct HScrollState {
    unsigned long start_time;
    int scroll_speed;           // px/sec
    int pending_speed;          // 0 = no change pending
    int total_distance;         // current cycle distance (0 = no scroll)
    int prev_total_distance;    // for wind-down when overflow disappears
    
    struct Result {
        int offset;             // current pixel offset
        int cycle_count;        // completed full cycles
        bool is_idle;           // true if offset ≈ 0
    };
    
    Result compute(unsigned long now) const;
    void reset(unsigned long now);
    
    // Call once per frame with the max headsign width across ALL visible slots
    void update_distance(int max_headsign_width, int marquee_gap);
};
```

This eliminates the duplication between `pinned_h_scroll_start_`, `h_scroll_start_time_`, `last_shared_scroll_dist_`, `last_scroll_distance_`, etc.

#### 3. Change detection via diff

Instead of separate change detectors for pinned/unpinned/data, there is ONE diff operation per frame:

```cpp
struct DisplayDiff {
    // Previous frame's slot assignments
    std::vector<std::string> prev_keys;
    std::vector<time_t> prev_departure_times;
    int prev_pinned_count;
    
    // Computed diff
    bool layout_changed;        // pinned_count changed
    bool pinned_set_changed;    // different routes pinned (same count)
    std::vector<int> departed_slots;    // slots whose trips disappeared
    std::vector<int> arrived_slots;     // slots with new trips
    std::vector<int> rolled_slots;      // same key, different departure time
    bool page_ready;                    // page timer expired + h-scroll idle
    
    void compute(const FrameState& current);
    void commit(const FrameState& current);  // update prev for next frame
};
```

The diff is computed ONCE, and then a single `plan_transitions()` function reads the diff and pushes the appropriate animations. No scattered if/else chains.

#### 4. Transition planner (replaces 14 phases)

```cpp
void TransitTracker::plan_transitions(const DisplayDiff& diff, unsigned long now) {
    // Don't plan new transitions while animations are running
    if (!animation_queue_.is_idle()) return;
    
    if (diff.layout_changed) {
        plan_layout_transition(diff, now);
        return;  // layout change takes priority
    }
    
    if (diff.pinned_set_changed) {
        plan_pin_swap(diff, now);
        return;
    }
    
    if (!diff.departed_slots.empty() || !diff.rolled_slots.empty()) {
        plan_data_transition(diff, now);
        return;
    }
    
    if (diff.page_ready) {
        plan_page_scroll(diff, now);
        return;
    }
}

void TransitTracker::plan_layout_transition(const DisplayDiff& diff, unsigned long now) {
    if (prefer_replace_over_scroll_) {
        // Staggered collapse all → reassign → staggered expand all
        for (int i = 0; i < limit_; i++) {
            animation_queue_.push_parallel({
                .type = AnimationType::COLLAPSE_ROW,
                .slot_index = i,
                .start_value = 1.0f,
                .end_value = 0.0f,
                .start_time = now + i * kReplaceStaggerMs,
                .duration_ms = kReplacePerRowMs,
            });
        }
        // The "reassign slots" happens in the completion callback
        // Then expand all:
        for (int i = 0; i < limit_; i++) {
            animation_queue_.push_sequential({
                .type = AnimationType::EXPAND_ROW,
                .slot_index = i,
                .start_value = 0.0f,
                .end_value = 1.0f,
                .start_time = 0,  // filled in when promoted from pending
                .duration_ms = kReplacePerRowMs,
            });
        }
    } else {
        // Vertical scroll mode: handled similarly with SLIDE_VERTICAL
        // ...
    }
}
```

#### 5. MQTT as a separate concern

MQTT discovery, subscriptions, and state publishing are extracted into their own methods that operate on the trip data and pin/hidden/next-only sets. They have ZERO interaction with the animation system.

```cpp
// Called from loop() when MQTT is connected and routes_pending_
void TransitTracker::sync_mqtt() {
    publish_route_entities();
    publish_color_entities();
}

// Called when MQTT command received — just updates the set, nothing else
void TransitTracker::on_mqtt_route_command(const std::string& key, const std::string& state) {
    if (state == "Off") hidden_routes_.insert(key);
    else if (state == "On") { hidden_routes_.erase(key); pinned_routes_.erase(key); }
    else if (state == "Pinned") { hidden_routes_.erase(key); pinned_routes_.insert(key); }
    persist_routes();
    // draw_schedule() will detect the change on next frame via DisplayDiff
}
```

### Rendering (the simple part)

```cpp
void TransitTracker::render_frame(const FrameState& state, unsigned long now) {
    int fh = font_->get_ascender() + font_->get_descender();
    int y_base = compute_y_base(fh);
    
    for (int i = 0; i < limit_; i++) {
        const Slot& slot = state.slots[i];
        if (slot.trip_key.empty() || slot.opacity <= 0.01f) continue;
        
        // Find the trip for this slot
        const Trip* trip = find_trip(state, slot.trip_key);
        if (!trip) continue;
        
        // Compute position
        int y = y_base + i * fh + (int)(slot.y_offset + 0.5f);
        int clip_h = (int)((float)fh * slot.opacity + 0.5f);
        
        // Apply clipping for partial visibility
        int clip_top, clip_bottom;
        if (slot.opacity < 1.0f) {
            // Growing from bottom (expand) or shrinking from bottom (collapse)
            clip_top = y + fh - clip_h;
            clip_bottom = y + fh;
        } else {
            clip_top = y;
            clip_bottom = y + fh;
        }
        
        // Container clipping (pinned section doesn't bleed into unpinned)
        if (slot.is_pinned_zone) {
            int divider = y_base + state.pinned_slot_count * fh;
            clip_bottom = std::min(clip_bottom, divider);
        } else {
            int divider = y_base + state.pinned_slot_count * fh;
            clip_top = std::max(clip_top, divider);
        }
        
        display_->start_clipping(0, clip_top, display_->get_width(), clip_bottom);
        draw_trip(*trip, y, fh, now, state.rtc_now,
                  false, nullptr, nullptr, 
                  state.h_scroll_offset, state.h_scroll_total_distance,
                  state.uniform_clip_start, state.uniform_clip_end,
                  slot.is_pinned_zone);
        display_->end_clipping();
    }
}
```

This is **the only place** `draw_trip` is called with `no_draw=false`. All 15+ current call sites collapse into one.

### Complete `draw_schedule()` flow

```cpp
void TransitTracker::draw_schedule() {
    // --- Early returns (unchanged from original) ---
    if (display_ == nullptr) return;
    if (!network::is_connected()) { draw_text_centered_("Waiting..."); return; }
    // ... etc ...
    
    // --- 1. Snapshot ---
    schedule_state_.mutex.lock();
    std::vector<Trip> all_trips = schedule_state_.trips;
    schedule_state_.mutex.unlock();
    // Mutex released immediately. Everything below works on the local copy.
    
    // --- 2. Filter ---
    auto visible = filter_trips(all_trips, hidden_routes_, active_next_only_routes_);
    if (visible.empty()) { draw_text_centered_("No upcoming..."); return; }
    
    // --- 3. Partition ---
    auto [pinned, unpinned] = partition_trips(visible, pinned_routes_, pinned_rows_count_);
    
    // --- 4. Build frame state ---
    FrameState frame;
    frame.pinned_slot_count = pinned.size(); // already deduped & clamped
    frame.unpinned_slot_count = limit_ - frame.pinned_slot_count;
    frame.all_trips = std::move(all_trips);
    frame.pinned_trips = std::move(pinned);
    frame.unpinned_trips = std::move(unpinned);
    frame.rtc_now = rtc_->now().timestamp;
    
    // --- 5. Paginate ---
    pinned_container_.paginate(frame.pinned_trips, frame.pinned_slot_count);
    unpinned_container_.paginate(frame.unpinned_trips, frame.unpinned_slot_count);
    
    // --- 6. Assign slots ---
    assign_slots(frame);
    
    // --- 7. Diff & plan ---
    unsigned long now = millis();
    DisplayDiff diff;
    diff.compute(frame, prev_frame_);
    
    if (animation_queue_.is_idle()) {
        plan_transitions(diff, now);
    }
    
    // --- 8. Tick animations ---
    animation_queue_.tick(now);
    apply_animations_to_slots(frame, now);
    
    // --- 9. H-scroll ---
    int max_hw = compute_max_headsign_width(frame);
    h_scroll_.update_distance(max_hw, marquee_gap);
    auto scroll = h_scroll_.compute(now);
    frame.h_scroll_offset = animation_queue_.is_idle() ? scroll.offset : 0;
    frame.h_scroll_total_distance = scroll.total_distance;
    
    // --- 10. Deferred changes (next-only, scroll speed) ---
    if (scroll.is_idle) {
        apply_pending_changes();
    }
    
    // --- 11. Render ---
    compute_uniform_clipping(frame);
    render_frame(frame, now);
    
    // --- 12. Commit for next frame ---
    diff.commit(frame);
    prev_frame_ = frame;
}
```

This is **~60 lines** for the entire orchestration, vs the current **~3,800 lines**.

### State variable reduction

| Current | New | Count reduction |
|---------|-----|----------------|
| ~60 member variables for animation state | `AnimationQueue` + `HScrollState` + `DisplayDiff` + 2× `ScrollContainer` | ~15 total |
| 14 pin transition phases | 4 animation types composed in recipes | 0 phases |
| 6 split scroll phases | `PAGE_SCROLL` animation | 0 phases |
| Separate pinned/unpinned data transition | Single `DisplayDiff` | 1 detector |
| 5+ frozen snapshot vectors | 1 `prev_frame_` | 1 snapshot |

---

## 3. Comparison: Why Start From Scratch

### The current code cannot be incrementally fixed

Every bug fix in the current code follows this pattern:
1. Identify a visual glitch (snap, flicker, stuck animation)
2. Trace it to a missing guard condition in one of the 14+ phases
3. Add the guard — which changes the timing/behavior of that phase
4. This causes a new interaction with another phase that wasn't previously reachable
5. New bug appears in a different scenario

This is because the **state space is combinatorial**. With 14 pin phases × 6 split phases × 2 data transition phases × 2 pinned data transition phases, there are **336 possible state combinations**, most of which have never been tested and many of which produce incorrect behavior. Each fix adds a new guard that prunes one path but inadvertently opens another.

### What the new design eliminates

| Problem | Current | New |
|---------|---------|-----|
| **Rendering duplication** | 15+ code paths call `draw_trip` with different parameters | 1 rendering loop |
| **State explosion** | 336+ phase combinations | Animations are independent per-slot; at most one transition recipe active |
| **Frozen snapshot management** | 5 separate snapshot vectors, manually kept in sync | 1 `prev_frame_` |
| **Guard condition proliferation** | ~30 compound boolean guards | `animation_queue_.is_idle()` |
| **Phase-dependent routing** | "Phases 1-2, 5-9, 12 use old routes; 3-4, 10-11, 13 use target" | Transition recipe captures before/after state at creation |
| **Timing races** | `seen_nonzero_offset_`, `kScrollNearZeroPx`, `pending_cycles_` | `h_scroll_.is_idle()` checked once |
| **Feature coupling** | Every feature must know about every other feature | Features interact only through `DisplayDiff` |

### Lines of code estimate

| Component | Estimated LOC |
|-----------|--------------|
| `Slot`, `FrameState`, `DisplayDiff` structs | ~80 |
| `AnimationQueue` | ~100 |
| `HScrollState` | ~60 |
| `ScrollContainer` (×2) | ~50 |
| Transition recipes (pin, unpin, data, page) | ~200 |
| `draw_schedule()` orchestration | ~80 |
| `render_frame()` | ~50 |
| Filter/partition/paginate helpers | ~60 |
| MQTT (unchanged from current, extracted) | ~350 |
| Everything else (setup, WS, config, etc.) | ~350 (unchanged) |
| **Total** | **~1,380** |

vs. the current **4,888 lines** — a **72% reduction**.

### Why the new design won't have the same bugs

1. **Animations always complete**: `SlotAnimation` tracks `start_time + duration_ms`. `is_complete()` is a pure function of time. There is no phase that can be skipped or stuck. The queue promotes pending animations only after all active ones complete.

2. **No snaps**: every visual change goes through the animation queue. The only way to skip an animation is `cancel_all()`, which is reserved for shutdown. A new change arriving mid-animation gets **queued**, not applied immediately.

3. **Single source of truth**: `prev_frame_` is the only record of "what was on screen last frame." There's no possibility of `data_transition_old_trips_` disagreeing with `pinned_dt_old_trips_` because they don't exist.

4. **Testable**: `AnimationQueue::tick()`, `DisplayDiff::compute()`, and the transition recipes are pure functions that can be unit-tested without a display, font, or network connection.

5. **Re-use between modes**: In replace mode, the recipes use `COLLAPSE_ROW` + `EXPAND_ROW`. In scroll mode, they use `SLIDE_VERTICAL`. The rendering code is identical either way — it just reads `slot.opacity` and `slot.y_offset`.

### Migration path

1. Branch from the original clean commit (`a22dac0`)
2. Add the new structs (`Slot`, `FrameState`, `AnimationQueue`, etc.) to new header files
3. Port `draw_schedule()` to the new architecture
4. Port MQTT code (mostly copy-paste of the current extraction methods)
5. Port YAML configuration from `transit-tracker.yaml`
6. Test each feature in isolation before combining

The MQTT, WebSocket, configuration parsing, and `draw_trip()` code can all be directly reused — they don't need redesign. The only thing that changes is the orchestration layer inside `draw_schedule()`.

---

## 4. Implementation File Structure

```
components/transit_tracker/
  __init__.py              # ESPHome config schema (keep, add new config keys)
  schedule_state.h         # Trip + ScheduleState (keep as-is)
  localization.h/cpp       # Keep as-is
  string_utils.h/cpp       # Keep as-is
  animation_utils.h        # Replace: smoothstep + SlotAnimation + AnimationQueue
  display_types.h          # NEW: Slot, FrameState, DisplayDiff, ScrollContainer, HScrollState
  transit_tracker.h         # Slimmed: ~100 lines (remove ~60 animation state vars)
  transit_tracker.cpp       # Slimmed: ~1,400 lines total
  mqtt_support.cpp          # NEW: extracted MQTT discovery/subscribe/publish  
```

### What to keep verbatim from current code

- `setup()`, `loop()`, `dump_config()`, `reconnect()`, `close()`, `on_shutdown()`
- `on_ws_message_()`, `on_ws_event_()`, `connect_ws_()`  
- `set_abbreviations_from_text()`, `set_route_styles_from_text()`
- `set_hidden_routes_from_text()`, `set_pinned_routes_from_text()`, `set_next_only_routes_from_text()`
- `rebuild_route_stop_map_()`
- `draw_text_centered_()`, `set_realtime_color()`
- `draw_realtime_icon_()` 
- `draw_trip()` (the per-row rendering — simplified to remove `h_scroll_offset` special casing and the `frame_pin_offset_override_` machinery)
- All MQTT code (moved to `mqtt_support.cpp`)

### What gets rewritten

- `draw_schedule()` — from ~3,800 lines to ~80 lines of orchestration
- `transit_tracker.h` — from ~250 lines / ~60 state vars to ~100 lines / ~15 state vars
- `animation_utils.h` — from easing + constants to full animation system

---

## 5. Summary

The current implementation has made the classic mistake of managing complex state transitions with an explicit phase machine that grows linearly with each new feature but whose **interaction space grows combinatorially**. Each fix addresses one specific phase interaction but creates new ones, because the architecture provides no mechanism for isolation.

The proposed redesign replaces the phase machine with a **slot-based layout model** and a **composable animation queue**. This provides:

- **O(1) rendering paths** instead of O(phases × features)
- **Isolation**: animations are per-slot and independent
- **Composability**: new transitions are built from 4 primitives, not new phases
- **Correctness by construction**: animations always complete because they're time-bounded
- **Debuggability**: `AnimationQueue::dump()` shows exactly what's in flight

The non-negotiable features (Pin UI, uniform scrolling, replace mode, vertical scroll mode, MQTT support) are all preserved. The MQTT code is nearly unchanged. The rendering code (`draw_trip`) is simplified. The only thing that fundamentally changes is the ~3,800 lines of orchestration code in `draw_schedule()`, which is replaced by ~500 lines of structured, composable logic.
