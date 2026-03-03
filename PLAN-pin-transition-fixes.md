# Pin Transition — Implementation Plan for Unhandled Scenarios

Based on the analysis of the [pin matrix](PLAN-pin-matrix.md) against the current
scrolling code, these are the gaps that need to be addressed, ordered by priority.

---

## Issue #5 — Rapid Successive Pins Lose Animation

**Severity:** High  
**Trigger:** User pins route A, then pins route B before A's transition completes.

### Problem

When pin B arrives during phase 1 or 2 of pin A's transition, `pin_transition_phase_ != 0`
so the change is ignored. When A's transition completes, it sets
`pin_transition_old_eff_pinned_ = -1` (force re-init) and
`pin_transition_old_pinned_routes_ = pinned_routes_` (which now includes both A and B).
On the next frame, old state is initialized to the *current* state — B was never seen as
"newly pinned" so it gets no collapse animation.

### Fix

Queue the incoming pin change so it triggers a fresh transition after the current one completes.

1. Add a member `bool pin_transition_pending_restart_ = false`.
2. In the detection block (`pin_transition_phase_ == 0 && eff != old_eff`), if
   `pin_transition_phase_ != 0`, set `pin_transition_pending_restart_ = true` instead.
3. When phase 2 completes (both split and flat paths), check `pin_transition_pending_restart_`:
   - If true: set `pin_transition_old_eff_pinned_ = -1` (force re-init) as today,
     clear the flag. The next frame will detect the mismatch and start a new transition
     for the remaining change.
   - If false: set `pin_transition_old_eff_pinned_` and `old_pinned_routes_` normally.

### Edge Cases

- Three rapid pins: each successive pin sets `pending_restart_ = true`, only the last
  transition's "new" state matters. All intermediate states collapse into one final animation.
  Acceptable — animating each individually would take too long.

---

## Issue #9 — Paging State Stale After Phase 2

**Severity:** High  
**Trigger:** Pin on a multi-page display; after collapse completes, page index exceeds new page count.

### Problem

During transition, old paging state is frozen. When phase 2 completes and the display
snaps to the new layout, `current_page_index_` may exceed the new `total_pages`. The code
clamps it, but the resulting page may not show the same trips the collapse animation ended on.

### Fix

When phase 2 completes, compute the correct page for the new layout:

**Flat path:**
1. After snap, compute `new_total_pages` based on the post-pin trip count.
2. The last non-collapsing entry in `phase2_entries` tells us which trip index was at the
   bottom of the display. Find which page contains that trip in the new layout.
3. Set `current_page_index_` to that page, reset `page_timer_start_` and
   `h_scroll_start_time_`.

**Split path:**
1. Reset `pinned_page_index_ = 0` and `split_unpinned_page_index_ = 0`.
2. Reset all split scroll state (`split_scroll_phase_ = 0`, timers, etc.).
3. On the next frame, the split layout initializes fresh.

This is simpler than trying to preserve the exact position, and gives a clean starting state.

---

## Issue #3 — Pin During Active Page Scroll

**Severity:** Medium  
**Trigger:** Pin triggered while a page-to-page scroll animation is in progress (flat path).

### Problem

Phase 1 checks `h_scroll_offset` for idle detection, but doesn't know a page scroll is
running. The page scroll resets `h_scroll_start_time_` on completion, which can break
the "seen nonzero then returned to 0" cycle tracking.

### Fix

In the phase 1 detection, also check for active page scroll:

```cpp
// Phase 1 completion check (flat path)
if (this->pin_transition_phase_ == 1) {
  // Don't advance to phase 2 while a page scroll is in progress
  if (in_page_scroll) {
    // Wait — do nothing until page scroll finishes
  } else if (total_scroll_distance > 0) {
    // existing h-scroll idle detection...
  } else {
    // existing 500ms no-scroll wait...
  }
}
```

**Note:** `in_page_scroll` is currently computed *after* the phase 1 check. Move the
`in_page_scroll` computation (or at least the detection of whether a page scroll is active)
*before* the pin transition blocks.

### Ordering change needed

Move these lines before the pin transition phase 1/2 blocks:
- `bool in_page_scroll = (page_scroll_start_ > 0 && ...)` — a lightweight check
- Or add a separate `bool page_scroll_active` flag computed earlier

---

## Issue #4 — Pin During Active Split Scroll Phases

**Severity:** Medium  
**Trigger:** Pin triggered while the split layout is mid-v-scroll (phases 2–4 of the split
scroll state machine).

### Problem

Split scroll phases animate pinned and unpinned sections independently. A pin transition
during this animation means the "on-screen trips" used for phase 2 entry building don't
match what's visually displayed (the display shows partially-scrolled positions).

### Fix

In the split path's phase 1 detection, also wait for split scroll completion:

```cpp
if (this->pin_transition_phase_ == 1) {
  bool split_scroll_active = (this->split_scroll_phase_ != 0);
  if (split_scroll_active) {
    // Wait — split scroll must finish first
  } else if (shared_scroll_dist > 0) {
    // existing h-scroll idle detection...
  } else {
    // existing 500ms wait...
  }
}
```

This ensures phase 1 doesn't complete until both h-scroll is idle **and** the split
v-scroll is at rest. The split scroll state machine will finish on its own, advancing
pages as needed, and then phase 1 can proceed.

---

## Issue #1 — Unpin Transition Uses Wrong Base

**Severity:** Medium  
**Trigger:** Unpinning a route (going from split layout back to flat, or from split with
2 pinned → split with 1 pinned).

### Problem

When unpinning, phase 2 runs in the split path because `has_split` is true under old
partitioning. `split_phase2_base = pinned_si` starts from the pinned section's page index.
The trip being unpinned is in the pinned section (top rows), but the phase 2 renderer
treats the entire `visible_trips` as a flat list and tries to collapse the unpinned trip
wherever it falls in the combined list. The visual position doesn't match.

### Fix

For unpin transitions in split path, the base index should reflect the **visible screen
position**, not just the pinned page index. When a trip is being *removed* from pinned:

1. Detect **direction** at transition start: `bool is_unpinning = (effective_pinned_count < pin_transition_old_eff_pinned_)`.
2. Store this as `pin_transition_is_unpin_`.
3. In split path phase 2, when `is_unpin`:
   - The collapsing trip is in the **pinned section** (rows 0..pinned_rows-1)
   - The fill-in comes from the **unpinned section** scrolling up
   - Adjust `split_phase2_base` to start from 0 (pinned section always visible from top)
   - Only mark entries in the pinned pool whose key matches the unpinned route as collapsing

### Alternative (simpler)

Don't animate unpin collapse at all — just snap. The user is removing a pin, the visual
change is less jarring because the unpinned trip will still be visible (it just moves from
the pinned section to the unpinned section). Defer proper unpin animation to steps 3–6.

---

## Issue #7 — H-Scroll Snap at Phase 1→2 Boundary

**Severity:** Low  
**Trigger:** Any config where headsign h-scroll was active before the pin.

### Problem

Phase 1 waits for `h_scroll_offset < 3`, then phase 2 renders with `h_scroll_offset = 0`.
At low frame rates, there could be a visible 1–2px horizontal jump.

### Fix

When phase 1 completes and transitions to phase 2, reset `h_scroll_start_time_ = uptime`
(flat path) or `pinned_h_scroll_start_ = uptime` (split path) so the h-scroll position is
genuinely at 0, not just "close to 0".

Already partially done — flat path sets `h_scroll_offset = 0` on line 1811. The split path
should do the same with `shared_h_offset = 0`.

Minimal code change, low risk.

---

## Issue #8 — Insufficient Fill Trips (Display Shrinks)

**Severity:** Low  
**Trigger:** T − P < limit_ (e.g., T=4 P=2 → only 2 non-collapsing trips to fill 3 slots).

### Problem

After both fill loops, `fill_needed > 0` means the entries list has fewer than `limit_`
items. The display shows fewer rows, with empty space at the bottom.

### Current Behavior

Functionally correct — the animation collapses 2 rows, fills with the 2 remaining trips,
and the display naturally shows 2 rows. After phase 2 completes, the new layout (1 pinned +
1 unpinned, or 2 pinned + 0 unpinned) correctly displays the reduced row count.

### Fix

No code change needed. The empty space is transient (only during the 500ms collapse) and
accurately represents the final state. The centering fix (using `page_size` instead of
`display_count`) ensures the rows stay top-aligned during the transition.

---

## Issue #10 — No Animation for Layout Structure Change

**Severity:** Deferred (Steps 3–6)  
**Trigger:** All pin/unpin transitions.

### Problem

The divider line, row repositioning, pinned section header, and pin icons all snap
instantly when phase 2 completes. The plan document outlines steps 3–6 for this:

3. **Push divider** — animate the divider line sliding into position
4. **Shift headers** — animate route names shifting as rows reposition
5. **Scroll pinned in** — animate the newly-pinned trip scrolling into the pinned section
6. **Animate pin icons** — fade in/out pin icons

### Status

These are all deferred to phase 3+ of the transition animation work. The current
implementation covers only steps 1 (wait for scroll idle) and 2 (collapse rows).

---

## Issue #2 — T=P Pinned-Only Renders via Flat Path

**Severity:** Info (no fix needed)  
**Trigger:** All trips belong to the pinned route (e.g., T=2 P=2).

### Current Behavior

`has_split = false` because there are no unpinned trips, so the flat path renders them.
No divider is drawn. Pin icons are shown. This is correct — there's nothing to divide.

### Fix

None needed. This is a valid rendering path for the pinned-only case.

---

## Issue #6 — V=0 Gives No Visual Feedback

**Severity:** Info (no fix needed now)  
**Trigger:** Multi-page display where the pinned route's trips are on a different page.

### Current Behavior

Phase 2 runs with `collapsing_count = 0`. The display sits static for 500ms + 2s debug
pause, then snaps to the new layout.

### Possible Future Enhancement

When V=0, skip the collapse animation entirely and go straight to the layout restructure
(steps 3–6). The user doesn't need to see a freeze followed by a jump — they should see
the display smoothly transition to the new layout.

---

## Implementation Order

| Step | Issue | Effort | Risk |
|:----:|:-----:|:------:|:----:|
| 1 | #7 — H-scroll snap fix | Trivial | None |
| 2 | #9 — Reset paging state after phase 2 | Small | Low |
| 3 | #5 — Queue successive pins | Small | Low |
| 4 | #3 — Wait for page scroll in phase 1 | Medium | Medium (ordering) |
| 5 | #4 — Wait for split scroll in phase 1 | Small | Low |
| 6 | #1 — Unpin base fix (or defer to snap) | Medium | Medium |
| 7 | #10 — Steps 3–6 animations | Large | High |

### Estimated LOC

| Step | Lines Changed | Lines Added |
|:----:|:------------:|:-----------:|
| 1 | 2 | 0 |
| 2 | 10 | 15 |
| 3 | 5 | 10 |
| 4 | 10 | 5 |
| 5 | 5 | 3 |
| 6 | 20 | 30 |
| 7 | — | 200+ |

---

## Also: Remove Temp Debug Pause

Both phase 2 completion checks currently have `&& phase2_elapsed >= 2500` (a 2-second
debug pause). This should be removed when the implementation is stable:

- **Flat path** (line ~1828): `if (collapse_t >= 1.0f && phase2_elapsed >= 2500)`
  → `if (collapse_t >= 1.0f)`
- **Split path** (line ~1393): same pattern
