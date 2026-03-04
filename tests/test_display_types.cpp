#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

// Include the stubs first (via -I), then the real headers
#include "schedule_state.h"
#include "display_types.h"
#include "string_utils.h"
#include "string_utils.cpp"  // pull in the implementation (single TU build)

using namespace esphome;
using namespace esphome::transit_tracker;

// =====================================================================
// Helper: build a Trip quickly
// =====================================================================
static Trip make_trip(const std::string &route_id, const std::string &headsign,
                      time_t dep = 1000, bool realtime = false,
                      const std::string &stop_id = "") {
  Trip t;
  t.route_id = route_id;
  t.route_name = route_id;  // same for tests
  t.route_color = Color(0x00FF00);
  t.headsign = headsign;
  t.stop_id = stop_id;
  t.arrival_time = dep - 30;
  t.departure_time = dep;
  t.is_realtime = realtime;
  return t;
}

// =====================================================================
// Trip::composite_key
// =====================================================================

TEST_SUITE("Trip") {
  TEST_CASE("composite_key without stop_id") {
    Trip t = make_trip("R1", "Downtown");
    CHECK(t.composite_key() == "R1:Downtown");
  }

  TEST_CASE("composite_key with stop_id") {
    Trip t = make_trip("R1", "Downtown", 1000, false, "S42");
    CHECK(t.composite_key() == "R1:Downtown:S42");
  }
}

// =====================================================================
// smoothstep
// =====================================================================

TEST_SUITE("smoothstep") {
  TEST_CASE("boundary values") {
    CHECK(smoothstep(0.0f) == doctest::Approx(0.0f));
    CHECK(smoothstep(1.0f) == doctest::Approx(1.0f));
    CHECK(smoothstep(0.5f) == doctest::Approx(0.5f));
  }

  TEST_CASE("clamping") {
    CHECK(smoothstep(-1.0f) == doctest::Approx(0.0f));
    CHECK(smoothstep(2.0f) == doctest::Approx(1.0f));
  }

  TEST_CASE("monotonic in [0,1]") {
    float prev = 0.0f;
    for (float t = 0.05f; t <= 1.0f; t += 0.05f) {
      float val = smoothstep(t);
      CHECK(val >= prev);
      prev = val;
    }
  }
}

// =====================================================================
// dedup_pinned_section
// =====================================================================

TEST_SUITE("dedup_pinned_section") {
  TEST_CASE("no duplicates — no change") {
    std::vector<Trip> trips = {
        make_trip("R1", "A"), make_trip("R2", "B"), make_trip("R3", "C")};
    int pc = 3;
    dedup_pinned_section(trips, pc);
    CHECK(pc == 3);
    CHECK(trips.size() == 3);
  }

  TEST_CASE("removes duplicate pinned keys, keeps first") {
    std::vector<Trip> trips = {
        make_trip("R1", "A", 100),
        make_trip("R1", "A", 200),  // dup
        make_trip("R2", "B", 300),
        make_trip("R3", "C", 400),  // unpinned
    };
    int pc = 3;  // first 3 are "pinned"
    dedup_pinned_section(trips, pc);
    CHECK(pc == 2);
    CHECK(trips.size() == 3);  // 2 pinned + 1 unpinned
    CHECK(trips[0].departure_time == 100);
    CHECK(trips[1].route_id == "R2");
    CHECK(trips[2].route_id == "R3");
  }

  TEST_CASE("stop_id makes keys distinct") {
    std::vector<Trip> trips = {
        make_trip("R1", "A", 100, false, "S1"),
        make_trip("R1", "A", 200, false, "S2"),
    };
    int pc = 2;
    dedup_pinned_section(trips, pc);
    CHECK(pc == 2);  // different stop_id => different key
  }
}

// =====================================================================
// HScrollState
// =====================================================================

TEST_SUITE("HScrollState") {
  TEST_CASE("idle when no distance") {
    HScrollState hs;
    hs.reset(1);
    hs.compute(1000);
    CHECK(hs.idle);
    CHECK(hs.offset == 0);
  }

  TEST_CASE("scrolls when distance set") {
    HScrollState hs;
    hs.scroll_speed = 100;  // 100px/s
    hs.reset(1);
    hs.update_distance(50, true);  // 50 + 20 gap = 70 total
    CHECK(hs.total_distance == 70);

    hs.compute(501);  // elapsed=500, 0.5s at 100px/s = 50px
    CHECK(hs.offset == 50);
    CHECK(!hs.idle);
  }

  TEST_CASE("wraps around at total_distance") {
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(1);
    hs.update_distance(80, true);  // total = 100
    CHECK(hs.total_distance == 100);

    // After reset: first cycle has no dwell (scroll_phase=1000ms).
    // Then start_time advances by 1000, standard cycles begin
    // (scroll_phase=1000ms + dwell=3000ms = 4000ms each).
    // To land in second cycle scroll phase at offset=50:
    //   first cycle ends at t=1001, standard start_time=1001
    //   t=1501: cycle_elapsed=500, offset=50, cycle_count=1+0=1
    hs.compute(1501);
    CHECK(hs.offset == 50);
    CHECK(hs.cycle_count == 1);
  }

  TEST_CASE("wind-down: finishes cycle then stops") {
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(1);
    hs.update_distance(80, true);  // total = 100

    // Let it scroll a bit first so it's mid-cycle
    hs.compute(201);  // elapsed=200 => 20px, not near zero
    CHECK(hs.offset == 20);
    CHECK(!hs.idle);

    // Simulate removing overflow
    hs.update_distance(0, false);
    CHECK(hs.total_distance == 0);
    CHECK(hs.prev_total_distance == 100);

    // At cycle boundary (near-zero) — stops
    hs.compute(1001);  // elapsed=1000 => 100px, 100 % 100 = 0 => near zero
    CHECK(hs.idle);
    CHECK(hs.prev_total_distance == 0);  // cleared
  }

  TEST_CASE("pending speed applied at cycle boundary") {
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(1);
    hs.update_distance(80, true);  // total = 100
    hs.pending_speed = 200;

    // Mid-scroll — pending not applied
    hs.compute(201);  // elapsed=200 => 20px, not near zero
    CHECK(hs.scroll_speed == 100);

    // At cycle boundary — applied
    hs.compute(1001);  // elapsed=1000 => 100px, 100 % 100 = 0 => near zero
    CHECK(hs.scroll_speed == 200);
    CHECK(hs.pending_speed == 0);
  }
}

// =====================================================================
// ScrollContainer
// =====================================================================

TEST_SUITE("ScrollContainer") {
  TEST_CASE("no paging when pool fits slots") {
    ScrollContainer sc;
    CHECK(!sc.needs_paging(3, 3));
    CHECK(!sc.needs_paging(2, 3));
    CHECK(sc.page_count(3, 3) == 1);
  }

  TEST_CASE("paging when pool > slots") {
    ScrollContainer sc;
    CHECK(sc.needs_paging(5, 3));
    CHECK(sc.page_count(5, 3) == 3);  // 0,1,2 start indices
  }

  TEST_CASE("advance_page wraps") {
    ScrollContainer sc;
    sc.page_offset = 0;
    sc.advance_page(5, 3);
    CHECK(sc.page_offset == 1);
    sc.advance_page(5, 3);
    CHECK(sc.page_offset == 2);
    sc.advance_page(5, 3);
    CHECK(sc.page_offset == 0);  // wraps
  }

  TEST_CASE("start/end index") {
    ScrollContainer sc;
    sc.page_offset = 1;
    CHECK(sc.start_index(5, 3) == 1);
    CHECK(sc.end_index(5, 3) == 4);
  }

  TEST_CASE("clamp resets if pool shrinks") {
    ScrollContainer sc;
    sc.page_offset = 4;
    sc.clamp(3, 3);  // pool=3, slots=3 => 1 page => offset clamped to 0
    CHECK(sc.page_offset == 0);
  }
}

// =====================================================================
// DisplayDiff
// =====================================================================

TEST_SUITE("DisplayDiff") {
  TEST_CASE("first frame — no diff") {
    DisplayDiff diff;
    std::vector<std::string> keys = {"R1:A", "R2:B"};
    std::vector<time_t> deps = {100, 200};
    std::set<std::string> pinned;

    diff.compute(keys, deps, 0, pinned);
    CHECK(!diff.has_changes());  // prev_pinned_count < 0
  }

  TEST_CASE("no change after commit") {
    DisplayDiff diff;
    std::vector<std::string> keys = {"R1:A", "R2:B"};
    std::vector<time_t> deps = {100, 200};
    std::set<std::string> pinned;
    std::vector<Trip> trips = {make_trip("R1", "A", 100), make_trip("R2", "B", 200)};
    std::vector<bool> is_p = {false, false};

    diff.commit(keys, deps, trips, is_p, 0, pinned);
    diff.compute(keys, deps, 0, pinned);
    CHECK(!diff.has_changes());
  }

  TEST_CASE("layout_changed on pinned count change") {
    DisplayDiff diff;
    std::vector<std::string> keys = {"R1:A", "R2:B"};
    std::vector<time_t> deps = {100, 200};
    std::set<std::string> pinned;
    std::vector<Trip> trips = {make_trip("R1", "A", 100), make_trip("R2", "B", 200)};
    std::vector<bool> is_p = {false, false};

    diff.commit(keys, deps, trips, is_p, 0, pinned);

    // Now pinned count changes to 1
    diff.compute(keys, deps, 1, pinned);
    CHECK(diff.layout_changed);
    CHECK(diff.has_changes());
  }

  TEST_CASE("pinned_set_changed on pinned routes change") {
    DisplayDiff diff;
    std::vector<std::string> keys = {"R1:A", "R2:B"};
    std::vector<time_t> deps = {100, 200};
    std::set<std::string> pinned_v1 = {"R1:A"};
    std::vector<Trip> trips = {make_trip("R1", "A", 100), make_trip("R2", "B", 200)};
    std::vector<bool> is_p = {true, false};

    diff.commit(keys, deps, trips, is_p, 1, pinned_v1);

    std::set<std::string> pinned_v2 = {"R2:B"};
    diff.compute(keys, deps, 1, pinned_v2);
    CHECK(diff.pinned_set_changed);
    CHECK(diff.has_changes());
  }

  TEST_CASE("data_changed on key change") {
    DisplayDiff diff;
    std::vector<std::string> keys1 = {"R1:A", "R2:B"};
    std::vector<time_t> deps1 = {100, 200};
    std::set<std::string> pinned;
    std::vector<Trip> trips = {make_trip("R1", "A", 100), make_trip("R2", "B", 200)};
    std::vector<bool> is_p = {false, false};

    diff.commit(keys1, deps1, trips, is_p, 0, pinned);

    std::vector<std::string> keys2 = {"R1:A", "R3:C"};
    std::vector<time_t> deps2 = {100, 300};
    diff.compute(keys2, deps2, 0, pinned);
    CHECK(diff.data_changed);
  }

  TEST_CASE("data_changed on departure time jump > 60s") {
    DisplayDiff diff;
    std::vector<std::string> keys = {"R1:A"};
    std::vector<time_t> deps1 = {1000};
    std::set<std::string> pinned;
    std::vector<Trip> trips = {make_trip("R1", "A", 1000)};
    std::vector<bool> is_p = {false};

    diff.commit(keys, deps1, trips, is_p, 0, pinned);

    std::vector<time_t> deps2 = {1100};  // +100s > 60s threshold
    diff.compute(keys, deps2, 0, pinned);
    CHECK(diff.data_changed);
  }

  TEST_CASE("no change on small departure drift") {
    DisplayDiff diff;
    std::vector<std::string> keys = {"R1:A"};
    std::vector<time_t> deps1 = {1000};
    std::set<std::string> pinned;
    std::vector<Trip> trips = {make_trip("R1", "A", 1000)};
    std::vector<bool> is_p = {false};

    diff.commit(keys, deps1, trips, is_p, 0, pinned);

    std::vector<time_t> deps2 = {1030};  // +30s < 60s threshold
    diff.compute(keys, deps2, 0, pinned);
    CHECK(!diff.has_changes());
  }
}

// =====================================================================
// Transition
// =====================================================================

TEST_SUITE("Transition") {
  TEST_CASE("timing helpers") {
    Transition tr;
    tr.stagger_rows = 3;
    // cascade = 250 * (3-1) = 500
    CHECK(tr.cascade_ms() == 500);
    // collapse = 500 + 500 = 1000
    CHECK(tr.collapse_duration_ms() == 1000);
    CHECK(tr.expand_duration_ms() == 1000);
    // total = 1000 + 500 + 1000 = 2500
    CHECK(tr.total_replace_ms() == 2500);
  }

  TEST_CASE("row_scale collapse") {
    Transition tr;
    tr.stagger_rows = 1;

    // t=0 => scale=1 (fully visible)
    CHECK(tr.row_scale(0, 0, false) == doctest::Approx(1.0f));
    // t=500 (kReplacePerRowMs) => scale=0 (fully collapsed)
    CHECK(tr.row_scale(0, 500, false) == doctest::Approx(0.0f));
    // t=250 => midpoint => scale=0.5
    CHECK(tr.row_scale(0, 250, false) == doctest::Approx(0.5f));
  }

  TEST_CASE("row_scale expand") {
    Transition tr;
    tr.stagger_rows = 1;

    CHECK(tr.row_scale(0, 0, true) == doctest::Approx(0.0f));
    CHECK(tr.row_scale(0, 500, true) == doctest::Approx(1.0f));
    CHECK(tr.row_scale(0, 250, true) == doctest::Approx(0.5f));
  }

  TEST_CASE("row_scale stagger offset") {
    Transition tr;
    tr.stagger_rows = 3;

    // Row 0 at t=0 starts immediately
    CHECK(tr.row_scale(0, 0, false) == doctest::Approx(1.0f));

    // Row 1 starts at t=250 (kReplaceStaggerMs)
    CHECK(tr.row_scale(1, 0, false) == doctest::Approx(1.0f));  // hasn't started yet
    CHECK(tr.row_scale(1, 250, false) == doctest::Approx(1.0f));  // just starting
    CHECK(tr.row_scale(1, 500, false) == doctest::Approx(0.5f));  // halfway

    // Row 2 starts at t=500
    CHECK(tr.row_scale(2, 500, false) == doctest::Approx(1.0f));
    CHECK(tr.row_scale(2, 1000, false) == doctest::Approx(0.0f));
  }

  TEST_CASE("reset clears state") {
    Transition tr;
    tr.phase = Transition::COLLAPSE;
    tr.stagger_rows = 5;
    tr.old_trips.push_back(make_trip("R1", "A"));
    tr.reset();

    CHECK(tr.phase == Transition::IDLE);
    CHECK(tr.stagger_rows == 0);
    CHECK(tr.old_trips.empty());
    CHECK(!tr.animate_pinned);
    CHECK(!tr.animate_unpinned);
  }
}

// =====================================================================
// split (string_utils)
// =====================================================================

TEST_SUITE("split") {
  TEST_CASE("basic split") {
    auto r = split("a;b;c", ';');
    REQUIRE(r.size() == 3);
    CHECK(r[0] == "a");
    CHECK(r[1] == "b");
    CHECK(r[2] == "c");
  }

  TEST_CASE("empty string") {
    auto r = split("", ';');
    CHECK(r.size() == 0);  // stringstream produces no tokens
  }

  TEST_CASE("no delimiter") {
    auto r = split("hello", ';');
    REQUIRE(r.size() == 1);
    CHECK(r[0] == "hello");
  }

  TEST_CASE("trailing delimiter") {
    auto r = split("a;b;", ';');
    REQUIRE(r.size() == 2);  // stringstream doesn't emit trailing empty
    CHECK(r[0] == "a");
    CHECK(r[1] == "b");
  }
}

// =====================================================================
// DisplayDiff — pin-change edge cases
// =====================================================================

TEST_SUITE("DisplayDiff — pin transitions") {
  // helper: commit a frame into the diff
  static void commit_frame(DisplayDiff &diff,
                           const std::vector<Trip> &trips,
                           int pinned_count,
                           const std::set<std::string> &pinned_routes) {
    std::vector<std::string> keys;
    std::vector<time_t> deps;
    std::vector<bool> is_p;
    for (size_t i = 0; i < trips.size(); i++) {
      keys.push_back(trips[i].composite_key());
      deps.push_back(trips[i].departure_time);
      is_p.push_back((int)i < pinned_count);
    }
    diff.commit(keys, deps, trips, is_p, pinned_count, pinned_routes);
  }

  static void compute_frame(DisplayDiff &diff,
                             const std::vector<Trip> &trips,
                             int pinned_count,
                             const std::set<std::string> &pinned_routes) {
    std::vector<std::string> keys;
    std::vector<time_t> deps;
    for (const auto &t : trips) {
      keys.push_back(t.composite_key());
      deps.push_back(t.departure_time);
    }
    diff.compute(keys, deps, pinned_count, pinned_routes);
  }

  TEST_CASE("add first pin — layout_changed") {
    DisplayDiff diff;
    std::vector<Trip> trips = {make_trip("R1", "A", 100), make_trip("R2", "B", 200)};
    std::set<std::string> no_pins;

    commit_frame(diff, trips, 0, no_pins);

    // Pin R1 — pinned count goes from 0→1
    std::set<std::string> pin_r1 = {"R1:A"};
    std::vector<Trip> reordered = {make_trip("R1", "A", 100), make_trip("R2", "B", 200)};
    compute_frame(diff, reordered, 1, pin_r1);

    CHECK(diff.layout_changed);
    CHECK(diff.has_changes());
    // Snapshot preserves old trips for transition
    CHECK(diff.prev_pinned_count == 0);
    CHECK(diff.prev_trips.size() == 2);
  }

  TEST_CASE("remove last pin — layout_changed") {
    DisplayDiff diff;
    std::vector<Trip> trips = {make_trip("R1", "A", 100), make_trip("R2", "B", 200)};
    std::set<std::string> pin_r1 = {"R1:A"};

    commit_frame(diff, trips, 1, pin_r1);

    // Remove pin
    std::set<std::string> no_pins;
    compute_frame(diff, trips, 0, no_pins);

    CHECK(diff.layout_changed);
    CHECK(diff.has_changes());
    CHECK(diff.prev_pinned_count == 1);
  }

  TEST_CASE("swap pinned route — pinned_set_changed (count stays same)") {
    DisplayDiff diff;
    std::vector<Trip> trips = {make_trip("R1", "A", 100), make_trip("R2", "B", 200), make_trip("R3", "C", 300)};
    std::set<std::string> pin_v1 = {"R1:A"};

    commit_frame(diff, trips, 1, pin_v1);

    // Swap R1 for R2 — count still 1 but set changed
    std::set<std::string> pin_v2 = {"R2:B"};
    std::vector<Trip> reordered = {make_trip("R2", "B", 200), make_trip("R1", "A", 100), make_trip("R3", "C", 300)};
    compute_frame(diff, reordered, 1, pin_v2);

    CHECK(diff.pinned_set_changed);
    CHECK(!diff.layout_changed);  // count didn't change
    CHECK(diff.has_changes());
  }

  TEST_CASE("rapid add then remove before commit — sees layout change each time") {
    DisplayDiff diff;
    std::vector<Trip> trips = {make_trip("R1", "A"), make_trip("R2", "B")};
    std::set<std::string> no_pins;

    // Frame 0: commit with no pins
    commit_frame(diff, trips, 0, no_pins);

    // Frame 1: compute with pin added (but don't commit — simulates transition active)
    std::set<std::string> pin_r1 = {"R1:A"};
    compute_frame(diff, trips, 1, pin_r1);
    CHECK(diff.layout_changed);

    // Frame 2: compute again with pin removed (still haven't committed)
    // diff.prev_pinned_count is still 0 from the commit
    compute_frame(diff, trips, 0, no_pins);
    CHECK(!diff.has_changes());  // same as committed state
  }

  TEST_CASE("commit preserves snapshot trips for transition old_trips") {
    DisplayDiff diff;
    Trip t1 = make_trip("R1", "A", 100);
    Trip t2 = make_trip("R2", "B", 200);
    std::vector<Trip> trips = {t1, t2};
    std::set<std::string> no_pins;

    commit_frame(diff, trips, 0, no_pins);

    // Verify snapshot
    REQUIRE(diff.prev_trips.size() == 2);
    CHECK(diff.prev_trips[0].route_id == "R1");
    CHECK(diff.prev_trips[1].route_id == "R2");
    CHECK(diff.prev_is_pinned[0] == false);
    CHECK(diff.prev_is_pinned[1] == false);

    // Modify original trips — snapshot should be independent
    trips[0] = make_trip("R3", "C", 300);
    CHECK(diff.prev_trips[0].route_id == "R1");
  }

  TEST_CASE("size change detected as data_changed") {
    DisplayDiff diff;
    std::vector<Trip> trips2 = {make_trip("R1", "A"), make_trip("R2", "B")};
    std::set<std::string> no_pins;
    commit_frame(diff, trips2, 0, no_pins);

    // Now 3 rows instead of 2
    std::vector<Trip> trips3 = {make_trip("R1", "A"), make_trip("R2", "B"), make_trip("R3", "C")};
    compute_frame(diff, trips3, 0, no_pins);
    CHECK(diff.data_changed);
  }

  TEST_CASE("multiple pins added at once — layout_changed") {
    DisplayDiff diff;
    std::vector<Trip> trips = {make_trip("R1", "A"), make_trip("R2", "B"), make_trip("R3", "C")};
    std::set<std::string> no_pins;
    commit_frame(diff, trips, 0, no_pins);

    std::set<std::string> pin_both = {"R1:A", "R2:B"};
    compute_frame(diff, trips, 2, pin_both);
    CHECK(diff.layout_changed);
    CHECK(diff.has_changes());
  }
}

// =====================================================================
// Transition — mid-animation interrupts
// =====================================================================

TEST_SUITE("Transition — mid-animation") {
  TEST_CASE("reset mid-collapse clears phase and snapshots") {
    Transition tr;
    tr.phase = Transition::COLLAPSE;
    tr.phase_start = 1000;
    tr.stagger_rows = 3;
    tr.animate_pinned = true;
    tr.animate_unpinned = true;
    tr.old_trips = {make_trip("R1", "A"), make_trip("R2", "B"), make_trip("R3", "C")};
    tr.old_is_pinned = {false, false, false};
    tr.old_eff_pinned = 0;

    // Simulate pin change mid-collapse → reset + re-enter
    tr.reset();
    CHECK(tr.phase == Transition::IDLE);
    CHECK(tr.old_trips.empty());
    CHECK(tr.stagger_rows == 0);

    // Re-populate for new transition
    tr.old_trips = {make_trip("R1", "A"), make_trip("R2", "B")};
    tr.old_is_pinned = {true, false};
    tr.old_eff_pinned = 1;
    tr.stagger_rows = 2;
    tr.phase = Transition::WAIT_SCROLL;
    tr.phase_start = 2000;

    CHECK(tr.phase == Transition::WAIT_SCROLL);
    CHECK(tr.old_eff_pinned == 1);
    CHECK(tr.old_trips.size() == 2);
  }

  TEST_CASE("reset mid-expand preserves ability to restart") {
    Transition tr;
    tr.phase = Transition::EXPAND;
    tr.phase_start = 5000;
    tr.stagger_rows = 3;
    tr.old_trips = {make_trip("R1", "A"), make_trip("R2", "B"), make_trip("R3", "C")};

    // Row 0 mid-expand: at t=250 into expand, partially visible
    float scale = tr.row_scale(0, 250, true);
    CHECK(scale > 0.0f);
    CHECK(scale < 1.0f);

    // Interrupted — reset
    tr.reset();
    CHECK(tr.phase == Transition::IDLE);

    // New transition can start cleanly
    tr.phase = Transition::COLLAPSE;
    tr.phase_start = 6000;
    tr.stagger_rows = 2;
    CHECK(tr.collapse_duration_ms() == kReplacePerRowMs + kReplaceStaggerMs);
  }

  TEST_CASE("single-row stagger has zero cascade") {
    Transition tr;
    tr.stagger_rows = 1;
    CHECK(tr.cascade_ms() == 0);
    CHECK(tr.collapse_duration_ms() == kReplacePerRowMs);
    CHECK(tr.expand_duration_ms() == kReplacePerRowMs);
    CHECK(tr.total_replace_ms() == 2 * kReplacePerRowMs + kReplaceMidPauseMs);
  }

  TEST_CASE("row_scale at exact phase boundaries") {
    Transition tr;
    tr.stagger_rows = 2;

    // Row 0, t=0, collapse: should be fully visible (about to start shrinking)
    CHECK(tr.row_scale(0, 0, false) == doctest::Approx(1.0f));

    // Row 0, t=kReplacePerRowMs, collapse: should be fully collapsed
    CHECK(tr.row_scale(0, kReplacePerRowMs, false) == doctest::Approx(0.0f));

    // Row 1, t=kReplaceStaggerMs, collapse: just starting — still visible
    CHECK(tr.row_scale(1, kReplaceStaggerMs, false) == doctest::Approx(1.0f));

    // Row 1, t=kReplaceStaggerMs + kReplacePerRowMs: fully collapsed
    CHECK(tr.row_scale(1, kReplaceStaggerMs + kReplacePerRowMs, false) == doctest::Approx(0.0f));

    // Row 0, t=0, expand: fully invisible (about to start growing)
    CHECK(tr.row_scale(0, 0, true) == doctest::Approx(0.0f));

    // Row 0, t=kReplacePerRowMs, expand: fully visible
    CHECK(tr.row_scale(0, kReplacePerRowMs, true) == doctest::Approx(1.0f));
  }

  TEST_CASE("row_scale with many rows — last row starts late") {
    Transition tr;
    tr.stagger_rows = 5;

    int last_row = 4;
    int last_row_start = last_row * kReplaceStaggerMs;  // 4 * 250 = 1000

    // Before its start time: still fully visible (collapse) or invisible (expand)
    CHECK(tr.row_scale(last_row, last_row_start - 1, false) == doctest::Approx(1.0f));
    CHECK(tr.row_scale(last_row, last_row_start - 1, true) == doctest::Approx(0.0f));

    // At its start time
    CHECK(tr.row_scale(last_row, last_row_start, false) == doctest::Approx(1.0f));
    CHECK(tr.row_scale(last_row, last_row_start, true) == doctest::Approx(0.0f));

    // At its end time
    CHECK(tr.row_scale(last_row, last_row_start + kReplacePerRowMs, false) == doctest::Approx(0.0f));
    CHECK(tr.row_scale(last_row, last_row_start + kReplacePerRowMs, true) == doctest::Approx(1.0f));
  }

  TEST_CASE("vscroll fields reset properly") {
    Transition tr;
    tr.is_vscroll = true;
    tr.vscroll_section_y = 10;
    tr.vscroll_section_rows = 3;
    tr.vscroll_is_pinned = true;
    tr.reset();
    CHECK(!tr.is_vscroll);
    // Note: vscroll fields aren't explicitly zeroed by reset(), but is_vscroll
    // being false means they won't be read.
  }

  TEST_CASE("old_respect_pin_inset defaults to true and resets") {
    Transition tr;
    CHECK(tr.old_respect_pin_inset == true);

    tr.old_respect_pin_inset = false;
    tr.phase = Transition::COLLAPSE;
    tr.reset();
    CHECK(tr.old_respect_pin_inset == true);
  }
}

// =====================================================================
// HScrollState — reset clears total_distance (phantom scroll fix)
// =====================================================================

TEST_SUITE("HScrollState — reset clears distance") {
  TEST_CASE("reset clears both total_distance and prev_total_distance") {
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(1);
    hs.update_distance(80, true);  // total = 100
    CHECK(hs.total_distance == 100);

    // Scroll for a while, then trigger wind-down so prev is populated
    hs.update_distance(0, false);
    CHECK(hs.prev_total_distance == 100);

    // Reset (like a transition would do)
    hs.reset(2000);
    CHECK(hs.total_distance == 0);
    CHECK(hs.prev_total_distance == 0);
  }

  TEST_CASE("no phantom scroll after reset when overflow disappears") {
    // Scenario: scrolling active → transition resets → new data has no overflow.
    // Before the fix, the stale total_distance survived reset() and
    // update_distance(0, false) moved it into prev_total_distance, causing
    // a phantom scroll cycle with skip_first_dwell=true (no dwell pause).
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(1);

    // Start scrolling (overflow exists)
    hs.update_distance(80, true);  // total = 100
    hs.compute(501);               // mid-scroll: offset=50
    CHECK(hs.offset == 50);
    CHECK(!hs.idle);

    // Transition fires: reset at current time
    hs.reset(501);

    // After transition, new trip data has no overflow
    hs.update_distance(0, false);

    // Should be fully idle — no phantom scroll cycle
    CHECK(hs.total_distance == 0);
    CHECK(hs.prev_total_distance == 0);

    hs.compute(600);
    CHECK(hs.idle);
    CHECK(hs.offset == 0);

    hs.compute(1000);
    CHECK(hs.idle);
    CHECK(hs.offset == 0);

    hs.compute(5000);
    CHECK(hs.idle);
    CHECK(hs.offset == 0);
  }

  TEST_CASE("reset then new overflow starts scrolling normally") {
    // After reset, if new data DOES overflow, scrolling should start fresh.
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(1);
    hs.update_distance(80, true);  // total = 100

    hs.compute(501);
    CHECK(hs.offset == 50);

    // Transition fires: reset
    hs.reset(1000);
    CHECK(hs.total_distance == 0);

    // New data still overflows (different headsign, same width)
    hs.update_distance(80, true);  // total = 100
    CHECK(hs.total_distance == 100);
    CHECK(!hs.deferred_start);  // skip_next_defer was set by reset

    // Scrolling works from offset 0 (skip_first_dwell means no initial dwell)
    hs.compute(1100);  // elapsed=100ms → 10px
    CHECK(hs.offset == 10);
    CHECK(!hs.idle);
  }

  TEST_CASE("repeated reset+no-overflow stays idle") {
    // Multiple rapid transitions where overflow disappears each time.
    // None should produce phantom scrolling.
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(1);
    hs.update_distance(80, true);
    hs.compute(501);  // scrolling

    for (int i = 0; i < 5; i++) {
      unsigned long t = 1000 + i * 500;
      hs.reset(t);
      hs.update_distance(0, false);
      hs.compute(t + 100);
      CHECK(hs.idle);
      CHECK(hs.offset == 0);
      CHECK(hs.total_distance == 0);
      CHECK(hs.prev_total_distance == 0);

      // Re-enable overflow for next iteration
      if (i < 4) {
        hs.update_distance(80, true);
        hs.compute(t + 200);
      }
    }
  }
}

// =====================================================================
// HScrollState — pin inset / total_distance change scenarios
// =====================================================================

TEST_SUITE("HScrollState — pin-change disruption") {
  TEST_CASE("total_distance change mid-scroll after reset") {
    // After reset, first cycle skips dwell.  Changing total_distance
    // mid-scroll shifts the first cycle’s scroll_phase_ms.
    HScrollState hs;
    hs.scroll_speed = 100;  // 100 px/s
    hs.reset(1);            // start_time=1 (0 is treated as uninitialized)

    // Initial: headsign width 80 → total=100 (80+20 gap)
    hs.update_distance(80, true);
    CHECK(hs.total_distance == 100);

    // Scroll to elapsed=750ms → 75px into first (no-dwell) cycle
    hs.compute(751);
    CHECK(hs.offset == 75);

    // Now simulate pin change making headsign 7px narrower → width 73, total=93
    hs.update_distance(73, true);
    CHECK(hs.total_distance == 93);

    // At elapsed=1000ms with new dist 93:
    // scroll_phase_ms = 930. First no-dwell cycle ends at elapsed=930.
    // start_time advances to 931, standard timing begins.
    // elapsed from 931 = 70ms, offset = 7px (in second cycle scroll phase).
    hs.compute(1001);
    CHECK(hs.offset == 7);
  }

  TEST_CASE("stable total_distance means smooth scrolling across cycles") {
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(0);
    hs.update_distance(80, true);  // total=100

    // Sample many frames — offset should increase monotonically within a cycle
    int prev_offset = -1;
    int prev_cycle = 0;
    for (unsigned long t = 0; t <= 2500; t += 50) {
      hs.compute(t);
      if (hs.cycle_count == prev_cycle) {
        CHECK(hs.offset >= prev_offset);
      }
      prev_offset = hs.offset;
      prev_cycle = hs.cycle_count;
    }
  }

  TEST_CASE("reset then immediately compute gives zero offset") {
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(1);
    hs.update_distance(80, true);

    // Scroll for a while
    hs.compute(501);  // elapsed=500 → 50px
    CHECK(hs.offset > 0);

    // Reset (simulating transition entering COLLAPSE)
    hs.reset(501);
    hs.compute(501);  // elapsed=0 → 0px
    CHECK(hs.offset == 0);
    CHECK(hs.idle);
  }

  TEST_CASE("wind-down followed by new overflow restarts cleanly") {
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(1);

    // Start scrolling
    hs.update_distance(80, true);
    CHECK(hs.total_distance == 100);

    // Remove overflow (wind-down)
    hs.update_distance(0, false);

    // Reach cycle boundary → wind-down clears prev
    hs.compute(1001);  // elapsed=1000, 100%100=0, near zero
    CHECK(hs.idle);
    CHECK(hs.prev_total_distance == 0);

    // In real code, a post-pause reset happens here
    hs.reset(1001);

    // New overflow appears (different route)
    hs.update_distance(60, true);  // total=80
    CHECK(hs.total_distance == 80);

    // Should scroll normally from offset 0
    hs.compute(1051);  // elapsed=50ms at 100px/s = 5px
    CHECK(hs.offset == 5);
  }

  TEST_CASE("zero distance throughout stays idle") {
    HScrollState hs;
    hs.reset(0);
    hs.update_distance(0, false);
    for (unsigned long t = 0; t < 5000; t += 100) {
      hs.compute(t);
      CHECK(hs.idle);
      CHECK(hs.offset == 0);
    }
  }

  TEST_CASE("update_distance with overflow=false but positive width has no effect") {
    HScrollState hs;
    hs.reset(0);
    // Width > 0 but no overflow — should not start scrolling
    hs.update_distance(80, false);
    CHECK(hs.total_distance == 0);
    hs.compute(1000);
    CHECK(hs.idle);
    CHECK(hs.offset == 0);
  }
}

// =====================================================================
// HScrollState — deferred start
// =====================================================================

TEST_SUITE("HScrollState — deferred start") {
  TEST_CASE("overflow appearing while fully idle defers scrolling") {
    // Simulates: headsign fit, then Wi-Fi icon shrinks available space
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.compute(1);           // init start_time (use 1, not 0 — 0 is sentinel)
    hs.update_distance(0, false);

    // Run idle for a while (no overflow)
    for (unsigned long t = 1; t <= 5000; t += 100) {
      hs.compute(t);
      CHECK(hs.idle);
      CHECK(hs.offset == 0);
    }

    // At t=5000, overflow appears (e.g. Wi-Fi icon reduces clip area)
    hs.update_distance(80, true);  // total_distance = 100
    CHECK(hs.total_distance == 100);
    CHECK(hs.deferred_start);

    // First compute after deferred_start sets deferred_since = 5100
    // Dwell duration = dist * 1000 / speed = 100 * 1000 / 100 = 1000ms
    // So dwell ends at 5100 + 1000 = 6100
    hs.compute(5100);
    CHECK(hs.offset == 0);
    CHECK(hs.idle);

    hs.compute(5500);
    CHECK(hs.offset == 0);
    CHECK(hs.idle);

    // Still dwelling at t=6099 (just before dwell ends)
    hs.compute(6099);
    CHECK(hs.offset == 0);
    CHECK(hs.idle);
    CHECK(hs.deferred_start);

    // At t=6100, dwell ends → scrolling begins from offset 0
    hs.compute(6100);
    CHECK(!hs.deferred_start);
    CHECK(hs.offset == 0);  // start_time was just set to 6100

    // At t=6200, scrolling is underway: 100ms * 100px/s = 10px
    hs.compute(6200);
    CHECK(hs.offset == 10);
    CHECK(!hs.idle);
  }

  TEST_CASE("overflow disappearing during dwell cancels without wind-down") {
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.compute(1);  // init start_time (use 1 not 0)
    hs.update_distance(0, false);
    hs.compute(1000);

    // Overflow appears → enters deferral
    hs.update_distance(80, true);  // total=100
    CHECK(hs.deferred_start);

    hs.compute(1200);  // mid-dwell
    CHECK(hs.offset == 0);
    CHECK(hs.deferred_start);

    // Overflow disappears before dwell finishes
    hs.update_distance(0, false);
    CHECK(!hs.deferred_start);
    CHECK(hs.total_distance == 0);
    CHECK(hs.prev_total_distance == 0);  // no wind-down queued

    // Stays fully idle
    hs.compute(2000);
    CHECK(hs.idle);
    CHECK(hs.offset == 0);
  }

  TEST_CASE("reset after transition suppresses deferral on next overflow") {
    // Simulates: data change → transition → post-pause → reset → overflow
    // The transition already provided a visual break, so no dwell needed.
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.compute(0);

    // Transition happens, reset at t=3000
    hs.reset(3000);
    CHECK(hs.skip_next_defer);

    // Overflow detected on first measurement after reset
    hs.update_distance(80, true);  // total=100
    CHECK(hs.total_distance == 100);
    CHECK(!hs.deferred_start);     // skip_next_defer consumed
    CHECK(!hs.skip_next_defer);

    // Scrolling starts immediately
    hs.compute(3100);  // elapsed=100ms → 10px
    CHECK(hs.offset == 10);
    CHECK(!hs.idle);
  }

  TEST_CASE("skip_next_defer consumed even when no overflow") {
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(1000);
    CHECK(hs.skip_next_defer);

    // No overflow on first measurement
    hs.update_distance(0, false);
    CHECK(!hs.skip_next_defer);  // consumed

    // Later, overflow appears — should now defer
    hs.compute(2000);
    hs.update_distance(80, true);
    CHECK(hs.deferred_start);
  }

  TEST_CASE("already scrolling + distance changes — no deferral") {
    // If we're already scrolling and the overflow magnitude changes
    // (e.g. different headsign), no deferral should happen.
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(1);  // use 1 not 0 (0 is sentinel in compute)
    hs.update_distance(80, true);  // skip_next_defer consumed, no defer

    hs.compute(501);  // scrolling: elapsed=500, offset=50
    CHECK(hs.offset == 50);
    CHECK(!hs.idle);

    // Distance changes (longer headsign)
    hs.update_distance(120, true);  // total=140
    CHECK(hs.total_distance == 140);
    CHECK(!hs.deferred_start);  // was_fully_idle is false

    // Scrolling continues
    hs.compute(601);
    CHECK(hs.offset > 0);
  }

  TEST_CASE("dwell duration scales with distance and speed") {
    HScrollState hs;
    hs.scroll_speed = 50;  // slower
    hs.compute(1);  // init start_time
    hs.update_distance(0, false);
    hs.compute(1000);

    hs.update_distance(80, true);  // total=100
    CHECK(hs.deferred_start);

    // First compute sets deferred_since=1100
    // Dwell = 100 * 1000 / 50 = 2000ms → ends at 1100+2000=3100
    hs.compute(1100);
    CHECK(hs.deferred_start);
    CHECK(hs.offset == 0);

    hs.compute(3099);
    CHECK(hs.deferred_start);

    hs.compute(3100);
    CHECK(!hs.deferred_start);
    CHECK(hs.offset == 0);  // start_time just set to 3100

    // Now scrolling at 50px/s: at t=3300, elapsed=200ms → 10px
    hs.compute(3300);
    CHECK(hs.offset == 10);
  }

  TEST_CASE("repeated overflow toggle only defers on 0→positive transition") {
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.compute(1);  // init start_time
    hs.update_distance(0, false);
    hs.compute(1000);

    // First overflow → deferred
    hs.update_distance(80, true);
    CHECK(hs.deferred_start);

    // First compute sets deferred_since=1100, dwell=1000ms, ends at 2100
    hs.compute(1100);
    CHECK(hs.deferred_start);
    hs.compute(2100);
    CHECK(!hs.deferred_start);

    // Scroll phase completes at 2100+1000=3100 (enters dwell)
    // start_time was set to 2100, scroll_phase_ms=1000, cycle_total_ms=4000
    hs.compute(3100);
    CHECK(hs.idle);

    // Remove overflow — wind-down
    hs.update_distance(0, false);
    // In dwell phase — wind-down clears prev_total_distance immediately
    hs.compute(3101);
    CHECK(hs.prev_total_distance == 0);

    // Second overflow → should defer again (fully idle)
    hs.update_distance(80, true);
    CHECK(hs.deferred_start);
  }

  TEST_CASE("wind-down followed by reset then overflow — no deferral") {
    // This is the existing "wind-down followed by new overflow" scenario
    // but verifying skip_next_defer prevents deferral.
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(1);

    hs.update_distance(80, true);  // skip_next_defer consumed, no defer
    CHECK(!hs.deferred_start);

    hs.update_distance(0, false);  // wind-down
    hs.compute(1001);              // reach boundary
    CHECK(hs.prev_total_distance == 0);

    hs.reset(1001);
    hs.update_distance(60, true);  // skip_next_defer consumed
    CHECK(!hs.deferred_start);

    hs.compute(1051);
    CHECK(hs.offset == 5);
  }
}

// =====================================================================
// ScrollContainer — edge cases during transitions
// =====================================================================

TEST_SUITE("ScrollContainer — transition edge cases") {
  TEST_CASE("clamp with drastically shrunk pool") {
    ScrollContainer sc;
    // Was on page 4 of a 10-item pool with 3 slots
    sc.page_offset = 4;
    CHECK(sc.start_index(10, 3) == 4);

    // Pool shrinks to 3 items — only 1 page
    sc.clamp(3, 3);
    CHECK(sc.page_offset == 0);
    CHECK(sc.start_index(3, 3) == 0);
    CHECK(sc.end_index(3, 3) == 3);
  }

  TEST_CASE("clamp with pool shrunk but still multi-page") {
    ScrollContainer sc;
    sc.page_offset = 5;
    // pool=8, slots=3 → 6 pages (0..5), offset 5 is still valid
    sc.clamp(8, 3);
    CHECK(sc.page_offset == 5);

    // pool shrinks to 6, slots=3 → 4 pages (0..3)
    sc.clamp(6, 3);
    CHECK(sc.page_offset == 3);
  }

  TEST_CASE("advance during an ongoing transition is safe") {
    ScrollContainer sc;
    sc.page_offset = 0;
    // Simulate: transition starts, page advances, new content prepared
    sc.advance_page(5, 3);  // goes to page 1
    CHECK(sc.page_offset == 1);
    CHECK(sc.start_index(5, 3) == 1);
    CHECK(sc.end_index(5, 3) == 4);

    // Another advance before the first transition completes
    sc.advance_page(5, 3);  // goes to page 2
    CHECK(sc.page_offset == 2);
    CHECK(sc.start_index(5, 3) == 2);
    CHECK(sc.end_index(5, 3) == 5);
  }

  TEST_CASE("reset clears page state for fresh pin layout") {
    ScrollContainer sc;
    sc.page_offset = 3;
    sc.page_timer = 999;
    sc.last_cycle_count = 5;

    sc.reset();
    CHECK(sc.page_offset == 0);
    CHECK(sc.page_timer == 0);
    CHECK(sc.last_cycle_count == 0);
  }

  TEST_CASE("start_index clamped when page_offset exceeds max") {
    ScrollContainer sc;
    sc.page_offset = 100;  // absurdly high
    // pool=5, slots=3 → max start=2
    int si = sc.start_index(5, 3);
    CHECK(si == 2);
    int ei = sc.end_index(5, 3);
    CHECK(ei == 5);
  }

  TEST_CASE("single-item pool, single slot — always page 0") {
    ScrollContainer sc;
    CHECK(sc.page_count(1, 1) == 1);
    CHECK(!sc.needs_paging(1, 1));
    CHECK(sc.start_index(1, 1) == 0);
    CHECK(sc.end_index(1, 1) == 1);
  }

  TEST_CASE("zero slots — no paging, safe indices") {
    ScrollContainer sc;
    CHECK(!sc.needs_paging(5, 0));
    CHECK(sc.page_count(5, 0) == 1);
  }
}

// =====================================================================
// Integration: DisplayDiff + Transition simulated multi-frame sequences
// =====================================================================

TEST_SUITE("Multi-frame integration") {

  /// Simulates the diff→commit→transition lifecycle for a single frame
  struct FrameContext {
    DisplayDiff diff;
    Transition transition;
    HScrollState h_scroll;
    ScrollContainer pinned_pager;
    ScrollContainer unpinned_pager;

    // Simulate frame_pin_inset_ calculation
    int pin_inset(int eff_pinned, bool show_pin_icon) const {
      return (eff_pinned > 0 && show_pin_icon) ? kPinIconWidth : 0;
    }
  };

  TEST_CASE("full lifecycle: idle → pin added → transition → idle") {
    FrameContext ctx;
    std::vector<Trip> trips = {make_trip("R1", "A", 100), make_trip("R2", "B", 200)};
    std::set<std::string> no_pins;

    // Frame 0: no pins, commit
    {
      std::vector<std::string> keys = {"R1:A", "R2:B"};
      std::vector<time_t> deps = {100, 200};
      std::vector<bool> is_p = {false, false};

      ctx.diff.compute(keys, deps, 0, no_pins);
      CHECK(!ctx.diff.has_changes());  // first frame
      ctx.diff.commit(keys, deps, trips, is_p, 0, no_pins);
    }

    // Frame 1: pin R1 — should detect layout change
    std::set<std::string> pin_r1 = {"R1:A"};
    {
      std::vector<std::string> keys = {"R1:A", "R2:B"};
      std::vector<time_t> deps = {100, 200};

      ctx.diff.compute(keys, deps, 1, pin_r1);
      CHECK(ctx.diff.layout_changed);

      // Start transition from old state
      ctx.transition.reset();
      ctx.transition.old_trips = ctx.diff.prev_trips;
      ctx.transition.old_is_pinned = ctx.diff.prev_is_pinned;
      ctx.transition.old_eff_pinned = ctx.diff.prev_pinned_count;
      ctx.transition.stagger_rows = (int)ctx.diff.prev_trips.size();
      ctx.transition.phase = Transition::COLLAPSE;
      ctx.transition.phase_start = 1000;

      CHECK(ctx.transition.old_eff_pinned == 0);
      CHECK(ctx.transition.phase == Transition::COLLAPSE);
      // Should NOT commit during transition
    }

    // Frame 2..N: still in COLLAPSE — diff.compute should still detect change
    // but we don't act on it (transition is not IDLE)
    {
      CHECK(ctx.transition.phase != Transition::IDLE);
      // Verify old snapshot is still from frame 0
      CHECK(ctx.transition.old_trips[0].route_id == "R1");
    }

    // Simulate collapse completing
    {
      unsigned long elapsed = ctx.transition.collapse_duration_ms();
      // Advance to MID_PAUSE
      ctx.transition.phase = Transition::MID_PAUSE;
      ctx.transition.phase_start = 1000 + elapsed;
    }

    // Simulate expand completing
    {
      ctx.transition.phase = Transition::EXPAND;
      ctx.transition.phase_start = 2500;
    }

    // Simulate POST_PAUSE completing → back to IDLE
    {
      ctx.transition.phase = Transition::POST_PAUSE;
      ctx.transition.phase_start = 4000;
      // After pause duration → reset
      ctx.transition.reset();
      CHECK(ctx.transition.phase == Transition::IDLE);

      // Now commit current state
      std::vector<std::string> keys = {"R1:A", "R2:B"};
      std::vector<time_t> deps = {100, 200};
      std::vector<bool> is_p = {true, false};
      ctx.diff.commit(keys, deps, trips, is_p, 1, pin_r1);
      CHECK(ctx.diff.prev_pinned_count == 1);
    }
  }

  TEST_CASE("pin change during COLLAPSE — reset and restart") {
    FrameContext ctx;

    // Initial state: R1 pinned
    std::vector<Trip> trips_v1 = {make_trip("R1", "A", 100), make_trip("R2", "B", 200)};
    std::set<std::string> pin_r1 = {"R1:A"};
    {
      std::vector<std::string> keys = {"R1:A", "R2:B"};
      std::vector<time_t> deps = {100, 200};
      std::vector<bool> is_p = {true, false};
      ctx.diff.commit(keys, deps, trips_v1, is_p, 1, pin_r1);
    }

    // Remove pin → triggers transition
    std::set<std::string> no_pins;
    {
      std::vector<std::string> keys = {"R1:A", "R2:B"};
      std::vector<time_t> deps = {100, 200};
      ctx.diff.compute(keys, deps, 0, no_pins);
      CHECK(ctx.diff.layout_changed);

      ctx.transition.reset();
      ctx.transition.old_trips = ctx.diff.prev_trips;
      ctx.transition.old_is_pinned = ctx.diff.prev_is_pinned;
      ctx.transition.old_eff_pinned = 1;
      ctx.transition.stagger_rows = 2;
      ctx.transition.phase = Transition::COLLAPSE;
      ctx.transition.phase_start = 1000;
    }

    // Mid-collapse (250ms in) — add a DIFFERENT pin
    {
      CHECK(ctx.transition.phase == Transition::COLLAPSE);
      float row0_scale = ctx.transition.row_scale(0, 250, false);
      CHECK(row0_scale > 0.0f);  // partially collapsed
      CHECK(row0_scale < 1.0f);

      // New pin change arrives — must reset transition
      std::set<std::string> pin_r2 = {"R2:B"};

      // Snapshot the partially-collapsed state for the new transition
      // In real code, diff.prev_trips still holds the last committed state
      CHECK(ctx.diff.prev_pinned_count == 1);
      CHECK(ctx.diff.prev_trips.size() == 2);

      ctx.transition.reset();
      CHECK(ctx.transition.phase == Transition::IDLE);

      // Start new transition
      ctx.transition.old_trips = ctx.diff.prev_trips;
      ctx.transition.old_is_pinned = ctx.diff.prev_is_pinned;
      ctx.transition.old_eff_pinned = ctx.diff.prev_pinned_count;
      ctx.transition.stagger_rows = 2;
      ctx.transition.phase = Transition::COLLAPSE;
      ctx.transition.phase_start = 1250;
      ctx.transition.animate_pinned = true;
      ctx.transition.animate_unpinned = true;

      CHECK(ctx.transition.old_eff_pinned == 1);
    }
  }

  TEST_CASE("pin change during EXPAND — reset and restart from current state") {
    FrameContext ctx;

    std::vector<Trip> trips = {make_trip("R1", "A", 100), make_trip("R2", "B", 200), make_trip("R3", "C", 300)};
    std::set<std::string> no_pins;
    {
      std::vector<std::string> keys = {"R1:A", "R2:B", "R3:C"};
      std::vector<time_t> deps = {100, 200, 300};
      std::vector<bool> is_p = {false, false, false};
      ctx.diff.commit(keys, deps, trips, is_p, 0, no_pins);
    }

    // Add pin — start transition
    std::set<std::string> pin_r1 = {"R1:A"};
    {
      ctx.transition.reset();
      ctx.transition.old_trips = trips;
      ctx.transition.old_is_pinned = {false, false, false};
      ctx.transition.old_eff_pinned = 0;
      ctx.transition.stagger_rows = 3;
      ctx.transition.phase = Transition::COLLAPSE;
      ctx.transition.phase_start = 0;
    }

    // Advance through collapse → mid_pause → expand
    {
      ctx.transition.phase = Transition::EXPAND;
      ctx.transition.phase_start = 2000;
      ctx.transition.stagger_rows = 3;
    }

    // Mid-expand: rows are partially grown
    {
      float scale0 = ctx.transition.row_scale(0, 300, true);
      CHECK(scale0 > 0.0f);
      CHECK(scale0 < 1.0f);

      // Second pin change arrives mid-expand — reset
      ctx.transition.reset();
      CHECK(ctx.transition.phase == Transition::IDLE);
      CHECK(ctx.transition.old_trips.empty());
    }
  }

  TEST_CASE("pin inset consistency: old layout should use old eff_pinned") {
    // This tests the logic our fix implements: when transition is rendering
    // old content, the pin inset used for measurement should match old state.
    FrameContext ctx;

    // Scenario 1: had pins (inset=7), removing last pin
    int old_inset = ctx.pin_inset(1, true);  // 7
    int new_inset = ctx.pin_inset(0, true);  // 0
    CHECK(old_inset == kPinIconWidth);
    CHECK(new_inset == 0);
    CHECK(old_inset != new_inset);

    // Scenario 2: had no pins, adding first pin
    old_inset = ctx.pin_inset(0, true);  // 0
    new_inset = ctx.pin_inset(1, true);  // 7
    CHECK(old_inset == 0);
    CHECK(new_inset == kPinIconWidth);
    CHECK(old_inset != new_inset);

    // Scenario 3: show_pin_icon=false → inset always 0
    CHECK(ctx.pin_inset(1, false) == 0);
    CHECK(ctx.pin_inset(0, false) == 0);
  }

  TEST_CASE("h-scroll total_distance preserved when measuring with old trips") {
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(1);

    // Measure with initial distance
    hs.update_distance(80, true);  // total=100
    hs.compute(501);  // elapsed=500 → 50px
    int offset_before = hs.offset;
    CHECK(offset_before == 50);

    // If we re-measure with the SAME width, offset stays consistent
    hs.update_distance(80, true);  // still total=100
    hs.compute(501);
    CHECK(hs.offset == offset_before);

    // If width changes (wrong inset used), after reset the first cycle
    // has no dwell, so the scroll wraps into the next cycle.
    hs.update_distance(73, true);  // total=93 (bug scenario)
    hs.compute(1001);  // elapsed=1000, scroll_phase=930 → first cycle ends,
                       // enters second cycle at offset=7
    CHECK(hs.offset == 7);
  }
}

// =====================================================================
// Respect Pin Inset — x_start and transition tests
// =====================================================================

TEST_SUITE("Respect Pin Inset") {

  // Helper: simulate the x_start logic from draw_trip
  static int compute_x_start(int frame_pin_inset, bool is_pinned, bool respect_pin_inset) {
    int x_start = frame_pin_inset;
    if (!is_pinned && !respect_pin_inset && x_start > 0) {
      x_start = 1;  // 1px margin when pin icon column is visible
    }
    return x_start;
  }

  TEST_CASE("respect=true: all rows use pin inset") {
    int inset = kPinIconWidth;
    CHECK(compute_x_start(inset, true, true) == kPinIconWidth);
    CHECK(compute_x_start(inset, false, true) == kPinIconWidth);
  }

  TEST_CASE("respect=false: pinned rows keep inset, unpinned get 1px margin") {
    int inset = kPinIconWidth;
    CHECK(compute_x_start(inset, true, false) == kPinIconWidth);
    CHECK(compute_x_start(inset, false, false) == 1);
  }

  TEST_CASE("respect=false with no pins: all rows at 0 (no-op)") {
    int inset = 0;  // no pins active
    CHECK(compute_x_start(inset, false, false) == 0);
    CHECK(compute_x_start(inset, true, false) == 0);
  }

  TEST_CASE("transition stores old_respect_pin_inset for collapse") {
    Transition tr;
    CHECK(tr.old_respect_pin_inset == true);  // default

    // Simulate storing old state when starting transition
    tr.old_respect_pin_inset = true;  // was respecting inset
    tr.old_eff_pinned = 1;
    tr.old_trips = {make_trip("R1", "A"), make_trip("R2", "B")};
    tr.old_is_pinned = {true, false};
    tr.phase = Transition::COLLAPSE;
    tr.phase_start = 1000;
    tr.stagger_rows = 2;

    // During COLLAPSE: old_respect_pin_inset should be used
    // Pinned row keeps inset, unpinned row keeps inset (old respect=true)
    CHECK(compute_x_start(kPinIconWidth, true, tr.old_respect_pin_inset) == kPinIconWidth);
    CHECK(compute_x_start(kPinIconWidth, false, tr.old_respect_pin_inset) == kPinIconWidth);

    // After transition, new respect=false would be active
    bool new_respect = false;
    CHECK(compute_x_start(kPinIconWidth, true, new_respect) == kPinIconWidth);
    CHECK(compute_x_start(kPinIconWidth, false, new_respect) == 1);
  }

  TEST_CASE("respect_pin_inset change detected as layout change") {
    // Simulates the detection logic in draw_schedule step 12
    DisplayDiff diff;
    std::vector<Trip> trips = {make_trip("R1", "A", 100), make_trip("R2", "B", 200)};
    std::set<std::string> pin_r1 = {"R1:A"};

    // Commit initial frame with 1 pin, respect=true
    std::vector<std::string> keys = {"R1:A", "R2:B"};
    std::vector<time_t> deps = {100, 200};
    std::vector<bool> is_p = {true, false};
    int eff_pinned = 1;
    diff.commit(keys, deps, trips, is_p, eff_pinned, pin_r1);

    bool committed_respect = true;
    bool desired_respect = false;  // user changed it

    // diff.compute finds no data change
    diff.compute(keys, deps, eff_pinned, pin_r1);
    CHECK(!diff.has_changes());

    // But respect_inset_changed triggers a transition
    bool respect_inset_changed = (desired_respect != committed_respect) && eff_pinned > 0;
    CHECK(respect_inset_changed);

    // Combined condition triggers transition
    bool should_transition = (diff.has_changes() || respect_inset_changed) && diff.prev_pinned_count >= 0;
    CHECK(should_transition);
  }

  TEST_CASE("respect change with no pins does not trigger transition") {
    DisplayDiff diff;
    std::vector<Trip> trips = {make_trip("R1", "A", 100), make_trip("R2", "B", 200)};
    std::set<std::string> no_pins;

    std::vector<std::string> keys = {"R1:A", "R2:B"};
    std::vector<time_t> deps = {100, 200};
    std::vector<bool> is_p = {false, false};
    diff.commit(keys, deps, trips, is_p, 0, no_pins);

    bool committed_respect = true;
    bool desired_respect = false;
    int eff_pinned = 0;

    diff.compute(keys, deps, eff_pinned, no_pins);
    CHECK(!diff.has_changes());

    bool respect_inset_changed = (desired_respect != committed_respect) && eff_pinned > 0;
    CHECK(!respect_inset_changed);  // no pins → no visual effect → no transition
  }

  TEST_CASE("simultaneous pin add and respect change triggers single transition") {
    DisplayDiff diff;
    std::vector<Trip> trips = {make_trip("R1", "A", 100), make_trip("R2", "B", 200)};
    std::set<std::string> no_pins;

    // Frame 0: no pins
    std::vector<std::string> keys = {"R1:A", "R2:B"};
    std::vector<time_t> deps = {100, 200};
    std::vector<bool> is_p = {false, false};
    diff.commit(keys, deps, trips, is_p, 0, no_pins);

    bool committed_respect = true;
    bool desired_respect = false;

    // Frame 1: pin added AND respect changed
    std::set<std::string> pin_r1 = {"R1:A"};
    int new_eff_pinned = 1;
    diff.compute(keys, deps, new_eff_pinned, pin_r1);
    CHECK(diff.layout_changed);  // pin count 0→1

    bool respect_inset_changed = (desired_respect != committed_respect) && new_eff_pinned > 0;
    CHECK(respect_inset_changed);

    // Both conditions true → single transition handles both
    bool layout_swap = diff.layout_changed || diff.pinned_set_changed || respect_inset_changed;
    CHECK(layout_swap);
  }

  TEST_CASE("full lifecycle: respect=true → false with active pins") {
    DisplayDiff diff;
    Transition transition;

    std::vector<Trip> trips = {make_trip("R1", "A", 100), make_trip("R2", "B", 200)};
    std::set<std::string> pin_r1 = {"R1:A"};

    // Frame 0: pins active, respect=true, commit
    std::vector<std::string> keys = {"R1:A", "R2:B"};
    std::vector<time_t> deps = {100, 200};
    std::vector<bool> is_p = {true, false};
    int eff_pinned = 1;
    bool committed_respect = true;
    bool desired_respect = true;
    diff.commit(keys, deps, trips, is_p, eff_pinned, pin_r1);

    // Frame 1: user sets respect=false
    desired_respect = false;
    diff.compute(keys, deps, eff_pinned, pin_r1);
    CHECK(!diff.has_changes());  // no data change

    bool respect_inset_changed = (desired_respect != committed_respect) && eff_pinned > 0;
    CHECK(respect_inset_changed);

    // Start transition
    transition.reset();
    transition.old_trips = diff.prev_trips;
    transition.old_is_pinned = diff.prev_is_pinned;
    transition.old_eff_pinned = diff.prev_pinned_count;
    transition.old_respect_pin_inset = committed_respect;  // true
    transition.stagger_rows = (int)diff.prev_trips.size();
    transition.phase = Transition::COLLAPSE;
    transition.phase_start = 1000;

    // During COLLAPSE: old respect is used
    CHECK(transition.old_respect_pin_inset == true);
    CHECK(compute_x_start(kPinIconWidth, false, transition.old_respect_pin_inset) == kPinIconWidth);

    // Simulate transition completing
    transition.reset();
    committed_respect = desired_respect;  // now false

    // After transition: new respect is active
    CHECK(compute_x_start(kPinIconWidth, false, committed_respect) == 1);
    CHECK(compute_x_start(kPinIconWidth, true, committed_respect) == kPinIconWidth);
  }
}
