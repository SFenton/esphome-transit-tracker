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

    // Standard cycle: scroll_phase=1000ms, dwell=3000ms, total=4000ms.
    // t=501: elapsed=500, offset=50, cycle_count=0 (in first scroll)
    hs.compute(501);
    CHECK(hs.offset == 50);
    CHECK(hs.cycle_count == 0);

    // t=1501: elapsed=1500, in first dwell, offset=0, cycle_count=1
    hs.compute(1501);
    CHECK(hs.offset == 0);
    CHECK(hs.cycle_count == 1);
    CHECK(hs.idle);

    // t=4501: elapsed=4500, in second scroll (4000+500), offset=50, cycle_count=1
    hs.compute(4501);
    CHECK(hs.offset == 50);
    CHECK(hs.cycle_count == 1);
  }

  TEST_CASE("cycle_count increments at first dwell") {
    // Verifies that cycle_count does not increment during the scroll phase
    // and only increments when entering the dwell phase.
    HScrollState hs;
    hs.scroll_speed = 100;  // 100px/s
    hs.reset(1);
    hs.update_distance(80, true);  // total=100, scroll_phase=1000ms

    // During first scroll phase: cycle_count = 0
    hs.compute(501);  // 500ms into scroll
    CHECK(hs.cycle_count == 0);
    CHECK(hs.offset == 50);
    CHECK(!hs.idle);

    // In first dwell: cycle_count = 1
    // Dwell starts at t=1001 (elapsed=1000 >= scroll_phase=1000)
    hs.compute(1501);  // 500ms into first dwell
    CHECK(hs.cycle_count == 1);
    CHECK(hs.offset == 0);
    CHECK(hs.idle);

    // In second scroll: cycle_count still 1
    // Second cycle starts at elapsed=4000 (4000ms cycle), i.e. t=4001
    hs.compute(4501);  // 500ms into second scroll
    CHECK(hs.cycle_count == 1);
    CHECK(hs.offset == 50);
    CHECK(!hs.idle);
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

    // Set pending speed AFTER the first compute (so offset > 0)
    hs.compute(201);  // elapsed=200 => 20px, not near zero
    CHECK(hs.offset == 20);
    hs.pending_speed = 200;

    // Mid-scroll — pending not applied (offset > 0)
    hs.compute(501);
    CHECK(hs.scroll_speed == 100);

    // At dwell (cycle boundary) — applied
    hs.compute(1501);  // in first dwell
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

    diff.commit(keys, deps, trips, is_p, is_p, 0, pinned);
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

    diff.commit(keys, deps, trips, is_p, is_p, 0, pinned);

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

    diff.commit(keys, deps, trips, is_p, is_p, 1, pinned_v1);

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

    diff.commit(keys1, deps1, trips, is_p, is_p, 0, pinned);

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

    diff.commit(keys, deps1, trips, is_p, is_p, 0, pinned);

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

    diff.commit(keys, deps1, trips, is_p, is_p, 0, pinned);

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
    diff.commit(keys, deps, trips, is_p, is_p, pinned_count, pinned_routes);
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

    // Scroll to elapsed=750ms, 75px into first scroll phase
    hs.compute(751);
    CHECK(hs.offset == 75);

    // Now simulate pin change making headsign 7px narrower
    hs.update_distance(73, true);
    CHECK(hs.total_distance == 93);

    // At elapsed=1000ms with new dist 93:
    // scroll_phase_ms = 930. Elapsed=1000 >= 930, in dwell.
    hs.compute(1001);
    CHECK(hs.offset == 0);
    CHECK(hs.idle);
  }

  TEST_CASE("stable total_distance means smooth scrolling across cycles") {
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(0);
    hs.update_distance(80, true);  // total=100

    // Sample many frames — offset should increase monotonically within each
    // scroll phase, and be 0 during dwell.
    int prev_offset = -1;
    int prev_cycle = 0;
    for (unsigned long t = 0; t <= 10000; t += 50) {
      hs.compute(t);
      if (hs.cycle_count == prev_cycle && !hs.idle) {
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
// HScrollState — page-rotation regression
//
// Regression tests for the bug where page pagers triggered immediately
// after every transition, causing an infinite rapid-fire collapse/expand
// loop. The root cause was skip_first_dwell incrementing cycle_count at
// its boundary. The fix removed skip_first_dwell entirely so that all
// cycles are standard (scroll + dwell) from the start.
// =====================================================================

TEST_SUITE("HScrollState — page-rotation regression") {

  TEST_CASE("cycle_count stays 0 through first scroll phase") {
    HScrollState hs;
    hs.scroll_speed = 30;  // realistic speed: 30px/s
    hs.reset(1);
    hs.update_distance(134, true);  // total = 154, scroll_phase = 5133ms

    // Sample throughout the first scroll phase — cycle_count must be 0
    for (unsigned long t = 1; t < 5133; t += 200) {
      hs.compute(t);
      CHECK(hs.cycle_count == 0);
    }
    CHECK(!hs.idle);
  }

  TEST_CASE("cycle_count increments to 1 at first dwell") {
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(1);
    hs.update_distance(80, true);  // total=100, scroll_phase=1000ms

    // Just before dwell
    hs.compute(990);
    CHECK(hs.cycle_count == 0);
    CHECK(!hs.idle);

    // Just into dwell (elapsed=1000 >= scroll_phase=1000)
    hs.compute(1050);
    CHECK(hs.cycle_count == 1);
    CHECK(hs.idle);
    CHECK(hs.offset == 0);
  }

  TEST_CASE("simulated page pager fires at first dwell, not before") {
    // Simulates the exact condition that caused the infinite loop:
    // pager.last_cycle_count = 0 (set at POST_PAUSE→IDLE),
    // then h-scroll starts.  Page must NOT trigger until the first dwell.
    HScrollState hs;
    hs.scroll_speed = 30;  // 30px/s (realistic)
    hs.reset(1);
    hs.update_distance(134, true);  // total=154, scroll_phase=5133ms

    int pager_last_cycle_count = 0;
    bool page_triggered_during_scroll = false;
    bool page_triggered_at_dwell = false;

    // scroll: 1..5134, dwell: 5134..8134
    for (unsigned long t = 1; t <= 9000; t += 50) {
      hs.compute(t);
      bool page_due = hs.idle && hs.cycle_count > pager_last_cycle_count;

      if (page_due) {
        if (t <= 5134) {
          page_triggered_during_scroll = true;
        } else {
          page_triggered_at_dwell = true;
          break;
        }
      }
    }

    CHECK(!page_triggered_during_scroll);
    CHECK(page_triggered_at_dwell);
  }

  TEST_CASE("cycle_count accumulates correctly across multiple cycles") {
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(1);
    hs.update_distance(80, true);  // total=100

    // Standard cycles (scroll+dwell): each 1000+3000 = 4000ms
    //   cycle 0: scroll 1..1001, dwell 1001..4001 → cycle_count=1
    //   cycle 1: scroll 4001..5001, dwell 5001..8001 → cycle_count=2
    //   cycle 2: scroll 8001..9001, dwell 9001..12001 → cycle_count=3

    hs.compute(1500);  // first dwell
    CHECK(hs.cycle_count == 1);

    hs.compute(4500);  // second scroll
    CHECK(hs.cycle_count == 1);

    hs.compute(5500);  // second dwell
    CHECK(hs.cycle_count == 2);

    hs.compute(9500);  // third dwell
    CHECK(hs.cycle_count == 3);
  }

  TEST_CASE("no double-scroll after transition") {
    // Verifies that after reset, the first scroll phase transitions
    // to a dwell (not another scroll), preventing the old double-scroll
    // artifact that skip_first_dwell caused.
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(1);
    hs.update_distance(80, true);  // total=100, scroll_phase=1000ms

    // End of first scroll (offset near max)
    hs.compute(950);
    CHECK(hs.offset >= 90);
    CHECK(hs.cycle_count == 0);

    // Immediately after scroll_phase, we're in dwell (not another scroll)
    hs.compute(1100);
    CHECK(hs.offset == 0);   // at home position
    CHECK(hs.idle);           // in dwell
    CHECK(hs.cycle_count == 1);
  }

  TEST_CASE("wind-down clears at first dwell") {
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(1);
    hs.update_distance(80, true);  // total=100

    // Scroll halfway
    hs.compute(501);
    CHECK(hs.offset == 50);

    // Remove overflow (triggers wind-down)
    hs.update_distance(0, false);
    CHECK(hs.total_distance == 0);
    CHECK(hs.prev_total_distance == 100);

    // At dwell: wind-down clears prev_total_distance
    hs.compute(1501);  // in first dwell
    CHECK(hs.idle);
    CHECK(hs.prev_total_distance == 0);
  }

  TEST_CASE("pending speed applied at home position before first scroll") {
    // On reboot, configured speed arrives as pending_speed after reset.
    // It should be applied before the first scroll begins.
    HScrollState hs;
    hs.scroll_speed = 10;  // default
    hs.reset(1);
    hs.update_distance(80, true);  // total=100
    hs.pending_speed = 30;

    // First compute: offset==0, so pending_speed applied immediately
    hs.compute(1);
    CHECK(hs.scroll_speed == 30);
    CHECK(hs.pending_speed == 0);

    // Subsequent scrolling uses the correct speed
    hs.compute(501);  // elapsed=500ms at 30px/s = 15px
    CHECK(hs.offset == 15);
  }

  TEST_CASE("pending speed not applied mid-scroll") {
    HScrollState hs;
    hs.scroll_speed = 100;
    hs.reset(1);
    hs.update_distance(80, true);

    // Start scrolling
    hs.compute(201);  // offset=20
    CHECK(hs.offset == 20);

    // Set pending speed mid-scroll
    hs.pending_speed = 50;

    // Next compute: offset > 0, so pending_speed NOT applied
    hs.compute(301);
    CHECK(hs.scroll_speed == 100);  // unchanged
    CHECK(hs.pending_speed == 50);  // still pending

    // Applied at dwell
    hs.compute(1501);
    CHECK(hs.scroll_speed == 50);
  }
}

// =====================================================================
// HScrollState — deferred start
// =====================================================================

TEST_SUITE("HScrollState — deferred start") {
  TEST_CASE("first overflow on boot defers with 3s dwell") {
    // On boot, the very first schedule arrival causes overflow.
    // This triggers a 3s deferred start (kScrollCycleDwellMs) so the
    // user can read the initial display before scrolling begins.
    HScrollState hs;
    hs.scroll_speed = 100;
    // No reset() called — simulating fresh struct as component would see it
    // before any transition has occurred.
    hs.compute(1);  // init start_time

    // First overflow appears (first schedule arrives)
    hs.update_distance(80, true);  // total=100
    CHECK(hs.total_distance == 100);
    CHECK(hs.deferred_start);  // deferred — 3s dwell before scrolling

    // First compute after deferred_start sets deferred_since.
    // compute(100): deferred_since=100, dwell ends at 100+3000=3100
    hs.compute(100);
    CHECK(hs.offset == 0);
    CHECK(hs.idle);

    // Still dwelling at t=3099
    hs.compute(3099);
    CHECK(hs.offset == 0);
    CHECK(hs.idle);
    CHECK(hs.deferred_start);

    // Dwell ends at t=3100
    hs.compute(3100);
    CHECK(!hs.deferred_start);
    CHECK(hs.offset == 0);  // start_time was just set

    // Scrolling begins
    hs.compute(3200);
    CHECK(hs.offset == 10);  // 100ms at 100px/s
    CHECK(!hs.idle);
  }

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
    // Dwell duration = kScrollCycleDwellMs = 3000ms
    // So dwell ends at 5100 + 3000 = 8100
    hs.compute(5100);
    CHECK(hs.offset == 0);
    CHECK(hs.idle);

    hs.compute(5500);
    CHECK(hs.offset == 0);
    CHECK(hs.idle);

    // Still dwelling at t=8099 (just before dwell ends)
    hs.compute(8099);
    CHECK(hs.offset == 0);
    CHECK(hs.idle);
    CHECK(hs.deferred_start);

    // At t=8100, dwell ends → scrolling begins from offset 0
    hs.compute(8100);
    CHECK(!hs.deferred_start);
    CHECK(hs.offset == 0);  // start_time was just set to 8100

    // At t=8200, scrolling is underway: 100ms * 100px/s = 10px
    hs.compute(8200);
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

  TEST_CASE("dwell duration is always 3s (kScrollCycleDwellMs)") {
    HScrollState hs;
    hs.scroll_speed = 50;  // slower — but dwell is fixed 3s regardless
    hs.compute(1);  // init start_time
    hs.update_distance(0, false);
    hs.compute(1000);

    hs.update_distance(80, true);  // total=100
    CHECK(hs.deferred_start);

    // First compute sets deferred_since=1100
    // Dwell = kScrollCycleDwellMs = 3000ms → ends at 1100+3000=4100
    hs.compute(1100);
    CHECK(hs.deferred_start);
    CHECK(hs.offset == 0);

    hs.compute(4099);
    CHECK(hs.deferred_start);

    hs.compute(4100);
    CHECK(!hs.deferred_start);
    CHECK(hs.offset == 0);  // start_time just set to 4100

    // Now scrolling at 50px/s: at t=4300, elapsed=200ms → 10px
    hs.compute(4300);
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

    // First compute sets deferred_since=1100, dwell=3000ms, ends at 4100
    hs.compute(1100);
    CHECK(hs.deferred_start);
    hs.compute(4100);
    CHECK(!hs.deferred_start);

    // Scroll phase completes at 4100+1000=5100 (enters dwell)
    // start_time was set to 4100, scroll_phase_ms=1000, cycle_total_ms=4000
    hs.compute(5100);
    CHECK(hs.idle);

    // Remove overflow — wind-down
    hs.update_distance(0, false);
    // In dwell phase — wind-down clears prev_total_distance immediately
    hs.compute(5101);
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
      ctx.diff.commit(keys, deps, trips, is_p, is_p, 0, no_pins);
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
      ctx.diff.commit(keys, deps, trips, is_p, is_p, 1, pin_r1);
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
      ctx.diff.commit(keys, deps, trips_v1, is_p, is_p, 1, pin_r1);
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
      ctx.diff.commit(keys, deps, trips, is_p, is_p, 0, no_pins);
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

    // If width changes (wrong inset used), it shifts scroll_phase_ms.
    // With standard cycles, the change takes effect immediately.
    hs.update_distance(73, true);  // total=93 (bug scenario)
    hs.compute(1001);  // elapsed=1000, scroll_phase=930 → in dwell (idle)
    CHECK(hs.offset == 0);
    CHECK(hs.idle);
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
    diff.commit(keys, deps, trips, is_p, is_p, eff_pinned, pin_r1);

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
    diff.commit(keys, deps, trips, is_p, is_p, 0, no_pins);

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
    diff.commit(keys, deps, trips, is_p, is_p, 0, no_pins);

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
    diff.commit(keys, deps, trips, is_p, is_p, eff_pinned, pin_r1);

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

// =====================================================================
// Always Scroll or Replace — idle self-replace detection
// =====================================================================

TEST_SUITE("Always Scroll or Replace") {

  // Mirrors the idle detection logic from draw_schedule step 12.
  // Returns true when a self-replace transition should fire.
  struct IdleReplaceState {
    bool always_scroll_or_replace{false};
    int page_interval{5000};  // ms
    unsigned long idle_since{0};

    // Returns true when a self-replace should fire.
    // Call after normal change-detection / paging logic runs.
    bool check(unsigned long now, bool triggered_transition,
               bool h_scroll_active, bool post_pause_active,
               int prev_pinned_count) {
      if (triggered_transition) {
        idle_since = 0;
        return false;
      }
      if (!always_scroll_or_replace || prev_pinned_count < 0)
        return false;

      if (!h_scroll_active && !post_pause_active) {
        if (idle_since == 0) {
          idle_since = now;
          return false;
        }
        if (now - idle_since >= (unsigned long)page_interval) {
          idle_since = 0;
          return true;
        }
        return false;
      }
      idle_since = 0;
      return false;
    }
  };

  TEST_CASE("disabled by default — never fires") {
    IdleReplaceState state;
    state.always_scroll_or_replace = false;
    state.page_interval = 5000;

    for (unsigned long t = 0; t <= 20000; t += 100) {
      CHECK(!state.check(t, false, false, false, 0));
    }
  }

  TEST_CASE("fires after page_interval when idle") {
    IdleReplaceState state;
    state.always_scroll_or_replace = true;
    state.page_interval = 5000;

    // First call starts the timer
    CHECK(!state.check(1000, false, false, false, 0));
    CHECK(state.idle_since == 1000);

    // Not enough time yet
    CHECK(!state.check(3000, false, false, false, 0));

    // Just before threshold
    CHECK(!state.check(5999, false, false, false, 0));

    // At threshold — fires!
    CHECK(state.check(6000, false, false, false, 0));
    CHECK(state.idle_since == 0);  // reset after firing
  }

  TEST_CASE("does not fire on first frame (prev_pinned_count < 0)") {
    IdleReplaceState state;
    state.always_scroll_or_replace = true;
    state.page_interval = 5000;

    for (unsigned long t = 0; t <= 10000; t += 100) {
      CHECK(!state.check(t, false, false, false, -1));
    }
    CHECK(state.idle_since == 0);
  }

  TEST_CASE("h-scroll active prevents idle timer") {
    IdleReplaceState state;
    state.always_scroll_or_replace = true;
    state.page_interval = 5000;

    // h-scroll active — timer doesn't start
    CHECK(!state.check(1000, false, true, false, 0));
    CHECK(state.idle_since == 0);

    CHECK(!state.check(10000, false, true, false, 0));
    CHECK(state.idle_since == 0);
  }

  TEST_CASE("post-pause active prevents idle timer") {
    IdleReplaceState state;
    state.always_scroll_or_replace = true;
    state.page_interval = 5000;

    CHECK(!state.check(1000, false, false, true, 0));
    CHECK(state.idle_since == 0);

    CHECK(!state.check(10000, false, false, true, 0));
    CHECK(state.idle_since == 0);
  }

  TEST_CASE("triggered transition resets idle timer") {
    IdleReplaceState state;
    state.always_scroll_or_replace = true;
    state.page_interval = 5000;

    // Start idle timer
    CHECK(!state.check(1000, false, false, false, 0));
    CHECK(state.idle_since == 1000);

    // Accumulate some time
    CHECK(!state.check(4000, false, false, false, 0));
    CHECK(state.idle_since == 1000);

    // Data change triggers a transition — idle timer resets
    CHECK(!state.check(4500, true, false, false, 0));
    CHECK(state.idle_since == 0);

    // Timer restarts from scratch after the interruption
    CHECK(!state.check(5000, false, false, false, 0));
    CHECK(state.idle_since == 5000);

    // Full interval from new start
    CHECK(!state.check(9999, false, false, false, 0));
    CHECK(state.check(10000, false, false, false, 0));
  }

  TEST_CASE("h-scroll becoming active mid-idle resets timer") {
    IdleReplaceState state;
    state.always_scroll_or_replace = true;
    state.page_interval = 5000;

    // Start idle timer
    CHECK(!state.check(1000, false, false, false, 0));
    CHECK(state.idle_since == 1000);

    // h-scroll becomes active at 3000 — resets timer
    CHECK(!state.check(3000, false, true, false, 0));
    CHECK(state.idle_since == 0);

    // h-scroll goes idle again — timer restarts
    CHECK(!state.check(4000, false, false, false, 0));
    CHECK(state.idle_since == 4000);

    CHECK(state.check(9000, false, false, false, 0));
  }

  TEST_CASE("fires repeatedly with correct interval spacing") {
    IdleReplaceState state;
    state.always_scroll_or_replace = true;
    state.page_interval = 5000;

    // First fire (use t=1, not 0 — 0 is sentinel for idle_since)
    CHECK(!state.check(1, false, false, false, 0));
    CHECK(state.check(5001, false, false, false, 0));

    // Timer restarted after fire
    CHECK(!state.check(5001, false, false, false, 0));  // re-initializes
    CHECK(!state.check(10000, false, false, false, 0));
    CHECK(state.check(10001, false, false, false, 0));

    // Third fire
    CHECK(!state.check(10001, false, false, false, 0));
    CHECK(state.check(15001, false, false, false, 0));
  }

  TEST_CASE("custom page_interval is respected") {
    IdleReplaceState state;
    state.always_scroll_or_replace = true;
    state.page_interval = 2000;  // shorter

    CHECK(!state.check(1000, false, false, false, 0));
    CHECK(!state.check(2999, false, false, false, 0));
    CHECK(state.check(3000, false, false, false, 0));
  }

  TEST_CASE("works with pinned rows (prev_pinned_count > 0)") {
    IdleReplaceState state;
    state.always_scroll_or_replace = true;
    state.page_interval = 5000;

    CHECK(!state.check(1000, false, false, false, 1));
    CHECK(state.idle_since == 1000);
    CHECK(state.check(6000, false, false, false, 1));
  }

  TEST_CASE("integration: idle → self-replace → post-pause → idle → repeat") {
    // Simulates the full lifecycle: idle timer fires, transition runs
    // (which shows as triggered_transition=true or post_pause_active=true),
    // then back to idle.
    IdleReplaceState state;
    state.always_scroll_or_replace = true;
    state.page_interval = 5000;

    // Phase 1: idle, timer accumulates (use t=1, not 0 — 0 is sentinel)
    CHECK(!state.check(1, false, false, false, 0));
    CHECK(!state.check(4000, false, false, false, 0));

    // Phase 2: self-replace fires at 5001
    CHECK(state.check(5001, false, false, false, 0));

    // Phase 3: transition is active (normally transition_.phase != IDLE,
    // so check() wouldn't be called, but if it were via triggered_transition):
    CHECK(!state.check(5100, true, false, false, 0));
    CHECK(state.idle_since == 0);

    // Phase 4: transition done, post-pause active (timer stays reset)
    CHECK(!state.check(6000, false, false, true, 0));
    CHECK(state.idle_since == 0);

    // Phase 5: post-pause ends, idle timer restarts
    CHECK(!state.check(7000, false, false, false, 0));
    CHECK(state.idle_since == 7000);

    // Phase 6: fires again after page_interval
    CHECK(state.check(12000, false, false, false, 0));
  }

  TEST_CASE("intermittent h-scroll prevents firing") {
    // Headsign overflow comes and goes — idle timer never accumulates enough
    IdleReplaceState state;
    state.always_scroll_or_replace = true;
    state.page_interval = 5000;

    for (unsigned long t = 0; t < 30000; t += 1000) {
      // Alternate: 2s idle, 1s scrolling
      bool scrolling = ((t / 1000) % 3 == 2);
      bool result = state.check(t, false, scrolling, false, 0);
      CHECK(!result);  // never accumulates 5 continuous seconds
    }
  }
}

// =====================================================================
// PinMode enum
// =====================================================================

TEST_SUITE("PinMode") {
  TEST_CASE("enum values are distinct") {
    CHECK(PIN_NONE == 0);
    CHECK(PIN_GENERAL == 1);
    CHECK(PIN_LEAVING_SOON == 2);
    CHECK(PIN_BOTH == 3);
    CHECK(PIN_NONE != PIN_GENERAL);
    CHECK(PIN_GENERAL != PIN_LEAVING_SOON);
    CHECK(PIN_LEAVING_SOON != PIN_BOTH);
  }
}

// =====================================================================
// Pinned Leaving Soon — partition & leaving-soon flag logic
// =====================================================================

// Helper that mimics draw_schedule step 4: partition visible trips into
// pinned-first order, and step 6: compute per-row leaving_soon flags.
// Returns {pinned_pool, unpinned_pool, pinned_leaving_soon_flags}.
struct PartitionResult {
  std::vector<Trip> pinned_pool;
  std::vector<Trip> unpinned_pool;
  std::vector<bool> pinned_leaving_soon;  // one per pinned_pool entry
};

static PartitionResult partition_trips(
    std::vector<Trip> visible,
    const std::map<std::string, PinMode> &pinned_routes,
    unsigned int rtc_now,
    int threshold_min) {

  int threshold_sec = threshold_min * 60;
  int actual_pinned = 0;

  if (!pinned_routes.empty()) {
    std::stable_partition(visible.begin(), visible.end(),
      [&](const Trip &t) {
        auto it = pinned_routes.find(t.composite_key());
        if (it == pinned_routes.end()) return false;
        switch (it->second) {
          case PIN_GENERAL: case PIN_BOTH: return true;
          case PIN_LEAVING_SOON:
            return (int)(t.departure_time - rtc_now) < threshold_sec;
          default: return false;
        }
      });
    for (const auto &t : visible) {
      auto it = pinned_routes.find(t.composite_key());
      if (it == pinned_routes.end()) break;
      bool eff = false;
      switch (it->second) {
        case PIN_GENERAL: case PIN_BOTH: eff = true; break;
        case PIN_LEAVING_SOON:
          eff = (int)(t.departure_time - rtc_now) < threshold_sec; break;
        default: break;
      }
      if (!eff) break;
      actual_pinned++;
    }
    dedup_pinned_section(visible, actual_pinned);
  }

  PartitionResult r;
  r.pinned_pool.assign(visible.begin(), visible.begin() + actual_pinned);
  r.unpinned_pool.assign(visible.begin() + actual_pinned, visible.end());

  for (const auto &t : r.pinned_pool) {
    auto it = pinned_routes.find(t.composite_key());
    bool ls = false;
    if (it != pinned_routes.end()) {
      int secs_until = (int)(t.departure_time - rtc_now);
      switch (it->second) {
        case PIN_LEAVING_SOON: ls = true; break;
        case PIN_BOTH: ls = (secs_until < threshold_sec); break;
        default: break;
      }
    }
    r.pinned_leaving_soon.push_back(ls);
  }
  return r;
}

TEST_SUITE("PinnedLeavingSoon") {

  // -- PIN_GENERAL: always pinned, never leaving-soon (red pin) --

  TEST_CASE("PIN_GENERAL — always in pinned pool regardless of time") {
    std::map<std::string, PinMode> pins = {{"R1:A", PIN_GENERAL}};
    // Trip departing in 30 minutes (1800s from now)
    auto r = partition_trips(
        {make_trip("R1", "A", 2800), make_trip("R2", "B", 2900)},
        pins, /*rtc_now=*/1000, /*threshold_min=*/10);
    CHECK(r.pinned_pool.size() == 1);
    CHECK(r.pinned_pool[0].route_id == "R1");
    CHECK(r.unpinned_pool.size() == 1);
    CHECK(r.unpinned_pool[0].route_id == "R2");
  }

  TEST_CASE("PIN_GENERAL — leaving_soon flag is always false") {
    std::map<std::string, PinMode> pins = {{"R1:A", PIN_GENERAL}};
    // Even when departure is 2 minutes away
    auto r = partition_trips(
        {make_trip("R1", "A", 1120)},
        pins, 1000, 10);
    CHECK(r.pinned_pool.size() == 1);
    CHECK(r.pinned_leaving_soon.size() == 1);
    CHECK(r.pinned_leaving_soon[0] == false);
  }

  // -- PIN_LEAVING_SOON: unpinned until within threshold --

  TEST_CASE("PIN_LEAVING_SOON — outside threshold stays unpinned") {
    std::map<std::string, PinMode> pins = {{"R1:A", PIN_LEAVING_SOON}};
    // Trip departing in 15 min, threshold is 10 min
    auto r = partition_trips(
        {make_trip("R1", "A", 1900), make_trip("R2", "B", 2000)},
        pins, 1000, 10);
    CHECK(r.pinned_pool.size() == 0);
    CHECK(r.unpinned_pool.size() == 2);
    // R1 is first in unpinned (stable order)
    CHECK(r.unpinned_pool[0].route_id == "R1");
  }

  TEST_CASE("PIN_LEAVING_SOON — inside threshold moves to pinned with green flag") {
    std::map<std::string, PinMode> pins = {{"R1:A", PIN_LEAVING_SOON}};
    // Trip departing in 5 min (300s), threshold is 10 min (600s)
    auto r = partition_trips(
        {make_trip("R1", "A", 1300), make_trip("R2", "B", 2000)},
        pins, 1000, 10);
    CHECK(r.pinned_pool.size() == 1);
    CHECK(r.pinned_pool[0].route_id == "R1");
    CHECK(r.pinned_leaving_soon[0] == true);  // green pin
    CHECK(r.unpinned_pool.size() == 1);
    CHECK(r.unpinned_pool[0].route_id == "R2");
  }

  TEST_CASE("PIN_LEAVING_SOON — exactly at threshold boundary") {
    std::map<std::string, PinMode> pins = {{"R1:A", PIN_LEAVING_SOON}};
    // Trip departing in exactly 10 min (600s), threshold is 10 min
    // (int)(1600 - 1000) = 600, and 600 < 600 is false → unpinned
    auto r = partition_trips(
        {make_trip("R1", "A", 1600)},
        pins, 1000, 10);
    CHECK(r.pinned_pool.size() == 0);
    CHECK(r.unpinned_pool.size() == 1);
  }

  TEST_CASE("PIN_LEAVING_SOON — one second under threshold moves to pinned") {
    std::map<std::string, PinMode> pins = {{"R1:A", PIN_LEAVING_SOON}};
    // Trip departing in 9min59s (599s), threshold is 10 min (600s)
    auto r = partition_trips(
        {make_trip("R1", "A", 1599)},
        pins, 1000, 10);
    CHECK(r.pinned_pool.size() == 1);
    CHECK(r.pinned_leaving_soon[0] == true);
  }

  // -- PIN_BOTH: always pinned; green when within threshold, red otherwise --

  TEST_CASE("PIN_BOTH — always in pinned pool, red pin when outside threshold") {
    std::map<std::string, PinMode> pins = {{"R1:A", PIN_BOTH}};
    // Trip departing in 20 min, threshold 10 min
    auto r = partition_trips(
        {make_trip("R1", "A", 2200)},
        pins, 1000, 10);
    CHECK(r.pinned_pool.size() == 1);
    CHECK(r.pinned_leaving_soon[0] == false);  // red pin
  }

  TEST_CASE("PIN_BOTH — always in pinned pool, green pin when inside threshold") {
    std::map<std::string, PinMode> pins = {{"R1:A", PIN_BOTH}};
    // Trip departing in 5 min, threshold 10 min
    auto r = partition_trips(
        {make_trip("R1", "A", 1300)},
        pins, 1000, 10);
    CHECK(r.pinned_pool.size() == 1);
    CHECK(r.pinned_leaving_soon[0] == true);  // green pin
  }

  // -- PIN_BOTH with multiple instances of same route (the "2 Line" scenario) --

  TEST_CASE("PIN_BOTH — mixed: some instances green, some red") {
    // Scenario from the spec: "2 Line" pinned both, threshold = 10 min
    // Departures at 2min, 8min, 13min from now
    std::map<std::string, PinMode> pins = {{"2:Downtown", PIN_BOTH}};
    unsigned int now = 1000;
    std::vector<Trip> trips = {
        make_trip("2", "Downtown", now + 120),   // 2 min
        make_trip("2", "Downtown", now + 480),   // 8 min
        make_trip("2", "Downtown", now + 780),   // 13 min
    };
    auto r = partition_trips(trips, pins, now, 10);

    // All three share the same composite_key. After dedup, only the first
    // (soonest) survives in the pinned pool. The duplicates are discarded
    // (dedup_pinned_section erases them from the vector entirely).
    CHECK(r.pinned_pool.size() == 1);
    CHECK(r.pinned_pool[0].departure_time == now + 120);
    CHECK(r.pinned_leaving_soon[0] == true);  // 2 min is within 10 min
    CHECK(r.unpinned_pool.size() == 0);
  }

  TEST_CASE("PIN_BOTH — distinct stop_ids show mixed green/red pins") {
    // Same route at different stops — composite_keys differ, so all survive dedup.
    // This matches the user scenario: 2 Line at 2min(green), 8min(green), 13min(red).
    std::map<std::string, PinMode> pins = {
        {"2:Downtown:S1", PIN_BOTH},
        {"2:Downtown:S2", PIN_BOTH},
        {"2:Downtown:S3", PIN_BOTH},
    };
    unsigned int now = 1000;
    std::vector<Trip> trips = {
        make_trip("2", "Downtown", now + 120, false, "S1"),   // 2 min
        make_trip("2", "Downtown", now + 480, false, "S2"),   // 8 min
        make_trip("2", "Downtown", now + 780, false, "S3"),   // 13 min
    };
    auto r = partition_trips(trips, pins, now, 10);

    CHECK(r.pinned_pool.size() == 3);
    // 2 min and 8 min are within 10 min threshold → green
    CHECK(r.pinned_leaving_soon[0] == true);
    CHECK(r.pinned_leaving_soon[1] == true);
    // 13 min is outside threshold → red
    CHECK(r.pinned_leaving_soon[2] == false);
  }

  TEST_CASE("PIN_BOTH — multiple distinct routes, mixed pin states") {
    // "2 Line" and "E Line" both PIN_BOTH, threshold 10 min
    std::map<std::string, PinMode> pins = {
        {"2:Downtown", PIN_BOTH},
        {"E:Airport", PIN_BOTH},
    };
    unsigned int now = 1000;
    std::vector<Trip> trips = {
        make_trip("2", "Downtown", now + 120),   // 2 min — green
        make_trip("E", "Airport", now + 900),    // 15 min — red
        make_trip("2", "Downtown", now + 780),   // 13 min — (deduped)
    };
    auto r = partition_trips(trips, pins, now, 10);

    // After dedup: one "2:Downtown" (first) and one "E:Airport"
    CHECK(r.pinned_pool.size() == 2);
    CHECK(r.pinned_pool[0].route_id == "2");
    CHECK(r.pinned_pool[1].route_id == "E");
    CHECK(r.pinned_leaving_soon[0] == true);   // 2 min < 10 min
    CHECK(r.pinned_leaving_soon[1] == false);  // 15 min > 10 min
  }

  // -- Mixed pin modes --

  TEST_CASE("mixed: PIN_GENERAL + PIN_LEAVING_SOON + unpinned") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    std::vector<Trip> trips = {
        make_trip("R1", "A", now + 1200),  // PIN_GENERAL, 20 min
        make_trip("R2", "B", now + 300),   // PIN_LEAVING_SOON, 5 min (within 10)
        make_trip("R3", "C", now + 400),   // unpinned
    };
    auto r = partition_trips(trips, pins, now, 10);

    // R1 (general) and R2 (leaving soon, within threshold) are pinned
    CHECK(r.pinned_pool.size() == 2);
    CHECK(r.unpinned_pool.size() == 1);
    CHECK(r.unpinned_pool[0].route_id == "R3");

    // Find R1 and R2 in pinned pool (order preserved by stable_partition)
    // R1 and R2 are both "effectively pinned", so they stay in their
    // relative order from the visible list
    bool found_r1 = false, found_r2 = false;
    for (size_t i = 0; i < r.pinned_pool.size(); i++) {
      if (r.pinned_pool[i].route_id == "R1") {
        found_r1 = true;
        CHECK(r.pinned_leaving_soon[i] == false);  // PIN_GENERAL → red
      }
      if (r.pinned_pool[i].route_id == "R2") {
        found_r2 = true;
        CHECK(r.pinned_leaving_soon[i] == true);   // PIN_LEAVING_SOON → green
      }
    }
    CHECK(found_r1);
    CHECK(found_r2);
  }

  TEST_CASE("mixed: PIN_LEAVING_SOON outside threshold stays unpinned with PIN_GENERAL") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    std::vector<Trip> trips = {
        make_trip("R1", "A", now + 1200),  // PIN_GENERAL
        make_trip("R2", "B", now + 900),   // PIN_LEAVING_SOON, 15 min (outside 10)
        make_trip("R3", "C", now + 400),   // unpinned
    };
    auto r = partition_trips(trips, pins, now, 10);

    // Only R1 is pinned; R2 is unpinned (outside threshold)
    CHECK(r.pinned_pool.size() == 1);
    CHECK(r.pinned_pool[0].route_id == "R1");
    CHECK(r.pinned_leaving_soon[0] == false);

    CHECK(r.unpinned_pool.size() == 2);
  }

  // -- Threshold variation --

  TEST_CASE("threshold 5 min — trip at 4 min is pinned") {
    std::map<std::string, PinMode> pins = {{"R1:A", PIN_LEAVING_SOON}};
    auto r = partition_trips(
        {make_trip("R1", "A", 1240)},  // 4 min from now
        pins, 1000, 5);
    CHECK(r.pinned_pool.size() == 1);
    CHECK(r.pinned_leaving_soon[0] == true);
  }

  TEST_CASE("threshold 5 min — trip at 6 min stays unpinned") {
    std::map<std::string, PinMode> pins = {{"R1:A", PIN_LEAVING_SOON}};
    auto r = partition_trips(
        {make_trip("R1", "A", 1360)},  // 6 min from now
        pins, 1000, 5);
    CHECK(r.pinned_pool.size() == 0);
    CHECK(r.unpinned_pool.size() == 1);
  }

  TEST_CASE("threshold 60 min — trip at 59 min is pinned") {
    std::map<std::string, PinMode> pins = {{"R1:A", PIN_LEAVING_SOON}};
    auto r = partition_trips(
        {make_trip("R1", "A", 4540)},  // 59 min from now
        pins, 1000, 60);
    CHECK(r.pinned_pool.size() == 1);
    CHECK(r.pinned_leaving_soon[0] == true);
  }

  // -- No pinned routes at all --

  TEST_CASE("no pinned routes — all unpinned") {
    std::map<std::string, PinMode> pins;
    auto r = partition_trips(
        {make_trip("R1", "A", 1300), make_trip("R2", "B", 1400)},
        pins, 1000, 10);
    CHECK(r.pinned_pool.size() == 0);
    CHECK(r.unpinned_pool.size() == 2);
  }

  // -- PIN_NONE should not pin --

  TEST_CASE("PIN_NONE — treated as unpinned") {
    std::map<std::string, PinMode> pins = {{"R1:A", PIN_NONE}};
    auto r = partition_trips(
        {make_trip("R1", "A", 1120)},
        pins, 1000, 10);
    CHECK(r.pinned_pool.size() == 0);
    CHECK(r.unpinned_pool.size() == 1);
  }

  // -- DisplayDiff stores leaving_soon correctly --

  TEST_CASE("DisplayDiff commit stores is_leaving_soon") {
    DisplayDiff diff;
    std::vector<std::string> keys = {"R1:A"};
    std::vector<time_t> deps = {1300};
    std::vector<Trip> trips = {make_trip("R1", "A", 1300)};
    std::vector<bool> is_p = {true};
    std::vector<bool> is_ls = {true};
    std::set<std::string> pinned = {"R1:A"};

    diff.commit(keys, deps, trips, is_p, is_ls, 1, pinned);
    CHECK(diff.prev_is_leaving_soon.size() == 1);
    CHECK(diff.prev_is_leaving_soon[0] == true);
  }

  TEST_CASE("DisplayDiff commit stores mixed leaving_soon flags") {
    DisplayDiff diff;
    std::vector<std::string> keys = {"R1:A", "R2:B"};
    std::vector<time_t> deps = {1300, 1900};
    std::vector<Trip> trips = {make_trip("R1", "A", 1300), make_trip("R2", "B", 1900)};
    std::vector<bool> is_p = {true, true};
    std::vector<bool> is_ls = {true, false};  // R1 leaving soon, R2 not
    std::set<std::string> pinned = {"R1:A", "R2:B"};

    diff.commit(keys, deps, trips, is_p, is_ls, 2, pinned);
    REQUIRE(diff.prev_is_leaving_soon.size() == 2);
    CHECK(diff.prev_is_leaving_soon[0] == true);
    CHECK(diff.prev_is_leaving_soon[1] == false);
  }

  // -- Transition stores leaving_soon --

  TEST_CASE("Transition reset clears old_is_leaving_soon") {
    Transition tr;
    tr.old_is_leaving_soon = {true, false};
    tr.reset();
    CHECK(tr.old_is_leaving_soon.empty());
  }
}

// =====================================================================
// Split-Pin Layout — general vs departure pin allocation
// =====================================================================

// Helper that mirrors draw_schedule steps 4, 4b, 4c: partition visible trips
// into pinned-first order, split into general/departure pools, and compute
// effective row counts for each sub-section.
struct SplitPinResult {
  std::vector<Trip> general_pool;    // PIN_GENERAL, or PIN_BOTH when NOT leaving soon
  std::vector<Trip> departure_pool;  // PIN_LEAVING_SOON, or PIN_BOTH when leaving soon
  std::vector<Trip> pinned_pool;     // combined: general first, then departure
  std::vector<Trip> unpinned_pool;
  std::vector<Trip> demoted_to_unpinned;
  int actual_pinned;
  int eff_pinned;
  int eff_general;
  int eff_departure;
  int eff_unpinned;
  bool has_pin_split;
};

static SplitPinResult split_pin_partition(
    std::vector<Trip> visible,
    const std::map<std::string, PinMode> &pinned_routes,
    unsigned int rtc_now,
    int threshold_min,
    int pinned_rows_count,
    int general_pins_count,
    int limit) {

  int threshold_sec = threshold_min * 60;
  int actual_pinned = 0;

  if (!pinned_routes.empty()) {
    std::stable_partition(visible.begin(), visible.end(),
      [&](const Trip &t) {
        auto it = pinned_routes.find(t.composite_key());
        if (it == pinned_routes.end()) return false;
        switch (it->second) {
          case PIN_GENERAL: case PIN_BOTH: return true;
          case PIN_LEAVING_SOON:
            return (int)(t.departure_time - rtc_now) < threshold_sec;
          default: return false;
        }
      });
    for (const auto &t : visible) {
      auto it = pinned_routes.find(t.composite_key());
      if (it == pinned_routes.end()) break;
      bool eff = false;
      switch (it->second) {
        case PIN_GENERAL: case PIN_BOTH: eff = true; break;
        case PIN_LEAVING_SOON:
          eff = (int)(t.departure_time - rtc_now) < threshold_sec; break;
        default: break;
      }
      if (!eff) break;
      actual_pinned++;
    }
  }
  int original_actual_pinned = actual_pinned;

  // Split into general and departure pools (with inline dedup)
  std::vector<Trip> general_pool;
  std::vector<Trip> departure_pool;
  std::vector<Trip> demoted_to_unpinned;
  {
    std::set<std::string> seen_pinned_keys;
    for (int i = 0; i < actual_pinned; i++) {
      const auto &t = visible[i];
      auto key = t.composite_key();
      auto it = pinned_routes.find(key);
      if (it == pinned_routes.end()) continue;
      bool is_dup = seen_pinned_keys.count(key) > 0;
      seen_pinned_keys.insert(key);
      int secs_until = (int)(t.departure_time - rtc_now);
      bool leaving_soon = (secs_until < threshold_sec);
      if (is_dup) {
        switch (it->second) {
          case PIN_GENERAL:
            general_pool.push_back(t);
            break;
          case PIN_BOTH:
            if (leaving_soon) departure_pool.push_back(t);
            else general_pool.push_back(t);
            break;
          case PIN_LEAVING_SOON:
            if (leaving_soon) departure_pool.push_back(t);  // still within threshold
            else demoted_to_unpinned.push_back(t);
            break;
          default:
            demoted_to_unpinned.push_back(t);
            break;
        }
      } else {
        switch (it->second) {
          case PIN_GENERAL: general_pool.push_back(t); break;
          case PIN_LEAVING_SOON: departure_pool.push_back(t); break;
          case PIN_BOTH:
            if (leaving_soon) departure_pool.push_back(t);
            else general_pool.push_back(t);
            break;
          default: break;
        }
      }
    }
  }
  actual_pinned = (int)(general_pool.size() + departure_pool.size());

  bool has_pin_split = (general_pool.size() > 0 && departure_pool.size() > 0);

  // Will the unpinned pool be empty?
  bool unpinned_pool_empty = (demoted_to_unpinned.empty() &&
                              original_actual_pinned >= (int)visible.size());

  int eff_pinned;
  int eff_general = 0;
  int eff_departure = 0;
  if (actual_pinned > 0) {
    if (unpinned_pool_empty) {
      // No unpinned trips — pins can use ALL rows
      if (has_pin_split) {
        eff_general = std::min(general_pins_count, (int)general_pool.size());
        int dep_slots = limit - eff_general;
        eff_departure = std::min(dep_slots, (int)departure_pool.size());
        int surplus = dep_slots - eff_departure;
        if (surplus > 0) {
          int extra = std::min(surplus, (int)general_pool.size() - eff_general);
          eff_general += extra;
        }
        eff_pinned = eff_general + eff_departure;
      } else {
        // Single pin type — expand to fill all available rows
        eff_pinned = std::min(limit, actual_pinned);
      }
    } else {
      // Unpinned trips exist — pinned_rows_count caps pinned rows as before
      eff_pinned = std::min(pinned_rows_count, actual_pinned);
      if (has_pin_split) {
        if (eff_pinned < 2) eff_pinned = std::min(2, limit - 1);
        int gen_want = std::min(general_pins_count, eff_pinned - 1);
        eff_general = std::min(gen_want, (int)general_pool.size());
        int dep_slots = eff_pinned - eff_general;
        eff_departure = std::min(dep_slots, (int)departure_pool.size());
        int surplus = dep_slots - eff_departure;
        if (surplus > 0) {
          int extra = std::min(surplus, (int)general_pool.size() - eff_general);
          eff_general += extra;
        }
        eff_pinned = eff_general + eff_departure;
      } else {
        // Single pin type — respect pinned_rows_count setting and allow paging
        eff_pinned = std::min(pinned_rows_count, actual_pinned);
      }
    }
  } else {
    eff_pinned = 0;
  }
  int eff_unpinned = limit - eff_pinned;

  // Build combined pool
  std::vector<Trip> pinned_pool;
  for (int i = 0; i < eff_general; i++) pinned_pool.push_back(general_pool[i]);
  for (int i = 0; i < eff_departure; i++) pinned_pool.push_back(departure_pool[i]);
  if (!has_pin_split && actual_pinned > 0) {
    pinned_pool.clear();
    auto &single_pool = general_pool.empty() ? departure_pool : general_pool;
    int take = std::min((int)single_pool.size(), eff_pinned);
    for (int i = 0; i < take; i++)
      pinned_pool.push_back(single_pool[i]);
  }
  std::vector<Trip> unpinned_pool(demoted_to_unpinned.begin(), demoted_to_unpinned.end());
  unpinned_pool.insert(unpinned_pool.end(), visible.begin() + original_actual_pinned, visible.end());
  if (!demoted_to_unpinned.empty()) {
    std::stable_sort(unpinned_pool.begin(), unpinned_pool.end(),
        [](const Trip &a, const Trip &b) { return a.departure_time < b.departure_time; });
  }

  return {general_pool, departure_pool, pinned_pool, unpinned_pool, demoted_to_unpinned,
          actual_pinned, eff_pinned, eff_general, eff_departure,
          eff_unpinned, has_pin_split};
}

TEST_SUITE("SplitPinLayout") {

  // ---- has_pin_split detection ----

  TEST_CASE("no pinned routes — no split, all unpinned") {
    std::map<std::string, PinMode> pins;
    auto r = split_pin_partition(
        {make_trip("R1", "A", 1300), make_trip("R2", "B", 1400)},
        pins, 1000, 10, /*pinned_rows=*/1, /*general_pins=*/1, /*limit=*/3);
    CHECK(!r.has_pin_split);
    CHECK(r.eff_pinned == 0);
    CHECK(r.eff_unpinned == 3);
    CHECK(r.pinned_pool.empty());
    CHECK(r.unpinned_pool.size() == 2);
  }

  TEST_CASE("only PIN_GENERAL — no split") {
    std::map<std::string, PinMode> pins = {{"R1:A", PIN_GENERAL}};
    auto r = split_pin_partition(
        {make_trip("R1", "A", 2800), make_trip("R2", "B", 2900)},
        pins, 1000, 10, 1, 1, 3);
    CHECK(!r.has_pin_split);
    CHECK(r.eff_pinned == 1);
    CHECK(r.eff_unpinned == 2);
    CHECK(r.general_pool.size() == 1);
    CHECK(r.departure_pool.empty());
  }

  TEST_CASE("only PIN_LEAVING_SOON (inside threshold) — no split") {
    std::map<std::string, PinMode> pins = {{"R1:A", PIN_LEAVING_SOON}};
    auto r = split_pin_partition(
        {make_trip("R1", "A", 1300), make_trip("R2", "B", 2900)},
        pins, 1000, 10, 1, 1, 3);
    CHECK(!r.has_pin_split);
    CHECK(r.eff_pinned == 1);
    CHECK(r.departure_pool.size() == 1);
    CHECK(r.general_pool.empty());
  }

  TEST_CASE("PIN_BOTH outside threshold — general only, no split") {
    std::map<std::string, PinMode> pins = {{"R1:A", PIN_BOTH}};
    auto r = split_pin_partition(
        {make_trip("R1", "A", 2800)},
        pins, 1000, 10, 1, 1, 3);
    CHECK(!r.has_pin_split);
    CHECK(r.general_pool.size() == 1);
    CHECK(r.departure_pool.empty());
  }

  TEST_CASE("PIN_BOTH inside threshold — departure only, no split") {
    std::map<std::string, PinMode> pins = {{"R1:A", PIN_BOTH}};
    auto r = split_pin_partition(
        {make_trip("R1", "A", 1300)},
        pins, 1000, 10, 1, 1, 3);
    CHECK(!r.has_pin_split);
    CHECK(r.departure_pool.size() == 1);
    CHECK(r.general_pool.empty());
  }

  // ---- Split detection: both types present ----

  TEST_CASE("PIN_GENERAL + PIN_LEAVING_SOON (inside) — split active") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800), make_trip("R2", "B", now + 300),
         make_trip("R3", "C", now + 400)},
        pins, now, 10, 1, 1, 3);
    CHECK(r.has_pin_split);
    CHECK(r.general_pool.size() == 1);
    CHECK(r.departure_pool.size() == 1);
  }

  TEST_CASE("PIN_GENERAL + PIN_LEAVING_SOON (outside) — no split, only general") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800), make_trip("R2", "B", now + 900)},
        pins, now, 10, 1, 1, 3);
    CHECK(!r.has_pin_split);  // R2 outside threshold → unpinned
    CHECK(r.general_pool.size() == 1);
    CHECK(r.departure_pool.empty());
  }

  TEST_CASE("PIN_BOTH — one inside, one outside threshold — split active") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_BOTH},
        {"R2:B", PIN_BOTH},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 300),   // 5 min, inside → departure
         make_trip("R2", "B", now + 1800)}, // 30 min, outside → general
        pins, now, 10, 1, 1, 3);
    CHECK(r.has_pin_split);
    CHECK(r.general_pool.size() == 1);
    CHECK(r.general_pool[0].route_id == "R2");
    CHECK(r.departure_pool.size() == 1);
    CHECK(r.departure_pool[0].route_id == "R1");
  }

  // ---- Auto-bump to 2 pinned rows ----

  TEST_CASE("split with pinned_rows=1 auto-bumps to 2") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800), make_trip("R2", "B", now + 300),
         make_trip("R3", "C", now + 400)},
        pins, now, 10, /*pinned_rows=*/1, /*general_pins=*/1, /*limit=*/3);
    CHECK(r.has_pin_split);
    CHECK(r.eff_pinned == 2);  // auto-bumped from 1 to 2
    CHECK(r.eff_general == 1);
    CHECK(r.eff_departure == 1);
    CHECK(r.eff_unpinned == 1);
  }

  TEST_CASE("split with pinned_rows=2 stays at 2") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800), make_trip("R2", "B", now + 300),
         make_trip("R3", "C", now + 400)},
        pins, now, 10, /*pinned_rows=*/2, /*general_pins=*/1, /*limit=*/3);
    CHECK(r.eff_pinned == 2);
    CHECK(r.eff_general == 1);
    CHECK(r.eff_departure == 1);
  }

  TEST_CASE("auto-bump capped at limit-1 when limit=2") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800), make_trip("R2", "B", now + 300),
         make_trip("R3", "C", now + 400)},
        pins, now, 10, /*pinned_rows=*/1, /*general_pins=*/1, /*limit=*/2);
    // limit=2, so auto-bump: min(2, limit-1=1) = 1
    // gen_want=min(1,0)=0, eff_general=0, dep_slots=1, eff_departure=1
    CHECK(r.eff_pinned == 1);
    CHECK(r.eff_departure == 1);
    CHECK(r.eff_general == 0);
    CHECK(r.eff_unpinned == 1);
  }

  // ---- general_pins_count allocation ----

  TEST_CASE("general_pins_count=1, 3 pinned rows — 1 general, 2 departure slots") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_GENERAL},
        {"R3:C", PIN_LEAVING_SOON},
        {"R4:D", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 1900),
         make_trip("R3", "C", now + 300),
         make_trip("R4", "D", now + 400),
         make_trip("R5", "E", now + 500)},
        pins, now, 10, /*pinned_rows=*/3, /*general_pins=*/1, /*limit=*/5);
    CHECK(r.has_pin_split);
    CHECK(r.eff_pinned == 3);
    CHECK(r.eff_general == 1);
    CHECK(r.eff_departure == 2);
    CHECK(r.eff_unpinned == 2);
  }

  TEST_CASE("general_pins_count=2, 3 pinned rows — 2 general, 1 departure") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_GENERAL},
        {"R3:C", PIN_LEAVING_SOON},
        {"R4:D", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 1900),
         make_trip("R3", "C", now + 300),
         make_trip("R4", "D", now + 400),
         make_trip("R5", "E", now + 500)},
        pins, now, 10, /*pinned_rows=*/3, /*general_pins=*/2, /*limit=*/5);
    CHECK(r.eff_pinned == 3);
    CHECK(r.eff_general == 2);
    CHECK(r.eff_departure == 1);
    CHECK(r.eff_unpinned == 2);
  }

  TEST_CASE("general_pins_count capped at eff_pinned-1 to leave room for departure") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_GENERAL},
        {"R3:C", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 1900),
         make_trip("R3", "C", now + 300),
         make_trip("R4", "D", now + 400)},
        pins, now, 10, /*pinned_rows=*/2, /*general_pins=*/5, /*limit=*/4);
    // gen_want = min(5, 2-1) = 1; eff_general = min(1, 2) = 1
    CHECK(r.eff_pinned == 2);
    CHECK(r.eff_general == 1);
    CHECK(r.eff_departure == 1);
  }

  // ---- Fill-up / shrink to actual ----

  TEST_CASE("3 pinned rows but only 1 general + 1 departure — shows 2 pinned") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 300),
         make_trip("R3", "C", now + 400)},
        pins, now, 10, /*pinned_rows=*/3, /*general_pins=*/1, /*limit=*/4);
    // Only 2 actual pins → shrink
    CHECK(r.eff_pinned == 2);
    CHECK(r.eff_general == 1);
    CHECK(r.eff_departure == 1);
    CHECK(r.eff_unpinned == 2);
  }

  TEST_CASE("surplus departure slots given back to general") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_GENERAL},
        {"R3:C", PIN_GENERAL},
        {"R4:D", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 1900),
         make_trip("R3", "C", now + 2000),
         make_trip("R4", "D", now + 300),
         make_trip("R5", "E", now + 400)},
        pins, now, 10, /*pinned_rows=*/3, /*general_pins=*/1, /*limit=*/5);
    // gen_want=1, eff_general=1, dep_slots=2, eff_departure=1
    // surplus=1, extra=min(1, 3-1)=1 → eff_general=2
    CHECK(r.eff_pinned == 3);
    CHECK(r.eff_general == 2);
    CHECK(r.eff_departure == 1);
    CHECK(r.eff_unpinned == 2);
  }

  // ---- Priority ordering: general first, then departure ----

  TEST_CASE("pinned pool order: general first, then departure") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_LEAVING_SOON},
        {"R2:B", PIN_GENERAL},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 300),   // departure pin
         make_trip("R2", "B", now + 1800),  // general pin
         make_trip("R3", "C", now + 400)},
        pins, now, 10, 2, 1, 3);
    CHECK(r.has_pin_split);
    CHECK(r.pinned_pool.size() == 2);
    CHECK(r.pinned_pool[0].route_id == "R2");  // general first
    CHECK(r.pinned_pool[1].route_id == "R1");  // departure second
  }

  TEST_CASE("pinned pool order: multiple general then multiple departure") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_GENERAL},
        {"R3:C", PIN_LEAVING_SOON},
        {"R4:D", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 1900),
         make_trip("R3", "C", now + 300),
         make_trip("R4", "D", now + 400),
         make_trip("R5", "E", now + 500)},
        pins, now, 10, 4, 2, 5);
    CHECK(r.pinned_pool.size() == 4);
    CHECK(r.pinned_pool[0].route_id == "R1");  // general
    CHECK(r.pinned_pool[1].route_id == "R2");  // general
    CHECK(r.pinned_pool[2].route_id == "R3");  // departure
    CHECK(r.pinned_pool[3].route_id == "R4");  // departure
  }

  // ---- No split fallback: original behavior ----

  TEST_CASE("no split — pinned_pool respects pinned_rows_count in non-split") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_GENERAL},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 1900),
         make_trip("R3", "C", now + 400)},
        pins, now, 10, 2, 1, 3);
    CHECK(!r.has_pin_split);
    CHECK(r.eff_pinned == 2);  // respects pinned_rows_count=2
    CHECK(r.pinned_pool.size() == 2);
    CHECK(r.pinned_pool[0].route_id == "R1");
    CHECK(r.pinned_pool[1].route_id == "R2");
  }

  // ---- PIN_BOTH dynamic movement ----

  TEST_CASE("PIN_BOTH transitions from general to departure as time passes") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_BOTH},
        {"R2:B", PIN_GENERAL},
    };
    unsigned int now = 1000;
    int threshold = 10;

    // Time 1: R1 at 20 min → general (red)
    auto r1 = split_pin_partition(
        {make_trip("R1", "A", now + 1200), make_trip("R2", "B", now + 1800),
         make_trip("R3", "C", now + 400)},
        pins, now, threshold, 2, 1, 3);
    CHECK(!r1.has_pin_split);  // both in general
    CHECK(r1.general_pool.size() == 2);
    CHECK(r1.departure_pool.empty());

    // Time 2: R1 now at 5 min → departure (green), R2 still general → split!
    auto r2 = split_pin_partition(
        {make_trip("R1", "A", now + 300), make_trip("R2", "B", now + 1800),
         make_trip("R3", "C", now + 400)},
        pins, now, threshold, 2, 1, 3);
    CHECK(r2.has_pin_split);
    CHECK(r2.general_pool.size() == 1);
    CHECK(r2.general_pool[0].route_id == "R2");
    CHECK(r2.departure_pool.size() == 1);
    CHECK(r2.departure_pool[0].route_id == "R1");
  }

  // ---- Edge cases ----

  TEST_CASE("all pinned leaving soon — no general at all, no split") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_LEAVING_SOON},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 300), make_trip("R2", "B", now + 400)},
        pins, now, 10, 2, 1, 3);
    CHECK(!r.has_pin_split);
    CHECK(r.general_pool.empty());
    CHECK(r.departure_pool.size() == 2);
    CHECK(r.eff_pinned == 2);  // respects pinned_rows_count=2
  }

  TEST_CASE("all pinned general — no departure, no split") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_GENERAL},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800), make_trip("R2", "B", now + 1900)},
        pins, now, 10, 2, 1, 3);
    CHECK(!r.has_pin_split);
    CHECK(r.general_pool.size() == 2);
    CHECK(r.departure_pool.empty());
  }

  TEST_CASE("single trip with PIN_BOTH straddles threshold — no split possible") {
    std::map<std::string, PinMode> pins = {{"R1:A", PIN_BOTH}};
    unsigned int now = 1000;
    // Inside threshold — goes to departure only
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 300)},
        pins, now, 10, 1, 1, 3);
    CHECK(!r.has_pin_split);
    CHECK(r.departure_pool.size() == 1);
    CHECK(r.general_pool.empty());
    CHECK(r.eff_pinned == 1);
  }

  TEST_CASE("limit=2 with split — all pinned expands to fill both rows") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800), make_trip("R2", "B", now + 300)},
        pins, now, 10, 1, 1, 2);
    // All trips pinned → unpinned_pool_empty → pins expand to fill all rows
    // general_pins_count=1 → eff_general=1, dep_slots=limit-1=1, eff_departure=1
    CHECK(r.eff_pinned == 2);
    CHECK(r.eff_general == 1);
    CHECK(r.eff_departure == 1);
    CHECK(r.eff_unpinned == 0);
  }

  TEST_CASE("limit=2 with split + unpinned — auto-bump capped at limit-1") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800), make_trip("R2", "B", now + 300),
         make_trip("R3", "C", now + 400)},
        pins, now, 10, 1, 1, 2);
    // Unpinned trip exists → old auto-bump behavior: min(2, limit-1=1) = 1
    CHECK(r.eff_pinned == 1);
    CHECK(r.eff_general == 0);
    CHECK(r.eff_departure == 1);
    CHECK(r.eff_unpinned == 1);
  }

  TEST_CASE("many general, few departure — surplus fills general") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_GENERAL},
        {"R3:C", PIN_GENERAL},
        {"R4:D", PIN_GENERAL},
        {"R5:E", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 1900),
         make_trip("R3", "C", now + 2000),
         make_trip("R4", "D", now + 2100),
         make_trip("R5", "E", now + 300),
         make_trip("R6", "F", now + 400)},
        pins, now, 10, /*pinned_rows=*/4, /*general_pins=*/1, /*limit=*/6);
    // gen_want=1, eff_general=1, dep_slots=3, eff_departure=min(3,1)=1
    // surplus=2, extra=min(2, 4-1)=2 → eff_general=3
    CHECK(r.eff_pinned == 4);
    CHECK(r.eff_general == 3);
    CHECK(r.eff_departure == 1);
    CHECK(r.eff_unpinned == 2);
  }

  TEST_CASE("many departure, few general — departure fills remaining slots") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
        {"R3:C", PIN_LEAVING_SOON},
        {"R4:D", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 300),
         make_trip("R3", "C", now + 400),
         make_trip("R4", "D", now + 500),
         make_trip("R5", "E", now + 600)},
        pins, now, 10, /*pinned_rows=*/4, /*general_pins=*/1, /*limit=*/5);
    CHECK(r.eff_pinned == 4);
    CHECK(r.eff_general == 1);
    CHECK(r.eff_departure == 3);
    CHECK(r.eff_unpinned == 1);
  }

  TEST_CASE("pinned_rows larger than actual pins — shrinks to actual") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 300),
         make_trip("R3", "C", now + 400),
         make_trip("R4", "D", now + 500)},
        pins, now, 10, /*pinned_rows=*/5, /*general_pins=*/1, /*limit=*/6);
    CHECK(r.eff_pinned == 2);
    CHECK(r.eff_general == 1);
    CHECK(r.eff_departure == 1);
    CHECK(r.eff_unpinned == 4);
  }

  // ---- Setter bounds ----

  TEST_CASE("general_pins_count minimum is 1") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800), make_trip("R2", "B", now + 300),
         make_trip("R3", "C", now + 400)},
        pins, now, 10, 2, /*general_pins=*/1, 3);
    CHECK(r.eff_general == 1);
    CHECK(r.eff_departure == 1);
  }

  // ---- Real-world scenario: the user's example ----

  TEST_CASE("real scenario: 3 rows, 1 general + 1 departure, gen_pins=1") {
    std::map<std::string, PinMode> pins = {
        {"A:Downtown", PIN_GENERAL},
        {"B:Airport", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("A", "Downtown", now + 1800),
         make_trip("B", "Airport", now + 300),
         make_trip("C", "Mall", now + 400),
         make_trip("D", "Park", now + 500)},
        pins, now, 10, /*pinned_rows=*/1, /*general_pins=*/1, /*limit=*/3);
    CHECK(r.has_pin_split);
    CHECK(r.eff_pinned == 2);
    CHECK(r.eff_general == 1);
    CHECK(r.eff_departure == 1);
    CHECK(r.eff_unpinned == 1);
    CHECK(r.pinned_pool[0].route_id == "A");   // general first
    CHECK(r.pinned_pool[1].route_id == "B");   // departure second
    CHECK(r.unpinned_pool[0].route_id == "C");
  }

  TEST_CASE("real scenario: B not yet leaving soon — no split, 1 pinned row") {
    std::map<std::string, PinMode> pins = {
        {"A:Downtown", PIN_GENERAL},
        {"B:Airport", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("A", "Downtown", now + 1800),
         make_trip("B", "Airport", now + 900),  // 15 min, outside 10
         make_trip("C", "Mall", now + 400)},
        pins, now, 10, 1, 1, 3);
    CHECK(!r.has_pin_split);
    CHECK(r.eff_pinned == 1);
    CHECK(r.eff_unpinned == 2);
    CHECK(r.pinned_pool[0].route_id == "A");
  }

  // ---- Dedup interaction ----

  TEST_CASE("dedup within split: duplicate PIN_GENERAL stays in general pool") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R1", "A", now + 2000),  // dup PIN_GENERAL → general (red)
         make_trip("R2", "B", now + 300),
         make_trip("R3", "C", now + 400)},
        pins, now, 10, 2, 1, 4);
    // Duplicate PIN_GENERAL goes to general_pool (red pin)
    CHECK(r.actual_pinned == 3);
    CHECK(r.general_pool.size() == 2);  // first + duplicate
    CHECK(r.departure_pool.size() == 1);
    CHECK(r.unpinned_pool.size() == 1);  // R3:C
  }

  TEST_CASE("PIN_BOTH dups: leaving-soon first → departure, not-LS dup → general") {
    // Simulates the real scenario: all routes PIN_BOTH, most leaving soon,
    // duplicates route by their own leaving-soon status (matching primary logic).
    std::map<std::string, PinMode> pins = {
        {"545:DT Seattle", PIN_BOTH},
        {"2Line:S Bellevue", PIN_BOTH},
        {"BLine:BTC", PIN_BOTH},
        {"BLine:DTR", PIN_BOTH},
    };
    unsigned int now = 1000;
    // 545 at +60 (LS), 2Line at +120 (LS), BLine BTC at +300 (LS),
    // 2Line dup at +720 (not LS), BLine DTR at +840 (not LS),
    // BLine BTC dup at +900 (not LS), 545 dup at +960 (not LS)
    auto r = split_pin_partition(
        {make_trip("545", "DT Seattle", now + 60),
         make_trip("2Line", "S Bellevue", now + 120),
         make_trip("BLine", "BTC", now + 300),
         make_trip("2Line", "S Bellevue", now + 720),   // dup PIN_BOTH, not LS → general
         make_trip("BLine", "DTR", now + 840),
         make_trip("BLine", "BTC", now + 900),           // dup PIN_BOTH, not LS → general
         make_trip("545", "DT Seattle", now + 960)},     // dup PIN_BOTH, not LS → general
        pins, now, 10, 2, 1, 3);

    // First-occurrences leaving soon → departure_pool (green)
    CHECK(r.departure_pool.size() == 3);  // 545, 2Line, BLine BTC
    // First-occurrence NOT leaving soon + not-LS duplicates → general_pool (red)
    CHECK(r.general_pool.size() == 4);    // BLine DTR (first) + 2Line dup + BLine BTC dup + 545 dup
    CHECK(r.has_pin_split);
    CHECK(r.actual_pinned == 7);  // all in pools, none erased
    CHECK(r.unpinned_pool.empty());  // no unpinned routes
    CHECK(r.demoted_to_unpinned.empty());  // no PIN_LEAVING_SOON dups
  }

  TEST_CASE("PIN_BOTH dups: leaving-soon dup → departure (not general)") {
    // Regression test for the two-green-pin bug:
    // When a PIN_BOTH route has two trips both within the leaving-soon threshold,
    // the duplicate must go to departure_pool (green), NOT general_pool.
    // Old behavior: dup always went to general_pool, but pin color was
    // re-evaluated as green → visual mismatch (two green pins).
    std::map<std::string, PinMode> pins = {
        {"BLine:DTR", PIN_BOTH},
        {"R2:B", PIN_GENERAL},  // need a general pin to trigger split
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("BLine", "DTR", now + 120),   // first, leaving soon → departure
         make_trip("BLine", "DTR", now + 300),   // dup, ALSO leaving soon → should be departure
         make_trip("R2", "B", now + 1800),       // general pin
         make_trip("R3", "C", now + 400)},
        pins, now, 10, 2, 1, 4);

    // With the fix, leaving-soon dup goes to departure_pool (not general_pool)
    CHECK(r.departure_pool.size() == 2);  // both BLine trips
    CHECK(r.general_pool.size() == 1);    // only R2
    CHECK(r.has_pin_split);
    CHECK(r.demoted_to_unpinned.empty());
  }

  TEST_CASE("PIN_BOTH dups: mixed LS — LS dup → departure, not-LS dup → general") {
    // Two dups of the same PIN_BOTH route: one leaving soon, one not.
    std::map<std::string, PinMode> pins = {
        {"BLine:DTR", PIN_BOTH},
        {"R2:B", PIN_GENERAL},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("BLine", "DTR", now + 120),   // first, LS → departure
         make_trip("BLine", "DTR", now + 300),   // dup, LS → departure
         make_trip("BLine", "DTR", now + 900),   // dup, NOT LS → general
         make_trip("R2", "B", now + 1800),
         make_trip("R3", "C", now + 400)},
        pins, now, 10, 2, 1, 5);

    CHECK(r.departure_pool.size() == 2);  // first + LS dup
    CHECK(r.general_pool.size() == 2);    // R2 + not-LS dup
    CHECK(r.has_pin_split);
  }

  TEST_CASE("PIN_LEAVING_SOON dups within threshold: all stay in departure") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_LEAVING_SOON},
        {"R2:B", PIN_GENERAL},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 300),   // first → departure (green)
         make_trip("R1", "A", now + 400),   // dup, still within threshold → departure (green)
         make_trip("R2", "B", now + 1800),
         make_trip("R3", "C", now + 500)},
        pins, now, 10, 2, 1, 4);
    CHECK(r.departure_pool.size() == 2);  // both R1 trips within threshold
    CHECK(r.general_pool.size() == 1);
    CHECK(r.demoted_to_unpinned.empty());
    CHECK(r.unpinned_pool.size() == 1);  // R3:C only
  }

  TEST_CASE("PIN_LEAVING_SOON dup outside threshold: stays in unpinned naturally") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_LEAVING_SOON},
        {"R2:B", PIN_GENERAL},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 300),   // first → departure (green)
         make_trip("R1", "A", now + 700),   // outside threshold → not eff. pinned → unpinned
         make_trip("R2", "B", now + 1800),
         make_trip("R3", "C", now + 500)},
        pins, now, 10, 2, 1, 4);
    CHECK(r.departure_pool.size() == 1);
    CHECK(r.general_pool.size() == 1);
    CHECK(r.demoted_to_unpinned.empty());  // never reached dedup — was unpinned from partition
    // unpinned: R1 dup (700) then R3 (500), original order from partition
    CHECK(r.unpinned_pool.size() == 2);
    CHECK(r.unpinned_pool[0].departure_time == (time_t)(now + 700));
    CHECK(r.unpinned_pool[1].departure_time == (time_t)(now + 500));
  }

  // ---- Non-split respects pinned_rows_count ----

  TEST_CASE("two green pins with pinned_rows=2 — both shown in non-split") {
    // With the fix, non-split mode respects pinned_rows_count
    std::map<std::string, PinMode> pins = {
        {"R1:A:S1", PIN_BOTH},
        {"R1:A:S2", PIN_BOTH},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 300, false, "S1"),   // green
         make_trip("R1", "A", now + 400, false, "S2"),   // green
         make_trip("R2", "B", now + 500)},
        pins, now, 10, /*pinned_rows=*/2, /*general_pins=*/1, /*limit=*/3);
    CHECK(!r.has_pin_split);
    CHECK(r.departure_pool.size() == 2);
    CHECK(r.general_pool.empty());
    CHECK(r.eff_pinned == 2);  // respects pinned_rows_count
    CHECK(r.eff_unpinned == 1);
  }

  TEST_CASE("two red pins with pinned_rows=2 — both shown in non-split") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_GENERAL},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 1900),
         make_trip("R3", "C", now + 400)},
        pins, now, 10, /*pinned_rows=*/2, /*general_pins=*/1, /*limit=*/3);
    CHECK(!r.has_pin_split);
    CHECK(r.eff_pinned == 2);  // respects pinned_rows_count
    CHECK(r.eff_unpinned == 1);
  }

  TEST_CASE("split active with pinned_rows=2 — uses full allocation") {
    // Contrast: when split IS active, pinned_rows=2 gives 2 rows
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 300),
         make_trip("R3", "C", now + 400)},
        pins, now, 10, /*pinned_rows=*/2, /*general_pins=*/1, /*limit=*/3);
    CHECK(r.has_pin_split);
    CHECK(r.eff_pinned == 2);
    CHECK(r.eff_general == 1);
    CHECK(r.eff_departure == 1);
  }

  TEST_CASE("PIN_BOTH all leaving soon with pinned_rows=1 — still 1 row") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_BOTH},
        {"R2:B", PIN_BOTH},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 300),
         make_trip("R2", "B", now + 400),
         make_trip("R3", "C", now + 500)},
        pins, now, 10, /*pinned_rows=*/1, /*general_pins=*/1, /*limit=*/3);
    CHECK(!r.has_pin_split);
    CHECK(r.eff_pinned == 1);
    CHECK(r.pinned_pool.size() == 1);
  }
}

// =====================================================================
// Pin-color consistency — verify row_leaving_soon matches pool assignment
// =====================================================================
// Mirrors draw_schedule section 6: for each pinned row, the pin color is
// derived from the trip's PinMode + departure time.  These tests verify
// that pool assignment (general vs departure) is consistent with the
// independently-computed pin color, preventing the "two green pins" bug.

/// Helper: computes row_leaving_soon for a pinned trip, mirroring section 6
/// of draw_schedule.  Returns true for green pin, false for red.
static bool compute_row_leaving_soon(
    const Trip &trip,
    const std::map<std::string, PinMode> &pinned_routes,
    unsigned int rtc_now, int threshold_sec) {
  auto it = pinned_routes.find(trip.composite_key());
  if (it == pinned_routes.end()) return false;
  int secs_until = (int)(trip.departure_time - rtc_now);
  switch (it->second) {
    case PIN_LEAVING_SOON: return true;  // always green when in pinned section
    case PIN_BOTH: return (secs_until < threshold_sec);
    default: return false;
  }
}

/// Returns true if any pool assignment contradicts the pin color that would
/// be rendered.  A "general" trip should be red (ls=false), a "departure"
/// trip should be green (ls=true).
static bool has_pool_color_mismatch(
    const SplitPinResult &r,
    const std::map<std::string, PinMode> &pinned_routes,
    unsigned int rtc_now, int threshold_min) {
  int threshold_sec = threshold_min * 60;
  // Check general pool: all should render as red (ls=false)
  for (const auto &t : r.general_pool) {
    if (compute_row_leaving_soon(t, pinned_routes, rtc_now, threshold_sec))
      return true;  // would render green but is in general (red) pool
  }
  // Check departure pool: all should render as green (ls=true)
  for (const auto &t : r.departure_pool) {
    if (!compute_row_leaving_soon(t, pinned_routes, rtc_now, threshold_sec))
      return true;  // would render red but is in departure (green) pool
  }
  return false;
}

TEST_SUITE("PinColorConsistency") {

  TEST_CASE("PIN_BOTH dup both leaving soon — no color mismatch") {
    // The exact two-green-pin bug scenario: same PIN_BOTH route, two trips
    // both within threshold.  Before the fix, the dup went to general_pool
    // but rendered with a green pin.
    std::map<std::string, PinMode> pins = {
        {"BLine:DTR", PIN_BOTH},
        {"R2:B", PIN_GENERAL},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("BLine", "DTR", now + 120),
         make_trip("BLine", "DTR", now + 300),
         make_trip("R2", "B", now + 1800),
         make_trip("R3", "C", now + 400)},
        pins, now, 10, 2, 1, 4);
    CHECK(!has_pool_color_mismatch(r, pins, now, 10));
  }

  TEST_CASE("PIN_BOTH dup not leaving soon — no color mismatch") {
    std::map<std::string, PinMode> pins = {
        {"BLine:DTR", PIN_BOTH},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("BLine", "DTR", now + 120),   // LS → departure
         make_trip("BLine", "DTR", now + 900),   // not LS → general
         make_trip("R2", "B", now + 300),
         make_trip("R3", "C", now + 400)},
        pins, now, 10, 2, 1, 4);
    CHECK(!has_pool_color_mismatch(r, pins, now, 10));
  }

  TEST_CASE("real-world: multiple PIN_BOTH routes, mixed thresholds") {
    // Mirrors the user's actual configuration from the screenshot
    std::map<std::string, PinMode> pins = {
        {"545:DT Seattle", PIN_BOTH},
        {"2Line:S Bellevue", PIN_BOTH},
        {"BLine:BTC", PIN_BOTH},
        {"BLine:DTR", PIN_BOTH},
    };
    unsigned int now = 1000;

    // Scenario A: some leaving soon, some not — classic split
    auto ra = split_pin_partition(
        {make_trip("545", "DT Seattle", now + 60),
         make_trip("2Line", "S Bellevue", now + 120),
         make_trip("BLine", "BTC", now + 300),
         make_trip("BLine", "DTR", now + 840),
         make_trip("545", "DT Seattle", now + 960)},
        pins, now, 10, 1, 1, 3);
    CHECK(!has_pool_color_mismatch(ra, pins, now, 10));

    // Scenario B: all leaving soon with duplicates (the rare bug trigger)
    auto rb = split_pin_partition(
        {make_trip("545", "DT Seattle", now + 60),
         make_trip("545", "DT Seattle", now + 120),  // dup, also LS
         make_trip("2Line", "S Bellevue", now + 180),
         make_trip("BLine", "BTC", now + 240),
         make_trip("BLine", "DTR", now + 300)},
        pins, now, 10, 1, 1, 3);
    CHECK(!has_pool_color_mismatch(rb, pins, now, 10));

    // Scenario C: dups straddle the threshold boundary
    auto rc = split_pin_partition(
        {make_trip("BLine", "DTR", now + 300),   // first, LS
         make_trip("BLine", "DTR", now + 599),   // dup, just inside threshold (599 < 600)
         make_trip("BLine", "DTR", now + 601),   // dup, just outside threshold (601 >= 600)
         make_trip("545", "DT Seattle", now + 1800)},
        pins, now, 10, 2, 1, 4);
    CHECK(!has_pool_color_mismatch(rc, pins, now, 10));
  }

  TEST_CASE("PIN_GENERAL dups never cause mismatch") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R1", "A", now + 2000),  // dup
         make_trip("R2", "B", now + 300),
         make_trip("R3", "C", now + 400)},
        pins, now, 10, 2, 1, 4);
    CHECK(!has_pool_color_mismatch(r, pins, now, 10));
  }

  TEST_CASE("PIN_LEAVING_SOON dups never cause mismatch") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_LEAVING_SOON},
        {"R2:B", PIN_GENERAL},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 300),
         make_trip("R1", "A", now + 400),  // dup, still LS
         make_trip("R2", "B", now + 1800),
         make_trip("R3", "C", now + 500)},
        pins, now, 10, 2, 1, 4);
    CHECK(!has_pool_color_mismatch(r, pins, now, 10));
  }

  TEST_CASE("no split — no mismatch possible") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_BOTH},
        {"R2:B", PIN_BOTH},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 300),
         make_trip("R2", "B", now + 400)},
        pins, now, 10, 2, 1, 3);
    CHECK(!r.has_pin_split);
    CHECK(!has_pool_color_mismatch(r, pins, now, 10));
  }
}

// =====================================================================
// Split-pin paging — verify ScrollContainer pages correctly
// over both general and departure sub-pools in split mode
// =====================================================================

TEST_SUITE("SplitPinPaging") {

  // Helper: simulates the split-mode paging logic from draw_schedule.
  // Uses pinned_pager_ for general and departure_pager_ for departure.
  struct SplitPagingFrame {
    std::vector<Trip> pinned_pool;
    std::vector<Trip> unpinned_pool;
  };

  static SplitPagingFrame build_split_frame(
      const SplitPinResult &r,
      ScrollContainer &gen_pager, ScrollContainer &dep_pager) {
    SplitPagingFrame f;
    if (r.has_pin_split) {
      gen_pager.clamp((int)r.general_pool.size(), r.eff_general);
      dep_pager.clamp((int)r.departure_pool.size(), r.eff_departure);
      int g_si = gen_pager.start_index((int)r.general_pool.size(), r.eff_general);
      int g_ei = gen_pager.end_index((int)r.general_pool.size(), r.eff_general);
      int d_si = dep_pager.start_index((int)r.departure_pool.size(), r.eff_departure);
      int d_ei = dep_pager.end_index((int)r.departure_pool.size(), r.eff_departure);
      for (int i = g_si; i < g_ei; i++)
        f.pinned_pool.push_back(r.general_pool[i]);
      for (int i = d_si; i < d_ei; i++)
        f.pinned_pool.push_back(r.departure_pool[i]);
    } else if (r.eff_pinned > 0) {
      f.pinned_pool = r.pinned_pool;
    }
    f.unpinned_pool = r.unpinned_pool;
    return f;
  }

  // Helper to advance both pagers together (mirrors draw_schedule behavior)
  static void advance_both(const SplitPinResult &r,
                            ScrollContainer &gen_pager, ScrollContainer &dep_pager) {
    gen_pager.advance_page((int)r.general_pool.size(), r.eff_general);
    dep_pager.advance_page((int)r.departure_pool.size(), r.eff_departure);
  }

  TEST_CASE("1 red + 2 green: departure pages, general stays fixed") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
        {"R3:C", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 300),
         make_trip("R3", "C", now + 400),
         make_trip("R4", "D", now + 500)},
        pins, now, 10, /*pinned_rows=*/2, /*general_pins=*/1, /*limit=*/3);

    CHECK(r.has_pin_split);
    CHECK(r.eff_general == 1);
    CHECK(r.eff_departure == 1);
    CHECK(r.departure_pool.size() == 2);

    ScrollContainer gen_p, dep_p;
    gen_p.reset(); dep_p.reset();

    // Page 0: general R1 + first departure R2
    auto f0 = build_split_frame(r, gen_p, dep_p);
    CHECK(f0.pinned_pool.size() == 2);
    CHECK(f0.pinned_pool[0].route_id == "R1");
    CHECK(f0.pinned_pool[1].route_id == "R2");

    // Departure needs paging; general does not
    CHECK(!gen_p.needs_paging((int)r.general_pool.size(), r.eff_general));
    CHECK(dep_p.needs_paging((int)r.departure_pool.size(), r.eff_departure));

    advance_both(r, gen_p, dep_p);

    // Page 1: general R1 stays, departure rotates to R3
    auto f1 = build_split_frame(r, gen_p, dep_p);
    CHECK(f1.pinned_pool[0].route_id == "R1");
    CHECK(f1.pinned_pool[1].route_id == "R3");

    // Wrap around
    advance_both(r, gen_p, dep_p);
    auto f2 = build_split_frame(r, gen_p, dep_p);
    CHECK(f2.pinned_pool[1].route_id == "R2");
  }

  TEST_CASE("2 red + 1 green: general pages, departure stays fixed") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_GENERAL},
        {"R3:C", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 1900),
         make_trip("R3", "C", now + 300),
         make_trip("R4", "D", now + 500)},
        pins, now, 10, /*pinned_rows=*/2, /*general_pins=*/1, /*limit=*/3);

    CHECK(r.has_pin_split);
    CHECK(r.eff_general == 1);
    CHECK(r.eff_departure == 1);
    CHECK(r.general_pool.size() == 2);
    CHECK(r.departure_pool.size() == 1);

    ScrollContainer gen_p, dep_p;
    gen_p.reset(); dep_p.reset();

    // Page 0: first general R1 + departure R3
    auto f0 = build_split_frame(r, gen_p, dep_p);
    CHECK(f0.pinned_pool.size() == 2);
    CHECK(f0.pinned_pool[0].route_id == "R1");
    CHECK(f0.pinned_pool[1].route_id == "R3");

    // General needs paging; departure does not
    CHECK(gen_p.needs_paging((int)r.general_pool.size(), r.eff_general));
    CHECK(!dep_p.needs_paging((int)r.departure_pool.size(), r.eff_departure));

    advance_both(r, gen_p, dep_p);

    // Page 1: general rotates to R2, departure R3 stays
    auto f1 = build_split_frame(r, gen_p, dep_p);
    CHECK(f1.pinned_pool[0].route_id == "R2");
    CHECK(f1.pinned_pool[1].route_id == "R3");

    // Wrap around
    advance_both(r, gen_p, dep_p);
    auto f2 = build_split_frame(r, gen_p, dep_p);
    CHECK(f2.pinned_pool[0].route_id == "R1");
    CHECK(f2.pinned_pool[1].route_id == "R3");
  }

  TEST_CASE("2 red + 2 green: both sub-pools page together") {
    std::map<std::string, PinMode> pins = {
        {"G1:A", PIN_GENERAL},
        {"G2:B", PIN_GENERAL},
        {"D1:C", PIN_LEAVING_SOON},
        {"D2:D", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("G1", "A", now + 1800),
         make_trip("G2", "B", now + 1900),
         make_trip("D1", "C", now + 300),
         make_trip("D2", "D", now + 400),
         make_trip("U1", "E", now + 500)},
        pins, now, 10, /*pinned_rows=*/2, /*general_pins=*/1, /*limit=*/3);

    CHECK(r.has_pin_split);
    CHECK(r.eff_general == 1);
    CHECK(r.eff_departure == 1);

    ScrollContainer gen_p, dep_p;
    gen_p.reset(); dep_p.reset();

    auto f0 = build_split_frame(r, gen_p, dep_p);
    CHECK(f0.pinned_pool[0].route_id == "G1");
    CHECK(f0.pinned_pool[1].route_id == "D1");

    advance_both(r, gen_p, dep_p);

    auto f1 = build_split_frame(r, gen_p, dep_p);
    CHECK(f1.pinned_pool[0].route_id == "G2");
    CHECK(f1.pinned_pool[1].route_id == "D2");

    advance_both(r, gen_p, dep_p);

    auto f2 = build_split_frame(r, gen_p, dep_p);
    CHECK(f2.pinned_pool[0].route_id == "G1");
    CHECK(f2.pinned_pool[1].route_id == "D1");
  }

  TEST_CASE("1 red + 3 green, 2 departure slots: pages 2 at a time") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
        {"R3:C", PIN_LEAVING_SOON},
        {"R4:D", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 200),
         make_trip("R3", "C", now + 300),
         make_trip("R4", "D", now + 400),
         make_trip("R5", "E", now + 500)},
        pins, now, 10, /*pinned_rows=*/3, /*general_pins=*/1, /*limit=*/4);

    CHECK(r.has_pin_split);
    CHECK(r.eff_general == 1);
    CHECK(r.eff_departure == 2);
    CHECK(r.departure_pool.size() == 3);

    ScrollContainer gen_p, dep_p;
    gen_p.reset(); dep_p.reset();

    auto f0 = build_split_frame(r, gen_p, dep_p);
    CHECK(f0.pinned_pool.size() == 3);
    CHECK(f0.pinned_pool[0].route_id == "R1");
    CHECK(f0.pinned_pool[1].route_id == "R2");
    CHECK(f0.pinned_pool[2].route_id == "R3");

    advance_both(r, gen_p, dep_p);

    auto f1 = build_split_frame(r, gen_p, dep_p);
    CHECK(f1.pinned_pool[0].route_id == "R1");
    CHECK(f1.pinned_pool[1].route_id == "R3");
    CHECK(f1.pinned_pool[2].route_id == "R4");
  }

  TEST_CASE("3 red + 1 green, 2 general slots: general pages 2 at a time") {
    std::map<std::string, PinMode> pins = {
        {"G1:A", PIN_GENERAL},
        {"G2:B", PIN_GENERAL},
        {"G3:C", PIN_GENERAL},
        {"D1:D", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("G1", "A", now + 1800),
         make_trip("G2", "B", now + 1900),
         make_trip("G3", "C", now + 2000),
         make_trip("D1", "D", now + 300),
         make_trip("U1", "E", now + 500)},
        pins, now, 10, /*pinned_rows=*/3, /*general_pins=*/2, /*limit=*/4);

    CHECK(r.has_pin_split);
    CHECK(r.eff_general == 2);
    CHECK(r.eff_departure == 1);
    CHECK(r.general_pool.size() == 3);

    ScrollContainer gen_p, dep_p;
    gen_p.reset(); dep_p.reset();

    auto f0 = build_split_frame(r, gen_p, dep_p);
    CHECK(f0.pinned_pool.size() == 3);
    CHECK(f0.pinned_pool[0].route_id == "G1");
    CHECK(f0.pinned_pool[1].route_id == "G2");
    CHECK(f0.pinned_pool[2].route_id == "D1");

    advance_both(r, gen_p, dep_p);

    auto f1 = build_split_frame(r, gen_p, dep_p);
    CHECK(f1.pinned_pool[0].route_id == "G2");
    CHECK(f1.pinned_pool[1].route_id == "G3");
    CHECK(f1.pinned_pool[2].route_id == "D1");  // departure stays

    advance_both(r, gen_p, dep_p);

    auto f2 = build_split_frame(r, gen_p, dep_p);
    CHECK(f2.pinned_pool[0].route_id == "G1");  // wraps
    CHECK(f2.pinned_pool[2].route_id == "D1");
  }

  TEST_CASE("no paging needed when both pools fit slots exactly") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 300),
         make_trip("R3", "C", now + 400)},
        pins, now, 10, 2, 1, 3);

    CHECK(r.has_pin_split);

    ScrollContainer gen_p, dep_p;
    gen_p.reset(); dep_p.reset();
    gen_p.clamp((int)r.general_pool.size(), r.eff_general);
    dep_p.clamp((int)r.departure_pool.size(), r.eff_departure);

    CHECK(!gen_p.needs_paging((int)r.general_pool.size(), r.eff_general));
    CHECK(!dep_p.needs_paging((int)r.departure_pool.size(), r.eff_departure));
  }

  TEST_CASE("non-split mode: paging works with pinned_rows_count=1 and pool>1") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_GENERAL},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 1900),
         make_trip("R3", "C", now + 400)},
        pins, now, 10, 1, 1, 3);

    CHECK(!r.has_pin_split);
    CHECK(r.eff_pinned == 1);
    CHECK(r.general_pool.size() == 2);

    // The single-pool pager should page through the general pool
    ScrollContainer pager;
    pager.reset();
    pager.clamp((int)r.general_pool.size(), r.eff_pinned);
    CHECK(pager.needs_paging((int)r.general_pool.size(), r.eff_pinned));

    // Page 0: R1
    int si = pager.start_index((int)r.general_pool.size(), r.eff_pinned);
    CHECK(r.general_pool[si].route_id == "R1");

    // Page 1: R2
    pager.advance_page((int)r.general_pool.size(), r.eff_pinned);
    si = pager.start_index((int)r.general_pool.size(), r.eff_pinned);
    CHECK(r.general_pool[si].route_id == "R2");
  }

  TEST_CASE("PIN_BOTH mixed: departure sub-pool pages correctly") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_BOTH},
        {"R2:B", PIN_BOTH},
        {"R3:C", PIN_BOTH},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 300),
         make_trip("R3", "C", now + 400),
         make_trip("R4", "D", now + 500)},
        pins, now, 10, 2, 1, 3);

    CHECK(r.has_pin_split);
    CHECK(r.general_pool.size() == 1);
    CHECK(r.departure_pool.size() == 2);

    ScrollContainer gen_p, dep_p;
    gen_p.reset(); dep_p.reset();

    auto f0 = build_split_frame(r, gen_p, dep_p);
    CHECK(f0.pinned_pool[0].route_id == "R1");
    CHECK(f0.pinned_pool[1].route_id == "R2");

    advance_both(r, gen_p, dep_p);

    auto f1 = build_split_frame(r, gen_p, dep_p);
    CHECK(f1.pinned_pool[0].route_id == "R1");
    CHECK(f1.pinned_pool[1].route_id == "R3");
  }

  // ---- Pin expansion when unpinned pool is empty ----

  TEST_CASE("non-split: all pinned general → expand to fill all rows") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_GENERAL},
        {"R3:C", PIN_GENERAL},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 1900),
         make_trip("R3", "C", now + 2000)},
        pins, now, 10, /*pinned_rows=*/1, /*general_pins=*/1, /*limit=*/3);
    CHECK(!r.has_pin_split);
    // All trips pinned, no unpinned → expand to fill all 3 rows
    CHECK(r.eff_pinned == 3);
    CHECK(r.eff_unpinned == 0);
    CHECK(r.unpinned_pool.empty());
    CHECK(r.pinned_pool.size() == 3);
  }

  TEST_CASE("non-split: all pinned leaving soon → expand to limit") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_LEAVING_SOON},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 300),
         make_trip("R2", "B", now + 400)},
        pins, now, 10, /*pinned_rows=*/1, /*general_pins=*/1, /*limit=*/3);
    CHECK(!r.has_pin_split);
    CHECK(r.eff_pinned == 2);  // min(limit=3, actual=2) = 2
    CHECK(r.eff_unpinned == 1);
  }

  TEST_CASE("split: all pinned → general_pins_count controls allocation") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_GENERAL},
        {"R3:C", PIN_LEAVING_SOON},
        {"R4:D", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    // general_pins_count=1 → 1 general row, limit-1=2 departure rows
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 1900),
         make_trip("R3", "C", now + 300),
         make_trip("R4", "D", now + 400)},
        pins, now, 10, /*pinned_rows=*/1, /*general_pins=*/1, /*limit=*/3);
    CHECK(r.has_pin_split);
    CHECK(r.eff_general == 1);
    CHECK(r.eff_departure == 2);
    CHECK(r.eff_pinned == 3);
    CHECK(r.eff_unpinned == 0);
  }

  TEST_CASE("split: all pinned, general_pins_count=2 → 2 general, 1 departure") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_GENERAL},
        {"R3:C", PIN_LEAVING_SOON},
        {"R4:D", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 1900),
         make_trip("R3", "C", now + 300),
         make_trip("R4", "D", now + 400)},
        pins, now, 10, /*pinned_rows=*/1, /*general_pins=*/2, /*limit=*/3);
    CHECK(r.has_pin_split);
    CHECK(r.eff_general == 2);
    CHECK(r.eff_departure == 1);
    CHECK(r.eff_pinned == 3);
    CHECK(r.eff_unpinned == 0);
  }

  TEST_CASE("split: all pinned, fewer general than general_pins_count → departure spillover") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
        {"R3:C", PIN_LEAVING_SOON},
        {"R4:D", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    // general_pins_count=2, but only 1 general trip → departure gets 2 rows
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 300),
         make_trip("R3", "C", now + 400),
         make_trip("R4", "D", now + 500)},
        pins, now, 10, /*pinned_rows=*/1, /*general_pins=*/2, /*limit=*/3);
    CHECK(r.has_pin_split);
    CHECK(r.eff_general == 1);  // only 1 general trip available
    CHECK(r.eff_departure == 2);  // departure takes the surplus
    CHECK(r.eff_pinned == 3);
    CHECK(r.eff_unpinned == 0);
  }

  TEST_CASE("split: some unpinned → expansion does NOT apply") {
    std::map<std::string, PinMode> pins = {
        {"R1:A", PIN_GENERAL},
        {"R2:B", PIN_LEAVING_SOON},
    };
    unsigned int now = 1000;
    auto r = split_pin_partition(
        {make_trip("R1", "A", now + 1800),
         make_trip("R2", "B", now + 300),
         make_trip("R3", "C", now + 400)},
        pins, now, 10, /*pinned_rows=*/1, /*general_pins=*/1, /*limit=*/3);
    CHECK(r.has_pin_split);
    // Unpinned exists → old auto-bump behavior applies
    CHECK(r.eff_pinned == 2);
    CHECK(r.eff_unpinned == 1);
  }
}

// =====================================================================
// DisplayDiff — pinned_set_changed + data_changed coexistence
// =====================================================================

TEST_SUITE("DisplayDiff — pinned_set_changed no early return") {
  // helpers (mirror the ones in "DisplayDiff — pin transitions")
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
    diff.commit(keys, deps, trips, is_p, is_p, pinned_count, pinned_routes);
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

  TEST_CASE("pinned_set_changed alone — no data_changed when keys identical") {
    DisplayDiff diff;
    // R1 pinned, R2 unpinned — same order and keys
    std::vector<Trip> trips = {make_trip("R1", "A", 100), make_trip("R2", "B", 200)};
    std::set<std::string> pin_v1 = {"R1:A"};
    commit_frame(diff, trips, 1, pin_v1);

    // Now pin R2 instead of R1, but keep the same count and exact same rows
    std::set<std::string> pin_v2 = {"R2:B"};
    compute_frame(diff, trips, 1, pin_v2);

    CHECK(diff.pinned_set_changed);
    CHECK(!diff.layout_changed);
    CHECK(!diff.data_changed);
    CHECK(diff.has_changes());  // pinned_set_changed counts
  }

  TEST_CASE("pinned_set_changed WITH data_changed — both set") {
    DisplayDiff diff;
    std::vector<Trip> trips = {make_trip("R1", "A", 100), make_trip("R2", "B", 200)};
    std::set<std::string> pin_v1 = {"R1:A"};
    commit_frame(diff, trips, 1, pin_v1);

    // Change pin set AND swap rows (different keys in display)
    std::set<std::string> pin_v2 = {"R2:B"};
    std::vector<Trip> reordered = {make_trip("R2", "B", 200), make_trip("R3", "C", 300)};
    compute_frame(diff, reordered, 1, pin_v2);

    CHECK(diff.pinned_set_changed);
    CHECK(diff.data_changed);  // keys changed — was masked before the fix
    CHECK(diff.has_changes());
  }

  TEST_CASE("pinned_set_changed WITH departure time jump — both set") {
    DisplayDiff diff;
    std::vector<Trip> trips = {make_trip("R1", "A", 100), make_trip("R2", "B", 200)};
    std::set<std::string> pin_v1 = {"R1:A"};
    commit_frame(diff, trips, 1, pin_v1);

    // Change pin set AND large departure time change
    std::set<std::string> pin_v2 = {"R2:B"};
    std::vector<Trip> updated = {make_trip("R1", "A", 250), make_trip("R2", "B", 200)};
    compute_frame(diff, updated, 1, pin_v2);

    CHECK(diff.pinned_set_changed);
    CHECK(diff.data_changed);  // 100→250 is >60s threshold
  }

  TEST_CASE("pinned_set_changed with row count change — both set") {
    DisplayDiff diff;
    std::vector<Trip> trips2 = {make_trip("R1", "A", 100), make_trip("R2", "B", 200)};
    std::set<std::string> pin_v1 = {"R1:A"};
    commit_frame(diff, trips2, 1, pin_v1);

    // Different pin set AND different row count
    std::set<std::string> pin_v2 = {"R2:B"};
    std::vector<Trip> trips3 = {
        make_trip("R1", "A", 100), make_trip("R2", "B", 200), make_trip("R3", "C", 300)};
    compute_frame(diff, trips3, 1, pin_v2);

    CHECK(diff.pinned_set_changed);
    CHECK(diff.data_changed);  // size changed
  }
}

// =====================================================================
// Transition — new_trips snapshot fields
// =====================================================================

TEST_SUITE("Transition — expand snapshot") {
  TEST_CASE("reset clears new_trips snapshot") {
    Transition tr;
    tr.phase = Transition::EXPAND;
    tr.stagger_rows = 2;
    tr.old_trips = {make_trip("R1", "A"), make_trip("R2", "B")};
    tr.new_trips = {make_trip("R3", "C"), make_trip("R4", "D")};
    tr.new_is_pinned = {false, false};
    tr.new_is_leaving_soon = {false, false};
    tr.new_eff_pinned = 0;

    tr.reset();

    CHECK(tr.phase == Transition::IDLE);
    CHECK(tr.old_trips.empty());
    CHECK(tr.new_trips.empty());
    CHECK(tr.new_is_pinned.empty());
    CHECK(tr.new_is_leaving_soon.empty());
    CHECK(tr.new_eff_pinned == 0);
  }

  TEST_CASE("new_trips snapshot is independent of source") {
    Transition tr;
    std::vector<Trip> rows = {make_trip("R1", "A", 100), make_trip("R2", "B", 200)};

    tr.new_trips = rows;
    tr.new_is_pinned = {true, false};
    tr.new_is_leaving_soon = {false, false};
    tr.new_eff_pinned = 1;

    // Modify source — snapshot should be independent
    rows[0] = make_trip("R5", "E", 500);
    CHECK(tr.new_trips[0].route_id == "R1");
    CHECK(tr.new_trips[0].departure_time == 100);
  }

  TEST_CASE("new_eff_pinned defaults to 0") {
    Transition tr;
    CHECK(tr.new_eff_pinned == 0);
    CHECK(tr.new_trips.empty());
  }

  TEST_CASE("expand snapshot survives phase progression") {
    // Simulate: snapshot taken during EXPAND, used through POST_PAUSE
    Transition tr;
    tr.phase = Transition::EXPAND;
    tr.phase_start = 1000;
    tr.stagger_rows = 2;
    tr.new_trips = {make_trip("R1", "A"), make_trip("R2", "B")};
    tr.new_is_pinned = {true, false};
    tr.new_is_leaving_soon = {false, false};
    tr.new_eff_pinned = 1;

    // Transition to POST_PAUSE — snapshot should still be present
    tr.phase = Transition::POST_PAUSE;
    tr.phase_start = 2000;
    CHECK(tr.new_trips.size() == 2);
    CHECK(tr.new_eff_pinned == 1);
    CHECK(tr.new_trips[0].route_id == "R1");

    // Only reset() clears it
    tr.reset();
    CHECK(tr.new_trips.empty());
  }
}

// =====================================================================
// Default Pins (YAML `pins:` section)
// =====================================================================

TEST_SUITE("Default Pins") {

  // Simulates the defaults application logic from transit_tracker.cpp
  struct DefaultPinsContext {
    std::map<std::string, PinMode> default_pinned_routes;
    std::set<std::string> default_hidden_routes;
    std::set<std::string> default_next_only_routes;
    bool defaults_applied{false};

    std::map<std::string, PinMode> pinned_routes;
    std::set<std::string> hidden_routes;
    std::set<std::string> next_only_routes;

    void apply_defaults(const std::vector<Trip> &trips) {
      if (defaults_applied) return;
      defaults_applied = true;

      bool has_runtime_pins = !pinned_routes.empty();
      bool has_runtime_hidden = !hidden_routes.empty();
      bool has_runtime_next_only = !next_only_routes.empty();

      if (!has_runtime_pins && !default_pinned_routes.empty()) {
        std::set<std::string> seen;
        for (const auto &t : trips) {
          if (seen.count(t.route_id)) continue;
          auto it = default_pinned_routes.find(t.route_id);
          if (it != default_pinned_routes.end()) {
            pinned_routes[t.composite_key()] = it->second;
            seen.insert(t.route_id);
          }
        }
      }

      if (!has_runtime_hidden && !default_hidden_routes.empty()) {
        std::set<std::string> seen;
        for (const auto &t : trips) {
          if (seen.count(t.route_id)) continue;
          if (default_hidden_routes.count(t.route_id)) {
            hidden_routes.insert(t.composite_key());
            seen.insert(t.route_id);
          }
        }
      }

      if (!has_runtime_next_only && !default_next_only_routes.empty()) {
        std::set<std::string> seen;
        for (const auto &t : trips) {
          if (seen.count(t.route_id)) continue;
          if (default_next_only_routes.count(t.route_id)) {
            next_only_routes.insert(t.composite_key());
            seen.insert(t.route_id);
          }
        }
      }
    }
  };

  TEST_CASE("pinned default applied on first schedule data") {
    DefaultPinsContext ctx;
    ctx.default_pinned_routes["R1"] = PIN_GENERAL;

    std::vector<Trip> trips = {make_trip("R1", "Downtown"), make_trip("R2", "Airport")};
    ctx.apply_defaults(trips);

    CHECK(ctx.pinned_routes.size() == 1);
    CHECK(ctx.pinned_routes.count("R1:Downtown") == 1);
    CHECK(ctx.pinned_routes["R1:Downtown"] == PIN_GENERAL);
    CHECK(ctx.hidden_routes.empty());
    CHECK(ctx.next_only_routes.empty());
  }

  TEST_CASE("hidden default applied on first schedule data") {
    DefaultPinsContext ctx;
    ctx.default_hidden_routes.insert("R2");

    std::vector<Trip> trips = {make_trip("R1", "Downtown"), make_trip("R2", "Airport")};
    ctx.apply_defaults(trips);

    CHECK(ctx.hidden_routes.size() == 1);
    CHECK(ctx.hidden_routes.count("R2:Airport") == 1);
    CHECK(ctx.pinned_routes.empty());
  }

  TEST_CASE("next_only default applied on first schedule data") {
    DefaultPinsContext ctx;
    ctx.default_next_only_routes.insert("R1");

    std::vector<Trip> trips = {make_trip("R1", "Downtown"), make_trip("R2", "Airport")};
    ctx.apply_defaults(trips);

    CHECK(ctx.next_only_routes.size() == 1);
    CHECK(ctx.next_only_routes.count("R1:Downtown") == 1);
  }

  TEST_CASE("multiple pin modes") {
    DefaultPinsContext ctx;
    ctx.default_pinned_routes["R1"] = PIN_LEAVING_SOON;
    ctx.default_pinned_routes["R2"] = PIN_BOTH;

    std::vector<Trip> trips = {make_trip("R1", "Downtown"), make_trip("R2", "Airport")};
    ctx.apply_defaults(trips);

    CHECK(ctx.pinned_routes.size() == 2);
    CHECK(ctx.pinned_routes["R1:Downtown"] == PIN_LEAVING_SOON);
    CHECK(ctx.pinned_routes["R2:Airport"] == PIN_BOTH);
  }

  TEST_CASE("defaults only apply once") {
    DefaultPinsContext ctx;
    ctx.default_pinned_routes["R1"] = PIN_GENERAL;

    std::vector<Trip> trips1 = {make_trip("R1", "Downtown")};
    ctx.apply_defaults(trips1);
    CHECK(ctx.pinned_routes.size() == 1);

    // Second call with different trips — should NOT apply again
    ctx.pinned_routes.clear();
    std::vector<Trip> trips2 = {make_trip("R1", "Uptown")};
    ctx.apply_defaults(trips2);
    CHECK(ctx.pinned_routes.empty());  // not re-applied
  }

  TEST_CASE("defaults skipped if runtime pinned_routes already exist") {
    DefaultPinsContext ctx;
    ctx.default_pinned_routes["R1"] = PIN_GENERAL;

    // Simulate runtime state loaded from text entity before schedule arrives
    ctx.pinned_routes["R2:Airport"] = PIN_BOTH;

    std::vector<Trip> trips = {make_trip("R1", "Downtown"), make_trip("R2", "Airport")};
    ctx.apply_defaults(trips);

    // R1 should NOT be pinned because runtime state existed
    CHECK(ctx.pinned_routes.size() == 1);
    CHECK(ctx.pinned_routes.count("R2:Airport") == 1);
    CHECK(ctx.pinned_routes.count("R1:Downtown") == 0);
  }

  TEST_CASE("defaults skipped if runtime hidden_routes already exist") {
    DefaultPinsContext ctx;
    ctx.default_hidden_routes.insert("R1");

    // Simulate runtime state
    ctx.hidden_routes.insert("R3:Mall");

    std::vector<Trip> trips = {make_trip("R1", "Downtown")};
    ctx.apply_defaults(trips);

    // R1 should NOT be hidden because runtime state existed
    CHECK(ctx.hidden_routes.size() == 1);
    CHECK(ctx.hidden_routes.count("R3:Mall") == 1);
    CHECK(ctx.hidden_routes.count("R1:Downtown") == 0);
  }

  TEST_CASE("route_id matches first trip only (dedup by route_id)") {
    DefaultPinsContext ctx;
    ctx.default_pinned_routes["R1"] = PIN_GENERAL;

    // Two trips with same route_id but different headsigns
    std::vector<Trip> trips = {
      make_trip("R1", "Downtown"),
      make_trip("R1", "Uptown"),
    };
    ctx.apply_defaults(trips);

    // Only the first trip's composite key should be pinned
    CHECK(ctx.pinned_routes.size() == 1);
    CHECK(ctx.pinned_routes.count("R1:Downtown") == 1);
    CHECK(ctx.pinned_routes.count("R1:Uptown") == 0);
  }

  TEST_CASE("no defaults configured — nothing applied") {
    DefaultPinsContext ctx;

    std::vector<Trip> trips = {make_trip("R1", "Downtown")};
    ctx.apply_defaults(trips);

    CHECK(ctx.pinned_routes.empty());
    CHECK(ctx.hidden_routes.empty());
    CHECK(ctx.next_only_routes.empty());
  }

  TEST_CASE("default for non-existent route_id — silently ignored") {
    DefaultPinsContext ctx;
    ctx.default_pinned_routes["NONEXISTENT"] = PIN_GENERAL;

    std::vector<Trip> trips = {make_trip("R1", "Downtown")};
    ctx.apply_defaults(trips);

    CHECK(ctx.pinned_routes.empty());
  }

  TEST_CASE("mixed defaults — pin one, hide another, next_only a third") {
    DefaultPinsContext ctx;
    ctx.default_pinned_routes["R1"] = PIN_GENERAL;
    ctx.default_hidden_routes.insert("R2");
    ctx.default_next_only_routes.insert("R3");

    std::vector<Trip> trips = {
      make_trip("R1", "Downtown"),
      make_trip("R2", "Airport"),
      make_trip("R3", "Mall"),
    };
    ctx.apply_defaults(trips);

    CHECK(ctx.pinned_routes.size() == 1);
    CHECK(ctx.pinned_routes["R1:Downtown"] == PIN_GENERAL);
    CHECK(ctx.hidden_routes.size() == 1);
    CHECK(ctx.hidden_routes.count("R2:Airport") == 1);
    CHECK(ctx.next_only_routes.size() == 1);
    CHECK(ctx.next_only_routes.count("R3:Mall") == 1);
  }

  TEST_CASE("composite key with stop_id") {
    DefaultPinsContext ctx;
    ctx.default_pinned_routes["R1"] = PIN_GENERAL;

    std::vector<Trip> trips = {make_trip("R1", "Downtown", 1000, false, "S42")};
    ctx.apply_defaults(trips);

    CHECK(ctx.pinned_routes.size() == 1);
    CHECK(ctx.pinned_routes.count("R1:Downtown:S42") == 1);
  }
}
