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

    hs.compute(1501);  // elapsed=1500 => 150px at 100px/s, 150 % 100 = 50
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
