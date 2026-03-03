#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "schedule_state.h"
#include "string_utils.h"
#include "string_utils.cpp"  // single TU build

#include <set>
#include <string>
#include <vector>

using namespace esphome;
using namespace esphome::transit_tracker;

// =====================================================================
// Helpers — mirror the persist / restore logic from transit_tracker.cpp
// =====================================================================

/// Serialize a set of composite keys using '\x1F' as the delimiter.
/// This mirrors persist_hidden_routes_(), persist_pinned_routes_(), and
/// persist_next_only_routes_().  A leading '\x1F' prefix ensures the
/// deserializer always detects the new format, even with a single key.
static std::string serialize_keys(const std::set<std::string> &keys) {
  std::string s;
  for (const auto &k : keys) {
    s += '\x1F';
    s += k;
  }
  return s;
}

/// Serialize using the OLD (buggy) ';' delimiter.
static std::string serialize_keys_old(const std::set<std::string> &keys) {
  std::string s;
  for (const auto &k : keys) {
    if (!s.empty()) s += ';';
    s += k;
  }
  return s;
}

/// Deserialize a persisted string back into a set of composite keys.
/// This mirrors set_pinned_routes_from_text() et al.: if '\x1F' is present
/// we split on it; otherwise we fall back to ';' for backward compatibility.
static std::set<std::string> deserialize_keys(const std::string &text) {
  std::set<std::string> out;
  if (text.empty()) return out;
  char delim = (text.find('\x1F') != std::string::npos) ? '\x1F' : ';';
  for (const auto &id : split(text, delim)) {
    if (!id.empty()) out.insert(id);
  }
  return out;
}

/// Build a composite key the same way Trip::composite_key() does.
static std::string make_key(const std::string &route_id,
                            const std::string &headsign,
                            const std::string &stop_id = "") {
  std::string key = route_id + ":" + headsign;
  if (!stop_id.empty()) key += ":" + stop_id;
  return key;
}

// =====================================================================
// Basic round-trip (no semicolons in data)
// =====================================================================

TEST_SUITE("serialization round-trip") {

  TEST_CASE("empty set round-trips") {
    std::set<std::string> keys;
    auto s = serialize_keys(keys);
    CHECK(s.empty());
    auto restored = deserialize_keys(s);
    CHECK(restored.empty());
  }

  TEST_CASE("single key round-trips") {
    std::set<std::string> keys = {make_key("R1", "Downtown", "S1")};
    auto s = serialize_keys(keys);
    auto restored = deserialize_keys(s);
    CHECK(restored == keys);
  }

  TEST_CASE("multiple keys round-trip") {
    std::set<std::string> keys = {
        make_key("R1", "Downtown", "S1"),
        make_key("R2", "Uptown"),
        make_key("R3", "Crosstown", "S7"),
    };
    auto s = serialize_keys(keys);
    auto restored = deserialize_keys(s);
    CHECK(restored == keys);
  }

// =====================================================================
// Keys containing semicolons (the bug scenario)
// =====================================================================

  TEST_CASE("key with semicolon in headsign round-trips correctly") {
    // Some transit agencies include semicolons in headsign text,
    // e.g. "Downtown; Express" or "A;B".  composite_key() would
    // produce "R1:Downtown; Express:S1".
    std::set<std::string> keys = {make_key("R1", "Downtown; Express", "S1")};
    auto s = serialize_keys(keys);

    // The serialized form uses '\x1F' as the delimiter/prefix,
    // correctly preserving the ';' that appears inside the key data.
    CHECK(s.find(';') != std::string::npos);   // ';' is in the DATA
    CHECK(s.find('\x1F') != std::string::npos); // '\x1F' prefix present

    auto restored = deserialize_keys(s);
    CHECK(restored == keys);
    CHECK(restored.size() == 1);
  }

  TEST_CASE("multiple keys, one with semicolon, round-trip correctly") {
    std::set<std::string> keys = {
        make_key("R1", "Downtown; Express", "S1"),
        make_key("R2", "Uptown"),
    };
    auto s = serialize_keys(keys);

    // '\x1F' is the record delimiter; ';' appears only inside key data
    CHECK(s.find('\x1F') != std::string::npos);

    auto restored = deserialize_keys(s);
    CHECK(restored == keys);
    CHECK(restored.size() == 2);
  }

  TEST_CASE("key with multiple semicolons in headsign") {
    std::set<std::string> keys = {
        make_key("R1", "A;B;C", "S1"),
    };
    auto s = serialize_keys(keys);
    auto restored = deserialize_keys(s);
    CHECK(restored == keys);
    CHECK(restored.size() == 1);
  }

// =====================================================================
// OLD delimiter: demonstrates the bug and backward compatibility
// =====================================================================

  TEST_CASE("OLD delimiter corrupts key with semicolon") {
    // This documents the bug that existed before the fix.
    std::set<std::string> original = {make_key("R1", "Downtown; Express", "S1")};
    auto s = serialize_keys_old(original);

    // Old-style deserialize (falls back to ';' because no '\x1F' present)
    auto restored = deserialize_keys(s);

    // The key gets split into TWO fragments — neither matches the original.
    CHECK(restored != original);
    CHECK(restored.size() == 2);  // "R1:Downtown" and " Express:S1"
  }

  TEST_CASE("backward compat: old-style string without semicolons in data") {
    // Keys that don't contain semicolons were serialized with ';' in the
    // old format.  The new deserializer must still handle them.
    std::set<std::string> keys = {
        make_key("R1", "Downtown", "S1"),
        make_key("R2", "Uptown"),
    };
    auto s = serialize_keys_old(keys);

    // No '\x1F' → falls back to ';' splitting
    CHECK(s.find('\x1F') == std::string::npos);

    auto restored = deserialize_keys(s);
    CHECK(restored == keys);
  }

// =====================================================================
// Edge cases
// =====================================================================

  TEST_CASE("key with only semicolons as headsign") {
    std::set<std::string> keys = {make_key("R1", ";;;")};
    auto s = serialize_keys(keys);
    auto restored = deserialize_keys(s);
    CHECK(restored == keys);
    CHECK(restored.size() == 1);
  }

  TEST_CASE("headsign with trailing semicolon") {
    std::set<std::string> keys = {make_key("R1", "Express;", "S1")};
    auto s = serialize_keys(keys);
    auto restored = deserialize_keys(s);
    CHECK(restored == keys);
  }

  TEST_CASE("headsign with leading semicolon") {
    std::set<std::string> keys = {make_key("R1", ";Local", "S1")};
    auto s = serialize_keys(keys);
    auto restored = deserialize_keys(s);
    CHECK(restored == keys);
  }

  TEST_CASE("many keys all with semicolons") {
    std::set<std::string> keys;
    for (int i = 0; i < 10; i++) {
      keys.insert(make_key("R" + std::to_string(i),
                            "Headsign;" + std::to_string(i),
                            "S" + std::to_string(i)));
    }
    auto s = serialize_keys(keys);
    auto restored = deserialize_keys(s);
    CHECK(restored == keys);
    CHECK(restored.size() == 10);
  }

  TEST_CASE("unit separator in text triggers new-style parsing") {
    // Even if ';' is also present (inside key data), the '\x1F' detection
    // must take precedence.
    std::string manual = "R1:A;B\x1F" "R2:C";
    auto restored = deserialize_keys(manual);
    CHECK(restored.size() == 2);
    CHECK(restored.count("R1:A;B") == 1);
    CHECK(restored.count("R2:C") == 1);
  }
}
