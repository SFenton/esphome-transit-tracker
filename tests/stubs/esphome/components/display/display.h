// Minimal stub for tests — provides Color so schedule_state.h compiles
#pragma once
#include <cstdint>

namespace esphome {

struct Color {
  uint8_t r{0}, g{0}, b{0}, w{0};

  Color() = default;
  explicit Color(uint32_t hex)
      : r((hex >> 16) & 0xFF), g((hex >> 8) & 0xFF), b(hex & 0xFF) {}
  Color(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}

  bool operator==(const Color &o) const { return r == o.r && g == o.g && b == o.b; }
  bool operator!=(const Color &o) const { return !(*this == o); }
};

}  // namespace esphome
