#pragma once

#include <map>
#include <set>
#include <ArduinoWebsockets.h>

#include "esphome/core/component.h"
#include "esphome/components/display/display.h"
#include "esphome/components/font/font.h"
#include "esphome/components/time/real_time_clock.h"

#ifdef USE_MQTT
#include "esphome/components/mqtt/mqtt_client.h"
#endif

#ifdef USE_TEXT
#include "esphome/components/text/text.h"
#endif

#include "schedule_state.h"
#include "display_types.h"
#include "localization.h"

namespace esphome {
namespace transit_tracker {

struct RouteStyle {
  std::string name;
  Color color;
};

class TransitTracker : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_shutdown() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void reconnect();
  void close(bool fully = false);
  void draw_schedule();

  Localization *get_localization() { return &this->localization_; }

  // ---- Config setters ----
  void set_display(display::Display *d) { display_ = d; }
  void set_font(font::Font *f) { font_ = f; }
  void set_rtc(time::RealTimeClock *r) { rtc_ = r; }
  void set_base_url(const std::string &u) { base_url_ = u; }
  void set_feed_code(const std::string &c) { feed_code_ = c; }
  void set_display_departure_times(bool v) { display_departure_times_ = v; }
  void set_schedule_string(const std::string &s) { schedule_string_ = s; rebuild_route_stop_map_(); }
  void set_list_mode(const std::string &m) { list_mode_ = m; }
  void set_limit(int n) { limit_ = n; }
  void set_scroll_headsigns(bool v) { scroll_headsigns_ = v; }
  void set_uniform_headsign_start(bool v) { uniform_headsign_start_ = v; }
  void set_uniform_headsign_end(bool v) { uniform_headsign_end_ = v; }
  void set_scroll_routes(bool v);
  void set_respect_pin_inset(bool v) { respect_pin_inset_ = v; }
  void set_always_scroll_or_replace(bool v) { always_scroll_or_replace_ = v; }
  void set_scroll_speed(const std::string &speed);
  void set_page_interval(int seconds) { page_interval_ = seconds * 1000; }
  void set_page_pause_duration(int seconds) { page_pause_duration_ = seconds * 1000; }
  void set_pinned_rows_count(int c) { pinned_rows_count_ = std::max(1, std::min(c, limit_ - 1)); }
  int get_pinned_rows_count() const { return pinned_rows_count_; }
  void set_general_pins_count(int c) { general_pins_count_ = std::max(1, c); }
  int get_general_pins_count() const { return general_pins_count_; }
  void set_leaving_soon_threshold(int minutes) { leaving_soon_threshold_min_ = std::max(5, std::min(60, minutes)); }
  int get_leaving_soon_threshold() const { return leaving_soon_threshold_min_; }
  void set_unit_display(UnitDisplay u) { localization_.set_unit_display(u); }
  void add_abbreviation(const std::string &from, const std::string &to) { abbreviations_[from] = to; }
  void set_default_route_color(const Color &c) { default_route_color_ = c; }
  void add_route_style(const std::string &id, const std::string &name, const Color &c) { route_styles_[id] = {name, c}; }
  void set_abbreviations_from_text(const std::string &text);
  void set_route_styles_from_text(const std::string &text);
  void set_hidden_routes_from_text(const std::string &text);
  void set_pinned_routes_from_text(const std::string &text);
  void set_next_only_routes_from_text(const std::string &text);
  void set_realtime_color(const Color &color);
  void set_route_color_overrides_from_text(const std::string &text);

  // Default pins from YAML config (matched by route_id on first schedule data)
  void add_default_pinned_route(const std::string &route_id, PinMode mode) { default_pinned_routes_[route_id] = mode; }
  void add_default_hidden_route(const std::string &route_id) { default_hidden_routes_.insert(route_id); }
  void add_default_next_only_route(const std::string &route_id) { default_next_only_routes_.insert(route_id); }

#ifdef USE_TEXT
  void set_hidden_routes_text(text::Text *t) { hidden_routes_text_ = t; }
  void set_pinned_routes_text(text::Text *t) { pinned_routes_text_ = t; }
  void set_next_only_routes_text(text::Text *t) { next_only_routes_text_ = t; }
  void set_route_color_overrides_text(text::Text *t) { route_color_overrides_text_ = t; }
#endif

#ifdef USE_MQTT
  void publish_mqtt_routes_();
  void publish_mqtt_route_colors_();
  void persist_hidden_routes_();
  void persist_pinned_routes_();
  void persist_next_only_routes_();
  void persist_route_color_overrides_();
  static std::string slugify_(const std::string &input);
#endif

 protected:
  // ---- Drawing helpers ----
  void draw_text_centered_(const char *text, Color color);
  void draw_realtime_icon_(int bottom_right_x, int bottom_right_y, unsigned long now);
  void draw_trip(const Trip &trip, int y_offset, int font_height,
                 unsigned long uptime, uint rtc_now,
                 bool no_draw = false,
                 int *headsign_overflow_out = nullptr,
                 int *headsign_width_out = nullptr,
                 int h_scroll_offset = -1, int total_scroll_distance = 0,
                 bool is_pinned = false, bool is_leaving_soon = false);

  // ---- draw_schedule helpers ----
  void rebuild_route_stop_map_();
  void compute_uniform_clipping_(const std::vector<Trip> &trips, uint rtc_now);
  void compute_h_scroll_(const std::vector<Trip> &trips, int fh,
                          unsigned long now, uint rtc_now);
  void begin_transition_(const std::vector<Trip> &old_trips,
                          const std::vector<bool> &old_is_pinned,
                          const std::vector<bool> &old_is_leaving_soon,
                          int old_eff_pinned, bool layout_or_swap,
                          unsigned long now);
  void begin_page_transition_(const std::vector<Trip> &pinned_pool,
                               const std::vector<Trip> &unpinned_pool,
                               int eff_pinned, int eff_unpinned,
                               int gen_pager_pool, int gen_pager_slots,
                               int dep_pager_pool, int dep_pager_slots,
                               bool pinned_due, bool unpinned_due,
                               const std::vector<Trip> &old_trips,
                               const std::vector<bool> &old_is_pinned,
                               const std::vector<bool> &old_is_leaving_soon,
                               unsigned long now);
  void tick_transition_(unsigned long now, int fh,
                         const std::vector<Trip> &pinned_pool,
                         const std::vector<Trip> &unpinned_pool,
                         int eff_pinned, int eff_unpinned);
  void render_frame_(const std::vector<Trip> &rows,
                      const std::vector<bool> &row_pinned,
                      const std::vector<bool> &row_leaving_soon,
                      int eff_pinned, unsigned long now, uint rtc_now,
                      int fh, int y_base);

  // ---- Core state ----
  Localization localization_{};
  ScheduleState schedule_state_;
  display::Display *display_{nullptr};
  font::Font *font_{nullptr};
  time::RealTimeClock *rtc_{nullptr};

  // ---- WebSocket ----
  websockets::WebsocketsClient ws_client_{};
  void on_ws_message_(websockets::WebsocketsMessage message);
  void on_ws_event_(websockets::WebsocketsEvent event, String data);
  void connect_ws_();
  int connection_attempts_{0};
  unsigned long last_heartbeat_{0};
  bool has_ever_connected_{false};
  bool fully_closed_{false};

  // ---- Config ----
  std::string base_url_;
  std::string feed_code_;
  std::string schedule_string_;
  std::string list_mode_;
  bool display_departure_times_{true};
  int limit_{3};
  std::map<std::string, std::string> abbreviations_;
  Color default_route_color_{Color(0x028e51)};
  std::map<std::string, RouteStyle> route_styles_;
  std::map<std::string, Color> route_color_overrides_;
  bool scroll_headsigns_{false};
  bool respect_pin_inset_{true};
  bool uniform_headsign_start_{false};
  bool uniform_headsign_end_{false};
  bool scroll_routes_{false};
  bool always_scroll_or_replace_{false};
  int page_interval_{5000};
  int page_pause_duration_{1000};
  int pinned_rows_count_{1};
  int general_pins_count_{1};
  int leaving_soon_threshold_min_{10};
  Color realtime_color_{Color(0x20FF00)};
  Color realtime_color_dark_{Color(0x00A700)};

  // ---- Route management ----
  std::set<std::string> hidden_routes_;
  std::map<std::string, PinMode> pinned_routes_;
  std::set<std::string> next_only_routes_;
  std::map<std::string, std::string> route_stop_map_;

  // Default pins from YAML (applied once on first schedule data)
  std::map<std::string, PinMode> default_pinned_routes_;
  std::set<std::string> default_hidden_routes_;
  std::set<std::string> default_next_only_routes_;
  bool defaults_applied_{false};

  // ---- Animation state (the ENTIRE animation state for the component) ----
  Transition transition_;
  DisplayDiff diff_;
  HScrollState h_scroll_;
  ScrollContainer pinned_pager_;
  ScrollContainer departure_pager_;
  ScrollContainer unpinned_pager_;
  unsigned long post_pause_end_{0};
  unsigned long idle_since_{0};

  // ---- Per-frame rendering state (set once at top of draw_schedule) ----
  int frame_pin_inset_{0};
  bool frame_respect_pin_inset_{true};
  bool committed_respect_pin_inset_{true};
  int frame_uniform_clip_start_{-1};
  int frame_uniform_clip_end_{-1};

#ifdef USE_TEXT
  text::Text *hidden_routes_text_{nullptr};
  text::Text *pinned_routes_text_{nullptr};
  text::Text *next_only_routes_text_{nullptr};
  text::Text *route_color_overrides_text_{nullptr};
#endif

#ifdef USE_MQTT
  std::set<std::string> mqtt_subscribed_routes_;
  std::set<std::string> mqtt_subscribed_route_colors_;
  std::set<std::string> mqtt_published_routes_;
  std::set<std::string> mqtt_published_route_color_ids_;
  bool mqtt_routes_pending_{false};
  bool mqtt_persist_pending_{false};
#endif
};

}  // namespace transit_tracker
}  // namespace esphome
