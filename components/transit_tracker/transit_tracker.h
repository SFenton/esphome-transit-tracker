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

    Localization* get_localization() { return &this->localization_; }

    void set_display(display::Display *display) { display_ = display; }
    void set_font(font::Font *font) { font_ = font; }
    void set_rtc(time::RealTimeClock *rtc) { rtc_ = rtc; }

    void set_base_url(const std::string &base_url) { base_url_ = base_url; }
    void set_feed_code(const std::string &feed_code) { feed_code_ = feed_code; }
    void set_display_departure_times(bool display_departure_times) { display_departure_times_ = display_departure_times; }
    void set_schedule_string(const std::string &schedule_string) { schedule_string_ = schedule_string; rebuild_route_stop_map_(); }
    void set_list_mode(const std::string &list_mode) { list_mode_ = list_mode; }
    void set_limit(int limit) { limit_ = limit; }
    void set_scroll_headsigns(bool scroll_headsigns) { scroll_headsigns_ = scroll_headsigns; }
    void set_uniform_headsign_start(bool v) { uniform_headsign_start_ = v; }
    void set_uniform_headsign_end(bool v) { uniform_headsign_end_ = v; }
    void set_scroll_routes(bool v);
    void set_show_pin_icon(bool v) { show_pin_icon_ = v; }
    void set_page_interval(int seconds) { page_interval_ = seconds * 1000; }
    void set_page_scroll_duration(float seconds) { page_scroll_duration_ = (int)(seconds * 1000); }
    void set_page_pause_duration(int seconds) { page_pause_duration_ = seconds * 1000; }
    void set_paging_style_rotate(bool v) { paging_rotate_ = v; }
    void set_pinned_rows_count(int count) { pinned_rows_count_ = std::max(1, std::min(count, limit_ - 1)); }
    int get_pinned_rows_count() const { return pinned_rows_count_; }

    void set_unit_display(UnitDisplay unit_display) { this->localization_.set_unit_display(unit_display); }
    void add_abbreviation(const std::string &from, const std::string &to) { abbreviations_[from] = to; }
    void set_default_route_color(const Color &color) { default_route_color_ = color; }
    void add_route_style(const std::string &route_id, const std::string &name, const Color &color) { route_styles_[route_id] = RouteStyle{name, color}; }

    void set_abbreviations_from_text(const std::string &text);
    void set_route_styles_from_text(const std::string &text);
    void set_hidden_routes_from_text(const std::string &text);
    void set_pinned_routes_from_text(const std::string &text);

    void set_realtime_color(const Color &color);
    void set_divider_color(const Color &color) { divider_color_ = color; }
    void set_divider_color_from_text(const std::string &text);
    void set_route_color_overrides_from_text(const std::string &text);

    void rebuild_route_stop_map_();

#ifdef USE_TEXT
    void set_hidden_routes_text(text::Text *text) { hidden_routes_text_ = text; }
    void set_pinned_routes_text(text::Text *text) { pinned_routes_text_ = text; }
    void set_divider_color_text(text::Text *text) { divider_color_text_ = text; }
    void set_route_color_overrides_text(text::Text *text) { route_color_overrides_text_ = text; }
#endif

#ifdef USE_MQTT
    void publish_mqtt_routes_();
    void publish_mqtt_divider_color_();
    void publish_mqtt_route_colors_();
    void update_mqtt_route_state_(const std::string &composite_key, bool visible);
    void persist_hidden_routes_();
    void persist_pinned_routes_();
    void persist_divider_color_();
    void persist_route_color_overrides_();
    static std::string slugify_(const std::string &input);
#endif

  protected:
    static constexpr int scroll_speed = 10; // pixels/second
    static constexpr int marquee_gap = 20; // pixels between repetitions in continuous scroll
    int page_interval_ = 5000; // ms per page when scroll_routes is on
    int page_scroll_duration_ = 500; // ms for page scroll transition
    int last_page_index_ = -1;
    int scroll_from_page_ = -1;
    unsigned long page_scroll_start_ = 0;
    int page_pause_duration_ = 1000;       // ms to pause after page scroll before h-scroll resumes
    unsigned long h_scroll_start_time_ = 0; // when horizontal scrolling began for current page
    unsigned long page_timer_start_ = 0;    // when current page interval timer started
    unsigned long page_pause_start_ = 0;    // when pause began (0 = not pausing)
    bool page_pause_is_pre_ = false;        // true = pre-scroll pause, false = post-scroll pause
    bool page_change_pending_ = false;      // waiting for scroll to return to position 0
    int current_page_index_ = 0;            // explicitly tracked page index
    int h_scroll_cycles_at_pending_ = -1;   // scroll cycles completed when pending was set

    // Split layout paging state (pinned section)
    int pinned_page_index_{0};
    unsigned long pinned_page_timer_{0};
    unsigned long pinned_h_scroll_start_{0};

    // Split layout paging state (unpinned section)
    int split_unpinned_page_index_{0};
    unsigned long split_unpinned_page_timer_{0};
    unsigned long split_unpinned_h_scroll_start_{0};

    // Split layout scroll animation state
    unsigned long split_scroll_start_{0};       // millis when conveyor animation began (0 = idle)
    int split_old_pinned_page_{0};              // pinned page index before animation
    int split_old_unpinned_page_{0};            // unpinned page index before animation
    unsigned long split_pause_start_{0};        // when split pause began (0 = not pausing)
    bool split_pause_is_pre_{false};            // true = pre-scroll pause, false = post-scroll pause

    std::string from_now_(time_t unix_timestamp, uint rtc_now) const;
    void draw_text_centered_(const char *text, Color color);
    void draw_realtime_icon_(int bottom_right_x, int bottom_right_y, unsigned long now);

    void draw_trip(
      const Trip &trip, int y_offset, int font_height, unsigned long uptime, uint rtc_now,
      bool no_draw = false, int *headsign_overflow_out = nullptr, int *headsign_width_out = nullptr,
      int h_scroll_offset = -1, int total_scroll_distance = 0,
      int headsign_clipping_start_override = -1, int headsign_clipping_end_override = -1,
      bool is_pinned = false
    );

    Localization localization_{};
    ScheduleState schedule_state_;

    display::Display *display_;
    font::Font *font_;
    time::RealTimeClock *rtc_;

    websockets::WebsocketsClient ws_client_{};

    void on_ws_message_(websockets::WebsocketsMessage message);
    void on_ws_event_(websockets::WebsocketsEvent event, String data);
    void connect_ws_();
    int connection_attempts_ = 0;
    unsigned long last_heartbeat_ = 0;
    bool has_ever_connected_ = false;
    bool fully_closed_ = false;

    std::string base_url_;
    std::string feed_code_;
    std::string schedule_string_;
    std::string list_mode_;
    bool display_departure_times_ = true;
    int limit_;

    std::map<std::string, std::string> abbreviations_;
    Color default_route_color_ = Color(0x028e51);
    std::map<std::string, RouteStyle> route_styles_;
    std::map<std::string, Color> route_color_overrides_;  // MQTT-set per-route color overrides
    bool scroll_headsigns_ = false;
    bool show_pin_icon_ = true;
    bool uniform_headsign_start_ = false;
    bool uniform_headsign_end_ = false;
    bool scroll_routes_ = false;
    bool paging_rotate_ = false; // false = full page, true = single rotate

    std::set<std::string> hidden_routes_;
    std::set<std::string> pinned_routes_;
    int pinned_rows_count_{1};
    std::map<std::string, std::string> route_stop_map_;  // route_id -> stop_id (from schedule_string_)

#ifdef USE_TEXT
    text::Text *hidden_routes_text_{nullptr};
    text::Text *pinned_routes_text_{nullptr};
    text::Text *divider_color_text_{nullptr};
    text::Text *route_color_overrides_text_{nullptr};
#endif

#ifdef USE_MQTT
    std::set<std::string> mqtt_subscribed_routes_;
    std::set<std::string> mqtt_subscribed_route_colors_;
    bool mqtt_routes_pending_{false};
    bool mqtt_divider_color_pending_{false};
    bool mqtt_divider_color_subscribed_{false};
#endif

    Color realtime_color_ = Color(0x20FF00);
    Color realtime_color_dark_ = Color(0x00A700);
    Color divider_color_ = Color(0xFF0000);
};


}  // namespace transit_tracker
}  // namespace esphome