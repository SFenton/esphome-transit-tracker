#pragma once

#include <map>
#include <set>
#include <ArduinoWebsockets.h>

#include "esphome/core/component.h"
#include "esphome/components/display/display.h"
#include "esphome/components/font/font.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/components/text_sensor/text_sensor.h"

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
    void set_route_names_sensor(text_sensor::TextSensor *sensor) { route_names_sensor_ = sensor; }

    void set_base_url(const std::string &base_url) { base_url_ = base_url; }
    void set_feed_code(const std::string &feed_code) { feed_code_ = feed_code; }
    void set_display_departure_times(bool display_departure_times) { display_departure_times_ = display_departure_times; }
    void set_schedule_string(const std::string &schedule_string) { schedule_string_ = schedule_string; }
    void set_list_mode(const std::string &list_mode) { list_mode_ = list_mode; }
    void set_limit(int limit) { limit_ = limit; }
    void set_scroll_headsigns(bool scroll_headsigns) { scroll_headsigns_ = scroll_headsigns; }
    void set_uniform_headsign_start(bool v) { uniform_headsign_start_ = v; }
    void set_uniform_headsign_end(bool v) { uniform_headsign_end_ = v; }
    void set_scroll_routes(bool v);
    void set_page_interval(int seconds) { page_interval_ = seconds * 1000; }
    void set_paging_style_rotate(bool v) { paging_rotate_ = v; }

    void set_unit_display(UnitDisplay unit_display) { this->localization_.set_unit_display(unit_display); }
    void add_abbreviation(const std::string &from, const std::string &to) { abbreviations_[from] = to; }
    void set_default_route_color(const Color &color) { default_route_color_ = color; }
    void add_route_style(const std::string &route_id, const std::string &name, const Color &color) { route_styles_[route_id] = RouteStyle{name, color}; }

    void set_abbreviations_from_text(const std::string &text);
    void set_route_styles_from_text(const std::string &text);
    void set_hidden_routes_from_text(const std::string &text);

    void set_realtime_color(const Color &color);

  protected:
    static constexpr int scroll_speed = 10; // pixels/second
    static constexpr int idle_time_left = 5000;
    static constexpr int idle_time_right = 1000;
    int page_interval_ = 5000; // ms per page when scroll_routes is on

    std::string from_now_(time_t unix_timestamp, uint rtc_now) const;
    void draw_text_centered_(const char *text, Color color);
    void draw_realtime_icon_(int bottom_right_x, int bottom_right_y, unsigned long now);

    void draw_trip(
      const Trip &trip, int y_offset, int font_height, unsigned long uptime, uint rtc_now,
      bool no_draw = false, int *headsign_overflow_out = nullptr, int scroll_cycle_duration = 0,
      int headsign_clipping_start_override = -1, int headsign_clipping_end_override = -1
    );

    Localization localization_{};
    ScheduleState schedule_state_;

    display::Display *display_;
    font::Font *font_;
    time::RealTimeClock *rtc_;
    text_sensor::TextSensor *route_names_sensor_{nullptr};

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
    bool scroll_headsigns_ = false;
    bool uniform_headsign_start_ = false;
    bool uniform_headsign_end_ = false;
    bool scroll_routes_ = false;
    bool paging_rotate_ = false; // false = full page, true = single rotate

    std::set<std::string> hidden_routes_;

    Color realtime_color_ = Color(0x20FF00);
    Color realtime_color_dark_ = Color(0x00A700);
};


}  // namespace transit_tracker
}  // namespace esphome