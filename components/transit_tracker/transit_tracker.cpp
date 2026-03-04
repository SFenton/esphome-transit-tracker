#include "transit_tracker.h"
#include "string_utils.h"

#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/components/json/json_util.h"
#include "esphome/components/watchdog/watchdog.h"
#include "esphome/components/network/util.h"

namespace esphome {
namespace transit_tracker {

static const char *TAG = "transit_tracker.component";

// =====================================================================
// Setup / Loop / Lifecycle
// =====================================================================

void TransitTracker::setup() {
  this->ws_client_.onMessage([this](websockets::WebsocketsMessage message) {
    this->on_ws_message_(message);
  });
  this->ws_client_.onEvent([this](websockets::WebsocketsEvent event, String data) {
    this->on_ws_event_(event, data);
  });
  this->rebuild_route_stop_map_();
  this->connect_ws_();

  this->set_interval("check_stale_trips", 10000, [this]() {
    if (this->ws_client_.available() && !this->schedule_state_.trips.empty()) {
      bool has_stale = false;
      this->schedule_state_.mutex.lock();
      auto now = this->rtc_->now();
      if (now.is_valid()) {
        for (auto &trip : this->schedule_state_.trips) {
          if (now.timestamp - trip.departure_time > 60) { has_stale = true; break; }
        }
      }
      this->schedule_state_.mutex.unlock();
      if (has_stale) {
        ESP_LOGD(TAG, "Stale trips detected, reconnecting");
        this->reconnect();
      }
    }
  });
}

void TransitTracker::loop() {
  this->ws_client_.poll();

#ifdef USE_MQTT
  if (mqtt::global_mqtt_client != nullptr && mqtt::global_mqtt_client->is_connected()) {
    if (this->mqtt_routes_pending_) {
      this->publish_mqtt_routes_();
      this->publish_mqtt_route_colors_();
    }
    if (this->mqtt_persist_pending_) {
      this->mqtt_persist_pending_ = false;
      this->persist_hidden_routes_();
      this->persist_pinned_routes_();
    }
  }
#endif

  if (this->last_heartbeat_ != 0 && millis() - this->last_heartbeat_ > 60000) {
    ESP_LOGW(TAG, "Heartbeat timeout, reconnecting");
    this->reconnect();
    return;
  }
}

void TransitTracker::dump_config() {
  ESP_LOGCONFIG(TAG, "Transit Tracker:");
  ESP_LOGCONFIG(TAG, "  Base URL: %s", this->base_url_.c_str());
  ESP_LOGCONFIG(TAG, "  Schedule: %s", this->schedule_string_.c_str());
  ESP_LOGCONFIG(TAG, "  Limit: %d", this->limit_);
  ESP_LOGCONFIG(TAG, "  List mode: %s", this->list_mode_.c_str());
  ESP_LOGCONFIG(TAG, "  Display departure times: %s", this->display_departure_times_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "  Scroll Headsigns: %s", this->scroll_headsigns_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "  Scroll Routes: %s", this->scroll_routes_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "  Show Pin Icon: %s", this->show_pin_icon_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "  Respect Pin Inset: %s", this->respect_pin_inset_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "  Always Scroll or Replace: %s", this->always_scroll_or_replace_ ? "true" : "false");
}

void TransitTracker::reconnect() { this->close(); this->connect_ws_(); }

void TransitTracker::close(bool fully) {
  if (fully) this->fully_closed_ = true;
  this->ws_client_.close();
}

void TransitTracker::on_shutdown() {
  this->cancel_interval("check_stale_trips");
  this->close(true);
}

// =====================================================================
// WebSocket
// =====================================================================

void TransitTracker::on_ws_message_(websockets::WebsocketsMessage message) {
  ESP_LOGV(TAG, "Received message: %s", message.rawData().c_str());

  bool valid = json::parse_json(message.rawData(), [this](JsonObject root) -> bool {
    if (root["event"].as<std::string>() == "heartbeat") {
      ESP_LOGD(TAG, "Received heartbeat");
      this->last_heartbeat_ = millis();
      return true;
    }
    if (root["event"].as<std::string>() != "schedule") return true;

    ESP_LOGD(TAG, "Received schedule update");
    this->schedule_state_.mutex.lock();
    this->schedule_state_.trips.clear();

    auto data = root["data"].as<JsonObject>();
    for (auto trip : data["trips"].as<JsonArray>()) {
      std::string headsign = trip["headsign"].as<std::string>();
      for (const auto &abbr : this->abbreviations_) {
        size_t pos = headsign.find(abbr.first);
        if (pos != std::string::npos)
          headsign.replace(pos, abbr.first.length(), abbr.second);
      }

      auto route_id = trip["routeId"].as<std::string>();
      auto route_style = this->route_styles_.find(route_id);

      Color route_color = this->default_route_color_;
      std::string route_name = trip["routeName"].as<std::string>();

      if (route_style != this->route_styles_.end()) {
        route_color = route_style->second.color;
        route_name = route_style->second.name;
      } else if (!trip["routeColor"].isNull()) {
        route_color = Color(std::stoul(trip["routeColor"].as<std::string>(), nullptr, 16));
      }

      // Apply MQTT color override (highest priority)
      auto color_override = this->route_color_overrides_.find(route_id);
      if (color_override != this->route_color_overrides_.end()) {
        route_color = color_override->second;
      }

      std::string stop_id;
      if (!trip["stopId"].isNull()) {
        stop_id = trip["stopId"].as<std::string>();
      } else {
        auto it = this->route_stop_map_.find(route_id);
        if (it != this->route_stop_map_.end()) stop_id = it->second;
      }

      std::string stop_name;
      if (!trip["stopName"].isNull()) stop_name = trip["stopName"].as<std::string>();

      this->schedule_state_.trips.push_back({
        .route_id = route_id,
        .route_name = route_name,
        .route_color = route_color,
        .headsign = headsign,
        .stop_id = stop_id,
        .stop_name = stop_name,
        .arrival_time = trip["arrivalTime"].as<time_t>(),
        .departure_time = trip["departureTime"].as<time_t>(),
        .is_realtime = trip["isRealtime"].as<bool>(),
      });
    }

    ESP_LOGD(TAG, "Schedule update: %d trip(s)", (int)this->schedule_state_.trips.size());
    for (size_t i = 0; i < this->schedule_state_.trips.size(); i++) {
      const auto &t = this->schedule_state_.trips[i];
      ESP_LOGD(TAG, "  [%d] %s  dep=%ld  rt=%s",
               (int)i, t.composite_key().c_str(), (long)t.departure_time, t.is_realtime ? "Y" : "N");
    }

    this->schedule_state_.mutex.unlock();

#ifdef USE_MQTT
    this->mqtt_routes_pending_ = true;
#endif
    return true;
  });

  if (!valid) this->status_set_error(LOG_STR("Failed to parse schedule data"));
}

void TransitTracker::on_ws_event_(websockets::WebsocketsEvent event, String data) {
  if (event == websockets::WebsocketsEvent::ConnectionOpened) {
    ESP_LOGD(TAG, "WebSocket connection opened");
    auto message = json::build_json([this](JsonObject root) {
      root["event"] = "schedule:subscribe";
      auto data = root["data"].to<JsonObject>();
      if (!this->feed_code_.empty()) data["feedCode"] = this->feed_code_;
      data["routeStopPairs"] = this->schedule_string_;
      data["limit"] = this->scroll_routes_ ? this->limit_ * 3 : this->limit_;
      data["sortByDeparture"] = this->display_departure_times_;
      data["listMode"] = this->list_mode_;
    });
    ESP_LOGV(TAG, "Sending message: %s", message.c_str());
    this->ws_client_.send(message.c_str());
  } else if (event == websockets::WebsocketsEvent::ConnectionClosed) {
    ESP_LOGD(TAG, "WebSocket connection closed");
    if (!this->fully_closed_ && this->connection_attempts_ == 0) {
      this->defer([this]() { this->connect_ws_(); });
    }
  } else if (event == websockets::WebsocketsEvent::GotPing) {
    ESP_LOGV(TAG, "Received ping");
  } else if (event == websockets::WebsocketsEvent::GotPong) {
    ESP_LOGV(TAG, "Received pong");
  }
}

void TransitTracker::connect_ws_() {
  if (this->base_url_.empty()) { ESP_LOGW(TAG, "No base URL set, not connecting"); return; }
  if (this->fully_closed_) { ESP_LOGW(TAG, "Connection fully closed, not reconnecting"); return; }
  if (this->ws_client_.available(true)) { ESP_LOGV(TAG, "Already connected"); return; }

  watchdog::WatchdogManager wdm(20000);
  this->last_heartbeat_ = 0;
  ESP_LOGD(TAG, "Connecting to WebSocket (attempt %d): %s", this->connection_attempts_, this->base_url_.c_str());

  bool ok = false;
  if (esphome::network::is_connected()) {
    ok = this->ws_client_.connect(this->base_url_.c_str());
  } else {
    ESP_LOGW(TAG, "Not connected to network; skipping connection attempt");
  }

  if (!ok) {
    this->connection_attempts_++;
    if (this->connection_attempts_ >= 3)
      this->status_set_error(LOG_STR("Failed to connect to WebSocket server"));
    if (this->connection_attempts_ >= 15) {
      ESP_LOGE(TAG, "Could not connect within 15 attempts; rebooting");
      App.reboot();
    }
    auto timeout = std::min(15000, this->connection_attempts_ * 5000);
    ESP_LOGW(TAG, "Failed to connect, retrying in %ds", timeout / 1000);
    this->set_timeout("reconnect", timeout, [this]() { this->connect_ws_(); });
  } else {
    this->has_ever_connected_ = true;
    this->connection_attempts_ = 0;
    this->status_clear_error();
  }
}

// =====================================================================
// Text Parsers & Config Setters
// =====================================================================

void TransitTracker::set_abbreviations_from_text(const std::string &text) {
  this->abbreviations_.clear();
  for (const auto &line : split(text, '\n')) {
    auto parts = split(line, ';');
    if (parts.size() == 1) { this->add_abbreviation(parts[0], ""); continue; }
    if (parts.size() != 2) { ESP_LOGW(TAG, "Invalid abbreviation: %s", line.c_str()); continue; }
    this->add_abbreviation(parts[0], parts[1]);
  }
}

void TransitTracker::set_route_styles_from_text(const std::string &text) {
  this->route_styles_.clear();
  for (const auto &line : split(text, '\n')) {
    auto parts = split(line, ';');
    if (parts.size() != 3) continue;
    this->add_route_style(parts[0], parts[1], Color(std::stoul(parts[2], nullptr, 16)));
  }
}

void TransitTracker::set_hidden_routes_from_text(const std::string &text) {
  this->hidden_routes_.clear();
  if (text.empty()) return;
  // Composite keys (route_id:headsign:stop_id) may contain semicolons from
  // transit agency headsign data.  We now persist with '\x1F' (ASCII Unit
  // Separator) to avoid splitting keys incorrectly.  Fall back to ';' for
  // backward compatibility with values persisted before this change.
  char delim = (text.find('\x1F') != std::string::npos) ? '\x1F' : ';';
  for (const auto &id : split(text, delim)) {
    if (!id.empty()) { this->hidden_routes_.insert(id); ESP_LOGD(TAG, "Hiding route: %s", id.c_str()); }
  }
}

void TransitTracker::set_pinned_routes_from_text(const std::string &text) {
  this->pinned_routes_.clear();
  if (text.empty()) return;
  // Composite keys may contain semicolons — use '\x1F' if present, else
  // fall back to ';' for backward compatibility with older persisted values.
  char delim = (text.find('\x1F') != std::string::npos) ? '\x1F' : ';';
  for (const auto &entry : split(text, delim)) {
    if (entry.empty()) continue;
    // Parse optional mode suffix: key=G, key=L, key=B
    // Default to PIN_GENERAL for backward compatibility (no suffix).
    std::string key = entry;
    PinMode mode = PIN_GENERAL;
    if (entry.size() >= 3) {
      auto eq = entry.rfind('=');
      if (eq != std::string::npos && eq == entry.size() - 2) {
        char m = entry.back();
        key = entry.substr(0, eq);
        if (m == 'L') mode = PIN_LEAVING_SOON;
        else if (m == 'B') mode = PIN_BOTH;
        // else default to PIN_GENERAL
      }
    }
    this->pinned_routes_[key] = mode;
    ESP_LOGD(TAG, "Pinning route: %s (mode=%d)", key.c_str(), (int)mode);
  }
}

void TransitTracker::set_next_only_routes_from_text(const std::string &text) {
  this->next_only_routes_.clear();
  if (text.empty()) return;
  // Composite keys may contain semicolons — use '\x1F' if present, else
  // fall back to ';' for backward compatibility with older persisted values.
  char delim = (text.find('\x1F') != std::string::npos) ? '\x1F' : ';';
  for (const auto &id : split(text, delim)) {
    if (!id.empty()) this->next_only_routes_.insert(id);
  }
}

void TransitTracker::rebuild_route_stop_map_() {
  this->route_stop_map_.clear();
  std::set<std::string> ambiguous;
  for (const auto &pair : split(this->schedule_string_, ';')) {
    auto parts = split(pair, ',');
    if (parts.size() >= 2) {
      auto it = this->route_stop_map_.find(parts[0]);
      if (it != this->route_stop_map_.end() && it->second != parts[1]) {
        ambiguous.insert(parts[0]);
      } else {
        this->route_stop_map_[parts[0]] = parts[1];
      }
    }
  }
  for (const auto &r : ambiguous) {
    this->route_stop_map_.erase(r);
    ESP_LOGD(TAG, "Route %s at multiple stops, stop_id omitted from key", r.c_str());
  }
}

void TransitTracker::set_route_color_overrides_from_text(const std::string &text) {
  this->route_color_overrides_.clear();
  if (text.empty()) return;
  for (const auto &entry : split(text, ';')) {
    auto cp = entry.find(':');
    if (cp == std::string::npos) continue;
    int r, g, b;
    if (sscanf(entry.substr(cp + 1).c_str(), "%d,%d,%d", &r, &g, &b) == 3)
      this->route_color_overrides_[entry.substr(0, cp)] = Color(r, g, b);
  }
}

void TransitTracker::draw_text_centered_(const char *text, Color color) {
  this->display_->print(this->display_->get_width() / 2, this->display_->get_height() / 2,
                        this->font_, color, display::TextAlign::CENTER, text);
}

void TransitTracker::set_realtime_color(const Color &color) {
  this->realtime_color_ = color;
  this->realtime_color_dark_ = Color(
    (uint8_t)(color.r * 0.5f),
    (uint8_t)(color.g * 0.5f),
    (uint8_t)(color.b * 0.5f));
}

void TransitTracker::set_scroll_routes(bool v) {
  if (this->scroll_routes_ != v) {
    this->scroll_routes_ = v;
    this->pinned_pager_.reset();
    this->departure_pager_.reset();
    this->unpinned_pager_.reset();
    if (this->ws_client_.available()) this->reconnect();
  }
}

void TransitTracker::set_scroll_speed(const std::string &speed) {
  int new_speed = 20;
  if (speed == "slow") new_speed = 10;
  else if (speed == "medium") new_speed = 20;
  else if (speed == "fast") new_speed = 30;
  else if (speed == "fastest") new_speed = 40;
  else {
    char *end = nullptr;
    long val = strtol(speed.c_str(), &end, 10);
    if (end != speed.c_str() && *end == '\0') new_speed = (int)val;
  }
  this->h_scroll_.pending_speed = new_speed;
}

// =====================================================================
// MQTT Auto-Discovery
// =====================================================================

#ifdef USE_MQTT

std::string TransitTracker::slugify_(const std::string &input) {
  std::string slug = input;
  for (auto &c : slug) {
    if (c == ':' || c == ' ' || c == '|') c = '_';
    else c = tolower(c);
  }
  return slug;
}

void TransitTracker::publish_mqtt_routes_() {
  if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) return;
  this->mqtt_routes_pending_ = false;

  std::string node = App.get_name();
  std::string topic_prefix = mqtt::global_mqtt_client->get_topic_prefix();

  std::set<std::string> seen_keys;
  for (const auto &t : this->schedule_state_.trips) {
    std::string key = t.composite_key();
    if (seen_keys.count(key)) continue;
    seen_keys.insert(key);

    if (this->mqtt_published_routes_.count(key)) continue;
    this->mqtt_published_routes_.insert(key);

    std::string slug = slugify_(key);
    std::string object_id = node + "_route_" + slug;
    std::string state_topic = topic_prefix + "/route/" + slug + "/state";
    std::string command_topic = topic_prefix + "/route/" + slug + "/set";

    std::string friendly_name = t.route_name;
    if (!t.headsign.empty()) friendly_name += " - " + t.headsign;
    if (!t.stop_name.empty()) friendly_name += " - " + t.stop_name;

    // Discovery JSON (select entity: Off/On/Pinned)
    std::string discovery_topic = "homeassistant/select/" + object_id + "/config";
    auto payload = json::build_json([&](JsonObject root) {
      root["name"] = friendly_name;
      root["unique_id"] = object_id;
      root["default_entity_id"] = "select." + object_id;
      root["state_topic"] = state_topic;
      root["command_topic"] = command_topic;
      root["icon"] = "mdi:bus";
      root["optimistic"] = true;

      auto options = root["options"].to<JsonArray>();
      options.add("Off");
      options.add("On");
      options.add("Pinned");
      options.add("Pinned (Leaving Soon)");
      options.add("Pinned (Both)");

      // Note: we intentionally omit availability_topic here.
      // These are configuration entities; the device's built-in
      // ESPHome entities already report online/offline status.
      // Including the device-level birth/will topic caused brief
      // unavailable flickers whenever the blocking WebSocket
      // reconnect starved the MQTT keepalive.

      auto device = root["device"].to<JsonObject>();
      auto ids = device["identifiers"].to<JsonArray>();
      ids.add(node);
      device["name"] = App.get_friendly_name().empty() ? node : App.get_friendly_name();
    });
    mqtt::global_mqtt_client->publish(discovery_topic, payload, 0, true);

    // Remove any stale switch discovery from before the conversion
    mqtt::global_mqtt_client->publish("homeassistant/switch/" + object_id + "/config", std::string(""), 0, true);

    // Publish current state
    bool hidden = this->hidden_routes_.count(key) > 0;
    auto pin_it = this->pinned_routes_.find(key);
    std::string state;
    if (hidden) {
      state = "Off";
    } else if (pin_it != this->pinned_routes_.end()) {
      switch (pin_it->second) {
        case PIN_LEAVING_SOON: state = "Pinned (Leaving Soon)"; break;
        case PIN_BOTH: state = "Pinned (Both)"; break;
        default: state = "Pinned"; break;
      }
    } else {
      state = "On";
    }
    mqtt::global_mqtt_client->publish(state_topic, state, 0, true);

    // Subscribe to commands
    if (!this->mqtt_subscribed_routes_.count(key)) {
      this->mqtt_subscribed_routes_.insert(key);
      std::string cap_key = key, cap_st = state_topic;
      mqtt::global_mqtt_client->subscribe(
        command_topic,
        [this, cap_key, cap_st](const std::string &topic, const std::string &payload) {
          if (payload == "Off") {
            // Enforce at least one route visible
            int visible_count = 0;
            std::set<std::string> current_keys;
            for (const auto &trip : this->schedule_state_.trips) {
              std::string k = trip.composite_key();
              if (current_keys.count(k)) continue;
              current_keys.insert(k);
              if (!this->hidden_routes_.count(k)) visible_count++;
            }
            if (visible_count <= 1) {
              ESP_LOGW(TAG, "Cannot hide last visible route %s", cap_key.c_str());
              auto pin_it = this->pinned_routes_.find(cap_key);
              std::string cur_state;
              if (pin_it != this->pinned_routes_.end()) {
                switch (pin_it->second) {
                  case PIN_LEAVING_SOON: cur_state = "Pinned (Leaving Soon)"; break;
                  case PIN_BOTH: cur_state = "Pinned (Both)"; break;
                  default: cur_state = "Pinned"; break;
                }
              } else {
                cur_state = "On";
              }
              mqtt::global_mqtt_client->publish(cap_st, cur_state, 0, true);
              return;
            }
            this->hidden_routes_.insert(cap_key);
            this->pinned_routes_.erase(cap_key);
          } else if (payload == "On") {
            this->hidden_routes_.erase(cap_key);
            this->pinned_routes_.erase(cap_key);
          } else if (payload == "Pinned") {
            this->hidden_routes_.erase(cap_key);
            this->pinned_routes_[cap_key] = PIN_GENERAL;
          } else if (payload == "Pinned (Leaving Soon)") {
            this->hidden_routes_.erase(cap_key);
            this->pinned_routes_[cap_key] = PIN_LEAVING_SOON;
          } else if (payload == "Pinned (Both)") {
            this->hidden_routes_.erase(cap_key);
            this->pinned_routes_[cap_key] = PIN_BOTH;
          } else {
            ESP_LOGW(TAG, "Unknown route command: %s", payload.c_str());
            return;
          }

          mqtt::global_mqtt_client->publish(cap_st, payload, 0, true);
          ESP_LOGD(TAG, "Route %s set to %s via MQTT", cap_key.c_str(), payload.c_str());
          this->mqtt_persist_pending_ = true;
        },
        0
      );
    }
  }
}

void TransitTracker::persist_hidden_routes_() {
  // Use '\x1F' (ASCII Unit Separator) instead of ';' to delimit composite
  // keys.  Composite keys contain the headsign, which may itself include
  // semicolons from transit agency data.  Using ';' would split a single
  // key into multiple fragments on the next restore, silently breaking
  // pin/hide state.
  //
  // A leading '\x1F' prefix ensures the deserializer detects the new format
  // even when only a single key is stored (where no inter-key delimiter
  // would otherwise appear).  The deserializer skips empty tokens from
  // the prefix.
  std::string s;
  for (const auto &k : this->hidden_routes_) { s += '\x1F'; s += k; }
#ifdef USE_TEXT
  if (this->hidden_routes_text_ && this->hidden_routes_text_->state != s) {
    auto call = this->hidden_routes_text_->make_call();
    call.set_value(s);
    call.perform();
    ESP_LOGD(TAG, "Persisted hidden routes: %s", s.c_str());
  }
#endif
}

void TransitTracker::persist_pinned_routes_() {
  // Use '\x1F' with leading prefix — see persist_hidden_routes_() for rationale.
  // Each entry is key=M where M is G (General), L (Leaving Soon), or B (Both).
  std::string s;
  for (const auto &kv : this->pinned_routes_) {
    s += '\x1F';
    s += kv.first;
    s += '=';
    switch (kv.second) {
      case PIN_LEAVING_SOON: s += 'L'; break;
      case PIN_BOTH: s += 'B'; break;
      default: s += 'G'; break;
    }
  }
#ifdef USE_TEXT
  if (this->pinned_routes_text_ && this->pinned_routes_text_->state != s) {
    auto call = this->pinned_routes_text_->make_call();
    call.set_value(s);
    call.perform();
    ESP_LOGD(TAG, "Persisted pinned routes: %s", s.c_str());
  }
#endif
}

void TransitTracker::persist_next_only_routes_() {
  // Use '\x1F' with leading prefix — see persist_hidden_routes_() for rationale.
  std::string s;
  for (const auto &k : this->next_only_routes_) { s += '\x1F'; s += k; }
#ifdef USE_TEXT
  if (this->next_only_routes_text_) {
    auto call = this->next_only_routes_text_->make_call();
    call.set_value(s);
    call.perform();
  }
#endif
}

void TransitTracker::persist_route_color_overrides_() {
  std::string result;
  for (const auto &entry : this->route_color_overrides_) {
    if (!result.empty()) result += ";";
    char buf[64];
    snprintf(buf, sizeof(buf), "%s:%d,%d,%d",
             entry.first.c_str(), entry.second.r, entry.second.g, entry.second.b);
    result += buf;
  }
#ifdef USE_TEXT
  if (this->route_color_overrides_text_) {
    auto call = this->route_color_overrides_text_->make_call();
    call.set_value(result);
    call.perform();
  }
#endif
}

void TransitTracker::publish_mqtt_route_colors_() {
  if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) return;

  std::string node = App.get_name();
  std::string topic_prefix = mqtt::global_mqtt_client->get_topic_prefix();

  for (const auto &t : this->schedule_state_.trips) {
    if (this->mqtt_published_route_color_ids_.count(t.route_id)) continue;
    this->mqtt_published_route_color_ids_.insert(t.route_id);

    std::string slug = slugify_(t.route_id);
    std::string object_id = node + "_route_color_" + slug;
    std::string state_topic = topic_prefix + "/route_color/" + slug + "/state";
    std::string command_topic = topic_prefix + "/route_color/" + slug + "/set";
    Color current_color = t.route_color;

    // Light discovery JSON (RGB only, no brightness)
    std::string discovery_topic = "homeassistant/light/" + object_id + "/config";
    auto payload = json::build_json([&](JsonObject root) {
      root["name"] = t.route_name + " Color";
      root["unique_id"] = object_id;
      root["default_entity_id"] = "light." + object_id;
      root["schema"] = "json";
      root["state_topic"] = state_topic;
      root["command_topic"] = command_topic;
      root["icon"] = "mdi:palette";
      root["brightness"] = false;

      auto modes = root["supported_color_modes"].to<JsonArray>();
      modes.add("rgb");

      // Omit availability_topic — same rationale as route select entities.

      auto device = root["device"].to<JsonObject>();
      auto ids = device["identifiers"].to<JsonArray>();
      ids.add(node);
      device["name"] = App.get_friendly_name().empty() ? node : App.get_friendly_name();
    });
    mqtt::global_mqtt_client->publish(discovery_topic, payload, 0, true);

    // Publish current color state
    auto state_payload = json::build_json([&](JsonObject root) {
      root["state"] = "ON";
      auto c = root["color"].to<JsonObject>();
      c["r"] = current_color.r;
      c["g"] = current_color.g;
      c["b"] = current_color.b;
      root["color_mode"] = "rgb";
    });
    mqtt::global_mqtt_client->publish(state_topic, state_payload, 0, true);

    // Subscribe to command topic
    if (!this->mqtt_subscribed_route_colors_.count(t.route_id)) {
      this->mqtt_subscribed_route_colors_.insert(t.route_id);
      std::string cap_rid = t.route_id, cap_st = state_topic;
      mqtt::global_mqtt_client->subscribe(
        command_topic,
        [this, cap_rid, cap_st](const std::string &topic, const std::string &cmd_payload) {
          bool parsed = json::parse_json(cmd_payload, [this, &cap_rid](JsonObject root) -> bool {
            if (root["color"].is<JsonObject>()) {
              JsonObject color = root["color"];
              uint8_t r = color["r"] | 255;
              uint8_t g = color["g"] | 0;
              uint8_t b = color["b"] | 0;
              Color new_color(r, g, b);
              this->route_color_overrides_[cap_rid] = new_color;
              this->schedule_state_.mutex.lock();
              for (auto &trip : this->schedule_state_.trips)
                if (trip.route_id == cap_rid) trip.route_color = new_color;
              this->schedule_state_.mutex.unlock();
            }
            return true;
          });
          if (parsed) {
            auto override_it = this->route_color_overrides_.find(cap_rid);
            if (override_it != this->route_color_overrides_.end()) {
              auto new_state = json::build_json([&](JsonObject root) {
                root["state"] = "ON";
                auto c = root["color"].to<JsonObject>();
                c["r"] = override_it->second.r;
                c["g"] = override_it->second.g;
                c["b"] = override_it->second.b;
                root["color_mode"] = "rgb";
              });
              mqtt::global_mqtt_client->publish(cap_st, new_state, 0, true);
            }
            this->persist_route_color_overrides_();
          }
        },
        0
      );
    }
  }
}

#endif  // USE_MQTT

// =====================================================================
// Drawing Helpers
// =====================================================================

const uint8_t realtime_icon[6][6] = {
  {0, 0, 0, 3, 3, 3},
  {0, 0, 3, 0, 0, 0},
  {0, 3, 0, 0, 2, 2},
  {3, 0, 0, 2, 0, 0},
  {3, 0, 2, 0, 0, 1},
  {3, 0, 2, 0, 1, 1}
};

void HOT TransitTracker::draw_realtime_icon_(int bottom_right_x, int bottom_right_y, unsigned long now) {
  const int num_frames = 6;
  const int idle_frame_duration = 3000;
  const int anim_frame_duration = 200;
  const int cycle_duration = idle_frame_duration + (num_frames - 1) * anim_frame_duration;

  unsigned long cycle_time = now % cycle_duration;
  int frame;
  if (cycle_time < (unsigned long)idle_frame_duration) {
    frame = 0;
  } else {
    frame = 1 + (cycle_time - idle_frame_duration) / anim_frame_duration;
  }

  auto is_segment_lit = [frame](uint8_t segment) {
    switch (segment) {
      case 1: return frame >= 1 && frame <= 3;
      case 2: return frame >= 2 && frame <= 4;
      case 3: return frame >= 3 && frame <= 5;
      default: return false;
    }
  };

  for (uint8_t i = 0; i < 6; ++i) {
    for (uint8_t j = 0; j < 6; ++j) {
      uint8_t segment_number = realtime_icon[i][j];
      if (segment_number == 0) continue;
      Color icon_color = is_segment_lit(segment_number) ? this->realtime_color_ : this->realtime_color_dark_;
      this->display_->draw_pixel_at(bottom_right_x - (5 - j), bottom_right_y - (5 - i), icon_color);
    }
  }
}

void TransitTracker::draw_trip(
    const Trip &trip, int y_offset, int fh, unsigned long uptime, uint rtc_now,
    bool no_draw, int *headsign_overflow_out, int *headsign_width_out,
    int h_scroll_offset, int total_scroll_distance, bool is_pinned, bool is_leaving_soon) {

  int x_start = this->frame_pin_inset_;
  if (!is_pinned && !this->frame_respect_pin_inset_ && x_start > 0) {
    x_start = 1;  // 1px margin when pin icon column is visible
  }
  int dw = this->display_->get_width();
  int dh = this->display_->get_height();

  // Pin icon (drawn in the inset margin if this row is pinned)
  if (is_pinned && x_start > 0 && this->show_pin_icon_ && !no_draw) {
    Color pin_color = is_leaving_soon ? Color(0x00FF00) : Color(0xFF0000); // green if leaving soon, else red
    int px = 1; // 1px left margin; icon occupies columns px..px+4 (5 wide)
    int py = y_offset + 1;
    int cx = px + 2; // center column of the 5px-wide icon

    // Row 0: 5px wide
    for (int c = 0; c < 5; c++) this->display_->draw_pixel_at(px + c, py, pin_color);
    py++;
    // Row 1: 3px centered
    for (int c = 1; c <= 3; c++) this->display_->draw_pixel_at(px + c, py, pin_color);
    py++;
    // Row 2: 3px centered
    for (int c = 1; c <= 3; c++) this->display_->draw_pixel_at(px + c, py, pin_color);
    py++;
    // Row 3: 5px wide
    for (int c = 0; c < 5; c++) this->display_->draw_pixel_at(px + c, py, pin_color);
    py++;
    // Remaining rows: 1px white vertical line centered
    int bottom = y_offset + fh - 3;
    for (int r = py; r < bottom; r++) {
      this->display_->draw_pixel_at(cx, r, Color(0xFFFFFF));
    }
  }

  // Route name
  if (!no_draw)
    this->display_->print(x_start, y_offset, this->font_, trip.route_color,
                          display::TextAlign::TOP_LEFT, trip.route_name.c_str());

  int route_width, _;
  this->font_->measure(trip.route_name.c_str(), &route_width, &_, &_, &_);
  route_width += x_start;

  // Time display
  auto time_str = this->localization_.fmt_duration_from_now(
      this->display_departure_times_ ? trip.departure_time : trip.arrival_time, rtc_now);
  int time_width;
  this->font_->measure(time_str.c_str(), &time_width, &_, &_, &_);

  // Headsign clipping bounds (uniform overrides per-trip bounds)
  int clip_start = (this->frame_uniform_clip_start_ >= 0) ? this->frame_uniform_clip_start_ : route_width + 3;
  int clip_end = (this->frame_uniform_clip_end_ >= 0) ? this->frame_uniform_clip_end_ : dw - time_width - 2;

  // Time
  if (!no_draw) {
    Color time_color = trip.is_realtime ? this->realtime_color_ : Color(0xa7a7a7);
    this->display_->print(dw + 1, y_offset, this->font_, time_color,
                          display::TextAlign::TOP_RIGHT, time_str.c_str());
  }

  // Realtime icon
  if (trip.is_realtime) {
    if (this->frame_uniform_clip_end_ < 0) clip_end -= 8;
    if (!no_draw)
      this->draw_realtime_icon_(dw - time_width - 2, y_offset + fh - 5, uptime);
  }

  // Headsign measurement
  int hw;
  this->font_->measure(trip.headsign.c_str(), &hw, &_, &_, &_);
  int overflow = hw - (clip_end - clip_start);
  if (headsign_overflow_out) *headsign_overflow_out = overflow;
  if (headsign_width_out) *headsign_width_out = hw;
  if (no_draw) return;

  // Headsign rendering (continuous marquee when scrolling)
  this->display_->start_clipping(clip_start, 0, clip_end, dh);
  if (overflow > 0 && h_scroll_offset >= 0 && total_scroll_distance > 0) {
    int x = clip_start - h_scroll_offset;
    this->display_->print(x, y_offset, this->font_, trip.headsign.c_str());
    this->display_->print(x + total_scroll_distance, y_offset, this->font_, trip.headsign.c_str());
  } else {
    this->display_->print(clip_start, y_offset, this->font_, trip.headsign.c_str());
  }
  this->display_->end_clipping();
}

// =====================================================================
// draw_schedule Helpers
// =====================================================================

void TransitTracker::compute_uniform_clipping_(const std::vector<Trip> &trips, uint rtc_now) {
  this->frame_uniform_clip_start_ = -1;
  this->frame_uniform_clip_end_ = -1;
  if (!this->uniform_headsign_start_ && !this->uniform_headsign_end_) return;

  int max_route_w = 0, max_time_w = 0;
  bool any_rt = false;
  int _;
  for (const auto &t : trips) {
    int rw;
    this->font_->measure(t.route_name.c_str(), &rw, &_, &_, &_);
    max_route_w = std::max(max_route_w, rw);

    auto ts = this->localization_.fmt_duration_from_now(
        this->display_departure_times_ ? t.departure_time : t.arrival_time, rtc_now);
    int tw;
    this->font_->measure(ts.c_str(), &tw, &_, &_, &_);
    max_time_w = std::max(max_time_w, tw);
    if (t.is_realtime) any_rt = true;
  }

  if (this->uniform_headsign_start_)
    this->frame_uniform_clip_start_ = this->frame_pin_inset_ + max_route_w + 3;

  if (this->uniform_headsign_end_) {
    int reserve = max_time_w + 2;
    if (any_rt) reserve += 8;
    this->frame_uniform_clip_end_ = this->display_->get_width() - reserve;
  }
}

void TransitTracker::compute_h_scroll_(const std::vector<Trip> &trips, int fh,
                                        unsigned long now, uint rtc_now) {
  if (!this->scroll_headsigns_) {
    this->h_scroll_.total_distance = 0;
    this->h_scroll_.prev_total_distance = 0;
    this->h_scroll_.offset = 0;
    this->h_scroll_.idle = true;
    this->h_scroll_.deferred_start = false;
    this->h_scroll_.deferred_since = 0;
    return;
  }

  // Freeze h-scroll during collapse/expand/post-pause animations
  bool anim_active = (this->transition_.phase >= Transition::COLLAPSE &&
                      this->transition_.phase <= Transition::POST_PAUSE);
  if (anim_active || (this->post_pause_end_ != 0 && now < this->post_pause_end_)) {
    this->h_scroll_.offset = 0;
    this->h_scroll_.idle = true;
    return;
  }

  // Restart h-scroll timer after post-pause ends
  if (this->post_pause_end_ != 0 && now >= this->post_pause_end_) {
    this->post_pause_end_ = 0;
    this->h_scroll_.reset(now);
  }

  // Find max headsign width and check overflow
  int max_hw = 0;
  bool any_overflow = false;
  for (const auto &t : trips) {
    int ov = 0, hw = 0;
    // Use is_pinned=true for worst-case measurement (pin inset reduces clip area)
    this->draw_trip(t, 0, fh, now, rtc_now, true, &ov, &hw, -1, 0, true);
    if (ov > 0) any_overflow = true;
    max_hw = std::max(max_hw, hw);
  }

  this->h_scroll_.update_distance(max_hw, any_overflow);
  this->h_scroll_.compute(now);
}

void TransitTracker::begin_transition_(
    const std::vector<Trip> &old_trips, const std::vector<bool> &old_is_pinned,
    const std::vector<bool> &old_is_leaving_soon,
    int old_eff_pinned, bool layout_or_swap, unsigned long now) {

  this->transition_.reset();
  this->transition_.old_trips = old_trips;
  this->transition_.old_is_pinned = old_is_pinned;
  this->transition_.old_is_leaving_soon = old_is_leaving_soon;
  this->transition_.old_eff_pinned = old_eff_pinned;
  this->transition_.old_respect_pin_inset = this->committed_respect_pin_inset_;

  // Animate all rows for all change types (simple and robust)
  this->transition_.animate_pinned = true;
  this->transition_.animate_unpinned = true;
  this->transition_.stagger_rows = std::max(1, (int)old_trips.size());

  // Start WAIT_SCROLL if h-scroll is active and not near zero
  bool scroll_active = (this->h_scroll_.total_distance > 0 || this->h_scroll_.prev_total_distance > 0);
  if (scroll_active && this->h_scroll_.offset >= kScrollNearZeroPx) {
    this->transition_.phase = Transition::WAIT_SCROLL;
    this->transition_.phase_start = now;
  } else {
    this->transition_.phase = Transition::COLLAPSE;
    this->transition_.phase_start = now;
    if (scroll_active) this->h_scroll_.reset(now);
  }
}

void TransitTracker::begin_page_transition_(
    const std::vector<Trip> &pinned_pool, const std::vector<Trip> &unpinned_pool,
    int eff_pinned, int eff_unpinned,
    int gen_pager_pool, int gen_pager_slots,
    int dep_pager_pool, int dep_pager_slots,
    bool pinned_due, bool unpinned_due,
    const std::vector<Trip> &old_trips, const std::vector<bool> &old_is_pinned,
    const std::vector<bool> &old_is_leaving_soon,
    unsigned long now) {

  // Snapshot current display before advancing pages
  this->transition_.reset();
  this->transition_.old_trips = old_trips;
  this->transition_.old_is_pinned = old_is_pinned;
  this->transition_.old_is_leaving_soon = old_is_leaving_soon;
  this->transition_.old_eff_pinned = eff_pinned;
  this->transition_.old_respect_pin_inset = this->committed_respect_pin_inset_;
  this->transition_.animate_pinned = pinned_due;
  this->transition_.animate_unpinned = unpinned_due;
  this->transition_.stagger_rows = std::max(1, (int)old_trips.size());

  // Advance page indices (EXPAND will use the new page content)
  if (pinned_due) {
    this->pinned_pager_.advance_page(gen_pager_pool, gen_pager_slots);
    this->departure_pager_.advance_page(dep_pager_pool, dep_pager_slots);
  }
  if (unpinned_due)
    this->unpinned_pager_.advance_page((int)unpinned_pool.size(), eff_unpinned);

  // Use collapse/expand (replace mode)
  bool scroll_active = (this->h_scroll_.total_distance > 0 || this->h_scroll_.prev_total_distance > 0);
  if (scroll_active && this->h_scroll_.offset >= kScrollNearZeroPx) {
    this->transition_.phase = Transition::WAIT_SCROLL;
    this->transition_.phase_start = now;
  } else {
    this->transition_.phase = Transition::COLLAPSE;
    this->transition_.phase_start = now;
    if (scroll_active) this->h_scroll_.reset(now);
  }
}

void TransitTracker::tick_transition_(unsigned long now, int fh,
                                       const std::vector<Trip> &pinned_pool,
                                       const std::vector<Trip> &unpinned_pool,
                                       int eff_pinned, int eff_unpinned) {
  if (this->transition_.phase == Transition::IDLE) return;
  unsigned long elapsed = now - this->transition_.phase_start;

  switch (this->transition_.phase) {
    case Transition::WAIT_SCROLL:
      // Wait for h-scroll to reach near-zero, then start collapse
      if (this->h_scroll_.idle || this->h_scroll_.total_distance == 0) {
        this->transition_.phase = Transition::COLLAPSE;
        this->transition_.phase_start = now;
        this->h_scroll_.reset(now);
      } else if (elapsed > 10000) {
        // Safety timeout: don't wait forever
        ESP_LOGW(TAG, "WAIT_SCROLL timeout, forcing collapse");
        this->transition_.phase = Transition::COLLAPSE;
        this->transition_.phase_start = now;
        this->h_scroll_.reset(now);
      }
      break;

    case Transition::COLLAPSE:
      if ((int)elapsed >= this->transition_.collapse_duration_ms()) {
        this->transition_.phase = Transition::MID_PAUSE;
        this->transition_.phase_start = now;
      }
      break;

    case Transition::MID_PAUSE:
      if ((int)elapsed >= kReplaceMidPauseMs) {
        this->transition_.phase = Transition::EXPAND;
        this->transition_.phase_start = now;
        // Update stagger_rows for expand (layout may have changed)
        int new_total = eff_pinned + std::min((int)unpinned_pool.size(), eff_unpinned);
        this->transition_.stagger_rows = std::max(1, std::min(new_total, this->limit_));
      }
      break;

    case Transition::EXPAND:
      if ((int)elapsed >= this->transition_.expand_duration_ms()) {
        this->transition_.phase = Transition::POST_PAUSE;
        this->transition_.phase_start = now;
      }
      break;

    case Transition::POST_PAUSE:
      if ((int)elapsed >= this->page_pause_duration_) {
        this->transition_.reset();
        // Brief h-scroll hold after animation
        this->post_pause_end_ = now + 500;
        this->pinned_pager_.page_timer = now;
        this->pinned_pager_.last_cycle_count = this->h_scroll_.cycle_count;
        this->departure_pager_.page_timer = now;
        this->departure_pager_.last_cycle_count = this->h_scroll_.cycle_count;
        this->unpinned_pager_.page_timer = now;
        this->unpinned_pager_.last_cycle_count = this->h_scroll_.cycle_count;
      }
      break;

    default:
      break;
  }
}

void TransitTracker::render_frame_(
    const std::vector<Trip> &rows, const std::vector<bool> &row_pinned,
    const std::vector<bool> &row_leaving_soon,
    int eff_pinned, unsigned long now, uint rtc_now, int fh, int y_base) {

  auto phase = this->transition_.phase;
  bool use_old = (phase == Transition::WAIT_SCROLL ||
                  phase == Transition::COLLAPSE ||
                  phase == Transition::MID_PAUSE);

  const auto &draw_rows = use_old ? this->transition_.old_trips : rows;
  const auto &draw_pinned = use_old ? this->transition_.old_is_pinned : row_pinned;
  const auto &draw_leaving_soon = use_old ? this->transition_.old_is_leaving_soon : row_leaving_soon;

  // Save/restore pin inset based on current phase
  int save_inset = this->frame_pin_inset_;
  bool save_respect = this->frame_respect_pin_inset_;
  if (use_old) {
    this->frame_pin_inset_ = (this->transition_.old_eff_pinned > 0 && this->show_pin_icon_)
                                 ? kPinIconWidth : 0;
    this->frame_respect_pin_inset_ = this->transition_.old_respect_pin_inset;
  }

  // H-scroll: active only in IDLE and WAIT_SCROLL
  int h_off = 0, h_dist = 0;
  if (phase == Transition::IDLE || phase == Transition::WAIT_SCROLL) {
    if (this->h_scroll_.total_distance > 0 || this->h_scroll_.prev_total_distance > 0) {
      h_off = this->h_scroll_.offset;
      h_dist = this->h_scroll_.total_distance > 0
                   ? this->h_scroll_.total_distance
                   : this->h_scroll_.prev_total_distance;
    }
  }

  unsigned long phase_elapsed = now - this->transition_.phase_start;
  int dw = this->display_->get_width();

  for (size_t i = 0; i < draw_rows.size() && (int)i < this->limit_; i++) {
    int y = y_base + (int)i * fh;
    bool ls = (i < draw_leaving_soon.size()) ? draw_leaving_soon[i] : false;

    if (phase == Transition::COLLAPSE) {
      // Rows shrink from bottom up (top stays visible longest)
      float scale = this->transition_.row_scale((int)i, phase_elapsed, false);
      if (scale <= kMinRowScale) continue;
      int clip_h = std::max(1, (int)(fh * scale));
      this->display_->start_clipping(0, y, dw, y + clip_h);
      this->draw_trip(draw_rows[i], y, fh, now, rtc_now, false, nullptr, nullptr, -1, 0, draw_pinned[i], ls);
      this->display_->end_clipping();

    } else if (phase == Transition::MID_PAUSE) {
      // Nothing visible during mid-pause
      continue;

    } else if (phase == Transition::EXPAND) {
      // Rows grow from top down (bottom appears last)
      float scale = this->transition_.row_scale((int)i, phase_elapsed, true);
      if (scale <= kMinRowScale) continue;
      int clip_h = std::max(1, (int)(fh * scale));
      int clip_top = y + fh - clip_h;
      this->display_->start_clipping(0, std::max(0, clip_top), dw, y + fh);
      this->draw_trip(draw_rows[i], y, fh, now, rtc_now, false, nullptr, nullptr, -1, 0, draw_pinned[i], ls);
      this->display_->end_clipping();

    } else {
      // IDLE, WAIT_SCROLL, POST_PAUSE — full rows with optional h-scroll
      this->draw_trip(draw_rows[i], y, fh, now, rtc_now, false, nullptr, nullptr, h_off, h_dist, draw_pinned[i], ls);
    }
  }

  this->frame_pin_inset_ = save_inset;
  this->frame_respect_pin_inset_ = save_respect;
}

// =====================================================================
// draw_schedule — the main orchestrator
// =====================================================================

void HOT TransitTracker::draw_schedule() {
  // ---- Early returns ----
  if (!this->display_) return;
  if (!esphome::network::is_connected()) {
    this->draw_text_centered_("Waiting for network", Color(0x252627));
    return;
  }
  if (!this->rtc_->now().is_valid()) {
    this->draw_text_centered_("Waiting for time sync", Color(0x252627));
    return;
  }
  if (this->base_url_.empty()) {
    this->draw_text_centered_("No base URL set", Color(0x252627));
    return;
  }
  if (this->status_has_error()) {
    this->draw_text_centered_("Error loading schedule", Color(0xFE4C5C));
    return;
  }
  if (!this->has_ever_connected_) {
    this->draw_text_centered_("Loading...", Color(0x252627));
    return;
  }
  if (this->schedule_state_.trips.empty()) {
    auto msg = this->display_departure_times_ ? "No upcoming departures" : "No upcoming arrivals";
    this->draw_text_centered_(msg, Color(0x252627));
    return;
  }

  // ---- 1. Snapshot trips ----
  this->schedule_state_.mutex.lock();
  auto all_trips = this->schedule_state_.trips;
  this->schedule_state_.mutex.unlock();

  // ---- 2. Filter hidden routes ----
  std::vector<Trip> visible;
  for (const auto &t : all_trips) {
    if (this->hidden_routes_.count(t.composite_key())) continue;
    visible.push_back(t);
  }

  // ---- 3. Apply next-only filter ----
  if (!this->next_only_routes_.empty()) {
    std::set<std::string> seen;
    std::vector<Trip> filtered;
    for (const auto &t : visible) {
      auto key = t.composite_key();
      if (this->next_only_routes_.count(key)) {
        if (seen.count(key)) continue;
        seen.insert(key);
      }
      filtered.push_back(t);
    }
    visible = std::move(filtered);
  }

  if (visible.empty()) {
    auto msg = this->display_departure_times_ ? "No upcoming departures" : "No upcoming arrivals";
    this->draw_text_centered_(msg, Color(0x252627));
    return;
  }

  // ---- 4. Partition: pinned first, then unpinned ----
  // We need rtc_now early for the leaving-soon threshold check.
  uint rtc_now = this->rtc_->now().timestamp;
  int leaving_soon_threshold_sec = this->leaving_soon_threshold_min_ * 60;

  // A trip is "effectively pinned" if:
  //   PIN_GENERAL or PIN_BOTH → always
  //   PIN_LEAVING_SOON → only when departure_time - rtc_now < threshold
  int actual_pinned = 0;
  if (!this->pinned_routes_.empty()) {
    std::stable_partition(visible.begin(), visible.end(), [this, rtc_now, leaving_soon_threshold_sec](const Trip &t) {
      auto it = this->pinned_routes_.find(t.composite_key());
      if (it == this->pinned_routes_.end()) return false;
      switch (it->second) {
        case PIN_GENERAL:
        case PIN_BOTH:
          return true;
        case PIN_LEAVING_SOON:
          return (int)(t.departure_time - rtc_now) < leaving_soon_threshold_sec;
        default:
          return false;
      }
    });
    for (const auto &t : visible) {
      auto it = this->pinned_routes_.find(t.composite_key());
      if (it == this->pinned_routes_.end()) break;
      bool eff_pinned_trip = false;
      switch (it->second) {
        case PIN_GENERAL: case PIN_BOTH: eff_pinned_trip = true; break;
        case PIN_LEAVING_SOON:
          eff_pinned_trip = (int)(t.departure_time - rtc_now) < leaving_soon_threshold_sec;
          break;
        default: break;
      }
      if (!eff_pinned_trip) break;
      actual_pinned++;
    }
  }
  int original_actual_pinned = actual_pinned;

  // ---- 4b. Split pinned trips into general (red) and departure (green) pools ----
  // First occurrence of each composite_key is routed by pin mode:
  //   PIN_GENERAL → general (red)
  //   PIN_LEAVING_SOON → departure (green) if within threshold
  //   PIN_BOTH → departure if leaving soon, else general
  // Duplicate occurrences:
  //   PIN_GENERAL / PIN_BOTH → general (red) — always-pinned routes stay visible
  //   PIN_LEAVING_SOON → departure (green) if within threshold, else unpinned
  std::vector<Trip> general_pool;
  std::vector<Trip> departure_pool;
  std::vector<Trip> demoted_to_unpinned;
  {
    std::set<std::string> seen_pinned_keys;
    for (int i = 0; i < actual_pinned; i++) {
      const auto &t = visible[i];
      auto key = t.composite_key();
      auto it = this->pinned_routes_.find(key);
      if (it == this->pinned_routes_.end()) continue;
      bool is_dup = seen_pinned_keys.count(key) > 0;
      seen_pinned_keys.insert(key);
      int secs_until = (int)(t.departure_time - rtc_now);
      bool leaving_soon = (secs_until < leaving_soon_threshold_sec);
      if (is_dup) {
        switch (it->second) {
          case PIN_GENERAL:
            general_pool.push_back(t);  // always-pinned duplicates → red pin
            break;
          case PIN_BOTH:
            if (leaving_soon) departure_pool.push_back(t);  // match primary routing
            else general_pool.push_back(t);
            break;
          case PIN_LEAVING_SOON:
            if (leaving_soon) departure_pool.push_back(t);  // still within threshold → green
            else demoted_to_unpinned.push_back(t);
            break;
          default:
            demoted_to_unpinned.push_back(t);
            break;
        }
      } else {
        switch (it->second) {
          case PIN_GENERAL:
            general_pool.push_back(t);
            break;
          case PIN_LEAVING_SOON:
            departure_pool.push_back(t);
            break;
          case PIN_BOTH:
            if (leaving_soon) departure_pool.push_back(t);
            else general_pool.push_back(t);
            break;
          default: break;
        }
      }
    }
  }
  // Update actual_pinned to reflect the pools (excluding demoted dups)
  actual_pinned = (int)(general_pool.size() + departure_pool.size());

  // Detect whether we have a split layout (both general and departure pins active)
  bool has_pin_split = (general_pool.size() > 0 && departure_pool.size() > 0);

  // Will the unpinned pool be empty?  Check before computing eff_pinned so
  // pins can expand to fill all rows when there's nothing else to show.
  bool unpinned_pool_empty = (demoted_to_unpinned.empty() &&
                              original_actual_pinned >= (int)visible.size());

  // ---- 4c. Compute effective pinned rows with split-aware logic ----
  int eff_pinned;
  int eff_general = 0;
  int eff_departure = 0;
  if (actual_pinned > 0) {
    if (unpinned_pool_empty) {
      // No unpinned trips — pins can use ALL rows
      if (has_pin_split) {
        // general_pins_count_ determines general rows; remainder → departure
        eff_general = std::min(this->general_pins_count_, (int)general_pool.size());
        int dep_slots = this->limit_ - eff_general;
        eff_departure = std::min(dep_slots, (int)departure_pool.size());

        // If departure didn't fill its slots, give surplus back to general
        int surplus = dep_slots - eff_departure;
        if (surplus > 0) {
          int extra = std::min(surplus, (int)general_pool.size() - eff_general);
          eff_general += extra;
        }

        eff_pinned = eff_general + eff_departure;
      } else {
        // Single pin type — expand to fill all available rows
        eff_pinned = std::min(this->limit_, actual_pinned);
      }
    } else {
      // Unpinned trips exist — pinned_rows_count_ caps pinned rows as before
      eff_pinned = std::min(this->pinned_rows_count_, actual_pinned);

      if (has_pin_split) {
        // Auto-bump to at least 2 pinned rows when both types are present
        if (eff_pinned < 2) eff_pinned = std::min(2, this->limit_ - 1);

        // Allocate rows: general first, then departure
        int gen_want = std::min(this->general_pins_count_, eff_pinned - 1);  // leave ≥1 for departure
        eff_general = std::min(gen_want, (int)general_pool.size());
        int dep_slots = eff_pinned - eff_general;
        eff_departure = std::min(dep_slots, (int)departure_pool.size());

        // If departure didn't fill its slots, give surplus back to general
        int surplus = dep_slots - eff_departure;
        if (surplus > 0) {
          int extra = std::min(surplus, (int)general_pool.size() - eff_general);
          eff_general += extra;
        }

        // Shrink eff_pinned to what we actually have
        eff_pinned = eff_general + eff_departure;
      } else {
        // Single pin type — respect pinned_rows_count setting and allow paging
        eff_pinned = std::min(this->pinned_rows_count_, actual_pinned);
      }
    }
  } else {
    eff_pinned = 0;
  }
  int eff_unpinned = this->limit_ - eff_pinned;

  // Build unpinned pool: demoted duplicate trips + originally-unpinned, sorted by departure
  std::vector<Trip> unpinned_pool(demoted_to_unpinned.begin(), demoted_to_unpinned.end());
  unpinned_pool.insert(unpinned_pool.end(), visible.begin() + original_actual_pinned, visible.end());
  if (!demoted_to_unpinned.empty()) {
    std::stable_sort(unpinned_pool.begin(), unpinned_pool.end(),
        [](const Trip &a, const Trip &b) { return a.departure_time < b.departure_time; });
  }

  // ---- 5. Paginate & build pinned_pool ----
  // In split mode, pinned_pager_ pages the general sub-pool and
  // departure_pager_ pages the departure sub-pool independently.
  // In non-split mode, pinned_pager_ pages the single active pool.
  std::vector<Trip> pinned_pool;
  if (has_pin_split) {
    this->pinned_pager_.clamp((int)general_pool.size(), eff_general);
    this->departure_pager_.clamp((int)departure_pool.size(), eff_departure);
    int g_si = this->pinned_pager_.start_index((int)general_pool.size(), eff_general);
    int g_ei = this->pinned_pager_.end_index((int)general_pool.size(), eff_general);
    int d_si = this->departure_pager_.start_index((int)departure_pool.size(), eff_departure);
    int d_ei = this->departure_pager_.end_index((int)departure_pool.size(), eff_departure);
    for (int i = g_si; i < g_ei; i++) pinned_pool.push_back(general_pool[i]);
    for (int i = d_si; i < d_ei; i++) pinned_pool.push_back(departure_pool[i]);
  } else if (actual_pinned > 0) {
    // Non-split: all pinned trips are the same type; page with pinned_pager_
    auto &single_pool = general_pool.empty() ? departure_pool : general_pool;
    this->pinned_pager_.clamp((int)single_pool.size(), eff_pinned);
    int si = this->pinned_pager_.start_index((int)single_pool.size(), eff_pinned);
    int ei = this->pinned_pager_.end_index((int)single_pool.size(), eff_pinned);
    for (int i = si; i < ei; i++) pinned_pool.push_back(single_pool[i]);
    this->departure_pager_.reset();
  } else {
    this->pinned_pager_.reset();
    this->departure_pager_.reset();
  }

  if (this->scroll_routes_) this->unpinned_pager_.clamp((int)unpinned_pool.size(), eff_unpinned);
  else this->unpinned_pager_.reset();

  int u_si = this->unpinned_pager_.start_index((int)unpinned_pool.size(), eff_unpinned);
  int u_ei = this->unpinned_pager_.end_index((int)unpinned_pool.size(), eff_unpinned);

  // ---- 6. Build display rows ----
  std::vector<Trip> rows;
  std::vector<bool> row_pinned;
  std::vector<bool> row_leaving_soon;
  std::vector<std::string> keys;
  std::vector<time_t> deps;

  for (int i = 0; i < (int)pinned_pool.size(); i++) {
    rows.push_back(pinned_pool[i]); row_pinned.push_back(true);
    // Determine leaving-soon state for this pinned trip
    auto it = this->pinned_routes_.find(pinned_pool[i].composite_key());
    bool ls = false;
    if (it != this->pinned_routes_.end()) {
      int secs_until = (int)(pinned_pool[i].departure_time - rtc_now);
      switch (it->second) {
        case PIN_LEAVING_SOON: ls = true; break; // always green when in pinned section
        case PIN_BOTH: ls = (secs_until < leaving_soon_threshold_sec); break;
        default: break;
      }
    }
    row_leaving_soon.push_back(ls);
    keys.push_back(pinned_pool[i].composite_key()); deps.push_back(pinned_pool[i].departure_time);
  }
  for (int i = u_si; i < u_ei; i++) {
    rows.push_back(unpinned_pool[i]); row_pinned.push_back(false);
    row_leaving_soon.push_back(false);
    keys.push_back(unpinned_pool[i].composite_key()); deps.push_back(unpinned_pool[i].departure_time);
  }

  // ---- 7. Timing ----
  unsigned long now = millis();
  // rtc_now already computed in step 4
  int fh = this->font_->get_ascender() + this->font_->get_descender();
  int max_h = (this->limit_ * this->font_->get_ascender()) + ((this->limit_ - 1) * this->font_->get_descender());
  int y_base = (this->display_->get_height() % max_h) / 2;

  // ---- 8. Per-frame state ----
  this->frame_pin_inset_ = (eff_pinned > 0 && this->show_pin_icon_) ? kPinIconWidth : 0;
  this->frame_respect_pin_inset_ = this->respect_pin_inset_;

  // Build a set of pinned route keys for diff tracking
  std::set<std::string> pinned_route_keys;
  for (const auto &kv : this->pinned_routes_) pinned_route_keys.insert(kv.first);

  // ---- 9. Determine rendering context (old vs new layout) ----
  // H-scroll and uniform clipping must measure against the layout actually
  // visible this frame.  When a transition renders old content (or is about to
  // start), the visible rows and pin inset come from the previous frame.
  // Computing measurements with the *new* inset would shift the headsign
  // mid-marquee on the very first frame of a pin change.
  const std::vector<Trip> *measure_rows = &rows;
  int measure_eff_pinned = eff_pinned;
  bool rendering_old_layout = false;

  if (this->transition_.phase != Transition::IDLE) {
    // Mid-transition: phases that render old content
    bool phase_uses_old = (this->transition_.phase == Transition::WAIT_SCROLL ||
                           this->transition_.phase == Transition::COLLAPSE ||
                           this->transition_.phase == Transition::MID_PAUSE);
    if (phase_uses_old && !this->transition_.old_trips.empty()) {
      measure_rows = &this->transition_.old_trips;
      measure_eff_pinned = this->transition_.old_eff_pinned;
      rendering_old_layout = true;
    }
  } else {
    // IDLE: peek at diff to detect an impending layout change so that the
    // measurements below (and begin_transition_'s WAIT vs COLLAPSE decision)
    // use h-scroll / clipping state consistent with the old layout.
    this->diff_.compute(keys, deps, eff_pinned, pinned_route_keys);
    bool respect_inset_changed = (this->respect_pin_inset_ != this->committed_respect_pin_inset_)
                                 && eff_pinned > 0;
    if ((this->diff_.has_changes() || respect_inset_changed) && this->diff_.prev_pinned_count >= 0) {
      measure_rows = &this->diff_.prev_trips;
      measure_eff_pinned = this->diff_.prev_pinned_count;
      rendering_old_layout = true;
    }
  }

  // ---- 10. H-scroll ----
  {
    int saved_inset = this->frame_pin_inset_;
    bool saved_respect = this->frame_respect_pin_inset_;
    if (rendering_old_layout) {
      this->frame_pin_inset_ = (measure_eff_pinned > 0 && this->show_pin_icon_) ? kPinIconWidth : 0;
      this->frame_respect_pin_inset_ = this->committed_respect_pin_inset_;
    }
    this->compute_h_scroll_(*measure_rows, fh, now, rtc_now);
    this->frame_pin_inset_ = saved_inset;
    this->frame_respect_pin_inset_ = saved_respect;
  }

  // ---- 11. Uniform clipping ----
  {
    int saved_inset = this->frame_pin_inset_;
    bool saved_respect = this->frame_respect_pin_inset_;
    if (rendering_old_layout) {
      this->frame_pin_inset_ = (measure_eff_pinned > 0 && this->show_pin_icon_) ? kPinIconWidth : 0;
      this->frame_respect_pin_inset_ = this->committed_respect_pin_inset_;
    }
    this->compute_uniform_clipping_(*measure_rows, rtc_now);
    this->frame_pin_inset_ = saved_inset;
    this->frame_respect_pin_inset_ = saved_respect;
  }

  // ---- 12. Change detection & transition planning (only when idle) ----
  if (this->transition_.phase == Transition::IDLE) {
    // diff_.compute() already ran above in step 9

    bool respect_inset_changed = (this->respect_pin_inset_ != this->committed_respect_pin_inset_)
                                 && eff_pinned > 0;
    bool triggered_transition = false;

    if ((this->diff_.has_changes() || respect_inset_changed) && this->diff_.prev_pinned_count >= 0) {
      // Data/layout changed — start transition
      bool layout_swap = this->diff_.layout_changed || this->diff_.pinned_set_changed || respect_inset_changed;
      this->begin_transition_(this->diff_.prev_trips, this->diff_.prev_is_pinned,
                              this->diff_.prev_is_leaving_soon,
                              this->diff_.prev_pinned_count, layout_swap, now);
      triggered_transition = true;
    } else if (this->scroll_routes_ && this->diff_.prev_pinned_count >= 0) {
      // Check page timers
      bool pinned_due = false, unpinned_due = false;
      bool h_scroll_active = (this->h_scroll_.total_distance > 0 || this->h_scroll_.prev_total_distance > 0);

      // In split mode, pinned_pager_ pages general_pool and departure_pager_
      // pages departure_pool.  Either needing a page triggers pinned_due
      // (both sub-pools advance together since they share the pinned section).
      // In non-split mode, pinned_pager_ pages the single active pool.
      int gen_pager_pool, gen_pager_slots, dep_pager_pool, dep_pager_slots;
      if (has_pin_split) {
        gen_pager_pool = (int)general_pool.size();
        gen_pager_slots = eff_general;
        dep_pager_pool = (int)departure_pool.size();
        dep_pager_slots = eff_departure;
      } else {
        int single_pool_size = (int)general_pool.size() + (int)departure_pool.size();
        gen_pager_pool = single_pool_size;
        gen_pager_slots = eff_pinned;
        dep_pager_pool = 0;
        dep_pager_slots = 0;
      }

      bool gen_needs = this->pinned_pager_.needs_paging(gen_pager_pool, gen_pager_slots);
      bool dep_needs = has_pin_split && this->departure_pager_.needs_paging(dep_pager_pool, dep_pager_slots);

      if (gen_needs || dep_needs) {
        if (this->pinned_pager_.page_timer == 0) {
          this->pinned_pager_.page_timer = now;
          this->pinned_pager_.last_cycle_count = this->h_scroll_.cycle_count;
        }
        if (has_pin_split && this->departure_pager_.page_timer == 0) {
          this->departure_pager_.page_timer = now;
          this->departure_pager_.last_cycle_count = this->h_scroll_.cycle_count;
        }
        if (h_scroll_active) {
          pinned_due = this->h_scroll_.idle &&
                       this->h_scroll_.cycle_count > this->pinned_pager_.last_cycle_count;
        } else {
          bool interval_met = (now - this->pinned_pager_.page_timer >= (unsigned long)this->page_interval_);
          pinned_due = interval_met;
        }
      }

      if (this->unpinned_pager_.needs_paging((int)unpinned_pool.size(), eff_unpinned)) {
        if (this->unpinned_pager_.page_timer == 0) {
          this->unpinned_pager_.page_timer = now;
          this->unpinned_pager_.last_cycle_count = this->h_scroll_.cycle_count;
        }
        if (h_scroll_active) {
          // Headsigns are scrolling — page when one full scroll cycle completes
          unpinned_due = this->h_scroll_.idle &&
                         this->h_scroll_.cycle_count > this->unpinned_pager_.last_cycle_count;
        } else {
          // No headsign scrolling — use fixed page_interval timer
          bool interval_met = (now - this->unpinned_pager_.page_timer >= (unsigned long)this->page_interval_);
          unpinned_due = interval_met;
        }
      }

      if (pinned_due || unpinned_due) {
        this->begin_page_transition_(pinned_pool, unpinned_pool, eff_pinned, eff_unpinned,
                                     gen_pager_pool, gen_pager_slots,
                                     dep_pager_pool, dep_pager_slots,
                                     pinned_due, unpinned_due, rows, row_pinned, row_leaving_soon, now);
        triggered_transition = true;

        // Re-build pinned_pool with new page windows
        if (has_pin_split) {
          pinned_pool.clear();
          int g_si = this->pinned_pager_.start_index((int)general_pool.size(), eff_general);
          int g_ei = this->pinned_pager_.end_index((int)general_pool.size(), eff_general);
          int d_si = this->departure_pager_.start_index((int)departure_pool.size(), eff_departure);
          int d_ei = this->departure_pager_.end_index((int)departure_pool.size(), eff_departure);
          for (int i = g_si; i < g_ei; i++) pinned_pool.push_back(general_pool[i]);
          for (int i = d_si; i < d_ei; i++) pinned_pool.push_back(departure_pool[i]);
        } else if (actual_pinned > 0) {
          pinned_pool.clear();
          auto &single_pool = general_pool.empty() ? departure_pool : general_pool;
          int si = this->pinned_pager_.start_index((int)single_pool.size(), eff_pinned);
          int ei = this->pinned_pager_.end_index((int)single_pool.size(), eff_pinned);
          for (int i = si; i < ei; i++) pinned_pool.push_back(single_pool[i]);
        }

        // Re-compute rows
        rows.clear(); row_pinned.clear(); row_leaving_soon.clear(); keys.clear(); deps.clear();
        u_si = this->unpinned_pager_.start_index((int)unpinned_pool.size(), eff_unpinned);
        u_ei = this->unpinned_pager_.end_index((int)unpinned_pool.size(), eff_unpinned);
        for (int i = 0; i < (int)pinned_pool.size(); i++) {
          rows.push_back(pinned_pool[i]); row_pinned.push_back(true);
          auto it = this->pinned_routes_.find(pinned_pool[i].composite_key());
          bool ls = false;
          if (it != this->pinned_routes_.end()) {
            int secs_until = (int)(pinned_pool[i].departure_time - rtc_now);
            switch (it->second) {
              case PIN_LEAVING_SOON: ls = true; break;
              case PIN_BOTH: ls = (secs_until < leaving_soon_threshold_sec); break;
              default: break;
            }
          }
          row_leaving_soon.push_back(ls);
          keys.push_back(pinned_pool[i].composite_key()); deps.push_back(pinned_pool[i].departure_time);
        }
        for (int i = u_si; i < u_ei; i++) {
          rows.push_back(unpinned_pool[i]); row_pinned.push_back(false);
          row_leaving_soon.push_back(false);
          keys.push_back(unpinned_pool[i].composite_key()); deps.push_back(unpinned_pool[i].departure_time);
        }
      }
    }

    // Always Scroll or Replace: when the display is completely static
    // (no h-scroll, no transition, no post-pause hold), trigger a
    // collapse/replace after page_interval to keep the display alive.
    if (!triggered_transition && this->always_scroll_or_replace_ && this->diff_.prev_pinned_count >= 0) {
      bool h_scroll_active = (this->h_scroll_.total_distance > 0 || this->h_scroll_.prev_total_distance > 0);
      bool post_pause_active = (this->post_pause_end_ != 0 && now < this->post_pause_end_);
      if (!h_scroll_active && !post_pause_active) {
        if (this->idle_since_ == 0) {
          this->idle_since_ = now;
        } else if (now - this->idle_since_ >= (unsigned long)this->page_interval_) {
          this->begin_transition_(rows, row_pinned, row_leaving_soon, eff_pinned, false, now);
          this->idle_since_ = 0;
        }
      } else {
        this->idle_since_ = 0;
      }
    } else if (triggered_transition) {
      this->idle_since_ = 0;
    }
  }

  // ---- 13. Tick transition ----
  this->tick_transition_(now, fh, pinned_pool, unpinned_pool, eff_pinned, eff_unpinned);

  // ---- 14. Render ----
  this->render_frame_(rows, row_pinned, row_leaving_soon, eff_pinned, now, rtc_now, fh, y_base);

  // ---- 15. Commit diff (only when idle) ----
  if (this->transition_.phase == Transition::IDLE) {
    this->diff_.commit(keys, deps, rows, row_pinned, row_leaving_soon, eff_pinned, pinned_route_keys);
    this->committed_respect_pin_inset_ = this->respect_pin_inset_;
  }
}

}  // namespace transit_tracker
}  // namespace esphome
