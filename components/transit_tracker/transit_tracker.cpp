#include "transit_tracker.h"
#include "string_utils.h"

#include <algorithm>

#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/components/json/json_util.h"
#include "esphome/components/watchdog/watchdog.h"
#include "esphome/components/network/util.h"

namespace esphome {
namespace transit_tracker {

static const char *TAG = "transit_tracker.component";

void TransitTracker::setup() {
  this->ws_client_.onMessage([this](websockets::WebsocketsMessage message) {
    this->on_ws_message_(message);
  });

  this->ws_client_.onEvent([this](websockets::WebsocketsEvent event, String data) {
    this->on_ws_event_(event, data);
  });

  // Build the route_id -> stop_id lookup from schedule_string_
  this->rebuild_route_stop_map_();

  this->connect_ws_();

  this->set_interval("check_stale_trips", 10000, [this]() {
    if (this->ws_client_.available() && !this->schedule_state_.trips.empty()) {
      bool has_stale_trips = false;

      this->schedule_state_.mutex.lock();

      auto now = this->rtc_->now();
      if (now.is_valid()) {
        for (auto &trip : this->schedule_state_.trips) {
          if (now.timestamp - trip.departure_time > 60) {
            has_stale_trips = true;
            break;
          }
        }
      }

      this->schedule_state_.mutex.unlock();

      if (has_stale_trips) {
        ESP_LOGD(TAG, "Stale trips detected, reconnecting");
        ESP_LOGD(TAG, "  Current RTC time: %d", now.timestamp);
        ESP_LOGD(TAG, "  Last heartbeat: %d", this->last_heartbeat_);
        this->reconnect();
      }
    }
  });
}

void TransitTracker::loop() {
  this->ws_client_.poll();

#ifdef USE_MQTT
  // Publish pending MQTT routes when MQTT becomes connected
  if (mqtt::global_mqtt_client != nullptr && mqtt::global_mqtt_client->is_connected()) {
    if (this->mqtt_routes_pending_) {
      this->publish_mqtt_routes_();
      this->publish_mqtt_route_colors_();
    }
    if (this->mqtt_divider_color_pending_) {
      this->publish_mqtt_divider_color_();
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
}

void TransitTracker::reconnect() {
  this->close();
  this->connect_ws_();
}

void TransitTracker::close(bool fully) {
  if (fully) {
    this->fully_closed_ = true;
  }

  this->ws_client_.close();
}

void TransitTracker::on_shutdown() {
  this->cancel_interval("check_stale_trips");
  this->close(true);
}

void TransitTracker::on_ws_message_(websockets::WebsocketsMessage message) {
  ESP_LOGV(TAG, "Received message: %s", message.rawData().c_str());

  bool valid = json::parse_json(message.rawData(), [this](JsonObject root) -> bool {
    if (root["event"].as<std::string>() == "heartbeat") {
      ESP_LOGD(TAG, "Received heartbeat");
      this->last_heartbeat_ = millis();
      return true;
    }

    if (root["event"].as<std::string>() != "schedule") {
      return true;
    }

    ESP_LOGD(TAG, "Received schedule update");

    this->schedule_state_.mutex.lock();

    this->schedule_state_.trips.clear();

    auto data = root["data"].as<JsonObject>();

    for (auto trip : data["trips"].as<JsonArray>()) {
      std::string headsign = trip["headsign"].as<std::string>();
      for (const auto &abbr : this->abbreviations_) {
        size_t pos = headsign.find(abbr.first);
        if (pos != std::string::npos) {
          ESP_LOGV(TAG, "Applying abbreviation '%s' -> '%s' in headsign", abbr.first.c_str(), abbr.second.c_str());
          headsign.replace(pos, abbr.first.length(), abbr.second);
        }
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

      // Apply MQTT color override if present (takes highest priority)
      auto color_override = this->route_color_overrides_.find(route_id);
      if (color_override != this->route_color_overrides_.end()) {
        route_color = color_override->second;
      }

      std::string stop_id;
      if (!trip["stopId"].isNull()) {
        stop_id = trip["stopId"].as<std::string>();
      } else {
        // Fall back to schedule_string_ lookup
        auto it = this->route_stop_map_.find(route_id);
        if (it != this->route_stop_map_.end()) {
          stop_id = it->second;
        }
      }

      std::string stop_name;
      if (!trip["stopName"].isNull()) {
        stop_name = trip["stopName"].as<std::string>();
      }

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

    // Log received trips for debugging
    ESP_LOGD(TAG, "Schedule update: %d trip(s) received", (int)this->schedule_state_.trips.size());
    for (size_t i = 0; i < this->schedule_state_.trips.size(); i++) {
      const auto &t = this->schedule_state_.trips[i];
      ESP_LOGD(TAG, "  [%d] %s  dep=%ld  rt=%s",
               (int)i, t.composite_key().c_str(), (long)t.departure_time, t.is_realtime ? "Y" : "N");
    }

    this->schedule_state_.mutex.unlock();

#ifdef USE_MQTT
    // Flag routes for MQTT discovery (will publish in loop or on connect)
    this->mqtt_routes_pending_ = true;
    if (!this->mqtt_divider_published_) {
      this->mqtt_divider_color_pending_ = true;
    }
#endif

    return true;
  });

  if (!valid) {
    this->status_set_error("Failed to parse schedule data");
    return;
  }
}

void TransitTracker::on_ws_event_(websockets::WebsocketsEvent event, String data) {
  if (event == websockets::WebsocketsEvent::ConnectionOpened) {
    ESP_LOGD(TAG, "WebSocket connection opened");

    auto message = json::build_json([this](JsonObject root) {
      root["event"] = "schedule:subscribe";

      auto data = root.createNestedObject("data");

      if (!this->feed_code_.empty()) {
        data["feedCode"] = this->feed_code_;
      }

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
      this->defer([this]() {
        this->connect_ws_();
      });
    }
  } else if (event == websockets::WebsocketsEvent::GotPing) {
    ESP_LOGV(TAG, "Received ping");
  } else if (event == websockets::WebsocketsEvent::GotPong) {
    ESP_LOGV(TAG, "Received pong");
  }
}

void TransitTracker::connect_ws_() {
  if (this->base_url_.empty()) {
    ESP_LOGW(TAG, "No base URL set, not connecting");
    return;
  }

  if (this->fully_closed_) {
    ESP_LOGW(TAG, "Connection fully closed, not reconnecting");
    return;
  }

  if (this->ws_client_.available(true)) {
    ESP_LOGV(TAG, "Not reconnecting, already connected");
    return;
  }

  watchdog::WatchdogManager wdm(20000);

  this->last_heartbeat_ = 0;

  ESP_LOGD(TAG, "Connecting to WebSocket server (attempt %d): %s", this->connection_attempts_, this->base_url_.c_str());

  bool connection_success = false;
  if (esphome::network::is_connected()) {
    connection_success = this->ws_client_.connect(this->base_url_.c_str());
  } else {
    ESP_LOGW(TAG, "Not connected to network; skipping connection attempt");
  }

  if (!connection_success) {
    this->connection_attempts_++;

    if (this->connection_attempts_ >= 3) {
      this->status_set_error("Failed to connect to WebSocket server");
    }

    if (this->connection_attempts_ >= 15) {
      ESP_LOGE(TAG, "Could not connect to WebSocket server within 15 attempts.");
      ESP_LOGE(TAG, "It's likely that the network is not truly connected; rebooting the device to try to recover.");
      App.reboot();
    }

    auto timeout = std::min(15000, this->connection_attempts_ * 5000);
    ESP_LOGW(TAG, "Failed to connect, retrying in %ds", timeout / 1000);

    this->set_timeout("reconnect", timeout, [this]() {
      this->connect_ws_();
    });
  } else {
    this->has_ever_connected_ = true;
    this->connection_attempts_ = 0;
    this->status_clear_error();
  }
}

void TransitTracker::set_abbreviations_from_text(const std::string &text) {
  this->abbreviations_.clear();
  for (const auto &line : split(text, '\n')) {
    auto parts = split(line, ';');

    if (parts.size() == 1) {
      // If only one part is provided, treat it as a removal (replace with empty string)
      this->add_abbreviation(parts[0], "");
      continue;
    }

    if (parts.size() != 2) {
      ESP_LOGW(TAG, "Invalid abbreviation line: %s", line.c_str());
      continue;
    }

    this->add_abbreviation(parts[0], parts[1]);
  }
}

void TransitTracker::set_route_styles_from_text(const std::string &text) {
  this->route_styles_.clear();
  for (const auto &line : split(text, '\n')) {
    auto parts = split(line, ';');
    if (parts.size() != 3) {
      ESP_LOGW(TAG, "Invalid route style line: %s", line.c_str());
      continue;
    }
    uint32_t color = std::stoul(parts[2], nullptr, 16);
    this->add_route_style(parts[0], parts[1], Color(color));
  }
}

void TransitTracker::set_hidden_routes_from_text(const std::string &text) {
  this->hidden_routes_.clear();
  if (text.empty()) return;
  for (const auto &route_id : split(text, ';')) {
    if (!route_id.empty()) {
      this->hidden_routes_.insert(route_id);
      ESP_LOGD(TAG, "Hiding route: %s", route_id.c_str());
    }
  }
}

void TransitTracker::set_pinned_routes_from_text(const std::string &text) {
  this->pinned_routes_.clear();
  if (text.empty()) return;
  for (const auto &route_id : split(text, ';')) {
    if (!route_id.empty()) {
      this->pinned_routes_.insert(route_id);
      ESP_LOGD(TAG, "Pinning route: %s", route_id.c_str());
    }
  }
}

void TransitTracker::rebuild_route_stop_map_() {
  // Parse schedule_string_ to build route_id -> stop_id lookup.
  // Format: route,stop_id,time_offset;route,stop_id,time_offset;...
  // If a route appears at multiple different stops, we can't disambiguate
  // from server data, so we leave it out (headsign still differentiates).
  this->route_stop_map_.clear();
  std::set<std::string> ambiguous_routes;
  for (const auto &pair : split(this->schedule_string_, ';')) {
    auto parts = split(pair, ',');
    if (parts.size() >= 2) {
      auto it = this->route_stop_map_.find(parts[0]);
      if (it != this->route_stop_map_.end() && it->second != parts[1]) {
        ambiguous_routes.insert(parts[0]);
      } else {
        this->route_stop_map_[parts[0]] = parts[1];
      }
    }
  }
  for (const auto &r : ambiguous_routes) {
    this->route_stop_map_.erase(r);
    ESP_LOGD(TAG, "Route %s appears at multiple stops, stop_id omitted from key", r.c_str());
  }
}

void TransitTracker::draw_text_centered_(const char *text, Color color) {
  int display_center_x = this->display_->get_width() / 2;
  int display_center_y = this->display_->get_height() / 2;
  this->display_->print(display_center_x, display_center_y, this->font_, color, display::TextAlign::CENTER, text);
}

void TransitTracker::set_realtime_color(const Color &color) {
  this->realtime_color_ = color;
  this->realtime_color_dark_ = Color(
    (color.r * 0.5),
    (color.g * 0.5),
    (color.b * 0.5)
  );
}

void TransitTracker::set_divider_color_from_text(const std::string &text) {
  if (text.empty()) return;
  // Parse "R,G,B" format
  int r = 255, g = 0, b = 0;
  if (sscanf(text.c_str(), "%d,%d,%d", &r, &g, &b) == 3) {
    this->divider_color_ = Color(r, g, b);
  }
}

void TransitTracker::set_route_color_overrides_from_text(const std::string &text) {
  this->route_color_overrides_.clear();
  if (text.empty()) return;
  // Format: "route_id:R,G,B;route_id:R,G,B;..."
  for (const auto &entry : split(text, ';')) {
    auto colon_pos = entry.find(':');
    if (colon_pos == std::string::npos) continue;
    std::string route_id = entry.substr(0, colon_pos);
    std::string rgb_str = entry.substr(colon_pos + 1);
    int r, g, b;
    if (sscanf(rgb_str.c_str(), "%d,%d,%d", &r, &g, &b) == 3) {
      this->route_color_overrides_[route_id] = Color(r, g, b);
    }
  }
}

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
  if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) {
    return;
  }

  this->mqtt_routes_pending_ = false;

  std::string node = App.get_name();
  std::string topic_prefix = mqtt::global_mqtt_client->get_topic_prefix();

  for (const auto &t : this->schedule_state_.trips) {
    std::string key = t.composite_key();
    if (this->mqtt_published_routes_.count(key)) continue;
    this->mqtt_published_routes_.insert(key);

    std::string slug = slugify_(key);
    std::string object_id = node + "_route_" + slug;
    std::string state_topic = topic_prefix + "/route/" + slug + "/state";
    std::string command_topic = topic_prefix + "/route/" + slug + "/set";

    std::string friendly_name = t.route_name;
    if (!t.headsign.empty()) {
      friendly_name += " - " + t.headsign;
    }
    if (!t.stop_name.empty()) {
      friendly_name += " - " + t.stop_name;
    }

    // Build discovery JSON (MQTT select entity with Off/On/Pinned)
    std::string discovery_topic = "homeassistant/select/" + object_id + "/config";
    auto payload = json::build_json([&](JsonObject root) {
      root["name"] = friendly_name;
      root["unique_id"] = object_id;
      root["default_entity_id"] = "select." + object_id;
      root["state_topic"] = state_topic;
      root["command_topic"] = command_topic;
      root["icon"] = "mdi:bus";

      auto options = root.createNestedArray("options");
      options.add("Off");
      options.add("On");
      options.add("Pinned");

      // Use the MQTT client's own availability info so topics match the birth/will messages
      const auto &avail = mqtt::global_mqtt_client->get_availability();
      if (!avail.topic.empty()) {
        root["availability_topic"] = avail.topic;
        root["payload_available"] = avail.payload_available;
        root["payload_not_available"] = avail.payload_not_available;
      }

      auto device = root.createNestedObject("device");
      auto ids = device.createNestedArray("identifiers");
      ids.add(node);
      device["name"] = App.get_friendly_name().empty() ? node : App.get_friendly_name();
    });

    mqtt::global_mqtt_client->publish(discovery_topic, payload, 0, true);

    // Also remove any stale switch discovery from before the conversion
    std::string old_switch_topic = "homeassistant/switch/" + object_id + "/config";
    mqtt::global_mqtt_client->publish(old_switch_topic, "", 0, true);

    ESP_LOGD(TAG, "Published MQTT discovery for route: %s", friendly_name.c_str());

    // Determine and publish current state
    bool hidden = (this->hidden_routes_.find(key) != this->hidden_routes_.end());
    bool pinned = (this->pinned_routes_.find(key) != this->pinned_routes_.end());
    std::string state = hidden ? "Off" : (pinned ? "Pinned" : "On");
    mqtt::global_mqtt_client->publish(state_topic, state, 0, true);

    // Subscribe to command topic if not already
    if (this->mqtt_subscribed_routes_.find(key) == this->mqtt_subscribed_routes_.end()) {
      this->mqtt_subscribed_routes_.insert(key);
      std::string captured_key = key;
      std::string captured_state_topic = state_topic;
      mqtt::global_mqtt_client->subscribe(
        command_topic,
        [this, captured_key, captured_state_topic](const std::string &topic, const std::string &payload) {
          if (payload == "Off") {
            // Enforce at least one route visible
            int visible_count = 0;
            std::set<std::string> current_keys;
            for (const auto &trip : this->schedule_state_.trips) {
              std::string k = trip.composite_key();
              if (current_keys.count(k)) continue;
              current_keys.insert(k);
              if (this->hidden_routes_.find(k) == this->hidden_routes_.end()) {
                visible_count++;
              }
            }
            if (visible_count <= 1) {
              ESP_LOGW(TAG, "Cannot hide route %s — at least one must remain visible", captured_key.c_str());
              // Re-publish current state to reject the command
              bool is_pinned = (this->pinned_routes_.find(captured_key) != this->pinned_routes_.end());
              mqtt::global_mqtt_client->publish(captured_state_topic, std::string(is_pinned ? "Pinned" : "On"), 0, true);
              return;
            }
            this->hidden_routes_.insert(captured_key);
            this->pinned_routes_.erase(captured_key);
          } else if (payload == "On") {
            this->hidden_routes_.erase(captured_key);
            this->pinned_routes_.erase(captured_key);
          } else if (payload == "Pinned") {
            this->hidden_routes_.erase(captured_key);
            this->pinned_routes_.insert(captured_key);
          } else {
            ESP_LOGW(TAG, "Unknown route command: %s", payload.c_str());
            return;
          }

          mqtt::global_mqtt_client->publish(captured_state_topic, payload, 0, true);
          ESP_LOGD(TAG, "Route %s set to %s via MQTT", captured_key.c_str(), payload.c_str());

          this->persist_hidden_routes_();
          this->persist_pinned_routes_();
        },
        0
      );
    }
  }
}

void TransitTracker::persist_hidden_routes_() {
  std::string hidden_str;
  for (const auto &k : this->hidden_routes_) {
    if (!hidden_str.empty()) hidden_str += ";";
    hidden_str += k;
  }

#ifdef USE_TEXT
  if (this->hidden_routes_text_ != nullptr) {
    auto call = this->hidden_routes_text_->make_call();
    call.set_value(hidden_str);
    call.perform();
    ESP_LOGD(TAG, "Persisted hidden routes to text entity: %s", hidden_str.c_str());
  }
#endif
}

void TransitTracker::persist_pinned_routes_() {
  std::string pinned_str;
  for (const auto &k : this->pinned_routes_) {
    if (!pinned_str.empty()) pinned_str += ";";
    pinned_str += k;
  }

#ifdef USE_TEXT
  if (this->pinned_routes_text_ != nullptr) {
    auto call = this->pinned_routes_text_->make_call();
    call.set_value(pinned_str);
    call.perform();
    ESP_LOGD(TAG, "Persisted pinned routes to text entity: %s", pinned_str.c_str());
  }
#endif
}

void TransitTracker::persist_divider_color_() {
  char buf[16];
  snprintf(buf, sizeof(buf), "%d,%d,%d", this->divider_color_.r, this->divider_color_.g, this->divider_color_.b);
  std::string color_str(buf);

#ifdef USE_TEXT
  if (this->divider_color_text_ != nullptr) {
    auto call = this->divider_color_text_->make_call();
    call.set_value(color_str);
    call.perform();
  }
#endif
}

void TransitTracker::publish_mqtt_divider_color_() {
  if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) {
    return;
  }

  this->mqtt_divider_color_pending_ = false;
  this->mqtt_divider_published_ = true;

  std::string node = App.get_name();
  std::string topic_prefix = mqtt::global_mqtt_client->get_topic_prefix();

  std::string object_id = node + "_divider_color";
  std::string state_topic = topic_prefix + "/divider_color/state";
  std::string command_topic = topic_prefix + "/divider_color/set";

  // Build MQTT Light discovery JSON (RGB only, no brightness)
  std::string discovery_topic = "homeassistant/light/" + object_id + "/config";
  auto payload = json::build_json([&](JsonObject root) {
    root["name"] = "Pinned Line Color";
    root["unique_id"] = object_id;
    root["default_entity_id"] = "light." + object_id;
    root["schema"] = "json";
    root["state_topic"] = state_topic;
    root["command_topic"] = command_topic;
    root["icon"] = "mdi:line-scan";
    root["brightness"] = false;

    auto color_modes = root.createNestedArray("supported_color_modes");
    color_modes.add("rgb");

    const auto &avail = mqtt::global_mqtt_client->get_availability();
    if (!avail.topic.empty()) {
      root["availability_topic"] = avail.topic;
      root["payload_available"] = avail.payload_available;
      root["payload_not_available"] = avail.payload_not_available;
    }

    auto device = root.createNestedObject("device");
    auto ids = device.createNestedArray("identifiers");
    ids.add(node);
    device["name"] = App.get_friendly_name().empty() ? node : App.get_friendly_name();
  });

  mqtt::global_mqtt_client->publish(discovery_topic, payload, 0, true);

  // Publish current state
  auto state_payload = json::build_json([&](JsonObject root) {
    root["state"] = "ON";
    auto color = root.createNestedObject("color");
    color["r"] = this->divider_color_.r;
    color["g"] = this->divider_color_.g;
    color["b"] = this->divider_color_.b;
    root["color_mode"] = "rgb";
  });
  mqtt::global_mqtt_client->publish(state_topic, state_payload, 0, true);

  // Subscribe to command topic
  if (!this->mqtt_divider_color_subscribed_) {
    this->mqtt_divider_color_subscribed_ = true;
    mqtt::global_mqtt_client->subscribe(
      command_topic,
      [this, state_topic](const std::string &topic, const std::string &cmd_payload) {
        // Parse JSON command: {"state":"ON","color":{"r":255,"g":0,"b":0}}
        bool parsed = json::parse_json(cmd_payload, [this](JsonObject root) -> bool {
          if (root.containsKey("color")) {
            JsonObject color = root["color"];
            uint8_t r = color["r"] | 255;
            uint8_t g = color["g"] | 0;
            uint8_t b = color["b"] | 0;
            this->divider_color_ = Color(r, g, b);
          }
          return true;
        });

        if (parsed) {
          // Publish updated state back
          auto new_state = json::build_json([this](JsonObject root) {
            root["state"] = "ON";
            auto color = root.createNestedObject("color");
            color["r"] = this->divider_color_.r;
            color["g"] = this->divider_color_.g;
            color["b"] = this->divider_color_.b;
            root["color_mode"] = "rgb";
          });
          mqtt::global_mqtt_client->publish(state_topic, new_state, 0, true);
          this->persist_divider_color_();
        }
      },
      0
    );
  }
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
  if (this->route_color_overrides_text_ != nullptr) {
    auto call = this->route_color_overrides_text_->make_call();
    call.set_value(result);
    call.perform();
  }
#endif
}

void TransitTracker::publish_mqtt_route_colors_() {
  if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) {
    return;
  }

  std::string node = App.get_name();
  std::string topic_prefix = mqtt::global_mqtt_client->get_topic_prefix();

  for (const auto &t : this->schedule_state_.trips) {
    if (this->mqtt_published_route_color_ids_.count(t.route_id)) continue;
    this->mqtt_published_route_color_ids_.insert(t.route_id);

    std::string slug = slugify_(t.route_id);
    std::string object_id = node + "_route_color_" + slug;
    std::string state_topic = topic_prefix + "/route_color/" + slug + "/state";
    std::string command_topic = topic_prefix + "/route_color/" + slug + "/set";

    // Use route_name for the friendly name
    std::string friendly_name = t.route_name + " Color";

    // Determine current color for this route
    Color current_color = t.route_color;

    // Build MQTT Light discovery JSON (RGB only, no brightness)
    std::string discovery_topic = "homeassistant/light/" + object_id + "/config";
    auto payload = json::build_json([&](JsonObject root) {
      root["name"] = friendly_name;
      root["unique_id"] = object_id;
      root["default_entity_id"] = "light." + object_id;
      root["schema"] = "json";
      root["state_topic"] = state_topic;
      root["command_topic"] = command_topic;
      root["icon"] = "mdi:palette";
      root["brightness"] = false;

      auto color_modes = root.createNestedArray("supported_color_modes");
      color_modes.add("rgb");

      const auto &avail = mqtt::global_mqtt_client->get_availability();
      if (!avail.topic.empty()) {
        root["availability_topic"] = avail.topic;
        root["payload_available"] = avail.payload_available;
        root["payload_not_available"] = avail.payload_not_available;
      }

      auto device = root.createNestedObject("device");
      auto ids = device.createNestedArray("identifiers");
      ids.add(node);
      device["name"] = App.get_friendly_name().empty() ? node : App.get_friendly_name();
    });

    mqtt::global_mqtt_client->publish(discovery_topic, payload, 0, true);

    // Publish current color state
    auto state_payload = json::build_json([&](JsonObject root) {
      root["state"] = "ON";
      auto color = root.createNestedObject("color");
      color["r"] = current_color.r;
      color["g"] = current_color.g;
      color["b"] = current_color.b;
      root["color_mode"] = "rgb";
    });
    mqtt::global_mqtt_client->publish(state_topic, state_payload, 0, true);

    // Subscribe to command topic if not already
    if (this->mqtt_subscribed_route_colors_.find(t.route_id) == this->mqtt_subscribed_route_colors_.end()) {
      this->mqtt_subscribed_route_colors_.insert(t.route_id);
      std::string captured_route_id = t.route_id;
      std::string captured_state_topic = state_topic;
      mqtt::global_mqtt_client->subscribe(
        command_topic,
        [this, captured_route_id, captured_state_topic](const std::string &topic, const std::string &cmd_payload) {
          bool parsed = json::parse_json(cmd_payload, [this, &captured_route_id](JsonObject root) -> bool {
            if (root.containsKey("color")) {
              JsonObject color = root["color"];
              uint8_t r = color["r"] | 255;
              uint8_t g = color["g"] | 0;
              uint8_t b = color["b"] | 0;
              Color new_color(r, g, b);

              // Store the override
              this->route_color_overrides_[captured_route_id] = new_color;

              // Update all current trips with this route_id
              this->schedule_state_.mutex.lock();
              for (auto &trip : this->schedule_state_.trips) {
                if (trip.route_id == captured_route_id) {
                  trip.route_color = new_color;
                }
              }
              this->schedule_state_.mutex.unlock();
            }
            return true;
          });

          if (parsed) {
            // Publish updated state back
            auto override_it = this->route_color_overrides_.find(captured_route_id);
            if (override_it != this->route_color_overrides_.end()) {
              auto new_state = json::build_json([&](JsonObject root) {
                root["state"] = "ON";
                auto color = root.createNestedObject("color");
                color["r"] = override_it->second.r;
                color["g"] = override_it->second.g;
                color["b"] = override_it->second.b;
                root["color_mode"] = "rgb";
              });
              mqtt::global_mqtt_client->publish(captured_state_topic, new_state, 0, true);
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

const uint8_t realtime_icon[6][6] = {
  {0, 0, 0, 3, 3, 3},
  {0, 0, 3, 0, 0, 0},
  {0, 3, 0, 0, 2, 2},
  {3, 0, 0, 2, 0, 0},
  {3, 0, 2, 0, 0, 1},
  {3, 0, 2, 0, 1, 1}
};

void HOT TransitTracker::draw_realtime_icon_(int bottom_right_x, int bottom_right_y, unsigned long uptime) {
  const int num_frames = 6;
  const int idle_frame_duration = 3000;
  const int anim_frame_duration = 200;
  const int cycle_duration = idle_frame_duration + (num_frames - 1) * anim_frame_duration;

  unsigned long cycle_time = uptime % cycle_duration;

  int frame;
  if (cycle_time < idle_frame_duration) {
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
      if (segment_number == 0) {
        continue;
      }

      Color icon_color = is_segment_lit(segment_number) ? this->realtime_color_ : this->realtime_color_dark_;
      this->display_->draw_pixel_at(bottom_right_x - (5 - j), bottom_right_y - (5 - i), icon_color);
    }
  }
}

void TransitTracker::draw_trip(
    const Trip &trip, int y_offset, int font_height, unsigned long uptime, uint rtc_now,
    bool no_draw, int *headsign_overflow_out, int *headsign_width_out,
    int h_scroll_offset, int total_scroll_distance,
    int headsign_clipping_start_override, int headsign_clipping_end_override,
    bool is_pinned
) {
    // Draw pin icon for pinned routes (5px wide + 1px margins = 7px total)
    // Non-pinned routes also respect the inset so route names align
    int pin_offset = 0;
    if (this->frame_pin_offset_override_ >= 0) {
      // Phase 4 slide: use override offset, don't draw icon
      pin_offset = this->frame_pin_offset_override_;
    } else if (this->show_pin_icon_ && this->frame_show_pin_icons_) {
      pin_offset = 7; // 1px margin + 5px icon + 1px margin
      if (is_pinned && !no_draw) {
        Color pin_color = Color(0xFF0000); // red
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
        // Row 3: 5px centered
        for (int c = 0; c < 5; c++) this->display_->draw_pixel_at(px + c, py, pin_color);
        py++;
        // Remaining rows: 1px white vertical line centered
        int bottom = y_offset + font_height - 3;
        for (int r = py; r < bottom; r++) {
          this->display_->draw_pixel_at(cx, r, Color(0xFFFFFF));
        }
      }
    }

    if (!no_draw) {
      this->display_->print(pin_offset, y_offset, this->font_, trip.route_color, display::TextAlign::TOP_LEFT, trip.route_name.c_str());
    }

    int route_width, _;
    this->font_->measure(trip.route_name.c_str(), &route_width, &_, &_, &_);
    route_width += pin_offset;

    auto time_display = this->localization_.fmt_duration_from_now(
      this->display_departure_times_ ? trip.departure_time : trip.arrival_time,
      rtc_now
    );

    int time_width;
    this->font_->measure(time_display.c_str(), &time_width, &_, &_, &_);

    int headsign_clipping_start = (headsign_clipping_start_override >= 0) ? headsign_clipping_start_override : route_width + 3;
    int headsign_clipping_end = (headsign_clipping_end_override >= 0) ? headsign_clipping_end_override : this->display_->get_width() - time_width - 2;

    if (!no_draw) {
      Color time_color = trip.is_realtime ? this->realtime_color_ : Color(0xa7a7a7);
      this->display_->print(this->display_->get_width() + 1, y_offset, this->font_, time_color, display::TextAlign::TOP_RIGHT, time_display.c_str());
    }

    if (trip.is_realtime) {
      if (headsign_clipping_end_override < 0) {
        headsign_clipping_end -= 8;
      }

      if(!no_draw) {
        int icon_bottom_right_x = this->display_->get_width() - time_width - 2;
        int icon_bottom_right_y = y_offset + font_height - 5;

        this->draw_realtime_icon_(icon_bottom_right_x, icon_bottom_right_y, uptime);
      }
    }

    int headsign_max_width = headsign_clipping_end - headsign_clipping_start;

    int headsign_actual_width;
    this->font_->measure(trip.headsign.c_str(), &headsign_actual_width, &_, &_, &_);

    int headsign_overflow = headsign_actual_width - headsign_max_width;
    if (headsign_overflow_out) {
      *headsign_overflow_out = headsign_overflow;
    }
    if (headsign_width_out) {
      *headsign_width_out = headsign_actual_width;
    }

    if (no_draw) {
      return;
    }

    // Draw headsign with optional continuous marquee scroll
    this->display_->start_clipping(headsign_clipping_start, 0, headsign_clipping_end, this->display_->get_height());
    if (headsign_overflow > 0 && h_scroll_offset >= 0 && total_scroll_distance > 0) {
      // Continuous marquee: draw text and its wrapping copy
      int x = headsign_clipping_start - h_scroll_offset;
      this->display_->print(x, y_offset, this->font_, trip.headsign.c_str());
      this->display_->print(x + total_scroll_distance, y_offset, this->font_, trip.headsign.c_str());
    } else {
      this->display_->print(headsign_clipping_start, y_offset, this->font_, trip.headsign.c_str());
    }
    this->display_->end_clipping();
}

void HOT TransitTracker::draw_schedule() {
  if (this->display_ == nullptr) {
    ESP_LOGW(TAG, "No display attached, cannot draw schedule");
    return;
  }

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
    ESP_LOGW(TAG, "draw_schedule: trips empty (pre-lock)");
    auto message = "No upcoming arrivals";
    if (this->display_departure_times_) {
      message = "No upcoming departures";
    }

    this->draw_text_centered_(message, Color(0x252627));
    return;
  }

  this->schedule_state_.mutex.lock();

  // Filter out hidden routes (matched by composite key: routeId:headsign[:stopId])
  std::vector<Trip> visible_trips;
  if (!this->hidden_routes_.empty()) {
    for (const Trip &trip : this->schedule_state_.trips) {
      if (this->hidden_routes_.find(trip.composite_key()) == this->hidden_routes_.end()) {
        visible_trips.push_back(trip);
      }
    }
  } else {
    visible_trips = this->schedule_state_.trips;
  }

  if (visible_trips.empty()) {
    ESP_LOGW(TAG, "draw_schedule: visible_trips empty after filtering (total trips=%d, hidden_routes=%d)",
             (int)this->schedule_state_.trips.size(), (int)this->hidden_routes_.size());
    this->schedule_state_.mutex.unlock();
    auto message = "No upcoming arrivals";
    if (this->display_departure_times_) {
      message = "No upcoming departures";
    }
    this->draw_text_centered_(message, Color(0x252627));
    return;
  }

  // Count distinct pinned routes (without reordering — partition and dedup happen after transition check)
  // Each pinned route occupies exactly one row; multiple trips from the same route share that row.
  int actual_pinned_in_visible = 0;
  if (!this->pinned_routes_.empty()) {
    std::set<std::string> seen_pinned_keys;
    for (const auto &trip : visible_trips) {
      if (this->pinned_routes_.find(trip.composite_key()) != this->pinned_routes_.end()) {
        seen_pinned_keys.insert(trip.composite_key());
      }
    }
    actual_pinned_in_visible = (int)seen_pinned_keys.size();
  }
  int effective_pinned_count = (actual_pinned_in_visible > 0)
    ? std::min(this->pinned_rows_count_, actual_pinned_in_visible)
    : 0;

  int nominal_font_height = this->font_->get_ascender() + this->font_->get_descender();
  unsigned long uptime = millis();
  uint rtc_now = this->rtc_->now().timestamp;

  // ====== PIN TRANSITION: detect pinned-count changes and animate ======
  // Initialize old effective pinned on first frame
  if (this->pin_transition_old_eff_pinned_ < 0) {
    this->pin_transition_old_eff_pinned_ = effective_pinned_count;
    this->pin_transition_old_pinned_routes_ = this->pinned_routes_;
    this->pin_transition_old_hidden_routes_ = this->hidden_routes_;
  }

  // Detect change in effective pinned count (new pins added or removed)
  if (this->pin_transition_phase_ == 0 && effective_pinned_count != this->pin_transition_old_eff_pinned_) {
    ESP_LOGD(TAG, "Pin transition: %d -> %d, entering phase 1 (wait for scroll idle)",
             this->pin_transition_old_eff_pinned_, effective_pinned_count);
    this->pin_transition_phase_ = 1;
    this->pin_transition_start_ = uptime;
    this->pin_transition_seen_nonzero_offset_ = false;
    this->pin_transition_target_eff_pinned_ = effective_pinned_count;
    this->pin_transition_target_pinned_routes_ = this->pinned_routes_;
    this->pin_transition_target_hidden_routes_ = this->hidden_routes_;
    this->pin_transition_pending_restart_ = false;
  } else if (this->pin_transition_phase_ == 0 &&
             effective_pinned_count == this->pin_transition_old_eff_pinned_ &&
             this->pinned_routes_ != this->pin_transition_old_pinned_routes_) {
    // Pinned route SET changed but effective count stayed the same
    // (e.g. pinned_rows_count_=1, already had route A pinned, now also pin route B)
    // Still need to animate the transition.
    ESP_LOGD(TAG, "Pin transition: route set changed (eff count %d unchanged), entering phase 1",
             effective_pinned_count);
    this->pin_transition_phase_ = 1;
    this->pin_transition_start_ = uptime;
    this->pin_transition_seen_nonzero_offset_ = false;
    this->pin_transition_target_eff_pinned_ = effective_pinned_count;
    this->pin_transition_target_pinned_routes_ = this->pinned_routes_;
    this->pin_transition_target_hidden_routes_ = this->hidden_routes_;
    this->pin_transition_pending_restart_ = false;
  } else if (this->pin_transition_phase_ != 0 && effective_pinned_count != this->pin_transition_target_eff_pinned_) {
    if (this->pin_transition_phase_ == 1) {
      // Still waiting for scroll idle — no animation has started yet, so we can
      // safely update the targets directly instead of queuing a restart.
      // This avoids a full close+reopen when e.g. unpinning route A then immediately
      // pinning route B (two MQTT messages in rapid succession).
      ESP_LOGD(TAG, "Pin transition: new change during phase 1 (target %d -> %d), updating target directly",
               this->pin_transition_target_eff_pinned_, effective_pinned_count);
      this->pin_transition_target_eff_pinned_ = effective_pinned_count;
      this->pin_transition_target_pinned_routes_ = this->pinned_routes_;
      this->pin_transition_target_hidden_routes_ = this->hidden_routes_;
      this->pin_transition_pending_restart_ = false;
      // If targets now match the old state (net zero change), cancel the transition entirely
      if (this->pin_transition_target_eff_pinned_ == this->pin_transition_old_eff_pinned_ &&
          this->pin_transition_target_pinned_routes_ == this->pin_transition_old_pinned_routes_ &&
          this->pin_transition_target_hidden_routes_ == this->pin_transition_old_hidden_routes_) {
        ESP_LOGD(TAG, "Pin transition: net zero change, cancelling transition");
        this->pin_transition_phase_ = 0;
      }
    } else {
      // Animation already in progress — queue restart
      // Don't update target_pinned_routes_ — let the current transition finish with
      // its original target. The pending restart will start a new transition afterward.
      ESP_LOGD(TAG, "Pin transition: new change mid-transition (target %d -> %d), queuing restart",
               this->pin_transition_target_eff_pinned_, effective_pinned_count);
      this->pin_transition_target_eff_pinned_ = effective_pinned_count;
      this->pin_transition_pending_restart_ = true;
    }
  } else if (this->pin_transition_phase_ != 0 && !this->pin_transition_pending_restart_ &&
             this->pinned_routes_ != this->pin_transition_target_pinned_routes_) {
    if (this->pin_transition_phase_ == 1) {
      // Still in phase 1 — update target route set directly
      ESP_LOGD(TAG, "Pin transition: pin set changed during phase 1 (same count), updating target directly");
      this->pin_transition_target_pinned_routes_ = this->pinned_routes_;
      this->pin_transition_target_hidden_routes_ = this->hidden_routes_;
      this->pin_transition_pending_restart_ = false;
      // If targets now match old state, cancel
      if (this->pin_transition_target_pinned_routes_ == this->pin_transition_old_pinned_routes_ &&
          this->pin_transition_target_hidden_routes_ == this->pin_transition_old_hidden_routes_) {
        ESP_LOGD(TAG, "Pin transition: net zero change, cancelling transition");
        this->pin_transition_phase_ = 0;
      }
    } else {
      // Pin set changed but effective count stayed the same (e.g. swapped which route is pinned)
      // Queue a restart so the current transition finishes with its target, then a new one starts
      ESP_LOGD(TAG, "Pin transition: pin set changed mid-transition (same count), queuing restart");
      this->pin_transition_pending_restart_ = true;
    }
  }

  // Reset per-frame pin offset override (phase 4 sets it)
  this->frame_pin_offset_override_ = -1;

  // During pin transitions, freeze the hidden-route filter to prevent newly-unhidden
  // routes from snapping in before the animation handles them.
  // Phases 1-2, 5-9, 12 use old hidden routes (pre-change); phases 3-4, 10-11, 13 use target.
  if (this->pin_transition_phase_ >= 1) {
    const auto &frozen_hidden =
      (this->pin_transition_phase_ <= 2 || (this->pin_transition_phase_ >= 5 && this->pin_transition_phase_ <= 9) || this->pin_transition_phase_ == 12)
        ? this->pin_transition_old_hidden_routes_
        : this->pin_transition_target_hidden_routes_;

    visible_trips.clear();
    for (const Trip &trip : this->schedule_state_.trips) {
      if (frozen_hidden.find(trip.composite_key()) == frozen_hidden.end()) {
        visible_trips.push_back(trip);
      }
    }

    if (visible_trips.empty()) {
      this->schedule_state_.mutex.unlock();
      auto message = "No upcoming arrivals";
      if (this->display_departure_times_) {
        message = "No upcoming departures";
      }
      this->draw_text_centered_(message, Color(0x252627));
      return;
    }
  }

  // Phase 1, 2, 9, 12: use OLD pinned routes for partitioning so the display doesn't change.
  // Phase 2 additionally animates collapsing newly-pinned rows.
  // Phase 3, 4, 10, 11, 13 use TARGET routes (frozen at transition start, not live pinned_routes_).
  // Unpin phases 5-8 all use OLD routes (rendering old split layout while animating out).
  // Phase 12 uses OLD routes for pinned section collapse + computes target unpinned internally.
  if ((this->pin_transition_phase_ >= 1 && this->pin_transition_phase_ <= 2) ||
      (this->pin_transition_phase_ >= 5 && this->pin_transition_phase_ <= 9) ||
      this->pin_transition_phase_ == 12) {
    // Partition with OLD pinned routes
    actual_pinned_in_visible = 0;
    if (!this->pin_transition_old_pinned_routes_.empty()) {
      std::stable_partition(visible_trips.begin(), visible_trips.end(), [this](const Trip &trip) {
        return this->pin_transition_old_pinned_routes_.find(trip.composite_key()) != this->pin_transition_old_pinned_routes_.end();
      });
      for (const auto &trip : visible_trips) {
        if (this->pin_transition_old_pinned_routes_.find(trip.composite_key()) != this->pin_transition_old_pinned_routes_.end()) {
          actual_pinned_in_visible++;
        } else {
          break;
        }
      }
      // Deduplicate pinned section: keep only the soonest trip per distinct route
      {
        std::set<std::string> seen_keys;
        int write = 0;
        for (int i = 0; i < actual_pinned_in_visible; i++) {
          std::string key = visible_trips[i].composite_key();
          if (seen_keys.find(key) == seen_keys.end()) {
            seen_keys.insert(key);
            if (write != i) visible_trips[write] = visible_trips[i];
            write++;
          }
        }
        if (write < actual_pinned_in_visible) {
          visible_trips.erase(visible_trips.begin() + write, visible_trips.begin() + actual_pinned_in_visible);
          actual_pinned_in_visible = write;
        }
      }
    }
    effective_pinned_count = (actual_pinned_in_visible > 0)
      ? std::min(this->pinned_rows_count_, actual_pinned_in_visible)
      : 0;

    // Set frame flag for pin icons based on old state
    this->frame_show_pin_icons_ = (effective_pinned_count > 0);
    // Suppress pin icons during phases 7 and 8 (pin inset already removed in phase 6)
    if (this->pin_transition_phase_ == 7 || this->pin_transition_phase_ == 8) {
      this->frame_show_pin_icons_ = false;
    }
  } else {
    // Determine which routes to use for partitioning:
    // During phases 3-4 and 10-11, use the transition target (frozen at start of transition)
    // to prevent mid-transition pin changes from snapping in immediately.
    bool use_target = (this->pin_transition_phase_ >= 3 && this->pin_transition_phase_ <= 4) ||
                      (this->pin_transition_phase_ >= 10 && this->pin_transition_phase_ <= 11);
    const auto &partition_routes = use_target
      ? this->pin_transition_target_pinned_routes_
      : this->pinned_routes_;

    if (!partition_routes.empty()) {
      std::stable_partition(visible_trips.begin(), visible_trips.end(), [&partition_routes](const Trip &trip) {
        return partition_routes.find(trip.composite_key()) != partition_routes.end();
      });
      // Recount after partition (order-dependent count)
      actual_pinned_in_visible = 0;
      for (const auto &trip : visible_trips) {
        if (partition_routes.find(trip.composite_key()) != partition_routes.end()) {
          actual_pinned_in_visible++;
        } else {
          break;
        }
      }
      // Deduplicate pinned section: keep only the soonest trip per distinct route
      {
        std::set<std::string> seen_keys;
        int write = 0;
        for (int i = 0; i < actual_pinned_in_visible; i++) {
          std::string key = visible_trips[i].composite_key();
          if (seen_keys.find(key) == seen_keys.end()) {
            seen_keys.insert(key);
            if (write != i) visible_trips[write] = visible_trips[i];
            write++;
          }
        }
        if (write < actual_pinned_in_visible) {
          visible_trips.erase(visible_trips.begin() + write, visible_trips.begin() + actual_pinned_in_visible);
          actual_pinned_in_visible = write;
        }
      }
    }
    // Recompute effective_pinned_count based on distinct pinned routes
    effective_pinned_count = (actual_pinned_in_visible > 0)
      ? std::min(this->pinned_rows_count_, actual_pinned_in_visible)
      : 0;
    // Set frame flag for pin icons based on current state
    this->frame_show_pin_icons_ = (effective_pinned_count > 0);
    // Suppress pin icons during phase 3 push-down, phase 4 slide, and phase 8 scroll-up
    if (this->pin_transition_phase_ == 3 || this->pin_transition_phase_ == 4 || this->pin_transition_phase_ == 8) {
      this->frame_show_pin_icons_ = false;
    }

    // Keep hidden-routes snapshot current while idle so it's ready for next transition
    if (this->pin_transition_phase_ == 0) {
      this->pin_transition_old_hidden_routes_ = this->hidden_routes_;
      this->pin_transition_old_pinned_routes_ = this->pinned_routes_;
      this->pin_transition_old_eff_pinned_ = effective_pinned_count;
    }
  }

  // Pre-pass: compute uniform headsign boundaries if enabled
  // (must be after transition logic so effective_pinned_count reflects old state during transitions)
  int uniform_clipping_start = -1;
  int uniform_clipping_end = -1;

  if (this->uniform_headsign_start_ || this->uniform_headsign_end_) {
    int max_route_width = 0;
    int max_time_width = 0;
    bool any_realtime = false;
    int _;

    for (const Trip &trip : visible_trips) {
      int route_width;
      this->font_->measure(trip.route_name.c_str(), &route_width, &_, &_, &_);
      max_route_width = max(max_route_width, route_width);

      auto time_display = this->localization_.fmt_duration_from_now(
        this->display_departure_times_ ? trip.departure_time : trip.arrival_time,
        rtc_now
      );
      int time_width;
      this->font_->measure(time_display.c_str(), &time_width, &_, &_, &_);
      max_time_width = max(max_time_width, time_width);

      if (trip.is_realtime) {
        any_realtime = true;
      }
    }

    if (this->uniform_headsign_start_) {
      uniform_clipping_start = max_route_width + 3;
      // Account for pin icon width when any pinned trips are visible
      if (this->frame_pin_offset_override_ >= 0) {
        uniform_clipping_start += this->frame_pin_offset_override_;
      } else if (this->frame_show_pin_icons_ && this->show_pin_icon_) {
        uniform_clipping_start += 7; // 1px margin + 5px icon + 1px margin
      }
    }
    if (this->uniform_headsign_end_) {
      uniform_clipping_end = this->display_->get_width() - max_time_width - 2;
      if (any_realtime) {
        uniform_clipping_end -= 8;
      }
    }
  }

  // ====== SPLIT LAYOUT: independent pinned + unpinned sections ======
  bool has_split = (effective_pinned_count > 0 && actual_pinned_in_visible < (int)visible_trips.size());

  if (has_split) {
    // Separate trip pools
    std::vector<Trip> pinned_pool(visible_trips.begin(), visible_trips.begin() + actual_pinned_in_visible);
    std::vector<Trip> unpinned_pool(visible_trips.begin() + actual_pinned_in_visible, visible_trips.end());

    int pinned_rows = effective_pinned_count;
    int unpinned_rows = this->limit_ - pinned_rows;
    int pinned_pool_size = (int)pinned_pool.size();
    int unpinned_pool_size = (int)unpinned_pool.size();

    // Centering based on full layout (limit_ rows)
    int total_display_rows = this->limit_;
    int max_trips_height = (total_display_rows * this->font_->get_ascender()) + ((total_display_rows - 1) * this->font_->get_descender());
    int y_base = (this->display_->get_height() % max_trips_height) / 2;

    // ---- Determine current page slices from stored indices ----
    int pinned_pages = std::max(1, pinned_pool_size - pinned_rows + 1);
    if (this->pinned_page_index_ >= pinned_pages) this->pinned_page_index_ = 0;
    int pinned_si = (pinned_pool_size > pinned_rows) ? this->pinned_page_index_ : 0;
    int pinned_ei = std::min(pinned_si + pinned_rows, pinned_pool_size);

    int unpinned_pages = std::max(1, unpinned_pool_size - unpinned_rows + 1);
    if (this->split_unpinned_page_index_ >= unpinned_pages) this->split_unpinned_page_index_ = 0;
    int unpinned_si = (unpinned_pool_size > unpinned_rows && this->scroll_routes_) ? this->split_unpinned_page_index_ : 0;
    int unpinned_ei = std::min(unpinned_si + unpinned_rows, unpinned_pool_size);

    // ---- Shared h-scroll across ALL on-screen trips (pinned + unpinned) ----
    // pinned_h_scroll_start_ = shared h-scroll start time
    // pinned_page_timer_     = shared page dwell timer (when current page was set)
    // split_unpinned_page_timer_      = 0 if no page change pending, 1 if pending
    // split_unpinned_h_scroll_start_  = h-scroll cycle count when pending was set
    if (this->pinned_h_scroll_start_ == 0) this->pinned_h_scroll_start_ = uptime;
    if (this->pinned_page_timer_ == 0) this->pinned_page_timer_ = uptime;

    // ---- Paging needs (used by state machine and drawing) ----
    bool needs_pinned_paging = (pinned_pool_size > pinned_rows);
    bool needs_unpinned_paging = (unpinned_pool_size > unpinned_rows && this->scroll_routes_);
    bool needs_any_paging = needs_pinned_paging || needs_unpinned_paging;

    // ---- Split scroll phase state machine ----
    // Phase 0: idle (normal h-scroll + paging)
    // Phase 1: pre-scroll pause
    // Phase 2: pinned section v-scroll animation
    // Phase 3: mid-pause (between pinned and unpinned scrolls)
    // Phase 4: unpinned section v-scroll animation
    // Phase 5: post-scroll pause
    bool in_split_scroll = false;
    bool in_split_pinned_scroll = false;
    bool in_split_unpinned_scroll = false;
    float split_scroll_progress = 0.0f;
    bool in_split_pause = false;

    if (this->split_scroll_phase_ > 0 && this->split_scroll_start_ > 0) {
      unsigned long phase_elapsed = uptime - this->split_scroll_start_;

      switch (this->split_scroll_phase_) {
        case 1: // Pre-scroll pause
          if (phase_elapsed >= (unsigned long)this->page_pause_duration_) {
            ESP_LOGD(TAG, "Split scroll: phase 1 (pre-pause) complete, advancing pages");
            // Pre-pause complete — advance pages and begin pinned scroll
            this->split_old_pinned_page_ = this->pinned_page_index_;
            this->split_old_unpinned_page_ = this->split_unpinned_page_index_;

            if (needs_pinned_paging)
              this->pinned_page_index_ = (this->pinned_page_index_ + 1) % pinned_pages;
            if (needs_unpinned_paging)
              this->split_unpinned_page_index_ = (this->split_unpinned_page_index_ + 1) % unpinned_pages;

            pinned_si = needs_pinned_paging ? this->pinned_page_index_ : 0;
            pinned_ei = std::min(pinned_si + pinned_rows, pinned_pool_size);
            unpinned_si = needs_unpinned_paging ? this->split_unpinned_page_index_ : 0;
            unpinned_ei = std::min(unpinned_si + unpinned_rows, unpinned_pool_size);

            if (needs_pinned_paging && this->page_scroll_duration_ > 0) {
              // Begin pinned section scroll
              this->split_scroll_phase_ = 2;
              this->split_scroll_start_ = uptime;
              in_split_pinned_scroll = true;
              in_split_scroll = true;
              split_scroll_progress = 0.0f;
            } else if (needs_unpinned_paging && this->page_scroll_duration_ > 0) {
              // No pinned paging — skip to unpinned section scroll
              this->split_scroll_phase_ = 4;
              this->split_scroll_start_ = uptime;
              in_split_unpinned_scroll = true;
              in_split_scroll = true;
              split_scroll_progress = 0.0f;
            } else {
              // No animation needed — go to post-pause
              this->split_scroll_phase_ = 5;
              this->split_scroll_start_ = uptime;
              in_split_pause = true;
            }
            this->split_unpinned_page_timer_ = 0;
          } else {
            in_split_pause = true;
          }
          break;

        case 2: // Pinned section v-scroll
          if (phase_elapsed >= (unsigned long)this->page_scroll_duration_) {
            if (needs_unpinned_paging) {
              // Pinned done — enter mid-pause before unpinned scroll
              this->split_scroll_phase_ = 3;
              this->split_scroll_start_ = uptime;
              in_split_pause = true;
            } else {
              // No unpinned scroll needed — go to post-pause
              this->split_scroll_phase_ = 5;
              this->split_scroll_start_ = uptime;
              in_split_pause = true;
            }
          } else {
            float t = (float)phase_elapsed / (float)this->page_scroll_duration_;
            split_scroll_progress = t * t * (3.0f - 2.0f * t);
            in_split_pinned_scroll = true;
            in_split_scroll = true;
          }
          break;

        case 3: // Mid-pause (between sections)
          if (phase_elapsed >= (unsigned long)this->page_pause_duration_) {
            // Mid-pause complete — begin unpinned scroll
            this->split_scroll_phase_ = 4;
            this->split_scroll_start_ = uptime;
            in_split_unpinned_scroll = true;
            in_split_scroll = true;
            split_scroll_progress = 0.0f;
          } else {
            in_split_pause = true;
          }
          break;

        case 4: // Unpinned section v-scroll
          if (phase_elapsed >= (unsigned long)this->page_scroll_duration_) {
            // Unpinned done — enter post-pause
            this->split_scroll_phase_ = 5;
            this->split_scroll_start_ = uptime;
            in_split_pause = true;
          } else {
            float t = (float)phase_elapsed / (float)this->page_scroll_duration_;
            split_scroll_progress = t * t * (3.0f - 2.0f * t);
            in_split_unpinned_scroll = true;
            in_split_scroll = true;
          }
          break;

        case 5: // Post-scroll pause
          if (phase_elapsed >= (unsigned long)this->page_pause_duration_) {
            // Post-pause complete — resume normal operation
            ESP_LOGD(TAG, "Split scroll: phase 5 (post-pause) complete, resuming normal");
            this->split_scroll_phase_ = 0;
            this->split_scroll_start_ = 0;
            this->pinned_h_scroll_start_ = uptime;
            this->pinned_page_timer_ = uptime;
            // Clear old keys so the data transition detector doesn't see the
            // page change as a trip departure.  The steady-state updater will
            // repopulate them with the new page's keys this same frame.
            this->data_transition_old_keys_.clear();
            this->pinned_dt_old_trips_.clear();
          } else {
            in_split_pause = true;
          }
          break;
      }
    }

    int shared_scroll_dist = 0;
    int shared_h_offset = 0;
    int shared_h_cycles = 0;
    if (!in_split_scroll && !in_split_pause && this->scroll_headsigns_) {
      int max_hw = 0;
      bool any_ov = false;
      // During pinned data transition, use cached old trips for scroll distance
      // so the h-scroll doesn't jump when the trip changes mid-cycle.
      if (this->pinned_dt_phase_ >= 1 && !this->pinned_dt_old_trips_.empty()) {
        for (const auto &trip : this->pinned_dt_old_trips_) {
          int ov = 0, hw = 0;
          this->draw_trip(trip, 0, nominal_font_height, uptime, rtc_now,
                          true, &ov, &hw, -1, 0,
                          uniform_clipping_start, uniform_clipping_end, true);
          if (ov > 0) any_ov = true;
          max_hw = std::max(max_hw, hw);
        }
      } else {
        for (int i = pinned_si; i < pinned_ei; i++) {
          int ov = 0, hw = 0;
          this->draw_trip(pinned_pool[i], 0, nominal_font_height, uptime, rtc_now,
                          true, &ov, &hw, -1, 0,
                          uniform_clipping_start, uniform_clipping_end, true);
          if (ov > 0) any_ov = true;
          max_hw = std::max(max_hw, hw);
        }
      }
      for (int i = unpinned_si; i < unpinned_ei; i++) {
        int ov = 0, hw = 0;
        this->draw_trip(unpinned_pool[i], 0, nominal_font_height, uptime, rtc_now,
                        true, &ov, &hw, -1, 0,
                        uniform_clipping_start, uniform_clipping_end, false);
        if (ov > 0) any_ov = true;
        max_hw = std::max(max_hw, hw);
      }
      if (any_ov) {
        shared_scroll_dist = max_hw + marquee_gap;
        unsigned long elapsed = uptime - this->pinned_h_scroll_start_;
        int total_px = (int)(elapsed * scroll_speed / 1000);
        shared_h_offset = total_px % shared_scroll_dist;
        shared_h_cycles = total_px / shared_scroll_dist;
        this->last_shared_scroll_dist_ = shared_scroll_dist;
      } else if (this->last_shared_scroll_dist_ > 0) {
        // Overflow just disappeared mid-scroll — finish current cycle using old distance
        unsigned long elapsed = uptime - this->pinned_h_scroll_start_;
        int total_px = (int)(elapsed * scroll_speed / 1000);
        int wind_down_offset = total_px % this->last_shared_scroll_dist_;
        shared_h_cycles = total_px / this->last_shared_scroll_dist_;
        if (wind_down_offset < 3) {
          // Close enough to start — stop scrolling
          this->last_shared_scroll_dist_ = 0;
          this->pinned_h_scroll_start_ = uptime;
        } else {
          // Keep scrolling with old distance until we wrap back
          shared_scroll_dist = this->last_shared_scroll_dist_;
          shared_h_offset = wind_down_offset;
        }
      }
    }

    // Pin transition phase 1 completion check (split-layout path)
    if (this->pin_transition_phase_ == 1) {
      // Wait for split v-scroll to finish before checking h-scroll idle
      bool split_scroll_active = (this->split_scroll_phase_ != 0);
      if (split_scroll_active) {
        // Wait — split scroll must finish first
      } else if (shared_scroll_dist > 0) {
        // Scrolling active — track when offset moves away from 0, then wait for return
        if (shared_h_offset >= 3) {
          this->pin_transition_seen_nonzero_offset_ = true;
        }
        if (this->pin_transition_seen_nonzero_offset_ && shared_h_offset < 3) {
          shared_h_offset = 0;
          this->pinned_h_scroll_start_ = uptime;  // reset h-scroll to genuine 0
          bool is_same_count_swap = (this->pin_transition_target_eff_pinned_ == this->pin_transition_old_eff_pinned_) &&
                                    (this->pin_transition_target_pinned_routes_ != this->pin_transition_old_pinned_routes_);
          bool is_pin_up = (this->pin_transition_target_eff_pinned_ >= this->pin_transition_old_eff_pinned_);
          bool is_pin_expand = is_pin_up && this->pin_transition_old_eff_pinned_ > 0;
          if (is_same_count_swap) {
            ESP_LOGD(TAG, "Pin transition: phase 1 complete (split swap), entering phase 5 (hide old pinned)");
            this->pin_transition_phase_ = 5;
          } else if (is_pin_expand) {
            ESP_LOGD(TAG, "Pin transition: phase 1 complete (split expand), entering phase 9 (push-down)");
            this->pin_transition_phase_ = 9;
          } else if (is_pin_up) {
            ESP_LOGD(TAG, "Pin transition: phase 1 complete (split), entering phase 2");
            this->pin_transition_phase_ = 2;
          } else if (this->pin_transition_target_eff_pinned_ > 0) {
            ESP_LOGD(TAG, "Pin transition: phase 1 complete (split shrink %d->%d), entering phase 12",
                     this->pin_transition_old_eff_pinned_, this->pin_transition_target_eff_pinned_);
            this->pin_transition_phase_ = 12;
          } else {
            ESP_LOGD(TAG, "Pin transition: phase 1 complete (split unpin), entering phase 5 (hide-pinned)");
            this->pin_transition_phase_ = 5;
          }
          this->pin_transition_start_ = uptime;
        }
      } else {
        // No scrolling — wait 500ms
        unsigned long phase1_elapsed = uptime - this->pin_transition_start_;
        if (phase1_elapsed >= 500) {
          bool is_same_count_swap = (this->pin_transition_target_eff_pinned_ == this->pin_transition_old_eff_pinned_) &&
                                    (this->pin_transition_target_pinned_routes_ != this->pin_transition_old_pinned_routes_);
          bool is_pin_up = (this->pin_transition_target_eff_pinned_ >= this->pin_transition_old_eff_pinned_);
          bool is_pin_expand = is_pin_up && this->pin_transition_old_eff_pinned_ > 0;
          if (is_same_count_swap) {
            ESP_LOGD(TAG, "Pin transition: phase 1 complete (split swap no-scroll), entering phase 5");
            this->pin_transition_phase_ = 5;
          } else if (is_pin_expand) {
            ESP_LOGD(TAG, "Pin transition: phase 1 complete (split expand no-scroll), entering phase 9");
            this->pin_transition_phase_ = 9;
          } else if (is_pin_up) {
            ESP_LOGD(TAG, "Pin transition: phase 1 complete (split no-scroll), entering phase 2");
            this->pin_transition_phase_ = 2;
          } else if (this->pin_transition_target_eff_pinned_ > 0) {
            ESP_LOGD(TAG, "Pin transition: phase 1 complete (split shrink no-scroll %d->%d), entering phase 12",
                     this->pin_transition_old_eff_pinned_, this->pin_transition_target_eff_pinned_);
            this->pin_transition_phase_ = 12;
          } else {
            ESP_LOGD(TAG, "Pin transition: phase 1 complete (split no-scroll unpin), entering phase 5");
            this->pin_transition_phase_ = 5;
          }
          this->pin_transition_start_ = uptime;
        }
      }
    }

    // Pin transition phase 2: collapse newly-pinned rows (split-layout path)
    if (this->pin_transition_phase_ == 2) {
      unsigned long phase2_elapsed = uptime - this->pin_transition_start_;
      float collapse_t = std::min(1.0f, (float)phase2_elapsed / 500.0f);
      collapse_t = collapse_t * collapse_t * (3.0f - 2.0f * collapse_t);

      if (collapse_t >= 1.0f && phase2_elapsed >= 1000) { // 500ms collapse + 500ms dwell
        bool is_pin_up = (this->pin_transition_target_eff_pinned_ >= this->pin_transition_old_eff_pinned_);
        if (is_pin_up) {
          ESP_LOGD(TAG, "Pin transition: phase 2 complete (split), entering phase 3 (push-divider)");
          this->pin_transition_phase_ = 3;
          this->pin_transition_start_ = uptime;
          // Don't return — let phase 2 render the dwell state one more time.
          // Phase 3 starts cleanly next frame with new partition.
        } else {
          // Unpin — should have gone to phase 5 from phase 1; this is a fallback
          ESP_LOGD(TAG, "Pin transition: phase 2 complete (split unpin), entering phase 5");
          this->pin_transition_phase_ = 5;
          this->pin_transition_start_ = uptime;
        }
      }

      // Render combined trips as flat list with collapsing rows.
      // Only collapse trips that were on-screen. Fill-in skips newly-pinned.
      // visible_trips = [old pinned trips | unpinned trips]
      int split_phase2_base = pinned_si;
      int split_on_screen_end = std::min(split_phase2_base + this->limit_, (int)visible_trips.size());

      // Build display list: (trip index, should_collapse)
      std::vector<std::pair<int, bool>> split_phase2_entries;
      int collapsing_count = 0;
      for (int i = split_phase2_base; i < split_on_screen_end; i++) {
        std::string ck = visible_trips[i].composite_key();
        bool newly_pinned = (this->pin_transition_target_pinned_routes_.find(ck) != this->pin_transition_target_pinned_routes_.end()) &&
                            (this->pin_transition_old_pinned_routes_.find(ck) == this->pin_transition_old_pinned_routes_.end());
        bool newly_unpinned = (this->pin_transition_target_pinned_routes_.find(ck) == this->pin_transition_target_pinned_routes_.end()) &&
                              (this->pin_transition_old_pinned_routes_.find(ck) != this->pin_transition_old_pinned_routes_.end());
        bool collapsing = newly_pinned || newly_unpinned;
        split_phase2_entries.push_back({i, collapsing});
        if (collapsing) collapsing_count++;
      }
      // Fill-in from beyond the page, skipping newly-pinned/unpinned
      int fill_needed = collapsing_count;
      // First scan: from split_on_screen_end to end of list
      for (int i = split_on_screen_end; i < (int)visible_trips.size() && fill_needed > 0; i++) {
        std::string ck = visible_trips[i].composite_key();
        bool newly_pinned = (this->pin_transition_target_pinned_routes_.find(ck) != this->pin_transition_target_pinned_routes_.end()) &&
                            (this->pin_transition_old_pinned_routes_.find(ck) == this->pin_transition_old_pinned_routes_.end());
        bool newly_unpinned = (this->pin_transition_target_pinned_routes_.find(ck) == this->pin_transition_target_pinned_routes_.end()) &&
                              (this->pin_transition_old_pinned_routes_.find(ck) != this->pin_transition_old_pinned_routes_.end());
        if (!newly_pinned && !newly_unpinned) {
          split_phase2_entries.push_back({i, false});
          fill_needed--;
        }
      }
      // Wrap around: if still need fill, scan from 0 up to split_phase2_base
      for (int i = 0; i < split_phase2_base && fill_needed > 0; i++) {
        std::string ck = visible_trips[i].composite_key();
        bool newly_pinned = (this->pin_transition_target_pinned_routes_.find(ck) != this->pin_transition_target_pinned_routes_.end()) &&
                            (this->pin_transition_old_pinned_routes_.find(ck) == this->pin_transition_old_pinned_routes_.end());
        bool newly_unpinned = (this->pin_transition_target_pinned_routes_.find(ck) == this->pin_transition_target_pinned_routes_.end()) &&
                              (this->pin_transition_old_pinned_routes_.find(ck) != this->pin_transition_old_pinned_routes_.end());
        if (!newly_pinned && !newly_unpinned) {
          split_phase2_entries.push_back({i, false});
          fill_needed--;
        }
      }

      // Centering based on the original limit_ rows (stable baseline)
      int stable_count = this->limit_;
      int max_trips_height = (stable_count * this->font_->get_ascender()) + ((stable_count - 1) * this->font_->get_descender());
      int y_base_collapse = (this->display_->get_height() % max_trips_height) / 2;

      int fh = nominal_font_height;
      int dw = this->display_->get_width();
      int dh = this->display_->get_height();

      // Clip entire collapse animation to original display area
      this->display_->start_clipping(0, 0, dw, dh);
      float y_accum = (float)y_base_collapse;
      bool drew_divider = false;
      int entry_idx = 0;
      for (auto &entry : split_phase2_entries) {
        const Trip &trip = visible_trips[entry.first];
        std::string key = trip.composite_key();
        bool was_pinned = (this->pin_transition_old_pinned_routes_.find(key) != this->pin_transition_old_pinned_routes_.end());
        bool should_collapse = entry.second;

        float row_scale = 1.0f;
        if (should_collapse) {
          row_scale = 1.0f - collapse_t;
        }

        float row_height = (float)fh * row_scale;
        int y_int = (int)(y_accum + 0.5f);

        // Draw divider at old position if applicable
        if (!drew_divider && entry_idx >= pinned_rows && pinned_rows > 0) {
          int dy = (int)(y_accum - 1.0f + 0.5f);
          if (dy >= 0 && dy < dh) {
            this->display_->horizontal_line(0, dy, dw, this->divider_color_);
          }
          drew_divider = true;
        }

        if (row_scale > 0.05f && y_int < dh) {
          this->display_->start_clipping(0, y_int, dw, std::min(y_int + (int)(row_height + 0.5f), dh));
          this->draw_trip(trip, y_int, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, was_pinned);
          this->display_->end_clipping();
        }

        y_accum += row_height;
        entry_idx++;
      }
      this->display_->end_clipping();

      this->schedule_state_.mutex.unlock();
      return;
    }

    // Pin transition phase 3: push unpinned trips down, then sweep divider left-to-right
    if (this->pin_transition_phase_ == 3) {
      unsigned long phase3_elapsed = uptime - this->pin_transition_start_;
      // Sub-stage A: push-down over 0.5s
      // Sub-stage B: divider sweep over 0.5s (starts at t=500ms)
      float push_t = std::min(1.0f, (float)phase3_elapsed / 500.0f);
      push_t = push_t * push_t * (3.0f - 2.0f * push_t); // smoothstep

      bool push_done = (push_t >= 1.0f);
      float sweep_t = 0.0f;
      if (push_done) {
        sweep_t = std::min(1.0f, (float)(phase3_elapsed - 500) / 500.0f);
        sweep_t = sweep_t * sweep_t * (3.0f - 2.0f * sweep_t); // smoothstep
      }

      if (push_done && sweep_t >= 1.0f) { // push-down + divider sweep complete → enter phase 4
        ESP_LOGD(TAG, "Pin transition: phase 3 complete, entering phase 4 (slide-route-names)");
        this->pin_transition_phase_ = 4;
        this->pin_transition_start_ = uptime;
        // Don't return — let phase 3 render one more frame at final positions.
        // Phase 4 starts cleanly next frame.
      }

      int fh = nominal_font_height;
      int dw = this->display_->get_width();
      int dh = this->display_->get_height();
      int push_pixels = (int)((float)(pinned_rows * fh) * (1.0f - push_t) + 0.5f);

      // Pinned section: NOT rendered during phase 3 (saved for future phase)

      // Divider line: only after push completes, animate left-to-right sweep
      int divider_y_final = y_base - 1 + pinned_rows * fh - 1;
      if (push_done && divider_y_final >= 0 && divider_y_final < dh) {
        int sweep_width = (int)((float)dw * sweep_t + 0.5f);
        if (sweep_width > 0) {
          this->display_->horizontal_line(0, divider_y_final, sweep_width, this->divider_color_);
        }
      }

      // Unpinned section (shifts down from flat-list positions)
      // Draw limit_ rows worth of unpinned trips so that trips being pushed off-screen
      // remain visible until they exit, rather than disappearing immediately.
      int unpinned_y_start_anim = y_base + pinned_rows * fh - push_pixels;
      int unpinned_draw_count = std::min((int)unpinned_pool.size() - unpinned_si, this->limit_);
      // Clip: during push, clip from top of display; after push, clip below divider
      int clip_top = push_done ? std::max(0, divider_y_final + 1) : 0;
      this->display_->start_clipping(0, clip_top, dw, dh);
      for (int i = 0; i < unpinned_draw_count; i++) {
        int y = unpinned_y_start_anim + i * fh;
        if (y >= dh) break;  // no point drawing below display
        if (y + fh > 0) {
          this->draw_trip(unpinned_pool[unpinned_si + i], y, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, false);
        }
      }
      this->display_->end_clipping();

      this->schedule_state_.mutex.unlock();
      return;
    }

    // Pin transition phase 4: slide route names right, then scroll pinned rows in from bottom
    if (this->pin_transition_phase_ == 4) {
      unsigned long phase4_elapsed = uptime - this->pin_transition_start_;
      // Sub-stage A: horizontal slide of unpinned rows (0–500ms)
      // Sub-stage B: pinned rows scroll in from bottom of each row (500–1000ms)
      // Pause: 2s dwell then snap (1000–3000ms)
      float slide_t = std::min(1.0f, (float)phase4_elapsed / 500.0f); // 0.5s slide
      slide_t = slide_t * slide_t * (3.0f - 2.0f * slide_t); // smoothstep

      bool slide_done = (slide_t >= 1.0f);
      float reveal_t = 0.0f;
      if (slide_done) {
        reveal_t = std::min(1.0f, (float)(phase4_elapsed - 500) / 500.0f); // 0.5s reveal
        reveal_t = reveal_t * reveal_t * (3.0f - 2.0f * reveal_t); // smoothstep
      }
      bool reveal_done = (slide_done && reveal_t >= 1.0f);

      if (reveal_done && phase4_elapsed >= 3000) { // 0.5s slide + 0.5s reveal + 2s pause → snap
        ESP_LOGD(TAG, "Pin transition: phase 4 complete, snapping to new layout");
        this->pin_transition_phase_ = 0;
        this->frame_pin_offset_override_ = -1; // clear override so snap uses normal pin icons
        if (this->pin_transition_pending_restart_) {
          ESP_LOGD(TAG, "Pin transition: pending restart, transitioning from target to live");
          // Set old to the target we just finished transitioning to, so the detection
          // block correctly sees the difference between target and current live state
          this->pin_transition_old_eff_pinned_ = this->pin_transition_target_eff_pinned_;
          this->pin_transition_old_pinned_routes_ = this->pin_transition_target_pinned_routes_;
          this->pin_transition_old_hidden_routes_ = this->pin_transition_target_hidden_routes_;
          this->pin_transition_pending_restart_ = false;
        } else {
          this->pin_transition_old_eff_pinned_ = this->pin_transition_target_eff_pinned_;
          this->pin_transition_old_pinned_routes_ = this->pin_transition_target_pinned_routes_;
          this->pin_transition_old_hidden_routes_ = this->pin_transition_target_hidden_routes_;
        }
        // Reset split paging state for clean start
        this->pinned_page_index_ = 0;
        this->split_unpinned_page_index_ = 0;
        this->split_scroll_phase_ = 0;
        this->split_unpinned_page_timer_ = 0;
        this->pinned_page_timer_ = 0;
        this->pinned_h_scroll_start_ = 0;
        this->last_shared_scroll_dist_ = 0;
        // Clear data transition state so the split path doesn't detect a false
        // trip change (old keys are stale from before the pin transition started)
        this->data_transition_old_keys_.clear();
        this->data_transition_phase_ = 0;
        this->data_transition_departing_.clear();
        this->data_transition_pending_cycles_ = -1;
        this->pinned_dt_phase_ = 0;
        this->pinned_dt_old_trips_.clear();
        this->pinned_dt_pending_cycles_ = -1;
        // Don't return — let the phase 4 rendering run one more frame
        // so the display isn't blank. Normal layout picks up next frame.
      }

      int fh = nominal_font_height;
      int dw = this->display_->get_width();
      int dh = this->display_->get_height();

      // Compute pin offset for this frame
      int slide_offset = (int)(7.0f * slide_t + 0.5f);

      // Recompute uniform_clipping_start with sliding offset
      int phase4_uniform_start = uniform_clipping_start;
      int phase4_uniform_end = uniform_clipping_end;
      if (this->uniform_headsign_start_ && this->show_pin_icon_ && phase4_uniform_start >= 0) {
        phase4_uniform_start += slide_offset;
      }

      // Sub-stage A & B share: set pin offset override for unpinned rows during slide
      if (!slide_done) {
        this->frame_pin_offset_override_ = slide_offset;
      } else {
        // After slide, use full 7px offset for unpinned (no icon drawn)
        this->frame_pin_offset_override_ = 7;
      }

      // Pinned section: only rendered during sub-stage B (scroll in from bottom of each row)
      if (slide_done) {
        // Clear the override for pinned rows — they draw with real pin icon
        this->frame_pin_offset_override_ = -1;
        this->frame_show_pin_icons_ = true;

        // Recompute uniform clipping with full pin offset for pinned rows
        int pinned_uniform_start = uniform_clipping_start;
        if (this->uniform_headsign_start_ && this->show_pin_icon_ && pinned_uniform_start >= 0) {
          pinned_uniform_start += 7;
        }

        for (int i = pinned_si; i < pinned_ei; i++) {
          int row = i - pinned_si;
          int row_top = y_base - 1 + row * fh;
          int row_bot = row_top + fh;
          // Slide up from bottom of row: at reveal_t=0, trip is at row_bot; at reveal_t=1, at row_top
          int y_anim = row_top + (int)((float)fh * (1.0f - reveal_t) + 0.5f);
          this->display_->start_clipping(0, row_top, dw, row_bot);
          this->draw_trip(pinned_pool[i], y_anim, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          pinned_uniform_start, phase4_uniform_end, true);
          this->display_->end_clipping();
        }

        // Restore override for unpinned rows below
        this->frame_pin_offset_override_ = 7;
        this->frame_show_pin_icons_ = false;
      }

      // Divider
      int divider_y = y_base - 1 + pinned_rows * fh - 1;
      if (divider_y >= 0 && divider_y < dh) {
        this->display_->horizontal_line(0, divider_y, dw, this->divider_color_);
      }

      // Unpinned section (always rendered; slides right during sub-stage A)
      for (int i = unpinned_si; i < unpinned_ei; i++) {
        int row = i - unpinned_si;
        int y = y_base + pinned_rows * fh + row * fh;
        if (y >= dh) break;
        this->draw_trip(unpinned_pool[i], y, fh, uptime, rtc_now,
                        false, nullptr, nullptr, 0, 0,
                        phase4_uniform_start, phase4_uniform_end, false);
      }

      this->schedule_state_.mutex.unlock();
      return;
    }

    // Pin transition phase 5 (unpin): scroll pinned rows down out of their rows (reverse of phase 4B reveal)
    if (this->pin_transition_phase_ == 5) {
      unsigned long phase5_elapsed = uptime - this->pin_transition_start_;
      float hide_t = std::min(1.0f, (float)phase5_elapsed / 500.0f); // 0.5s
      hide_t = hide_t * hide_t * (3.0f - 2.0f * hide_t); // smoothstep

      if (hide_t >= 1.0f) {
        if (this->pin_transition_old_eff_pinned_ == this->pin_transition_target_eff_pinned_) {
          // Same-count swap: old pinned route is now hidden; skip phases 6-9 and go
          // directly to phase 10 to reveal the new pinned route. Set old_eff to 0 so
          // phase 10 treats ALL pinned rows as "new" and scrolls them in from bottom.
          ESP_LOGD(TAG, "Pin transition: phase 5 complete (swap), skipping to phase 10 (reveal new pinned)");
          // Cache on-screen unpinned trips for smooth swap animation
          this->pin_swap_old_unpinned_.clear();
          for (int i = unpinned_si; i < unpinned_ei; i++) {
            this->pin_swap_old_unpinned_.push_back(unpinned_pool[i]);
          }
          this->pin_transition_old_eff_pinned_ = 0;
          this->pin_transition_phase_ = 10;
          this->pin_transition_start_ = uptime;
        } else {
          ESP_LOGD(TAG, "Pin transition: phase 5 complete, entering phase 6 (slide-left)");
          this->pin_transition_phase_ = 6;
          this->pin_transition_start_ = uptime;
        }
      }

      int fh = nominal_font_height;
      int dw = this->display_->get_width();
      int dh = this->display_->get_height();

      // Pinned rows scroll down (out of view within their row bounds)
      for (int i = pinned_si; i < pinned_ei; i++) {
        int row = i - pinned_si;
        int row_top = y_base - 1 + row * fh;
        int row_bot = row_top + fh;
        // At hide_t=0: trip at row_top (normal); at hide_t=1: trip at row_bot (hidden)
        int y_anim = row_top + (int)((float)fh * hide_t + 0.5f);
        this->display_->start_clipping(0, row_top, dw, row_bot);
        this->draw_trip(pinned_pool[i], y_anim, fh, uptime, rtc_now,
                        false, nullptr, nullptr, 0, 0,
                        uniform_clipping_start, uniform_clipping_end, true);
        this->display_->end_clipping();
      }

      // Divider (static)
      int divider_y = y_base - 1 + pinned_rows * fh - 1;
      if (divider_y >= 0 && divider_y < dh) {
        this->display_->horizontal_line(0, divider_y, dw, this->divider_color_);
      }

      // Unpinned section (static)
      for (int i = unpinned_si; i < unpinned_ei; i++) {
        int row = i - unpinned_si;
        int y = y_base + pinned_rows * fh + row * fh;
        if (y >= dh) break;
        this->draw_trip(unpinned_pool[i], y, fh, uptime, rtc_now,
                        false, nullptr, nullptr, 0, 0,
                        uniform_clipping_start, uniform_clipping_end, false);
      }

      this->schedule_state_.mutex.unlock();
      return;
    }

    // Pin transition phase 6 (unpin): slide route names left, removing pin inset (reverse of phase 4A)
    if (this->pin_transition_phase_ == 6) {
      unsigned long phase6_elapsed = uptime - this->pin_transition_start_;
      float slide_t = std::min(1.0f, (float)phase6_elapsed / 500.0f); // 0.5s
      slide_t = slide_t * slide_t * (3.0f - 2.0f * slide_t); // smoothstep

      if (slide_t >= 1.0f) {
        ESP_LOGD(TAG, "Pin transition: phase 6 complete, entering phase 7 (sweep-out)");
        this->pin_transition_phase_ = 7;
        this->pin_transition_start_ = uptime;
      }

      int fh = nominal_font_height;
      int dw = this->display_->get_width();
      int dh = this->display_->get_height();

      // Slide offset goes from 7 → 0
      int slide_offset = (int)(7.0f * (1.0f - slide_t) + 0.5f);
      this->frame_pin_offset_override_ = slide_offset;

      // Recompute uniform_clipping with sliding offset
      // uniform_clipping_start already has +7 baked in (from frame_show_pin_icons_ during pre-pass);
      // replace that with the current slide_offset so the headsign slides left with the route name.
      int phase6_uniform_start = uniform_clipping_start;
      if (this->uniform_headsign_start_ && this->show_pin_icon_ && phase6_uniform_start >= 0) {
        phase6_uniform_start = phase6_uniform_start - 7 + slide_offset;
      }

      // Pinned area is empty (rows hidden in phase 5)

      // Divider (static)
      int divider_y = y_base - 1 + pinned_rows * fh - 1;
      if (divider_y >= 0 && divider_y < dh) {
        this->display_->horizontal_line(0, divider_y, dw, this->divider_color_);
      }

      // Unpinned section (slides left)
      for (int i = unpinned_si; i < unpinned_ei; i++) {
        int row = i - unpinned_si;
        int y = y_base + pinned_rows * fh + row * fh;
        if (y >= dh) break;
        this->draw_trip(unpinned_pool[i], y, fh, uptime, rtc_now,
                        false, nullptr, nullptr, 0, 0,
                        phase6_uniform_start, uniform_clipping_end, false);
      }

      this->schedule_state_.mutex.unlock();
      return;
    }

    // Pin transition phase 7 (unpin): sweep divider out right-to-left (reverse of phase 3B)
    if (this->pin_transition_phase_ == 7) {
      unsigned long phase7_elapsed = uptime - this->pin_transition_start_;
      float sweep_t = std::min(1.0f, (float)phase7_elapsed / 500.0f); // 0.5s
      sweep_t = sweep_t * sweep_t * (3.0f - 2.0f * sweep_t); // smoothstep

      if (sweep_t >= 1.0f) {
        ESP_LOGD(TAG, "Pin transition: phase 7 complete, entering phase 8 (scroll-up)");
        this->pin_transition_phase_ = 8;
        this->pin_transition_start_ = uptime;
        // Don't return — render this frame then phase 8 starts with NEW partition next frame.
      }

      int fh = nominal_font_height;
      int dw = this->display_->get_width();
      int dh = this->display_->get_height();

      // Divider sweeps out: at sweep_t=0 full width, at sweep_t=1 gone
      int divider_y = y_base - 1 + pinned_rows * fh - 1;
      if (divider_y >= 0 && divider_y < dh) {
        int sweep_width = (int)((float)dw * (1.0f - sweep_t) + 0.5f);
        if (sweep_width > 0) {
          this->display_->horizontal_line(0, divider_y, sweep_width, this->divider_color_);
        }
      }

      // Unpinned section (static, no pin offset)
      this->frame_pin_offset_override_ = 0;
      for (int i = unpinned_si; i < unpinned_ei; i++) {
        int row = i - unpinned_si;
        int y = y_base + pinned_rows * fh + row * fh;
        if (y >= dh) break;
        this->draw_trip(unpinned_pool[i], y, fh, uptime, rtc_now,
                        false, nullptr, nullptr, 0, 0,
                        uniform_clipping_start, uniform_clipping_end, false);
      }

      this->schedule_state_.mutex.unlock();
      return;
    }

    // Pin transition phase 8 (unpin): scroll unpinned trips up to flat positions
    if (this->pin_transition_phase_ == 8) {
      unsigned long phase8_elapsed = uptime - this->pin_transition_start_;
      float scroll_t = std::min(1.0f, (float)phase8_elapsed / 500.0f); // 0.5s
      scroll_t = scroll_t * scroll_t * (3.0f - 2.0f * scroll_t); // smoothstep

      int fh = nominal_font_height;
      int dh = this->display_->get_height();

      if (scroll_t >= 1.0f) {
        ESP_LOGD(TAG, "Pin transition: phase 8 complete, snapping to flat layout");
        this->pin_transition_phase_ = 0;
        if (this->pin_transition_pending_restart_) {
          ESP_LOGD(TAG, "Pin transition: pending restart, transitioning from target to live");
          // Set old to the target we just finished transitioning to, so the detection
          // block correctly sees the difference between target and current live state
          this->pin_transition_old_eff_pinned_ = this->pin_transition_target_eff_pinned_;
          this->pin_transition_old_pinned_routes_ = this->pin_transition_target_pinned_routes_;
          this->pin_transition_old_hidden_routes_ = this->pin_transition_target_hidden_routes_;
          this->pin_transition_pending_restart_ = false;
        } else {
          this->pin_transition_old_eff_pinned_ = this->pin_transition_target_eff_pinned_;
          this->pin_transition_old_pinned_routes_ = this->pin_transition_target_pinned_routes_;
          this->pin_transition_old_hidden_routes_ = this->pin_transition_target_hidden_routes_;
        }
        // Reset both split and flat paging state for clean start
        this->pinned_page_index_ = 0;
        this->split_unpinned_page_index_ = 0;
        this->split_scroll_phase_ = 0;
        this->split_unpinned_page_timer_ = 0;
        this->pinned_page_timer_ = 0;
        this->pinned_h_scroll_start_ = 0;
        this->last_shared_scroll_dist_ = 0;
        this->current_page_index_ = 0;
        this->last_page_index_ = 0;
        this->page_timer_start_ = 0;
        this->h_scroll_start_time_ = 0;
        this->last_scroll_distance_ = 0;
        this->page_scroll_start_ = 0;
        this->page_pause_start_ = 0;
        // Clear data transition state so the flat path doesn't detect a false
        // trip change (old keys are stale from before the pin transition started)
        this->data_transition_old_keys_.clear();
        this->data_transition_phase_ = 0;
        this->data_transition_departing_.clear();
        this->data_transition_pending_cycles_ = -1;
        this->pinned_dt_phase_ = 0;
        this->pinned_dt_old_trips_.clear();
        this->pinned_dt_pending_cycles_ = -1;
        this->pin_swap_old_unpinned_.clear();
        ESP_LOGV(TAG, "Pin snap (flat): visible=%d, showing [0..%d)",
                 (int)visible_trips.size(), std::min((int)visible_trips.size(), this->limit_));
        for (int i = 0; i < std::min((int)visible_trips.size(), this->limit_); i++) {
          ESP_LOGV(TAG, "  [%d] %s", i, visible_trips[i].composite_key().c_str());
        }
        // Don't return — render one more frame at final position
      }

      // Build flat-order trip list (undo the partition to get original sort order)
      std::vector<Trip> flat_trips;
      if (!this->hidden_routes_.empty()) {
        for (const Trip &trip : this->schedule_state_.trips) {
          if (this->hidden_routes_.find(trip.composite_key()) == this->hidden_routes_.end()) {
            flat_trips.push_back(trip);
          }
        }
      } else {
        flat_trips = this->schedule_state_.trips;
      }
      int flat_count = std::min((int)flat_trips.size(), this->limit_);

      // Build lookup: unpinned trip key → display row in old split layout
      std::map<std::string, int> unpinned_row_map;
      int unpinned_vis_count = std::min((int)unpinned_pool.size() - unpinned_si, unpinned_rows);
      for (int i = 0; i < unpinned_vis_count; i++) {
        unpinned_row_map[unpinned_pool[unpinned_si + i].composite_key()] = i;
      }

      // Draw ALL trips in flat order, interpolating from old positions to flat positions.
      // Unpinned trips: old_y = split position (y_base + pinned_rows*fh + row*fh)
      // Formerly-pinned trips: old_y = just above visible area (slide in from top)
      for (int fi = 0; fi < flat_count; fi++) {
        const Trip &trip = flat_trips[fi];
        int target_y = y_base + fi * fh;
        int old_y;

        auto it = unpinned_row_map.find(trip.composite_key());
        if (it != unpinned_row_map.end()) {
          old_y = y_base + pinned_rows * fh + it->second * fh;
        } else {
          // Formerly pinned: start just below the last visible unpinned row
          // so it enters from the bottom like a new row, not from above
          old_y = y_base + pinned_rows * fh + unpinned_vis_count * fh;
        }

        int y = old_y + (int)((float)(target_y - old_y) * scroll_t + 0.5f);
        if (y >= dh) continue;
        if (y + fh > 0) {
          this->draw_trip(trip, y, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, false);
        }
      }

      this->schedule_state_.mutex.unlock();
      return;
    }

    // Pin transition phase 9 (expand pins): push down barrier + unpinned content
    if (this->pin_transition_phase_ == 9) {
      unsigned long phase9_elapsed = uptime - this->pin_transition_start_;
      float push_t = std::min(1.0f, (float)phase9_elapsed / 500.0f);
      push_t = push_t * push_t * (3.0f - 2.0f * push_t); // smoothstep

      if (push_t >= 1.0f) {
        ESP_LOGD(TAG, "Pin transition: phase 9 complete, entering phase 10 (reveal expanded pins)");
        this->pin_transition_phase_ = 10;
        this->pin_transition_start_ = uptime;
        // Don't return — render one more frame at pushed position.
        // Note: do NOT populate pin_swap_old_unpinned_ here — the expand push-down
        // already handles the visual transition, and phase 11's scroll covers the
        // content change. Caching would cause a double-scroll effect.
      }

      int fh = nominal_font_height;
      int dw = this->display_->get_width();
      int dh = this->display_->get_height();
      int old_pinned_rows = effective_pinned_count; // using OLD partition
      int new_pinned_rows = this->pin_transition_target_eff_pinned_;
      int delta_rows = new_pinned_rows - old_pinned_rows;
      int push_pixels = (int)((float)(delta_rows * fh) * push_t + 0.5f);

      // Old pinned rows: static if staying, scroll-out if being replaced
      int old_divider_y = y_base - 1 + old_pinned_rows * fh - 1;
      for (int i = pinned_si; i < pinned_ei; i++) {
        int row = i - pinned_si;
        int row_top = y_base - 1 + row * fh;
        int row_bot = row_top + fh;
        bool staying = this->pin_transition_target_pinned_routes_.find(pinned_pool[i].composite_key())
                       != this->pin_transition_target_pinned_routes_.end();
        this->display_->start_clipping(0, row_top, dw, std::min(row_bot, old_divider_y));
        if (staying) {
          this->draw_trip(pinned_pool[i], row_top, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, true);
        } else {
          // Scroll down out of row (like phase 5 hide)
          int y_anim = row_top + (int)((float)fh * push_t + 0.5f);
          this->draw_trip(pinned_pool[i], y_anim, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, true);
        }
        this->display_->end_clipping();
      }

      // Divider: animates from old position to new position
      int divider_y_anim = old_divider_y + push_pixels;
      if (divider_y_anim >= 0 && divider_y_anim < dh) {
        this->display_->horizontal_line(0, divider_y_anim, dw, this->divider_color_);
      }

      // Unpinned section: pushes down with divider, clipped to below animated divider
      int old_unpinned_y_start = y_base + old_pinned_rows * fh;
      int animated_unpinned_y_start = old_unpinned_y_start + push_pixels;
      this->display_->start_clipping(0, divider_y_anim + 1, dw, dh);
      for (int i = unpinned_si; i < unpinned_ei; i++) {
        int row = i - unpinned_si;
        int y = animated_unpinned_y_start + row * fh;
        if (y >= dh) break;
        if (y + fh > 0) {
          this->draw_trip(unpinned_pool[i], y, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, false);
        }
      }
      this->display_->end_clipping();

      this->schedule_state_.mutex.unlock();
      return;
    }

    // Pin transition phase 10 (expand pins): reveal new pinned rows from bottom
    if (this->pin_transition_phase_ == 10) {
      unsigned long phase10_elapsed = uptime - this->pin_transition_start_;
      float reveal_t = std::min(1.0f, (float)phase10_elapsed / 500.0f);
      reveal_t = reveal_t * reveal_t * (3.0f - 2.0f * reveal_t); // smoothstep

      if (reveal_t >= 1.0f) {
        ESP_LOGD(TAG, "Pin transition: phase 10 complete, entering phase 11 (dwell + unpinned scroll)");
        this->pin_swap_old_unpinned_.clear(); // swap animation complete
        this->pin_transition_phase_ = 11;
        this->pin_transition_start_ = uptime;
        // Don't return — render one more frame at fully revealed position.
      }

      int fh = nominal_font_height;
      int dw = this->display_->get_width();
      int dh = this->display_->get_height();
      int new_pinned_rows = effective_pinned_count; // using TARGET partition now
      int old_pinned_rows = this->pin_transition_old_eff_pinned_;

      // Reset pinned page to show from start during expansion
      int ph10_pinned_si = 0;
      int ph10_pinned_ei = std::min(new_pinned_rows, (int)pinned_pool.size());

      // Old pinned rows: static if same route, scroll-in if replaced
      for (int i = ph10_pinned_si; i < std::min(ph10_pinned_si + old_pinned_rows, ph10_pinned_ei); i++) {
        int row = i - ph10_pinned_si;
        int row_top = y_base - 1 + row * fh;
        bool was_old_pinned = this->pin_transition_old_pinned_routes_.find(pinned_pool[i].composite_key())
                              != this->pin_transition_old_pinned_routes_.end();
        if (was_old_pinned) {
          // Truly old pinned trip — render static
          this->draw_trip(pinned_pool[i], row_top, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, true);
        } else {
          // New route replacing old one — animate scroll-in from bottom
          int row_bot = row_top + fh;
          int y_anim = row_top + (int)((float)fh * (1.0f - reveal_t) + 0.5f);
          this->display_->start_clipping(0, row_top, dw, row_bot);
          this->draw_trip(pinned_pool[i], y_anim, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, true);
          this->display_->end_clipping();
        }
      }

      // New pinned rows: scroll in from bottom of each row
      for (int i = ph10_pinned_si + old_pinned_rows; i < ph10_pinned_ei; i++) {
        int row = i - ph10_pinned_si;
        int row_top = y_base - 1 + row * fh;
        int row_bot = row_top + fh;
        // At reveal_t=0: trip at row_bot (hidden); at reveal_t=1: trip at row_top (visible)
        int y_anim = row_top + (int)((float)fh * (1.0f - reveal_t) + 0.5f);
        this->display_->start_clipping(0, row_top, dw, row_bot);
        this->draw_trip(pinned_pool[i], y_anim, fh, uptime, rtc_now,
                        false, nullptr, nullptr, 0, 0,
                        uniform_clipping_start, uniform_clipping_end, true);
        this->display_->end_clipping();
      }

      // Divider: static at new position
      int divider_y = y_base - 1 + new_pinned_rows * fh - 1;
      if (divider_y >= 0 && divider_y < dh) {
        this->display_->horizontal_line(0, divider_y, dw, this->divider_color_);
      }

      // Unpinned section
      int new_unpinned_rows = this->limit_ - new_pinned_rows;
      int unpinned_y_start_ph10 = y_base + new_pinned_rows * fh;

      if (!this->pin_swap_old_unpinned_.empty()) {
        // Pin swap: animate unpinned section transition in sync with pinned reveal
        // Newly-pinned trips collapse out, newly-unpinned trips expand in
        std::set<std::string> newly_pinned_keys;
        for (const auto &key : this->pin_transition_target_pinned_routes_) {
          if (this->pin_transition_old_pinned_routes_.find(key) == this->pin_transition_old_pinned_routes_.end()) {
            newly_pinned_keys.insert(key);
          }
        }
        struct SwapEntry { const Trip *trip; float scale; };
        std::vector<SwapEntry> swap_entries;
        // Old unpinned trips: collapsing if newly pinned, static otherwise
        for (const auto &trip : this->pin_swap_old_unpinned_) {
          float s = newly_pinned_keys.count(trip.composite_key()) ? (1.0f - reveal_t) : 1.0f;
          swap_entries.push_back({&trip, s});
        }
        // New unpinned trips not in old set: expanding
        std::set<std::string> old_unp_keys;
        for (const auto &trip : this->pin_swap_old_unpinned_)
          old_unp_keys.insert(trip.composite_key());
        int ph10_unpinned_ei = std::min(new_unpinned_rows, (int)unpinned_pool.size());
        for (int i = 0; i < ph10_unpinned_ei; i++) {
          if (old_unp_keys.find(unpinned_pool[i].composite_key()) == old_unp_keys.end()) {
            swap_entries.push_back({&unpinned_pool[i], reveal_t});
          }
        }
        // Render with clipping
        this->display_->start_clipping(0, divider_y + 1, dw, dh);
        float y_accum = (float)unpinned_y_start_ph10;
        for (auto &entry : swap_entries) {
          float row_height = (float)fh * entry.scale;
          int y_int = (int)(y_accum + 0.5f);
          if (entry.scale > 0.05f && y_int < dh) {
            this->display_->start_clipping(0, std::max(y_int, divider_y + 1), dw, std::min(y_int + (int)(row_height + 0.5f), dh));
            this->draw_trip(*entry.trip, y_int, fh, uptime, rtc_now,
                            false, nullptr, nullptr, 0, 0,
                            uniform_clipping_start, uniform_clipping_end, false);
            this->display_->end_clipping();
          }
          y_accum += row_height;
        }
        this->display_->end_clipping();
      } else {
        // Normal: static at new position using target partition
        int ph10_unpinned_si = 0;
        int ph10_unpinned_ei = std::min(new_unpinned_rows, (int)unpinned_pool.size());
        for (int i = ph10_unpinned_si; i < ph10_unpinned_ei; i++) {
          int row = i - ph10_unpinned_si;
          int y = unpinned_y_start_ph10 + row * fh;
          if (y >= dh) break;
          this->draw_trip(unpinned_pool[i], y, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, false);
        }
      }

      this->schedule_state_.mutex.unlock();
      return;
    }

    // Pin transition phase 11 (expand pins): dwell for page_interval, then scroll
    // unpinned rows and resume horizontal scrolling.
    if (this->pin_transition_phase_ == 11) {
      unsigned long phase11_elapsed = uptime - this->pin_transition_start_;

      int fh = nominal_font_height;
      int dw = this->display_->get_width();
      int dh = this->display_->get_height();
      int new_pinned_rows = effective_pinned_count; // using TARGET partition
      int new_unpinned_rows = this->limit_ - new_pinned_rows;
      int unpinned_pool_size_11 = (int)unpinned_pool.size();
      bool needs_unpinned_scroll = (unpinned_pool_size_11 > new_unpinned_rows && this->scroll_routes_);

      // Sub-stages:
      //   A) 0..page_pause_duration_: dwell (show static new layout, no h-scroll)
      //   B) page_pause_duration_..+page_scroll_duration_: scroll unpinned section (if needed)
      //   C) +page_scroll_duration_..+page_pause_duration_: post-scroll pause
      //   D) complete → snap to phase 0

      unsigned long dwell_end = (unsigned long)this->page_pause_duration_;
      unsigned long scroll_end = dwell_end + (needs_unpinned_scroll ? (unsigned long)this->page_scroll_duration_ : 0);
      unsigned long pause_end = scroll_end + (needs_unpinned_scroll ? (unsigned long)this->page_pause_duration_ : 0);

      bool phase11_complete = false;
      float unpinned_scroll_t = 0.0f;
      bool in_unpinned_scroll = false;
      int ph11_unpinned_si = 0;
      int ph11_unpinned_ei = std::min(new_unpinned_rows, unpinned_pool_size_11);

      if (phase11_elapsed >= pause_end) {
        // Complete — snap to new layout
        phase11_complete = true;
        // Update indices so the final "don't return" frame shows the scrolled position
        if (needs_unpinned_scroll) {
          int unpinned_pages = std::max(1, unpinned_pool_size_11 - new_unpinned_rows + 1);
          ph11_unpinned_si = 1 % unpinned_pages;
          ph11_unpinned_ei = std::min(ph11_unpinned_si + new_unpinned_rows, unpinned_pool_size_11);
        }
      } else if (needs_unpinned_scroll && phase11_elapsed >= dwell_end && phase11_elapsed < scroll_end) {
        // Sub-stage B: scrolling unpinned section
        unsigned long scroll_elapsed = phase11_elapsed - dwell_end;
        unpinned_scroll_t = std::min(1.0f, (float)scroll_elapsed / (float)this->page_scroll_duration_);
        unpinned_scroll_t = unpinned_scroll_t * unpinned_scroll_t * (3.0f - 2.0f * unpinned_scroll_t);
        in_unpinned_scroll = true;
        // Advance to next page for scroll animation
        int unpinned_pages = std::max(1, unpinned_pool_size_11 - new_unpinned_rows + 1);
        ph11_unpinned_si = 1 % unpinned_pages;  // scroll by one row
        ph11_unpinned_ei = std::min(ph11_unpinned_si + new_unpinned_rows, unpinned_pool_size_11);
      } else if (needs_unpinned_scroll && phase11_elapsed >= scroll_end) {
        // Sub-stage C: post-scroll pause — show scrolled position
        int unpinned_pages = std::max(1, unpinned_pool_size_11 - new_unpinned_rows + 1);
        ph11_unpinned_si = 1 % unpinned_pages;
        ph11_unpinned_ei = std::min(ph11_unpinned_si + new_unpinned_rows, unpinned_pool_size_11);
      }
      // Sub-stage A (dwell): ph11_unpinned_si/ei remain at 0..new_unpinned_rows

      if (phase11_complete) {
        ESP_LOGD(TAG, "Pin transition: phase 11 complete, snapping to expanded split layout");
        this->pin_transition_phase_ = 0;
        if (this->pin_transition_pending_restart_) {
          ESP_LOGD(TAG, "Pin transition: pending restart after expand");
          this->pin_transition_old_eff_pinned_ = this->pin_transition_target_eff_pinned_;
          this->pin_transition_old_pinned_routes_ = this->pin_transition_target_pinned_routes_;
          this->pin_transition_old_hidden_routes_ = this->pin_transition_target_hidden_routes_;
          this->pin_transition_pending_restart_ = false;
        } else {
          this->pin_transition_old_eff_pinned_ = this->pin_transition_target_eff_pinned_;
          this->pin_transition_old_pinned_routes_ = this->pin_transition_target_pinned_routes_;
          this->pin_transition_old_hidden_routes_ = this->pin_transition_target_hidden_routes_;
        }
        // Reset split paging state for clean start
        this->pinned_page_index_ = 0;
        this->split_unpinned_page_index_ = needs_unpinned_scroll ? (1 % std::max(1, unpinned_pool_size_11 - new_unpinned_rows + 1)) : 0;
        this->split_scroll_phase_ = 0;
        this->split_unpinned_page_timer_ = 0;
        this->pinned_page_timer_ = uptime;
        this->pinned_h_scroll_start_ = uptime;
        this->last_shared_scroll_dist_ = 0;
        this->pin_swap_old_unpinned_.clear();
        // Clear data transition state so the split path doesn't detect a false
        // trip change (old keys are stale from before the pin transition started)
        this->data_transition_old_keys_.clear();
        this->data_transition_phase_ = 0;
        this->data_transition_departing_.clear();
        this->data_transition_pending_cycles_ = -1;
        this->pinned_dt_phase_ = 0;
        this->pinned_dt_old_trips_.clear();
        this->pinned_dt_pending_cycles_ = -1;
        // Don't return — render one final frame, then normal layout picks up.
      }

      // Draw pinned rows
      int ph11_pinned_si = 0;
      int ph11_pinned_ei = std::min(new_pinned_rows, (int)pinned_pool.size());
      for (int i = ph11_pinned_si; i < ph11_pinned_ei; i++) {
        int row = i - ph11_pinned_si;
        int y_row = y_base - 1 + row * fh;
        this->draw_trip(pinned_pool[i], y_row, fh, uptime, rtc_now,
                        false, nullptr, nullptr, 0, 0,
                        uniform_clipping_start, uniform_clipping_end, true);
      }

      // Divider
      int divider_y = y_base - 1 + new_pinned_rows * fh - 1;
      if (divider_y >= 0 && divider_y < dh) {
        this->display_->horizontal_line(0, divider_y, dw, this->divider_color_);
      }

      // Unpinned section
      int unpinned_y_start_ph11 = y_base + new_pinned_rows * fh;
      this->display_->start_clipping(0, divider_y + 1, dw, dh);
      if (in_unpinned_scroll) {
        // Animate: scroll from old page (si=0) to new page (si=ph11_unpinned_si)
        int pixel_shift = (int)(unpinned_scroll_t * fh);

        // Draw old page (scrolling up/out)
        int old_ei = std::min(new_unpinned_rows, unpinned_pool_size_11);
        for (int i = 0; i < old_ei; i++) {
          int y = unpinned_y_start_ph11 + i * fh - pixel_shift;
          if (y + fh > divider_y && y < dh) {
            this->draw_trip(unpinned_pool[i], y, fh, uptime, rtc_now,
                            false, nullptr, nullptr, 0, 0,
                            uniform_clipping_start, uniform_clipping_end, false);
          }
        }
        // Draw incoming row (scrolling in from bottom)
        if (ph11_unpinned_ei > old_ei - 1) {
          int new_trip_idx = ph11_unpinned_ei - 1;
          if (new_trip_idx >= 0 && new_trip_idx < unpinned_pool_size_11) {
            int y = unpinned_y_start_ph11 + new_unpinned_rows * fh - pixel_shift;
            if (y + fh > divider_y && y < dh) {
              this->draw_trip(unpinned_pool[new_trip_idx], y, fh, uptime, rtc_now,
                              false, nullptr, nullptr, 0, 0,
                              uniform_clipping_start, uniform_clipping_end, false);
            }
          }
        }
      } else {
        // Static: draw current page
        for (int i = ph11_unpinned_si; i < ph11_unpinned_ei; i++) {
          int row = i - ph11_unpinned_si;
          int y = unpinned_y_start_ph11 + row * fh;
          if (y >= dh) break;
          this->draw_trip(unpinned_pool[i], y, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, false);
        }
      }
      this->display_->end_clipping();

      this->schedule_state_.mutex.unlock();
      return;
    }

    // Pin transition phase 12 (shrink pins): compress extra pinned rows, raise barrier,
    // and simultaneously scroll bottom container (old free rows out, new free rows in).
    if (this->pin_transition_phase_ == 12) {
      unsigned long phase12_elapsed = uptime - this->pin_transition_start_;
      unsigned long pre_pause = (unsigned long)this->page_pause_duration_;

      // Sub-stages: [0..pre_pause) = pre-pause, [pre_pause..pre_pause+500) = collapse anim
      bool in_pre_pause = (phase12_elapsed < pre_pause);
      unsigned long anim_elapsed = in_pre_pause ? 0 : (phase12_elapsed - pre_pause);
      float anim_t = in_pre_pause ? 0.0f : std::min(1.0f, (float)anim_elapsed / 500.0f);
      anim_t = anim_t * anim_t * (3.0f - 2.0f * anim_t); // smoothstep

      bool anim_done = (!in_pre_pause && anim_t >= 1.0f);

      int fh = nominal_font_height;
      int dw = this->display_->get_width();
      int dh = this->display_->get_height();

      int old_pinned_rows = this->pin_transition_old_eff_pinned_;
      int new_pinned_rows = this->pin_transition_target_eff_pinned_;
      int old_unpinned_rows = this->limit_ - old_pinned_rows;
      int new_unpinned_rows = this->limit_ - new_pinned_rows;
      int delta_rows = old_pinned_rows - new_pinned_rows; // > 0 since pin count dropped

      // Compute target unpinned pool (re-partition visible_trips with target routes)
      std::vector<Trip> target_unpinned;
      {
        std::vector<Trip> target_visible;
        for (const Trip &trip : this->schedule_state_.trips) {
          if (this->pin_transition_target_hidden_routes_.find(trip.composite_key()) == this->pin_transition_target_hidden_routes_.end()) {
            target_visible.push_back(trip);
          }
        }
        if (!this->pin_transition_target_pinned_routes_.empty()) {
          std::stable_partition(target_visible.begin(), target_visible.end(), [this](const Trip &trip) {
            return this->pin_transition_target_pinned_routes_.find(trip.composite_key()) != this->pin_transition_target_pinned_routes_.end();
          });
          int tpc = 0;
          for (const auto &trip : target_visible) {
            if (this->pin_transition_target_pinned_routes_.find(trip.composite_key()) != this->pin_transition_target_pinned_routes_.end()) {
              tpc++;
            } else break;
          }
          // Deduplicate pinned section
          {
            std::set<std::string> seen_keys;
            int write = 0;
            for (int i = 0; i < tpc; i++) {
              std::string key = target_visible[i].composite_key();
              if (seen_keys.find(key) == seen_keys.end()) {
                seen_keys.insert(key);
                if (write != i) target_visible[write] = target_visible[i];
                write++;
              }
            }
            if (write < tpc) {
              target_visible.erase(target_visible.begin() + write, target_visible.begin() + tpc);
              tpc = write;
            }
          }
          for (int i = tpc; i < (int)target_visible.size(); i++) {
            target_unpinned.push_back(target_visible[i]);
          }
        } else {
          target_unpinned = target_visible;
        }
      }

      // Completion check (pre-pause + animation done + post-dwell)
      if (anim_done && phase12_elapsed >= (unsigned long)(pre_pause + 500 + this->page_pause_duration_)) {
        ESP_LOGD(TAG, "Pin transition: phase 12 complete, snapping to shrunk split layout");
        this->pin_transition_phase_ = 0;
        if (this->pin_transition_pending_restart_) {
          ESP_LOGD(TAG, "Pin transition: pending restart after shrink");
          this->pin_transition_old_eff_pinned_ = this->pin_transition_target_eff_pinned_;
          this->pin_transition_old_pinned_routes_ = this->pin_transition_target_pinned_routes_;
          this->pin_transition_old_hidden_routes_ = this->pin_transition_target_hidden_routes_;
          this->pin_transition_pending_restart_ = false;
        } else {
          this->pin_transition_old_eff_pinned_ = this->pin_transition_target_eff_pinned_;
          this->pin_transition_old_pinned_routes_ = this->pin_transition_target_pinned_routes_;
          this->pin_transition_old_hidden_routes_ = this->pin_transition_target_hidden_routes_;
        }
        // Reset split paging state for clean start
        this->pinned_page_index_ = 0;
        this->split_unpinned_page_index_ = 0;
        this->split_scroll_phase_ = 0;
        this->split_unpinned_page_timer_ = 0;
        this->pinned_page_timer_ = uptime;
        this->pinned_h_scroll_start_ = uptime;
        this->last_shared_scroll_dist_ = 0;
        this->pin_swap_old_unpinned_.clear();
        // Clear data transition state so the split path doesn't detect a false
        // trip change (old keys are stale from before the pin transition started)
        this->data_transition_old_keys_.clear();
        this->data_transition_phase_ = 0;
        this->data_transition_departing_.clear();
        this->data_transition_pending_cycles_ = -1;
        this->pinned_dt_phase_ = 0;
        this->pinned_dt_old_trips_.clear();
        this->pinned_dt_pending_cycles_ = -1;
        // Don't return — render one final frame at destination, then normal picks up.
      }

      // --- Pinned section: rows 0..new_pinned_rows-1 static, rows new_pinned_rows..old_pinned_rows-1 collapsing ---
      float y_accum_pin = (float)(y_base - 1);
      for (int i = pinned_si; i < std::min(pinned_si + old_pinned_rows, (int)pinned_pool.size()); i++) {
        int row = i - pinned_si;
        bool collapsing = (row >= new_pinned_rows);
        float row_scale = collapsing ? (1.0f - anim_t) : 1.0f;
        float row_height = (float)fh * row_scale;
        int y_int = (int)(y_accum_pin + 0.5f);

        if (row_scale > 0.05f && y_int < dh) {
          this->display_->start_clipping(0, y_int, dw, std::min(y_int + (int)(row_height + 0.5f), dh));
          this->draw_trip(pinned_pool[i], y_int, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, true);
          this->display_->end_clipping();
        }
        y_accum_pin += row_height;
      }

      // --- Animated divider: tracks bottom of compressed pinned section ---
      int divider_y_anim = (int)(y_accum_pin + 0.5f) - 1;
      if (divider_y_anim >= 0 && divider_y_anim < dh) {
        this->display_->horizontal_line(0, divider_y_anim, dw, this->divider_color_);
      }

      // --- Unpinned section: scroll old content out, new content in ---
      // Content anchored at old unpinned start, scrolled up by new_free rows total.
      // old_unpinned_start = y_base + old_pinned_rows * fh (fixed reference point)
      // pixel_shift = anim_t * new_unpinned_rows * fh
      // The viewport clips to below the animated divider, so as the divider rises,
      // rows naturally become visible in the expanding viewport.
      int old_unpinned_start = y_base + old_pinned_rows * fh;
      int pixel_shift_px = (int)(anim_t * (float)(new_unpinned_rows * fh) + 0.5f);

      this->display_->start_clipping(0, divider_y_anim + 1, dw, dh);

      // Draw old unpinned rows (scrolling up/out)
      for (int i = unpinned_si; i < unpinned_ei; i++) {
        int row = i - unpinned_si;
        int y = old_unpinned_start + row * fh - pixel_shift_px;
        if (y + fh > divider_y_anim && y < dh) {
          this->draw_trip(unpinned_pool[i], y, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, false);
        }
      }

      // Draw new unpinned rows (scrolling in from below)
      int new_content_base = old_unpinned_start + old_unpinned_rows * fh;
      int target_unp_count = std::min(new_unpinned_rows, (int)target_unpinned.size());
      for (int j = 0; j < target_unp_count; j++) {
        int y = new_content_base + j * fh - pixel_shift_px;
        if (y + fh > divider_y_anim && y < dh) {
          this->draw_trip(target_unpinned[j], y, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, false);
        }
      }

      this->display_->end_clipping();

      this->schedule_state_.mutex.unlock();
      return;
    }

    // ---- Paging trigger (only in phase 0, and not during data transition) ----
    if (this->split_scroll_phase_ == 0 && this->data_transition_phase_ == 0 && needs_any_paging) {
      bool page_interval_elapsed = (uptime - this->pinned_page_timer_ >= (unsigned long)this->page_interval_);
      if (page_interval_elapsed && this->split_unpinned_page_timer_ == 0) {
        ESP_LOGD(TAG, "Paging: interval elapsed (%lums since timer), setting pending",
                 (unsigned long)(uptime - this->pinned_page_timer_));
        this->split_unpinned_page_timer_ = 1;
        this->split_unpinned_h_scroll_start_ = (unsigned long)shared_h_cycles;
      }

      if (this->split_unpinned_page_timer_ != 0) {
        bool can_change = (shared_scroll_dist == 0) ||
                          (shared_h_cycles > (int)this->split_unpinned_h_scroll_start_);
        if (can_change) {
          // Enter pre-scroll pause
          ESP_LOGD(TAG, "Paging: starting split scroll phase 1 (pre-pause)");
          shared_h_offset = 0;
          this->split_unpinned_page_timer_ = 0;
          this->split_scroll_phase_ = 1;
          this->split_scroll_start_ = uptime;
        }
      }
    } else if (this->split_scroll_phase_ == 0 && !needs_any_paging) {
      this->pinned_page_index_ = 0;
      this->split_unpinned_page_index_ = 0;
      this->split_unpinned_page_timer_ = 0;
      this->pinned_page_timer_ = uptime;
    }

    // ====== SPLIT DATA TRANSITION: detect on-screen trip changes and animate ======
    // Build current on-screen unpinned keys
    std::vector<std::string> split_on_screen_keys;
    for (int i = unpinned_si; i < unpinned_ei; i++)
      split_on_screen_keys.push_back(unpinned_pool[i].composite_key());

    // Detect changes (only in steady state: no split scroll, no pin transition)
    if (this->data_transition_phase_ == 0 &&
        !in_split_scroll && !in_split_pause &&
        this->pin_transition_phase_ == 0 &&
        !this->data_transition_old_keys_.empty()) {
      bool keys_changed = (split_on_screen_keys.size() != this->data_transition_old_keys_.size());
      if (!keys_changed) {
        for (size_t i = 0; i < split_on_screen_keys.size(); i++) {
          if (split_on_screen_keys[i] != this->data_transition_old_keys_[i]) {
            keys_changed = true;
            break;
          }
        }
      }
      if (keys_changed) {
        std::set<std::string> new_key_set(split_on_screen_keys.begin(), split_on_screen_keys.end());
        this->data_transition_departing_.clear();
        for (const auto &old_key : this->data_transition_old_keys_) {
          if (new_key_set.find(old_key) == new_key_set.end()) {
            bool found = false;
            for (const auto &trip : unpinned_pool) {
              if (trip.composite_key() == old_key) { this->data_transition_departing_.push_back(trip); found = true; break; }
            }
            if (!found) {
              for (const auto &trip : visible_trips) {
                if (trip.composite_key() == old_key) { this->data_transition_departing_.push_back(trip); found = true; break; }
              }
            }
            if (!found) {
              Trip placeholder;
              placeholder.route_id = old_key;
              placeholder.route_name = "?";
              placeholder.headsign = "";
              placeholder.route_color = Color(0x333333);
              placeholder.arrival_time = 0;
              placeholder.departure_time = 0;
              placeholder.is_realtime = false;
              this->data_transition_departing_.push_back(placeholder);
            }
          }
        }
        if (!this->data_transition_departing_.empty()) {
          for (const auto &dep : this->data_transition_departing_)
            ESP_LOGD(TAG, "Split on-screen: DEPARTED '%s'", dep.composite_key().c_str());
          std::set<std::string> old_set2(this->data_transition_old_keys_.begin(), this->data_transition_old_keys_.end());
          for (const auto &key : split_on_screen_keys)
            if (old_set2.find(key) == old_set2.end())
              ESP_LOGD(TAG, "Split on-screen: APPEARED '%s'", key.c_str());

          // Cancel any pending or in-progress paging scroll.
          // The paging trigger runs BEFORE data-transition detection each frame,
          // so split_scroll_phase_ may have just been set to 1 in this same frame.
          this->split_unpinned_page_timer_ = 0;
          this->pinned_page_timer_ = uptime;
          if (this->split_scroll_phase_ != 0) {
            ESP_LOGD(TAG, "Split data transition: cancelling active split scroll (phase %d)", this->split_scroll_phase_);
            this->split_scroll_phase_ = 0;
            this->split_scroll_start_ = 0;
          }

          if (shared_scroll_dist > 0 && shared_h_offset > 0) {
            ESP_LOGD(TAG, "Split data transition: %d trip(s) departed, waiting for h-scroll",
                     (int)this->data_transition_departing_.size());
            this->data_transition_phase_ = 1;
            this->data_transition_pending_cycles_ = shared_h_cycles;
          } else {
            ESP_LOGD(TAG, "Split data transition: %d trip(s) departed, starting collapse",
                     (int)this->data_transition_departing_.size());
            this->data_transition_phase_ = 2;
            this->data_transition_start_ = uptime;
          }
        } else {
          std::set<std::string> old_set(this->data_transition_old_keys_.begin(), this->data_transition_old_keys_.end());
          for (const auto &key : split_on_screen_keys)
            if (old_set.find(key) == old_set.end())
              ESP_LOGD(TAG, "Split on-screen: APPEARED (no depart) '%s'", key.c_str());
          this->data_transition_old_keys_ = split_on_screen_keys;
        }
      }
    }

    // Phase 1: waiting for h-scroll cycle to complete (split path)
    if (this->data_transition_phase_ == 1 && this->pin_transition_phase_ == 0) {
      bool scroll_idle = (shared_scroll_dist == 0) ||
                         (shared_h_cycles > this->data_transition_pending_cycles_);
      if (scroll_idle) {
        ESP_LOGD(TAG, "Split data transition: h-scroll idle, starting collapse");
        this->data_transition_phase_ = 2;
        this->data_transition_start_ = uptime;
        this->data_transition_pending_cycles_ = -1;
      }
    }

    // Phase 2 completion check (split path)
    if (this->data_transition_phase_ == 2 && this->pin_transition_phase_ == 0) {
      unsigned long dt_elapsed = uptime - this->data_transition_start_;
      float collapse_t = std::min(1.0f, (float)dt_elapsed / 500.0f);
      if (collapse_t >= 1.0f) {
        ESP_LOGD(TAG, "Split data transition: phase 2 collapse complete, resuming normal");
        this->data_transition_phase_ = 0;
        this->data_transition_departing_.clear();
        this->data_transition_pending_cycles_ = -1;
        this->data_transition_old_keys_ = split_on_screen_keys;
        // Reset page timer so a full interval elapses before the next v-scroll,
        // preventing a data-transition collapse from being immediately followed
        // by a paging scroll on the unpinned section.
        this->pinned_page_timer_ = uptime;
        // Also cancel any pending paging scroll that was queued before the
        // data transition started.
        this->split_unpinned_page_timer_ = 0;
        // Fall through to normal drawing
      }
    }

    // Update cached keys (steady state only, not during transitions)
    if (this->data_transition_phase_ == 0 && this->pin_transition_phase_ == 0 &&
        this->pinned_dt_phase_ == 0 &&
        !in_split_scroll && !in_split_pause) {
      this->data_transition_old_keys_ = split_on_screen_keys;
    }

    // ====== PINNED DATA TRANSITION: detect on-screen pinned trip changes ======
    // (e.g., "Now" trip departs and is replaced by "10m" for the same route)
    if (this->pinned_dt_phase_ == 0 &&
        !in_split_scroll && !in_split_pause &&
        this->pin_transition_phase_ == 0 &&
        !this->pinned_dt_old_trips_.empty()) {
      bool pinned_changed = false;
      int cur_pinned_count = pinned_ei - pinned_si;
      if (cur_pinned_count != (int)this->pinned_dt_old_trips_.size()) {
        pinned_changed = true;
      } else {
        for (int i = 0; i < cur_pinned_count; i++) {
          const auto &cur = pinned_pool[pinned_si + i];
          const auto &old_t = this->pinned_dt_old_trips_[i];
          if (cur.departure_time != old_t.departure_time ||
              cur.composite_key() != old_t.composite_key()) {
            pinned_changed = true;
            break;
          }
        }
      }

      if (pinned_changed) {
        // Cancel any pending paging scroll
        this->split_unpinned_page_timer_ = 0;
        this->pinned_page_timer_ = uptime;
        if (this->split_scroll_phase_ != 0) {
          ESP_LOGD(TAG, "Pinned data transition: cancelling active split scroll (phase %d)", this->split_scroll_phase_);
          this->split_scroll_phase_ = 0;
          this->split_scroll_start_ = 0;
        }

        if (shared_scroll_dist > 0 && shared_h_offset > 0) {
          ESP_LOGD(TAG, "Pinned data transition: trip changed, waiting for h-scroll");
          this->pinned_dt_phase_ = 1;
          this->pinned_dt_pending_cycles_ = shared_h_cycles;
        } else {
          ESP_LOGD(TAG, "Pinned data transition: trip changed, starting animation");
          this->pinned_dt_phase_ = 2;
          this->pinned_dt_start_ = uptime;
        }
      }
    }

    // Pinned data transition phase 1: wait for h-scroll cycle to complete
    if (this->pinned_dt_phase_ == 1 && this->pin_transition_phase_ == 0) {
      bool scroll_idle = (shared_scroll_dist == 0) ||
                         (shared_h_cycles > this->pinned_dt_pending_cycles_);
      if (scroll_idle) {
        ESP_LOGD(TAG, "Pinned data transition: h-scroll idle, starting animation");
        this->pinned_dt_phase_ = 2;
        this->pinned_dt_start_ = uptime;
        this->pinned_dt_pending_cycles_ = -1;
      }
    }

    // Pinned data transition phase 2 completion check
    if (this->pinned_dt_phase_ == 2 && this->pin_transition_phase_ == 0) {
      unsigned long pdt_elapsed = uptime - this->pinned_dt_start_;
      float pdt_t = std::min(1.0f, (float)pdt_elapsed / 500.0f);
      if (pdt_t >= 1.0f) {
        ESP_LOGD(TAG, "Pinned data transition: animation complete");
        this->pinned_dt_phase_ = 0;
        this->pinned_dt_old_trips_.clear();
        for (int i = pinned_si; i < pinned_ei; i++) {
          this->pinned_dt_old_trips_.push_back(pinned_pool[i]);
        }
        this->pinned_dt_pending_cycles_ = -1;
        this->pinned_page_timer_ = uptime;
        this->pinned_h_scroll_start_ = uptime;
        this->last_shared_scroll_dist_ = 0;
        // Fall through to normal drawing
      }
    }

    // Update cached pinned trips (steady state only)
    if (this->pinned_dt_phase_ == 0 && this->pin_transition_phase_ == 0 &&
        this->data_transition_phase_ == 0 &&
        !in_split_scroll && !in_split_pause) {
      this->pinned_dt_old_trips_.clear();
      for (int i = pinned_si; i < pinned_ei; i++) {
        this->pinned_dt_old_trips_.push_back(pinned_pool[i]);
      }
    }

    // ---- Drawing ----
    int fh = nominal_font_height;
    int divider_y = y_base - 1 + pinned_rows * fh - 1;
    int unpinned_y_start = y_base + pinned_rows * fh;
    int dw = this->display_->get_width();
    int dh = this->display_->get_height();

    if (in_split_pinned_scroll) {
      // Phase 2: Pinned section animates, unpinned stays at old position
      int pixel_shift_one = (int)(split_scroll_progress * fh);

      // ==== Pinned section (animated, clipped) ====
      this->display_->start_clipping(0, 0, dw, divider_y);
      int old_psi = this->split_old_pinned_page_;
      int new_psi = this->pinned_page_index_;
      if (old_psi + 1 == new_psi) {
        // Conveyor: combined range scrolls up by one row height
        int combined_end = std::min(new_psi + pinned_rows, pinned_pool_size);
        for (int i = old_psi; i < combined_end; i++) {
          int row = i - old_psi;
          int y = (y_base - 1) + row * fh - pixel_shift_one;
          this->draw_trip(pinned_pool[i], y, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, true);
        }
      } else {
        // Full section scroll (wrap-around)
        int section_h = pinned_rows * fh;
        int ps_full = (int)(split_scroll_progress * section_h);
        int old_ei_p = std::min(old_psi + pinned_rows, pinned_pool_size);
        for (int i = old_psi; i < old_ei_p; i++) {
          int row = i - old_psi;
          int y = (y_base - 1) + row * fh - ps_full;
          this->draw_trip(pinned_pool[i], y, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, true);
        }
        for (int i = pinned_si; i < pinned_ei; i++) {
          int row = i - pinned_si;
          int y = (y_base - 1) + section_h + row * fh - ps_full;
          this->draw_trip(pinned_pool[i], y, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, true);
        }
      }
      this->display_->end_clipping();

      // ==== Unpinned section (static at OLD position, no h-scroll) ====
      int old_usi = this->split_old_unpinned_page_;
      int old_uei = std::min(old_usi + unpinned_rows, unpinned_pool_size);
      this->display_->start_clipping(0, divider_y + 1, dw, dh);
      for (int i = old_usi; i < old_uei; i++) {
        int row = i - old_usi;
        int y = unpinned_y_start + row * fh;
        this->draw_trip(unpinned_pool[i], y, fh, uptime, rtc_now,
                        false, nullptr, nullptr, 0, 0,
                        uniform_clipping_start, uniform_clipping_end, false);
      }
      this->display_->end_clipping();

      this->display_->horizontal_line(0, divider_y, dw, this->divider_color_);

    } else if (in_split_unpinned_scroll) {
      // Phase 4: Unpinned section animates, pinned stays at new position
      int pixel_shift_one = (int)(split_scroll_progress * fh);

      // ==== Pinned section (static at NEW position, no h-scroll) ====
      this->display_->start_clipping(0, 0, dw, divider_y);
      for (int i = pinned_si; i < pinned_ei; i++) {
        int row = i - pinned_si;
        int y = (y_base - 1) + row * fh;
        this->draw_trip(pinned_pool[i], y, fh, uptime, rtc_now,
                        false, nullptr, nullptr, 0, 0,
                        uniform_clipping_start, uniform_clipping_end, true);
      }
      this->display_->end_clipping();

      // ==== Unpinned section (animated, clipped) ====
      this->display_->start_clipping(0, divider_y + 1, dw, dh);
      int old_usi = this->split_old_unpinned_page_;
      int new_usi = this->split_unpinned_page_index_;
      if (old_usi + 1 == new_usi) {
        int combined_end = std::min(new_usi + unpinned_rows, unpinned_pool_size);
        for (int i = old_usi; i < combined_end; i++) {
          int row = i - old_usi;
          int y = unpinned_y_start + row * fh - pixel_shift_one;
          this->draw_trip(unpinned_pool[i], y, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, false);
        }
      } else {
        int section_h = unpinned_rows * fh;
        int ps_full = (int)(split_scroll_progress * section_h);
        int old_ei_u = std::min(old_usi + unpinned_rows, unpinned_pool_size);
        for (int i = old_usi; i < old_ei_u; i++) {
          int row = i - old_usi;
          int y = unpinned_y_start + row * fh - ps_full;
          this->draw_trip(unpinned_pool[i], y, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, false);
        }
        for (int i = unpinned_si; i < unpinned_ei; i++) {
          int row = i - unpinned_si;
          int y = unpinned_y_start + section_h + row * fh - ps_full;
          this->draw_trip(unpinned_pool[i], y, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, false);
        }
      }
      this->display_->end_clipping();

      this->display_->horizontal_line(0, divider_y, dw, this->divider_color_);

    } else if (in_split_pause) {
      // Paused — draw both sections statically, no h-scroll
      // Phase 3 (mid-pause): pinned at new, unpinned at old
      // Phases 1, 5: both at current indices
      int draw_unp_si = unpinned_si;
      int draw_unp_ei = unpinned_ei;
      if (this->split_scroll_phase_ == 3) {
        draw_unp_si = this->split_old_unpinned_page_;
        draw_unp_ei = std::min(draw_unp_si + unpinned_rows, unpinned_pool_size);
      }

      this->display_->start_clipping(0, 0, dw, divider_y);
      for (int i = pinned_si; i < pinned_ei; i++) {
        int row = i - pinned_si;
        int y = y_base - 1 + row * fh;
        this->draw_trip(pinned_pool[i], y, fh, uptime, rtc_now,
                        false, nullptr, nullptr, 0, 0,
                        uniform_clipping_start, uniform_clipping_end, true);
      }
      this->display_->end_clipping();
      this->display_->horizontal_line(0, divider_y, dw, this->divider_color_);
      this->display_->start_clipping(0, divider_y + 1, dw, dh);
      for (int i = draw_unp_si; i < draw_unp_ei; i++) {
        int row = i - draw_unp_si;
        int y = unpinned_y_start + row * fh;
        this->draw_trip(unpinned_pool[i], y, fh, uptime, rtc_now,
                        false, nullptr, nullptr, 0, 0,
                        uniform_clipping_start, uniform_clipping_end, false);
      }
      this->display_->end_clipping();

    } else if (this->data_transition_phase_ == 2 || this->pinned_dt_phase_ == 2) {
      // Split data/pinned transition: animate one or both sections

      // == Pinned section ==
      this->display_->start_clipping(0, 0, dw, divider_y);
      if (this->pinned_dt_phase_ == 2 && !this->pinned_dt_old_trips_.empty()) {
        // Animate: old pinned trips scroll down out, new scroll in from above
        unsigned long pdt_elapsed2 = uptime - this->pinned_dt_start_;
        float pdt_t2 = std::min(1.0f, (float)pdt_elapsed2 / 500.0f);
        pdt_t2 = pdt_t2 * pdt_t2 * (3.0f - 2.0f * pdt_t2); // smoothstep
        int old_count = (int)this->pinned_dt_old_trips_.size();
        int new_count = pinned_ei - pinned_si;
        int max_count = std::max(old_count, new_count);
        for (int ri = 0; ri < max_count; ri++) {
          int row_top = y_base - 1 + ri * fh;
          // Clip to this row within the pinned area
          this->display_->start_clipping(0, std::max(row_top, 0), dw, std::min(row_top + fh, divider_y));
          // Old trip scrolls down
          if (ri < old_count) {
            int old_y = row_top + (int)((float)fh * pdt_t2 + 0.5f);
            this->draw_trip(this->pinned_dt_old_trips_[ri], old_y, fh, uptime, rtc_now,
                            false, nullptr, nullptr, 0, 0,
                            uniform_clipping_start, uniform_clipping_end, true);
          }
          // New trip scrolls in from above
          if (ri < new_count) {
            int new_y = row_top - fh + (int)((float)fh * pdt_t2 + 0.5f);
            this->draw_trip(pinned_pool[pinned_si + ri], new_y, fh, uptime, rtc_now,
                            false, nullptr, nullptr, 0, 0,
                            uniform_clipping_start, uniform_clipping_end, true);
          }
          this->display_->end_clipping();
        }
      } else {
        // Static pinned (no h-scroll during unpinned collapse)
        // Use old trips if in pinned_dt phase 1 (waiting for h-scroll)
        if (this->pinned_dt_phase_ == 1 && !this->pinned_dt_old_trips_.empty()) {
          for (int ri = 0; ri < (int)this->pinned_dt_old_trips_.size() && ri < pinned_rows; ri++) {
            int y = y_base - 1 + ri * fh;
            this->draw_trip(this->pinned_dt_old_trips_[ri], y, fh, uptime, rtc_now,
                            false, nullptr, nullptr, 0, 0,
                            uniform_clipping_start, uniform_clipping_end, true);
          }
        } else {
          for (int i = pinned_si; i < pinned_ei; i++) {
            int row = i - pinned_si;
            int y = y_base - 1 + row * fh;
            this->draw_trip(pinned_pool[i], y, fh, uptime, rtc_now,
                            false, nullptr, nullptr, 0, 0,
                            uniform_clipping_start, uniform_clipping_end, true);
          }
        }
      }
      this->display_->end_clipping();
      this->display_->horizontal_line(0, divider_y, dw, this->divider_color_);

      // == Unpinned section ==
      if (this->data_transition_phase_ == 2) {
        // Unpinned collapse animation (departing rows shrink, others fill from below)
        unsigned long dt_elapsed2 = uptime - this->data_transition_start_;
        float collapse_t2 = std::min(1.0f, (float)dt_elapsed2 / 500.0f);
        collapse_t2 = collapse_t2 * collapse_t2 * (3.0f - 2.0f * collapse_t2); // smoothstep

        this->display_->start_clipping(0, divider_y + 1, dw, dh);
        std::set<std::string> dep_keys;
        for (const auto &dep : this->data_transition_departing_)
          dep_keys.insert(dep.composite_key());

        struct SplitDTEntry { const Trip *trip; float scale; };
        std::vector<SplitDTEntry> dt_entries;

        for (const auto &old_key : this->data_transition_old_keys_) {
          bool is_departing = dep_keys.count(old_key) > 0;
          if (is_departing) {
            const Trip *dep_trip = nullptr;
            for (const auto &dep : this->data_transition_departing_) {
              if (dep.composite_key() == old_key) { dep_trip = &dep; break; }
            }
            if (dep_trip) dt_entries.push_back({dep_trip, 1.0f - collapse_t2});
          } else {
            for (int i = unpinned_si; i < unpinned_ei; i++) {
              if (unpinned_pool[i].composite_key() == old_key) {
                dt_entries.push_back({&unpinned_pool[i], 1.0f});
                break;
              }
            }
          }
        }

        std::set<std::string> old_key_set(this->data_transition_old_keys_.begin(), this->data_transition_old_keys_.end());
        for (int i = unpinned_si; i < unpinned_ei; i++) {
          if (old_key_set.find(unpinned_pool[i].composite_key()) == old_key_set.end()) {
            dt_entries.push_back({&unpinned_pool[i], 1.0f});
          }
        }

        float y_accum_dt = (float)unpinned_y_start;
        for (auto &entry : dt_entries) {
          float row_height = (float)fh * entry.scale;
          int y_int = (int)(y_accum_dt + 0.5f);
          if (entry.scale > 0.05f && y_int < dh) {
            this->display_->start_clipping(0, std::max(y_int, divider_y + 1), dw, std::min(y_int + (int)(row_height + 0.5f), dh));
            this->draw_trip(*entry.trip, y_int, fh, uptime, rtc_now,
                            false, nullptr, nullptr, 0, 0,
                            uniform_clipping_start, uniform_clipping_end, false);
            this->display_->end_clipping();
          }
          y_accum_dt += row_height;
        }
        this->display_->end_clipping();
      } else {
        // Static unpinned (no h-scroll during pinned animation)
        this->display_->start_clipping(0, divider_y + 1, dw, dh);
        for (int i = unpinned_si; i < unpinned_ei; i++) {
          int row = i - unpinned_si;
          int y = unpinned_y_start + row * fh;
          this->draw_trip(unpinned_pool[i], y, fh, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, false);
        }
        this->display_->end_clipping();
      }

    } else {
      // Phase 0: Normal — draw with h-scroll
      this->display_->start_clipping(0, 0, dw, divider_y);
      if (this->pinned_dt_phase_ == 1 && !this->pinned_dt_old_trips_.empty()) {
        // Frozen: draw cached old pinned trips while waiting for h-scroll to complete
        for (int ri = 0; ri < (int)this->pinned_dt_old_trips_.size() && ri < pinned_rows; ri++) {
          int y = y_base - 1 + ri * fh;
          this->draw_trip(this->pinned_dt_old_trips_[ri], y, fh, uptime, rtc_now,
                          false, nullptr, nullptr, shared_h_offset, shared_scroll_dist,
                          uniform_clipping_start, uniform_clipping_end, true);
        }
      } else {
        for (int i = pinned_si; i < pinned_ei; i++) {
          int row = i - pinned_si;
          int y = y_base - 1 + row * fh;
          this->draw_trip(pinned_pool[i], y, fh, uptime, rtc_now,
                          false, nullptr, nullptr, shared_h_offset, shared_scroll_dist,
                          uniform_clipping_start, uniform_clipping_end, true);
        }
      }
      this->display_->end_clipping();
      this->display_->horizontal_line(0, divider_y, dw, this->divider_color_);
      this->display_->start_clipping(0, divider_y + 1, dw, dh);
      for (int i = unpinned_si; i < unpinned_ei; i++) {
        int row = i - unpinned_si;
        int y = unpinned_y_start + row * fh;
        this->draw_trip(unpinned_pool[i], y, fh, uptime, rtc_now,
                        false, nullptr, nullptr, shared_h_offset, shared_scroll_dist,
                        uniform_clipping_start, uniform_clipping_end, false);
      }
      this->display_->end_clipping();
    }

    this->schedule_state_.mutex.unlock();
    return;
  }

  // Reset split paging state when not in split mode
  this->pinned_page_index_ = 0;
  this->pinned_page_timer_ = 0;
  this->pinned_h_scroll_start_ = 0;
  this->last_shared_scroll_dist_ = 0;
  this->split_unpinned_page_index_ = 0;
  this->split_unpinned_page_timer_ = 0;
  this->split_unpinned_h_scroll_start_ = 0;
  this->split_scroll_start_ = 0;
  this->split_scroll_phase_ = 0;
  this->split_old_pinned_page_ = 0;
  this->split_old_unpinned_page_ = 0;

  // Determine which trips to display (paging)
  int total_visible = visible_trips.size();
  int page_size = this->limit_;
  int start_index = 0;
  int end_index = total_visible;
  int current_page = 0;
  int total_pages = 1;
  bool paging_active = this->scroll_routes_ && total_visible > page_size;

  if (paging_active) {
    total_pages = this->paging_rotate_
      ? total_visible - page_size + 1
      : (total_visible + page_size - 1) / page_size;

    // Initialize paging state on first frame
    if (this->page_timer_start_ == 0) {
      this->page_timer_start_ = uptime;
      this->h_scroll_start_time_ = uptime;
      this->current_page_index_ = 0;
      this->page_change_pending_ = false;
      this->h_scroll_cycles_at_pending_ = -1;
      this->last_page_index_ = 0;
      this->page_pause_start_ = uptime;
      this->page_pause_is_pre_ = false;
    }

    // Clamp page index if total_pages changed (e.g. trip expired)
    if (this->current_page_index_ >= total_pages) {
      // Snap to last valid page (minimizes visual discontinuity vs jumping to 0)
      this->current_page_index_ = total_pages - 1;
      // Don't reset h_scroll_start_time_ — let wind-down handle scroll gracefully
      this->page_change_pending_ = false;
      this->h_scroll_cycles_at_pending_ = -1;
      // Let any active page scroll animation continue to finish smoothly;
      // the old page will render with whatever trips are still valid.
    }

    current_page = this->current_page_index_;
    start_index = this->paging_rotate_ ? current_page : current_page * page_size;
    end_index = std::min(start_index + page_size, total_visible);
  } else {
    // Reset paging state when not active
    // Don't reset h_scroll_start_time_ — let wind-down finish any active scroll gracefully
    this->page_timer_start_ = 0;
    this->current_page_index_ = 0;
    this->page_change_pending_ = false;
    this->h_scroll_cycles_at_pending_ = -1;
    this->last_page_index_ = -1;
    this->page_scroll_start_ = 0;
    this->page_pause_start_ = 0;
    this->page_pause_is_pre_ = false;
  }

  // Helper: compute total continuous scroll distance for a range of trips.
  // Returns the total pixel distance for one full marquee cycle, or 0 if no scrolling needed.
  auto compute_scroll_distance = [&](int si, int ei) -> int {
    if (!this->scroll_headsigns_) return 0;
    int max_headsign_width = 0;
    bool any_overflow = false;
    for (int i = si; i < ei; i++) {
      int overflow = 0;
      int headsign_width = 0;
      this->draw_trip(visible_trips[i], 0, nominal_font_height, uptime, rtc_now,
                      true, &overflow, &headsign_width, -1, 0,
                      uniform_clipping_start, uniform_clipping_end, i < effective_pinned_count);
      if (overflow > 0) any_overflow = true;
      max_headsign_width = std::max(max_headsign_width, headsign_width);
    }
    if (any_overflow) {
      return max_headsign_width + marquee_gap;
    }
    return 0;
  };

  // Compute horizontal scroll parameters for the current page
  int total_scroll_distance = compute_scroll_distance(start_index, end_index);
  int h_scroll_offset = -1;
  int h_scroll_cycles = 0;
  if (total_scroll_distance > 0) {
    unsigned long elapsed = uptime - this->h_scroll_start_time_;
    int total_px = (int)((unsigned long)elapsed * scroll_speed / 1000);
    h_scroll_offset = total_px % total_scroll_distance;
    h_scroll_cycles = total_px / total_scroll_distance;
    this->last_scroll_distance_ = total_scroll_distance;
  } else if (this->last_scroll_distance_ > 0) {
    // Overflow just disappeared mid-scroll — finish current cycle using old distance
    unsigned long elapsed = uptime - this->h_scroll_start_time_;
    int total_px = (int)((unsigned long)elapsed * scroll_speed / 1000);
    int wind_down_offset = total_px % this->last_scroll_distance_;
    h_scroll_cycles = total_px / this->last_scroll_distance_;
    if (wind_down_offset < 3) {
      // Close enough to start — stop scrolling
      this->last_scroll_distance_ = 0;
      h_scroll_offset = -1;
      this->h_scroll_start_time_ = uptime;
    } else {
      // Keep scrolling with old distance until we wrap back
      total_scroll_distance = this->last_scroll_distance_;
      h_scroll_offset = wind_down_offset;
    }
  }

  // Pin transition phase 1 completion check (flat-list path)
  if (this->pin_transition_phase_ == 1) {
    // Check if a page scroll animation is in progress
    bool page_scroll_active = (this->page_scroll_duration_ > 0 && this->page_scroll_start_ > 0);
    if (page_scroll_active) {
      // Wait — page scroll must finish first
    } else if (total_scroll_distance > 0) {
      // Scrolling active — track when offset moves away from 0, then wait for return
      if (h_scroll_offset >= 3) {
        this->pin_transition_seen_nonzero_offset_ = true;
      }
      if (this->pin_transition_seen_nonzero_offset_ && h_scroll_offset < 3) {
        h_scroll_offset = 0;
        this->h_scroll_start_time_ = uptime;  // reset h-scroll to genuine 0
        bool is_pin_up = (this->pin_transition_target_eff_pinned_ >= this->pin_transition_old_eff_pinned_);
        if (is_pin_up) {
          ESP_LOGD(TAG, "Pin transition: phase 1 complete (flat), entering phase 2 (collapse)");
          this->pin_transition_phase_ = 2;
        } else {
          ESP_LOGD(TAG, "Pin transition: phase 1 complete (flat unpin), entering phase 5 (hide-pinned)");
          this->pin_transition_phase_ = 5;
        }
        this->pin_transition_start_ = uptime;
      }
    } else {
      // No scrolling — wait 500ms
      unsigned long phase1_elapsed = uptime - this->pin_transition_start_;
      if (phase1_elapsed >= 500) {
        bool is_pin_up = (this->pin_transition_target_eff_pinned_ >= this->pin_transition_old_eff_pinned_);
        if (is_pin_up) {
          ESP_LOGD(TAG, "Pin transition: phase 1 complete (flat no-scroll), entering phase 2");
          this->pin_transition_phase_ = 2;
        } else {
          ESP_LOGD(TAG, "Pin transition: phase 1 complete (flat no-scroll unpin), entering phase 5");
          this->pin_transition_phase_ = 5;
        }
        this->pin_transition_start_ = uptime;
      }
    }
  }

  // Pin transition phase 2: collapse newly-pinned rows out of the flat list
  if (this->pin_transition_phase_ == 2) {
    unsigned long phase2_elapsed = uptime - this->pin_transition_start_;
    float collapse_t = std::min(1.0f, (float)phase2_elapsed / 500.0f);
    // smoothstep
    collapse_t = collapse_t * collapse_t * (3.0f - 2.0f * collapse_t);

    if (collapse_t >= 1.0f && phase2_elapsed >= 1000) { // 500ms collapse + 500ms dwell
      bool is_pin_up = (this->pin_transition_target_eff_pinned_ >= this->pin_transition_old_eff_pinned_);
      if (is_pin_up) {
        // Phase 2 complete — enter phase 3 (push-divider)
        ESP_LOGD(TAG, "Pin transition: phase 2 complete (flat), entering phase 3 (push-divider)");
        this->pin_transition_phase_ = 3;
        this->pin_transition_start_ = uptime;
        // Don't return — let phase 2 render the dwell state one more time.
        // Phase 3 starts cleanly next frame with new partition.
      } else {
        // Unpin — should have gone to phase 5 from phase 1; this is a fallback
        ESP_LOGD(TAG, "Pin transition: phase 2 complete (flat unpin), entering phase 5");
        this->pin_transition_phase_ = 5;
        this->pin_transition_start_ = uptime;
      }
    }

    // Render the flat list with collapsing rows.
    // Only collapse trips that were on-screen during phase 1.
    // Fill-in from below skips newly-pinned trips (they're leaving for pinned section).
    int phase2_base = start_index;  // from paging logic above
    int on_screen_end = std::min(phase2_base + this->limit_, (int)visible_trips.size());

    // Build display list: (trip index, should_collapse)
    // First: on-screen trips (some collapsing)
    std::vector<std::pair<int, bool>> phase2_entries;
    int collapsing_count = 0;
    for (int i = phase2_base; i < on_screen_end; i++) {
      std::string ck = visible_trips[i].composite_key();
      bool newly_pinned = (this->pin_transition_target_pinned_routes_.find(ck) != this->pin_transition_target_pinned_routes_.end()) &&
                          (this->pin_transition_old_pinned_routes_.find(ck) == this->pin_transition_old_pinned_routes_.end());
      bool newly_unpinned = (this->pin_transition_target_pinned_routes_.find(ck) == this->pin_transition_target_pinned_routes_.end()) &&
                            (this->pin_transition_old_pinned_routes_.find(ck) != this->pin_transition_old_pinned_routes_.end());
      bool collapsing = newly_pinned || newly_unpinned;
      phase2_entries.push_back({i, collapsing});
      if (collapsing) collapsing_count++;
    }
    // Then: fill-in trips from beyond the page, skipping newly-pinned/unpinned
    int fill_needed = collapsing_count;
    // First scan: from on_screen_end to end of list
    for (int i = on_screen_end; i < (int)visible_trips.size() && fill_needed > 0; i++) {
      std::string ck = visible_trips[i].composite_key();
      bool newly_pinned = (this->pin_transition_target_pinned_routes_.find(ck) != this->pin_transition_target_pinned_routes_.end()) &&
                          (this->pin_transition_old_pinned_routes_.find(ck) == this->pin_transition_old_pinned_routes_.end());
      bool newly_unpinned = (this->pin_transition_target_pinned_routes_.find(ck) == this->pin_transition_target_pinned_routes_.end()) &&
                            (this->pin_transition_old_pinned_routes_.find(ck) != this->pin_transition_old_pinned_routes_.end());
      if (!newly_pinned && !newly_unpinned) {
        phase2_entries.push_back({i, false});
        fill_needed--;
      }
    }
    // Wrap around: if still need fill, scan from 0 up to phase2_base (like pager wrapping to page 1)
    for (int i = 0; i < phase2_base && fill_needed > 0; i++) {
      std::string ck = visible_trips[i].composite_key();
      bool newly_pinned = (this->pin_transition_target_pinned_routes_.find(ck) != this->pin_transition_target_pinned_routes_.end()) &&
                          (this->pin_transition_old_pinned_routes_.find(ck) == this->pin_transition_old_pinned_routes_.end());
      bool newly_unpinned = (this->pin_transition_target_pinned_routes_.find(ck) == this->pin_transition_target_pinned_routes_.end()) &&
                            (this->pin_transition_old_pinned_routes_.find(ck) != this->pin_transition_old_pinned_routes_.end());
      if (!newly_pinned && !newly_unpinned) {
        phase2_entries.push_back({i, false});
        fill_needed--;
      }
    }

    // Centering based on the original limit_ rows (stable baseline)
    int stable_count = this->limit_;
    int max_trips_height = (stable_count * this->font_->get_ascender()) + ((stable_count - 1) * this->font_->get_descender());
    int y_base = (this->display_->get_height() % max_trips_height) / 2;
    int dw = this->display_->get_width();
    int dh = this->display_->get_height();

    // Clip entire collapse animation to original display area
    this->display_->start_clipping(0, 0, dw, dh);
    float y_accum = (float)y_base;
    for (auto &entry : phase2_entries) {
      const Trip &trip = visible_trips[entry.first];
      std::string key = trip.composite_key();
      bool should_collapse = entry.second;

      float row_scale = 1.0f;
      if (should_collapse) {
        row_scale = 1.0f - collapse_t; // shrink to 0
      }

      float row_height = (float)nominal_font_height * row_scale;
      int y_int = (int)(y_accum + 0.5f);

      if (row_scale > 0.05f && y_int < dh) {
        // Clip to current row height
        this->display_->start_clipping(0, y_int, dw, std::min(y_int + (int)(row_height + 0.5f), dh));
        this->draw_trip(trip, y_int, nominal_font_height, uptime, rtc_now,
                        false, nullptr, nullptr, 0, 0,
                        uniform_clipping_start, uniform_clipping_end,
                        this->pin_transition_old_pinned_routes_.find(key) != this->pin_transition_old_pinned_routes_.end());
        this->display_->end_clipping();
      }

      y_accum += row_height;
    }
    this->display_->end_clipping();

    this->schedule_state_.mutex.unlock();
    return;
  }

  // Page scroll transition handling
  bool in_page_scroll = false;
  int old_start_index = 0, old_end_index = 0;
  float scroll_progress = 0.0f;

  if (paging_active) {
    // Handle active scroll transition
    if (this->page_scroll_duration_ > 0 && this->page_scroll_start_ > 0) {
      // Note: even if scroll_from_page_ >= total_pages (trip expired),
      // we let the scroll continue — old page renders with capped indices
      // (may show fewer trips) rather than snapping instantly.
      {
        unsigned long elapsed = uptime - this->page_scroll_start_;
        if (elapsed >= (unsigned long)this->page_scroll_duration_) {
          // Transition complete - begin post-scroll pause
          this->page_scroll_start_ = 0;
          this->page_pause_start_ = uptime;
          this->page_pause_is_pre_ = false;
          total_scroll_distance = compute_scroll_distance(start_index, end_index);
          h_scroll_offset = 0;
          h_scroll_cycles = 0;
        } else {
          float t = (float)elapsed / (float)this->page_scroll_duration_;
          scroll_progress = t * t * (3.0f - 2.0f * t); // smoothstep ease-in-out
          in_page_scroll = true;
          h_scroll_offset = 0; // Freeze horizontal scroll during transition

          old_start_index = this->paging_rotate_ ? this->scroll_from_page_ : this->scroll_from_page_ * page_size;
          old_end_index = std::min(old_start_index + page_size, total_visible);
        }
      }
    }

    // Handle pause (pre-scroll or post-scroll)
    if (!in_page_scroll && this->page_pause_start_ > 0) {
      unsigned long pause_elapsed = uptime - this->page_pause_start_;
      if (pause_elapsed >= (unsigned long)this->page_pause_duration_) {
        this->page_pause_start_ = 0;
        if (this->page_pause_is_pre_) {
          // Pre-scroll pause complete - begin page transition
          this->scroll_from_page_ = this->current_page_index_;
          this->current_page_index_ = (this->current_page_index_ + 1) % total_pages;
          this->last_page_index_ = this->current_page_index_;

          current_page = this->current_page_index_;
          start_index = this->paging_rotate_ ? current_page : current_page * page_size;
          end_index = std::min(start_index + page_size, total_visible);

          ESP_LOGD(TAG, "Page change: page %d->%d  showing [%d..%d) of %d visible",
                   this->scroll_from_page_, this->current_page_index_,
                   start_index, end_index, total_visible);

          old_start_index = this->paging_rotate_ ? this->scroll_from_page_ : this->scroll_from_page_ * page_size;
          old_end_index = std::min(old_start_index + page_size, total_visible);

          if (this->page_scroll_duration_ > 0) {
            this->page_scroll_start_ = uptime;
            in_page_scroll = true;
            scroll_progress = 0.0f;
          } else {
            // Instant page change - begin post-scroll pause
            this->page_pause_start_ = uptime;
            this->page_pause_is_pre_ = false;
            total_scroll_distance = compute_scroll_distance(start_index, end_index);
          }
          // Clear cached keys so page change doesn't trigger data transition
          this->data_transition_old_keys_.clear();
          this->data_transition_phase_ = 0;
          this->data_transition_departing_.clear();
          this->data_transition_pending_cycles_ = -1;
          this->pinned_dt_phase_ = 0;
          this->pinned_dt_old_trips_.clear();
          this->pinned_dt_pending_cycles_ = -1;
          h_scroll_offset = 0;
          h_scroll_cycles = 0;
        } else {
          // Post-scroll pause complete - begin horizontal scrolling
          this->h_scroll_start_time_ = uptime;
          this->page_timer_start_ = uptime;
          h_scroll_offset = 0;
          h_scroll_cycles = 0;
        }
      } else {
        // Still pausing - freeze horizontal scroll at 0
        h_scroll_offset = 0;
      }
    }

    // Check if page change should be triggered
    if (!in_page_scroll && this->page_scroll_start_ == 0 && this->page_pause_start_ == 0) {
      // Check if page interval has elapsed
      if (!this->page_change_pending_ &&
          uptime - this->page_timer_start_ >= (unsigned long)this->page_interval_) {
        this->page_change_pending_ = true;
        this->h_scroll_cycles_at_pending_ = h_scroll_cycles;
      }

      // Check if conditions are met to trigger page change
      if (this->page_change_pending_) {
        bool can_change = (total_scroll_distance == 0) ||
                          (h_scroll_cycles > this->h_scroll_cycles_at_pending_);

        if (can_change) {
          // Enter pre-scroll pause before page transition
          h_scroll_offset = 0;
          this->page_change_pending_ = false;
          this->h_scroll_cycles_at_pending_ = -1;
          this->page_pause_start_ = uptime;
          this->page_pause_is_pre_ = true;
        }
      }
    }
  }

  // Helper: draw a page of trips at a given vertical base offset
  auto draw_page = [&](int si, int ei, int y_base, int scroll_dist, int scroll_off) {
    if (si < 0 || si >= ei || ei > total_visible) return;

    int display_count = ei - si;
    // Always center based on page_size (limit_) so rows stay at fixed positions
    // even when fewer trips are displayed (avoids jump when pin transition starts)
    int center_count = page_size;
    int max_trips_height = (center_count * this->font_->get_ascender()) + ((center_count - 1) * this->font_->get_descender());
    int y_offset = y_base + (this->display_->get_height() % max_trips_height) / 2;

    // Determine how many pinned rows are at the start of this slice
    int pinned_in_slice = std::max(0, std::min(effective_pinned_count, ei) - si);
    bool has_divider = (pinned_in_slice > 0 && pinned_in_slice < display_count);

    for (int i = si; i < ei; i++) {
      int row_index = i - si;
      int y;
      if (has_divider && row_index < pinned_in_slice) {
        y = y_offset - 1 + row_index * nominal_font_height;  // shift pinned rows up 1px
      } else {
        y = y_offset + row_index * nominal_font_height;
      }
      const Trip &trip = visible_trips[i];
      this->draw_trip(trip, y, nominal_font_height, uptime, rtc_now,
                      false, nullptr, nullptr, scroll_off, scroll_dist,
                      uniform_clipping_start, uniform_clipping_end, i < effective_pinned_count);
    }

    // Draw divider line between pinned and unpinned sections
    if (has_divider) {
      int divider_y = (y_offset - 1) + pinned_in_slice * nominal_font_height - 1;
      this->display_->horizontal_line(0, divider_y, this->display_->get_width(), this->divider_color_);
    }
  };

  // ====== DATA TRANSITION: detect on-screen trip changes and animate ======
  // Build current on-screen keys
  std::vector<std::string> current_on_screen_keys;
  for (int i = start_index; i < end_index; i++) {
    current_on_screen_keys.push_back(visible_trips[i].composite_key());
  }

  // Detect changes (only in steady state: no page scroll, no pin transition, no active data transition)
  if (this->data_transition_phase_ == 0 &&
      !in_page_scroll && this->page_scroll_start_ == 0 &&
      this->pin_transition_phase_ == 0 &&
      !this->data_transition_old_keys_.empty()) {
    // Check if on-screen trip keys changed
    bool keys_changed = (current_on_screen_keys.size() != this->data_transition_old_keys_.size());
    if (!keys_changed) {
      for (size_t i = 0; i < current_on_screen_keys.size(); i++) {
        if (current_on_screen_keys[i] != this->data_transition_old_keys_[i]) {
          keys_changed = true;
          break;
        }
      }
    }
    if (keys_changed) {
      // Find departing trips (in old but not in new)
      std::set<std::string> new_key_set(current_on_screen_keys.begin(), current_on_screen_keys.end());
      this->data_transition_departing_.clear();
      for (const auto &old_key : this->data_transition_old_keys_) {
        if (new_key_set.find(old_key) == new_key_set.end()) {
          // This trip departed — find it in visible_trips (may still be there, just off-screen)
          // or reconstruct from old data. Search visible_trips first.
          bool found = false;
          for (const auto &trip : visible_trips) {
            if (trip.composite_key() == old_key) {
              this->data_transition_departing_.push_back(trip);
              found = true;
              break;
            }
          }
          // If not found in visible_trips, the trip is truly gone — use a placeholder
          if (!found) {
            Trip placeholder;
            placeholder.route_id = old_key;
            placeholder.route_name = "?";
            placeholder.headsign = "";
            placeholder.route_color = Color(0x333333);
            placeholder.arrival_time = 0;
            placeholder.departure_time = 0;
            placeholder.is_realtime = false;
            this->data_transition_departing_.push_back(placeholder);
          }
        }
      }
      if (!this->data_transition_departing_.empty()) {
        // Log departing and arriving trips
        for (const auto &dep : this->data_transition_departing_) {
          ESP_LOGD(TAG, "On-screen: DEPARTED '%s'", dep.composite_key().c_str());
        }
        std::set<std::string> old_key_set2(this->data_transition_old_keys_.begin(), this->data_transition_old_keys_.end());
        for (const auto &key : current_on_screen_keys) {
          if (old_key_set2.find(key) == old_key_set2.end()) {
            ESP_LOGD(TAG, "On-screen: APPEARED (replacing) '%s'", key.c_str());
          }
        }

        // Wait for h-scroll to finish current cycle before animating
        if (total_scroll_distance > 0 && h_scroll_offset > 0) {
          ESP_LOGD(TAG, "Data transition: %d trip(s) departed, waiting for h-scroll to finish",
                   (int)this->data_transition_departing_.size());
          this->data_transition_phase_ = 1;  // pending: wait for scroll idle
          this->data_transition_pending_cycles_ = h_scroll_cycles;
          this->data_transition_old_start_ = start_index;
          this->data_transition_old_end_ = end_index;
        } else {
          // No h-scroll active — start collapse immediately
          ESP_LOGD(TAG, "Data transition: %d trip(s) departed, starting collapse animation",
                   (int)this->data_transition_departing_.size());
          this->data_transition_phase_ = 2;  // animating
          this->data_transition_start_ = uptime;
          this->data_transition_old_start_ = start_index;
          this->data_transition_old_end_ = end_index;
        }
      } else {
        // Trips changed but nothing departed (new trips appeared) — just update keys
        // Log which trips appeared
        std::set<std::string> old_set(this->data_transition_old_keys_.begin(), this->data_transition_old_keys_.end());
        for (const auto &key : current_on_screen_keys) {
          if (old_set.find(key) == old_set.end()) {
            ESP_LOGD(TAG, "On-screen: APPEARED (no depart) '%s'", key.c_str());
          }
        }
        this->data_transition_old_keys_ = current_on_screen_keys;
      }
    }
  }

  // Phase 1: waiting for h-scroll cycle to complete
  if (this->data_transition_phase_ == 1) {
    bool scroll_idle = (total_scroll_distance == 0) ||
                       (h_scroll_cycles > this->data_transition_pending_cycles_);
    if (scroll_idle) {
      ESP_LOGD(TAG, "Data transition: h-scroll idle, starting collapse animation");
      this->data_transition_phase_ = 2;  // advance to animation
      this->data_transition_start_ = uptime;
      this->data_transition_pending_cycles_ = -1;
    }
  }

  // Data transition animation rendering (phase 2 = animating)
  if (this->data_transition_phase_ == 2) {
    unsigned long dt_elapsed = uptime - this->data_transition_start_;
    float collapse_t = std::min(1.0f, (float)dt_elapsed / 500.0f);
    collapse_t = collapse_t * collapse_t * (3.0f - 2.0f * collapse_t); // smoothstep

    if (collapse_t >= 1.0f) {
      // Animation complete — snap to new layout
      this->data_transition_phase_ = 0;
      this->data_transition_departing_.clear();
      this->data_transition_pending_cycles_ = -1;
      this->data_transition_old_keys_ = current_on_screen_keys;
      // Fall through to normal rendering below
    } else {
      // Build merged display list: old on-screen positions with departing trips collapsing
      // and new trips filling in from below
      std::set<std::string> departing_set;
      for (const auto &dep : this->data_transition_departing_) {
        departing_set.insert(dep.composite_key());
      }

      // Centering
      int center_count = page_size;
      int max_trips_height = (center_count * this->font_->get_ascender()) + ((center_count - 1) * this->font_->get_descender());
      int y_base_dt = (this->display_->get_height() % max_trips_height) / 2;
      int dw = this->display_->get_width();
      int dh = this->display_->get_height();

      // Build ordered list: for each old on-screen slot, draw old trip (collapsing if departing)
      // then append fill-in trips from new on-screen set
      struct DtEntry {
        const Trip *trip;
        bool collapsing;
        bool is_pinned;
      };
      std::vector<DtEntry> dt_entries;

      // First: reconstruct old on-screen in order
      for (const auto &old_key : this->data_transition_old_keys_) {
        if (departing_set.count(old_key)) {
          // Find in departing cache
          for (const auto &dep : this->data_transition_departing_) {
            if (dep.composite_key() == old_key) {
              dt_entries.push_back({&dep, true, false});
              break;
            }
          }
        } else {
          // Still present — find in current visible_trips
          for (int i = start_index; i < end_index; i++) {
            if (visible_trips[i].composite_key() == old_key) {
              dt_entries.push_back({&visible_trips[i], false, i < effective_pinned_count});
              break;
            }
          }
        }
      }

      // Then: fill in new trips that weren't in old set
      std::set<std::string> old_key_set(this->data_transition_old_keys_.begin(), this->data_transition_old_keys_.end());
      int fill_count = 0;
      for (int i = start_index; i < end_index; i++) {
        std::string ck = visible_trips[i].composite_key();
        if (old_key_set.find(ck) == old_key_set.end()) {
          dt_entries.push_back({&visible_trips[i], false, i < effective_pinned_count});
          fill_count++;
        }
      }

      // Render with collapse
      this->display_->start_clipping(0, 0, dw, dh);
      float y_accum = (float)y_base_dt;
      for (auto &entry : dt_entries) {
        float row_scale = entry.collapsing ? (1.0f - collapse_t) : 1.0f;
        float row_height = (float)nominal_font_height * row_scale;
        int y_int = (int)(y_accum + 0.5f);

        if (row_scale > 0.05f && y_int < dh) {
          this->display_->start_clipping(0, y_int, dw, std::min(y_int + (int)(row_height + 0.5f), dh));
          this->draw_trip(*entry.trip, y_int, nominal_font_height, uptime, rtc_now,
                          false, nullptr, nullptr, 0, 0,
                          uniform_clipping_start, uniform_clipping_end, entry.is_pinned);
          this->display_->end_clipping();
        }
        y_accum += row_height;
      }
      this->display_->end_clipping();

      this->schedule_state_.mutex.unlock();
      return;
    }
  }

  // Update cached keys for next frame (steady state)
  // Don't update during pin transitions — frozen partition makes keys unreliable,
  // and we want the data transition detector to catch schedule changes that occurred
  // during the pin transition once it completes.
  if (this->data_transition_phase_ == 0 && this->pin_transition_phase_ == 0) {
    this->data_transition_old_keys_ = current_on_screen_keys;
  }

  if (in_page_scroll) {
    // Single-step rotate: adjacent pages in rotate mode share all but one trip,
    // so we scroll the combined range up by one line height (conveyor belt effect).
    bool single_line = this->paging_rotate_ &&
                       this->scroll_from_page_ + 1 == this->current_page_index_;

    if (single_line) {
      int combined_start = old_start_index;
      int combined_end = end_index;
      int pixel_shift = (int)(scroll_progress * nominal_font_height);
      int combined_scroll_dist = compute_scroll_distance(combined_start, combined_end);

      // Centering based on page_size (same for old and new page)
      int max_trips_height = (page_size * this->font_->get_ascender()) + ((page_size - 1) * this->font_->get_descender());
      int y_center = (this->display_->get_height() % max_trips_height) / 2;

      // Pinned row awareness during conveyor scroll
      int pinned_in_combined = std::max(0, std::min(effective_pinned_count, combined_end) - combined_start);
      bool has_divider = (pinned_in_combined > 0 && pinned_in_combined < (combined_end - combined_start));

      for (int i = combined_start; i < combined_end; i++) {
        int row_index = i - combined_start;
        int y_pos;
        if (has_divider && row_index < pinned_in_combined) {
          y_pos = y_center - 1 + row_index * nominal_font_height - pixel_shift;
        } else {
          y_pos = y_center + row_index * nominal_font_height - pixel_shift;
        }
        this->draw_trip(visible_trips[i], y_pos, nominal_font_height, uptime, rtc_now,
                        false, nullptr, nullptr, 0, combined_scroll_dist,
                        uniform_clipping_start, uniform_clipping_end, i < effective_pinned_count);
      }

      // Draw divider during conveyor scroll
      if (has_divider) {
        int divider_y = (y_center - 1) + pinned_in_combined * nominal_font_height - 1 - pixel_shift;
        if (divider_y >= 0 && divider_y < this->display_->get_height()) {
          this->display_->horizontal_line(0, divider_y, this->display_->get_width(), this->divider_color_);
        }
      }
    } else {
      // Full page scroll: old page exits upward, new page enters from below
      int dh = this->display_->get_height();
      int pixel_shift = (int)(scroll_progress * dh);
      int old_scroll_dist = compute_scroll_distance(old_start_index, old_end_index);
      draw_page(old_start_index, old_end_index, -pixel_shift, old_scroll_dist, 0);
      draw_page(start_index, end_index, dh - pixel_shift, total_scroll_distance, 0);
    }
  } else {
    int effective_offset = (h_scroll_offset >= 0) ? h_scroll_offset : 0;
    draw_page(start_index, end_index, 0, total_scroll_distance, effective_offset);
  }

  this->schedule_state_.mutex.unlock();
}

void TransitTracker::set_scroll_routes(bool v) {
  if (this->scroll_routes_ != v) {
    this->scroll_routes_ = v;
    this->page_timer_start_ = 0; // Reset paging state
    if (this->ws_client_.available()) {
      this->reconnect();
    }
  }
}

}  // namespace transit_tracker
}  // namespace esphome