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

    this->schedule_state_.mutex.unlock();

#ifdef USE_MQTT
    // Flag routes for MQTT discovery (will publish in loop or on connect)
    this->mqtt_routes_pending_ = true;
    this->mqtt_divider_color_pending_ = true;
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

  std::set<std::string> seen;
  for (const auto &t : this->schedule_state_.trips) {
    std::string key = t.composite_key();
    if (seen.count(key)) continue;
    seen.insert(key);

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

  std::set<std::string> seen_route_ids;
  for (const auto &t : this->schedule_state_.trips) {
    if (seen_route_ids.count(t.route_id)) continue;
    seen_route_ids.insert(t.route_id);

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
    if (this->show_pin_icon_ && !this->pinned_routes_.empty()) {
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

  // Partition visible trips: pinned first, then unpinned (stable to preserve order within each group)
  int actual_pinned_in_visible = 0;
  if (!this->pinned_routes_.empty()) {
    std::stable_partition(visible_trips.begin(), visible_trips.end(), [this](const Trip &trip) {
      return this->pinned_routes_.find(trip.composite_key()) != this->pinned_routes_.end();
    });
    for (const auto &trip : visible_trips) {
      if (this->pinned_routes_.find(trip.composite_key()) != this->pinned_routes_.end()) {
        actual_pinned_in_visible++;
      } else {
        break;
      }
    }
  }
  int effective_pinned_count = (actual_pinned_in_visible > 0)
    ? std::min(this->pinned_rows_count_, actual_pinned_in_visible)
    : 0;

  int nominal_font_height = this->font_->get_ascender() + this->font_->get_descender();
  unsigned long uptime = millis();
  uint rtc_now = this->rtc_->now().timestamp;

  // Pre-pass: compute uniform headsign boundaries if enabled
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
      if (effective_pinned_count > 0 && this->show_pin_icon_) {
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
            this->split_scroll_phase_ = 0;
            this->split_scroll_start_ = 0;
            this->pinned_h_scroll_start_ = uptime;
            this->pinned_page_timer_ = uptime;
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
      for (int i = pinned_si; i < pinned_ei; i++) {
        int ov = 0, hw = 0;
        this->draw_trip(pinned_pool[i], 0, nominal_font_height, uptime, rtc_now,
                        true, &ov, &hw, -1, 0,
                        uniform_clipping_start, uniform_clipping_end, true);
        if (ov > 0) any_ov = true;
        max_hw = std::max(max_hw, hw);
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
      }
    }

    // ---- Paging trigger (only in phase 0) ----
    if (this->split_scroll_phase_ == 0 && needs_any_paging) {
      bool page_interval_elapsed = (uptime - this->pinned_page_timer_ >= (unsigned long)this->page_interval_);
      if (page_interval_elapsed && this->split_unpinned_page_timer_ == 0) {
        this->split_unpinned_page_timer_ = 1;
        this->split_unpinned_h_scroll_start_ = (unsigned long)shared_h_cycles;
      }

      if (this->split_unpinned_page_timer_ != 0) {
        bool can_change = (shared_scroll_dist == 0) ||
                          (shared_h_cycles > (int)this->split_unpinned_h_scroll_start_);
        if (can_change) {
          // Enter pre-scroll pause
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

    } else {
      // Phase 0: Normal — draw with h-scroll
      this->display_->start_clipping(0, 0, dw, divider_y);
      for (int i = pinned_si; i < pinned_ei; i++) {
        int row = i - pinned_si;
        int y = y_base - 1 + row * fh;
        this->draw_trip(pinned_pool[i], y, fh, uptime, rtc_now,
                        false, nullptr, nullptr, shared_h_offset, shared_scroll_dist,
                        uniform_clipping_start, uniform_clipping_end, true);
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

    // Clamp page index if total_pages changed
    if (this->current_page_index_ >= total_pages) {
      this->current_page_index_ = 0;
      this->page_timer_start_ = uptime;
      this->h_scroll_start_time_ = uptime;
      this->page_change_pending_ = false;
      this->h_scroll_cycles_at_pending_ = -1;
      this->page_pause_start_ = 0;
      this->page_pause_is_pre_ = false;
    }

    current_page = this->current_page_index_;
    start_index = this->paging_rotate_ ? current_page : current_page * page_size;
    end_index = std::min(start_index + page_size, total_visible);
  } else {
    // Reset paging state when not active
    this->page_timer_start_ = 0;
    this->current_page_index_ = 0;
    this->page_change_pending_ = false;
    this->h_scroll_cycles_at_pending_ = -1;
    this->h_scroll_start_time_ = 0;
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
  }

  // Page scroll transition handling
  bool in_page_scroll = false;
  int old_start_index = 0, old_end_index = 0;
  float scroll_progress = 0.0f;

  if (paging_active) {
    // Handle active scroll transition
    if (this->page_scroll_duration_ > 0 && this->page_scroll_start_ > 0) {
      if (this->scroll_from_page_ >= total_pages) {
        // Old page no longer valid - cancel transition
        this->page_scroll_start_ = 0;
      } else {
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
    int max_trips_height = (display_count * this->font_->get_ascender()) + ((display_count - 1) * this->font_->get_descender());
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