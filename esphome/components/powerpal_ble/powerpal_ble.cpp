#include "powerpal_ble.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/automation.h"
#include <sstream>
#include <iomanip>

#ifdef USE_ESP32
namespace esphome {
namespace powerpal_ble {

static const char *const TAG = "powerpal_ble";

void Powerpal::dump_config() {
  ESP_LOGCONFIG(TAG, "POWERPAL");
  LOG_SENSOR(" ", "Battery", this->battery_);
  LOG_SENSOR(" ", "Power", this->power_sensor_);
  LOG_SENSOR(" ", "Daily Energy", this->daily_energy_sensor_);
  LOG_SENSOR(" ", "Total Energy", this->energy_sensor_);
  }

void Powerpal::setup() {
  this->authenticated_ = false;
  this->pulse_multiplier_ = ((seconds_in_minute * this->reading_batch_size_[0]) / (this->pulses_per_kwh_ / kw_to_w_conversion));
  ESP_LOGI(TAG, "pulse_multiplier_: %f", this->pulse_multiplier_ );
}


std::string Powerpal::pkt_to_hex_(const uint8_t *data, uint16_t len) {
  char buf[64];
  memset(buf, 0, 64);
  for (int i = 0; i < len; i++)
    sprintf(&buf[i * 2], "%02x", data[i]);
  std::string ret = buf;
  return ret;
}


void Powerpal::decode_(const uint8_t *data, uint16_t length) {
  ESP_LOGD(TAG, "DEC(%d): 0x%s", length, this->pkt_to_hex_(data, length).c_str());
}

void Powerpal::parse_battery_(const uint8_t *data, uint16_t length) {
  ESP_LOGD(TAG, "Battery: DEC(%d): 0x%s", length, this->pkt_to_hex_(data, length).c_str());
  if (length == 1) {
    this->battery_->publish_state(data[0]);
  }
}

void Powerpal::parse_measurement_(const uint8_t *data, uint16_t length) {
  ESP_LOGD(TAG, "Meaurement: DEC(%d): 0x%s", length, this->pkt_to_hex_(data, length).c_str());
  if (length >= 6) {
    time_t unix_time = data[0];
    unix_time += (data[1] << 8);
    unix_time += (data[2] << 16);
    unix_time += (data[3] << 24);
    long int new_time = unix_time;
    //
        uint16_t pulses_within_interval = data[4];
    pulses_within_interval += data[5] << 8;
    
    // float total_kwh_within_interval = pulses_within_interval / this->pulses_per_kwh_;
    float avg_watts_within_interval = pulses_within_interval * this->pulse_multiplier_;
    
    ESP_LOGI(TAG, "Timestamp: %ld, Pulses: %d, Average Watts within interval: %f W, Daily Pulses: %d", unix_time, pulses_within_interval,
             avg_watts_within_interval, daily_pulses_);

    if (this->power_sensor_ != nullptr) {
      this->power_sensor_->publish_state(avg_watts_within_interval);
      //
    }

    if (this->cost_sensor_ != nullptr) {
      double mycost = (pulses_within_interval / this->pulses_per_kwh_) * this->energy_cost_;
      this->cost_sensor_->publish_state(mycost);
    }

    if (this->pulses_sensor_ != nullptr) {
       this->pulses_sensor_->publish_state(pulses_within_interval);
    }

    if (this->watt_hours_sensor_ != nullptr) {
      int mywatt_hrs = (uint32_t)roundf(pulses_within_interval * (this->pulses_per_kwh_ / kw_to_w_conversion));
       this->watt_hours_sensor_->publish_state(mywatt_hrs);
    }
     if (this->timestamp_sensor_ != nullptr) {
      //int mywatt_hrs = (uint32_t)roundf(pulses_within_interval * (this->pulses_per_kwh_ / kw_to_w_conversion));
       this->timestamp_sensor_->publish_state(new_time);
    }
    if (this->energy_sensor_ != nullptr) {
      this->total_pulses_ += pulses_within_interval;
      float energy = this->total_pulses_ / this->pulses_per_kwh_;
      this->energy_sensor_->publish_state(energy);
    }

    if (this->daily_energy_sensor_ != nullptr) {
      // even if new day, publish last measurement window before resetting
      this->daily_pulses_ += pulses_within_interval;
      float energy = this->daily_pulses_ / this->pulses_per_kwh_;
      this->daily_energy_sensor_->publish_state(energy);
      
      if (this->daily_pulses_sensor_ != nullptr) {
      this->daily_pulses_sensor_->publish_state(daily_pulses_);
      }
      // if esphome device has a valid time component set up, use that (preferred)
      // else, use the powerpal measurement timestamps
#ifdef USE_TIME
      auto *time_ = *this->time_;
      esphome::ESPTime date_of_measurement = time_->now();
      if (date_of_measurement.is_valid()) {
        if (this->day_of_last_measurement_ == 0) { this->day_of_last_measurement_ = date_of_measurement.day_of_year;}
        else if (this->day_of_last_measurement_ != date_of_measurement.day_of_year) {
          this->daily_pulses_ = 0;
          this->day_of_last_measurement_ = date_of_measurement.day_of_year;
        }
      } else {
        // if !date_of_measurement.is_valid(), user may have a bare "time:" in their yaml without a specific platform selected, so fallback to date of powerpal measurement
#else
        // avoid using ESPTime here so we don't need a time component in the config
        struct tm *date_of_measurement = ::localtime(&unix_time);
        // date_of_measurement.tm_yday + 1 because we are matching ESPTime day of year (1-366 instead of 0-365), which lets us catch a day_of_last_measurement_ of 0 as uninitialised
        if (this->day_of_last_measurement_ == 0) { this->day_of_last_measurement_ = date_of_measurement->tm_yday + 1 ;}
        else if (this->day_of_last_measurement_ != date_of_measurement->tm_yday + 1) {
          this->daily_pulses_ = 0;
          this->day_of_last_measurement_ = date_of_measurement->tm_yday + 1;
        }
#endif
#ifdef USE_TIME
      }
#endif
    }


  }
}

std::string Powerpal::uuid_to_device_id_(const uint8_t *data, uint16_t length) {
  const char* hexmap[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "a", "b", "c", "d", "e", "f"};
  std::string device_id;
  for (int i = length-1; i >= 0; i--) {
    device_id.append(hexmap[(data[i] & 0xF0) >> 4]);
    device_id.append(hexmap[data[i] & 0x0F]);
  }
  return device_id;
}

std::string Powerpal::serial_to_apikey_(const uint8_t *data, uint16_t length) {
  const char* hexmap[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "a", "b", "c", "d", "e", "f"};
  std::string api_key;
  for (int i = 0; i < length; i++) {
    if ( i == 4 || i == 6 || i == 8 || i == 10 ) {
      api_key.append("-");
    }
    api_key.append(hexmap[(data[i] & 0xF0) >> 4]);
    api_key.append(hexmap[data[i] & 0x0F]);
  }
  return api_key;
}


// Helper function to convert GATTC event enum to string for debugging
static const char* gattc_event_to_str(esp_gattc_cb_event_t event) {
  switch (event) {
    case ESP_GATTC_REG_EVT: return "ESP_GATTC_REG_EVT";
    case ESP_GATTC_READ_CHAR_EVT: return "ESP_GATTC_READ_CHAR_EVT";
    case ESP_GATTC_WRITE_CHAR_EVT: return "ESP_GATTC_WRITE_CHAR_EVT";
    case ESP_GATTC_CONNECT_EVT: return "ESP_GATTC_CONNECT_EVT";
    case ESP_GATTC_DISCONNECT_EVT: return "ESP_GATTC_DISCONNECT_EVT";
    case ESP_GATTC_SEARCH_CMPL_EVT: return "ESP_GATTC_SEARCH_CMPL_EVT";
    case ESP_GATTC_NOTIFY_EVT: return "ESP_GATTC_NOTIFY_EVT";
    case ESP_GATTC_OPEN_EVT: return "ESP_GATTC_OPEN_EVT";
    case ESP_GATTC_CFG_MTU_EVT: return "ESP_GATTC_CFG_MTU_EVT";
    case ESP_GATTC_SEARCH_RES_EVT: return "ESP_GATTC_SEARCH_RES_EVT";
    case ESP_GATTC_READ_DESCR_EVT: return "ESP_GATTC_READ_DESCR_EVT";
    case ESP_GATTC_WRITE_DESCR_EVT: return "ESP_GATTC_WRITE_DESCR_EVT";
    case ESP_GATTC_SRVC_CHG_EVT: return "ESP_GATTC_SRVC_CHG_EVT";
    case ESP_GATTC_ENC_CMPL_CB_EVT: return "ESP_GATTC_ENC_CMPL_CB_EVT";
    case ESP_GATTC_UNREG_EVT: return "ESP_GATTC_UNREG_EVT";
    case ESP_GATTC_CLOSE_EVT: return "ESP_GATTC_CLOSE_EVT";
    case ESP_GATTC_SET_ASSOC_EVT: return "ESP_GATTC_SET_ASSOC_EVT";
    case ESP_GATTC_GET_ADDR_LIST_EVT: return "ESP_GATTC_GET_ADDR_LIST_EVT";
    // Add more cases as needed for your platform
    default: {
      static char buf[32];
      snprintf(buf, sizeof(buf), "UNKNOWN_EVT_%d", event);
      return buf;
    }
  }
}

void Powerpal::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                   esp_ble_gattc_cb_param_t *param) {
  ESP_LOGD(TAG, "GATTC event: %d (%s)", event, gattc_event_to_str(event));
  switch (event) {
    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "BLE disconnected from Powerpal.");
      this->authenticated_ = false;
      break;
    }
    case ESP_GATTC_CONNECT_EVT: {
      ESP_LOGI(TAG, "BLE connected to Powerpal.");
      break;
    }
    case ESP_GATTC_OPEN_EVT: {
      // This event can occur before/after CONNECT_EVT, but is not used for state changes here.
      ESP_LOGD(TAG, "ESP_GATTC_OPEN_EVT received. This is normal and may occur outside connecting state.");
      break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      // auto *pairing_code_char_ = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID,
      // POWERPAL_CHARACTERISTIC_PAIRING_CODE_UUID); if (pairing_code_char_ == nullptr) {
      //   ESP_LOGE(TAG, "[%s] No Powerpal service or Pairing Code Characteristic found at device, not a POWERPAL..?",
      //             this->parent_->address_str().c_str());
      //   break;
      // } else {
      //   this->pairing_code_char_handle_ = pairing_code_char_->handle;
      // }

      // auto *reading_batch_size_char_ = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID,
      // POWERPAL_CHARACTERISTIC_READING_BATCH_SIZE_UUID); if (reading_batch_size_char_ == nullptr) {
      //   ESP_LOGE(TAG, "[%s] No Powerpal service or Reading Batch Size Characteristic found at device, not a
      //   POWERPAL..?",
      //             this->parent_->address_str().c_str());
      //   break;
      // } else {
      //   this->reading_batch_size_char_handle_ = reading_batch_size_char_->handle;
      // }

      // auto *measurement_char_ = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID,
      // POWERPAL_CHARACTERISTIC_MEASUREMENT_UUID); if (measurement_char_ == nullptr) {
      //   ESP_LOGE(TAG, "[%s] No Powerpal service or Measurement Characteristic found at device, not a POWERPAL..?",
      //             this->parent_->address_str().c_str());
      //   break;
      // } else {
      //   this->measurement_char_handle_ = measurement_char_->handle;
      // }

      // auto *uuid_char_ = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID,
      // POWERPAL_CHARACTERISTIC_UUID_UUID); if (uuid_char_ == nullptr) {
      //   ESP_LOGE(TAG, "[%s] No Powerpal service or Measurement Characteristic found at device, not a POWERPAL..?",
      //             this->parent_->address_str().c_str());
      //   break;
      // } else {
      //   this->uuid_char_handle_ = uuid_char_->handle;
      //   ESP_LOGE(TAG, "UUID HANDLE: %d",this->uuid_char_handle_);
      // }

      // auto *serial_char_ = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID,
      // POWERPAL_CHARACTERISTIC_SERIAL_UUID); if (serial_char_ == nullptr) {
      //   ESP_LOGE(TAG, "[%s] No Powerpal service or Measurement Characteristic found at device, not a POWERPAL..?",
      //             this->parent_->address_str().c_str());
      //   break;
      // } else {
      //   this->serial_number_char_handle_ = serial_char_->handle;
      //   ESP_LOGE(TAG, "SERIAL HANDLE: %d",this->serial_number_char_handle_);
      // }

      break;
    }
    case ESP_GATTC_READ_CHAR_EVT: {
      ESP_LOGD(TAG, "[%s] ESP_GATTC_READ_CHAR_EVT (Received READ)", this->parent_->address_str().c_str());
      if (param->read.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Error reading char at handle %d, status=%d", param->read.handle, param->read.status);
        break;
      }
      // reading batch size
      if (param->read.handle == this->reading_batch_size_char_handle_) {
        ESP_LOGD(TAG, "Recieved reading_batch_size read event");
        this->decode_(param->read.value, param->read.value_len);
        if (param->read.value_len == 4) {
          if (param->read.value[0] != this->reading_batch_size_[0]) {
            // reading batch size needs changing, so write
            auto status =
                esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                         this->reading_batch_size_char_handle_, sizeof(this->reading_batch_size_),
                                         this->reading_batch_size_, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
            if (status) {
              ESP_LOGW(TAG, "Error sending write request for batch_size, status=%d", status);
            }
          } else {
            // reading batch size is set correctly so subscribe to measurement notifications
            auto status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(),
                                                            this->measurement_char_handle_);
            if (status) {
              ESP_LOGW(TAG, "[%s] esp_ble_gattc_register_for_notify failed, status=%d",
                       this->parent_->address_str().c_str(), status);
            }
          }
        } else {
          // error, length should be 4
        }
        break;
      }

      // battery
      if (param->read.handle == this->battery_char_handle_) {
        ESP_LOGD(TAG, "Recieved battery read event");
        this->parse_battery_(param->read.value, param->read.value_len);
        break;
      }

      // firmware
      if (param->read.handle == this->firmware_char_handle_) {
        ESP_LOGD(TAG, "Recieved firmware read event");
        this->decode_(param->read.value, param->read.value_len);
        break;
      }

      // led sensitivity
      if (param->read.handle == this->led_sensitivity_char_handle_) {
        ESP_LOGD(TAG, "Recieved led sensitivity read event");
        this->decode_(param->read.value, param->read.value_len);
        break;
      }

      // serialNumber
      if (param->read.handle == this->serial_number_char_handle_) {
        ESP_LOGI(TAG, "Recieved uuid read event");
        this->powerpal_device_id_ = this->uuid_to_device_id_(param->read.value, param->read.value_len);
        ESP_LOGI(TAG, "Powerpal device id: %s", this->powerpal_device_id_.c_str());

        break;
      }

      // uuid
      if (param->read.handle == this->uuid_char_handle_) {
        ESP_LOGI(TAG, "Recieved serial_number read event");
        this->powerpal_apikey_ = this->serial_to_apikey_(param->read.value, param->read.value_len);
        ESP_LOGI(TAG, "Powerpal apikey: %s", this->powerpal_apikey_.c_str());

        break;
      }

      break;
    }

    case ESP_GATTC_WRITE_CHAR_EVT: {
      ESP_LOGD(TAG, "[%s] ESP_GATTC_WRITE_CHAR_EVT (Write confirmed)", this->parent_->address_str().c_str());
      if (param->write.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Error writing value to char at handle %d, status=%d", param->write.handle, param->write.status);
        break;
      }

      if (param->write.handle == this->pairing_code_char_handle_ && !this->authenticated_) {
        this->authenticated_ = true;

        // Add a short delay to allow BLE stack to settle before subscribing to notifications
        esphome::delay(100); // 100ms delay

        auto read_reading_batch_size_status =
            esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                    this->reading_batch_size_char_handle_, ESP_GATT_AUTH_REQ_NONE);
        if (read_reading_batch_size_status) {
          ESP_LOGW(TAG, "Error sending read request for reading batch size, status=%d", read_reading_batch_size_status);
        }

        if (!this->powerpal_apikey_.length()) {
          // read uuid (apikey)
          auto read_uuid_status = esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                                            this->uuid_char_handle_, ESP_GATT_AUTH_REQ_NONE);
          if (read_uuid_status) {
            ESP_LOGW(TAG, "Error sending read request for powerpal uuid, status=%d", read_uuid_status);
          }
        }
        if (!this->powerpal_device_id_.length()) {
          // read serial number (device id)
          auto read_serial_number_status = esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                                            this->serial_number_char_handle_, ESP_GATT_AUTH_REQ_NONE);
          if (read_serial_number_status) {
            ESP_LOGW(TAG, "Error sending read request for powerpal serial number, status=%d", read_serial_number_status);
          }
        }

        if (this->battery_ != nullptr) {
          // read battery
          auto read_battery_status = esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                                             this->battery_char_handle_, ESP_GATT_AUTH_REQ_NONE);
          if (read_battery_status) {
            ESP_LOGW(TAG, "Error sending read request for battery, status=%d", read_battery_status);
          }
          // Enable notifications for battery
          auto notify_battery_status = esp_ble_gattc_register_for_notify(
              this->parent_->get_gattc_if(), this->parent_->get_remote_bda(), this->battery_char_handle_);
          if (notify_battery_status) {
            ESP_LOGW(TAG, "[%s] esp_ble_gattc_register_for_notify failed, status=%d",
                     this->parent_->address_str().c_str(), notify_battery_status);
          }
        }

        // read firmware version
        auto read_firmware_status =
            esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                    this->firmware_char_handle_, ESP_GATT_AUTH_REQ_NONE);
        if (read_firmware_status) {
          ESP_LOGW(TAG, "Error sending read request for led sensitivity, status=%d", read_firmware_status);
        }

        // read led sensitivity
        auto read_led_sensitivity_status =
            esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                    this->led_sensitivity_char_handle_, ESP_GATT_AUTH_REQ_NONE);
        if (read_led_sensitivity_status) {
          ESP_LOGW(TAG, "Error sending read request for led sensitivity, status=%d", read_led_sensitivity_status);
        }

        break;
      }
      if (param->write.handle == this->reading_batch_size_char_handle_) {
        // reading batch size is now set correctly so subscribe to measurement notifications
        auto status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(),
                                                        this->measurement_char_handle_);
        if (status) {
          ESP_LOGW(TAG, "[%s] esp_ble_gattc_register_for_notify failed, status=%d",
                   this->parent_->address_str().c_str(), status);
        }
        break;
      }

      ESP_LOGW(TAG, "[%s] Missed all handle matches: %d",
               this->parent_->address_str().c_str(), param->write.handle);
      break;
    }  // ESP_GATTC_WRITE_CHAR_EVT

    case ESP_GATTC_NOTIFY_EVT: {
      ESP_LOGD(TAG, "[%s] Received Notification", this->parent_->address_str().c_str());

      // battery
      if (param->notify.handle == this->battery_char_handle_) {
        ESP_LOGD(TAG, "Recieved battery notify event");
        this->parse_battery_(param->notify.value, param->notify.value_len);
        break;
      }

      // measurement
      if (param->notify.handle == this->measurement_char_handle_) {
        ESP_LOGD(TAG, "Recieved measurement notify event");
        this->parse_measurement_(param->notify.value, param->notify.value_len);
        break;
      }
      // historical measurement
      if (param->notify.handle == this->measurement_access_char_handle_) {
        ESP_LOGD(TAG, "Received historical measurement notify event");
        this->parse_historical_measurement_(param->notify.value, param->notify.value_len);
        break;
      }
      break;  // registerForNotify
    }
    case ESP_GATTC_SEARCH_RES_EVT: {
      auto &sr = param->search_res;
      ESP_LOGD(TAG, "SEARCH_RES: char handle: %d", sr.handle);

      if (sr.uuid == POWERPAL_CHARACTERISTIC_PAIRING_CODE_UUID) {
        this->pairing_code_char_handle_ = sr.handle;
        ESP_LOGD(TAG, "Found pairing code characteristic handle: %d", sr.handle);
      } else if (sr.uuid == POWERPAL_CHARACTERISTIC_READING_BATCH_SIZE_UUID) {
        this->reading_batch_size_char_handle_ = sr.handle;
        ESP_LOGD(TAG, "Found reading batch size characteristic handle: %d", sr.handle);
      } else if (sr.uuid == POWERPAL_CHARACTERISTIC_MEASUREMENT_UUID) {
        this->measurement_char_handle_ = sr.handle;
        ESP_LOGD(TAG, "Found measurement characteristic handle: %d", sr.handle);
      } else if (sr.uuid == POWERPAL_CHARACTERISTIC_UUID_UUID) {
        this->uuid_char_handle_ = sr.handle;
        ESP_LOGD(TAG, "Found UUID characteristic handle: %d", sr.handle);
      } else if (sr.uuid == POWERPAL_CHARACTERISTIC_SERIAL_UUID) {
        this->serial_number_char_handle_ = sr.handle;
        ESP_LOGD(TAG, "Found serial number characteristic handle: %d", sr.handle);
      } else if (sr.uuid == POWERPAL_CHARACTERISTIC_BATTERY_UUID) {
        this->battery_char_handle_ = sr.handle;
        ESP_LOGD(TAG, "Found battery characteristic handle: %d", sr.handle);
      } else if (sr.uuid == POWERPAL_CHARACTERISTIC_FIRMWARE_UUID) {
        this->firmware_char_handle_ = sr.handle;
        ESP_LOGD(TAG, "Found firmware characteristic handle: %d", sr.handle);
      } else if (sr.uuid == POWERPAL_CHARACTERISTIC_LED_SENSITIVITY_UUID) {
        this->led_sensitivity_char_handle_ = sr.handle;
        ESP_LOGD(TAG, "Found LED sensitivity characteristic handle: %d", sr.handle);
      } else if (sr.uuid == POWERPAL_CHARACTERISTIC_MEASUREMENT_ACCESS_UUID) {
        this->measurement_access_char_handle_ = sr.handle;
        ESP_LOGD(TAG, "Found measurement access characteristic handle: %d", sr.handle);
      }
      // ...add more as needed...
      break;
    }
    default:
      ESP_LOGD(TAG, "Unhandled GATTC event: %d (%s)", event, gattc_event_to_str(event));
      break;
  }
}

void Powerpal::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    // This event is sent once authentication has completed
    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
      if (param->ble_security.auth_cmpl.success) {
        ESP_LOGI(TAG, "[%s] Writing pairing code to Powerpal", this->parent_->address_str().c_str());
        auto status = esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                               this->pairing_code_char_handle_, sizeof(this->pairing_code_),
                                               this->pairing_code_, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (status) {
          ESP_LOGW(TAG, "Error sending write request for pairing_code, status=%d", status);
        }
      }
      break;
    }
    default:
      break;
  }
}

// In your main loop or after receiving all historical data:
void Powerpal::loop() {
  // Only publish pending measurements, do NOT call poll_historical_if_configured_ automatically
  for (auto &m : pending_measurements_) {
    publish_measurement_with_time(this->power_sensor_, m.value, m.timestamp);
    // publish other sensors as needed
  }
  pending_measurements_.clear();
}

void Powerpal::trigger_manual_historical_polling(const std::string& start_str, const std::string& end_str) {
  ESP_LOGI(TAG, "Manual API/Web event: Triggering historical polling.");

  auto parse_datetime = [](const std::string& datetime) -> time_t {
    // If string is all digits, treat as unix timestamp
    if (!datetime.empty() && std::all_of(datetime.begin(), datetime.end(), ::isdigit)) {
      return static_cast<time_t>(std::stoll(datetime));
    }
    std::tm tm = {};
    std::istringstream ss(datetime);
    ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");
    if (ss.fail()) {
      ESP_LOGW(TAG, "Failed to parse datetime string: %s", datetime.c_str());
      return 0;
    }
    tm.tm_isdst = -1;
    return mktime(&tm);
  };

  time_t start = parse_datetime(start_str);
  time_t end = parse_datetime(end_str);

  this->historical_polled_ = false; // allow polling again if desired

  if (start > 0 && end > start) {
    ESP_LOGI(TAG, "Manual polling with custom range: start=%ld end=%ld", start, end);
    request_historical_measurements(start, end);
    historical_polled_ = true;
  } else {
    ESP_LOGI(TAG, "Manual polling: invalid range, skipping historical polling.");
  }
}

void Powerpal::request_historical_measurements(time_t start, time_t end) {
  // Prepare payload: start and end timestamps, little endian
  uint8_t payload[8];
  payload[0] = start & 0xFF;
  payload[1] = (start >> 8) & 0xFF;
  payload[2] = (start >> 16) & 0xFF;
  payload[3] = (start >> 24) & 0xFF;
  payload[4] = end & 0xFF;
  payload[5] = (end >> 8) & 0xFF;
  payload[6] = (end >> 16) & 0xFF;
  payload[7] = (end >> 24) & 0xFF;

  // Use connection MTU to optimize transmission
  size_t max_payload = this->get_mtu() - 3; // 3 bytes for ATT header
  if (sizeof(payload) <= max_payload) {
    esp_err_t err = esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                           this->measurement_access_char_handle_, sizeof(payload), payload,
                           ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to request historical measurements: %d", err);
    }
  } else {
    ESP_LOGW(TAG, "Payload larger than negotiated MTU (%d), implement chunking if needed.", this->get_mtu());
    // ...chunking logic if needed...
  }
}

void Powerpal::parse_historical_measurement_(const uint8_t *data, uint16_t length) {
  // Assume same format as parse_measurement_
  if (length >= 6) {
    time_t unix_time = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
    uint16_t pulses_within_interval = data[4] + (data[5] << 8);
    float avg_watts_within_interval = pulses_within_interval * this->pulse_multiplier_;
    ESP_LOGD(TAG, "Historical Measurement: timestamp=%ld pulses=%d avg_watts=%f", unix_time, pulses_within_interval, avg_watts_within_interval);
    // Store for later publishing
    pending_measurements_.push_back({avg_watts_within_interval, unix_time});
    // Optionally parse and store other sensors as needed
  }
}

void Powerpal::publish_measurement_with_time(sensor::Sensor *sensor, float value, time_t timestamp) {
  if (sensor != nullptr) {
    ESP_LOGD(TAG, "Publishing sensor value=%f at timestamp=%ld", value, timestamp);
    sensor->publish_state(value); // Replace with timestamped publish if available
    // If Home Assistant supports timestamped sensors, use appropriate API
  }
}

}  // namespace powerpal_ble
}  // namespace esphome

#endif
