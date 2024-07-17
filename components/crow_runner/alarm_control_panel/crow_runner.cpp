#include "crow_runner.h"
#include <utility>
#include <iostream>
#include <sstream>
#include <iomanip>
#include "esphome/components/alarm_control_panel/alarm_control_panel.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"


namespace esphome {
namespace crow_runner {

using namespace esphome::alarm_control_panel;

static const char *TAG = "crow_runner.alarm_control_panel";

///
// CrowRunnerAlarmControlPanel
///

CrowRunnerAlarmControlPanel::CrowRunnerAlarmControlPanel() {
    // noop
};

void CrowRunnerAlarmControlPanel::setup() {
    ESP_LOGCONFIG(TAG, "Setting up Crow Runner AlarmControlPanel '%s'...", name_.c_str());

    bus_.setup(pin_clock_, pin_data_);

    // switch (restore_mode_) {
    //   case ALARM_CONTROL_PANEL_ALWAYS_DISARMED:
    //     current_state_ = ACP_STATE_DISARMED;
    //     break;
    //   case ALARM_CONTROL_PANEL_RESTORE_DEFAULT_DISARMED: {
    //     uint8_t value;
    //     pref_ = global_preferences->make_preference<uint8_t>(get_object_id_hash());
    //     if (pref_.load(&value)) {
    //       current_state_ = static_cast<AlarmControlPanelState>(value);
    //     } else {
    //       current_state_ = ACP_STATE_DISARMED;
    //     }
    //     break;
    //   }
    // }
    // desired_state_ = current_state_;
}
void CrowRunnerAlarmControlPanel::loop() {
    // noop
}


void CrowRunnerAlarmControlPanel::dump_config() {
  ESP_LOGCONFIG(TAG, "CrowRunnerAlarmControlPanel:");
  ESP_LOGCONFIG(TAG, "  Current State: %s", LOG_STR_ARG(alarm_control_panel_state_to_string(current_state_)));
  ESP_LOGCONFIG(TAG, "  Number of Codes: %u", codes_.size());

  LOG_PIN("  Clock Pin: ", pin_clock_);
  LOG_PIN("  Data Pin: ", pin_data_);

  ESP_LOGCONFIG(TAG, "  Supported Features: %" PRIu32, get_supported_features());
}


uint32_t CrowRunnerAlarmControlPanel::get_supported_features() const {
  return ACP_FEAT_ARM_AWAY | ACP_FEAT_TRIGGER;
}


bool CrowRunnerAlarmControlPanel::is_code_valid_(optional<std::string> code) {
  if (!codes_.empty()) {
    if (code.has_value()) {
      ESP_LOGVV(TAG, "Checking code: %s", code.value().c_str());
      return (std::count(codes_.begin(), codes_.end(), code.value()) == 1);
    }
    ESP_LOGD(TAG, "No code provided");
    return false;
  }
  return true;
}


void CrowRunnerAlarmControlPanel::control(const AlarmControlPanelCall &call) {
  if (call.get_state()) {
    if (call.get_state() == ACP_STATE_ARMED_AWAY) {
      arm_(call.get_code(), ACP_STATE_ARMED_AWAY, 0);
    } else if (call.get_state() == ACP_STATE_DISARMED) {
      if (!is_code_valid_(call.get_code())) {
        ESP_LOGW(TAG, "Not disarming code doesn't match");
        return;
      }
      desired_state_ = ACP_STATE_DISARMED;
      publish_state(ACP_STATE_DISARMED);
    } else if (call.get_state() == ACP_STATE_TRIGGERED) {
      publish_state(ACP_STATE_TRIGGERED);
    } else if (call.get_state() == ACP_STATE_PENDING) {
      publish_state(ACP_STATE_PENDING);
    } else {
      ESP_LOGE(TAG, "State not yet implemented: %s",
               LOG_STR_ARG(alarm_control_panel_state_to_string(*call.get_state())));
    }
  }
}


void CrowRunnerAlarmControlPanel::arm_(optional<std::string> code, AlarmControlPanelState state,
                                     uint32_t delay) {
  if (current_state_ != ACP_STATE_DISARMED) {
    ESP_LOGW(TAG, "Cannot arm when not disarmed");
    return;
  }
  if (!is_code_valid_(std::move(code))) {
    ESP_LOGW(TAG, "Not arming code doesn't match");
    return;
  }
  desired_state_ = state;
  if (delay > 0) {
    publish_state(ACP_STATE_ARMING);
  } else {
    publish_state(state);
  }
}


}  // namespace empty_spi_sensor
}  // namespace esphome
