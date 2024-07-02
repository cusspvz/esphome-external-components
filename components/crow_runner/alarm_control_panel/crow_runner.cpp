#include "esphome/core/log.h"
#include "crow_runner.h"

namespace esphome {
namespace crow_runner {

static const char *TAG = "crow_runner.alarm_control_panel";

CrowRunnerAlarmControlPanel::CrowRunnerAlarmControlPanel(){
    // noop
};

void CrowRunnerAlarmControlPanel::setup() {
    ESP_LOGCONFIG(TAG, "Setting up Crow Runner AlarmControlPanel '%s'...", this->name_.c_str());
    // switch (this->restore_mode_) {
    //   case ALARM_CONTROL_PANEL_ALWAYS_DISARMED:
    //     this->current_state_ = ACP_STATE_DISARMED;
    //     break;
    //   case ALARM_CONTROL_PANEL_RESTORE_DEFAULT_DISARMED: {
    //     uint8_t value;
    //     this->pref_ = global_preferences->make_preference<uint8_t>(this->get_object_id_hash());
    //     if (this->pref_.load(&value)) {
    //       this->current_state_ = static_cast<alarm_control_panel::AlarmControlPanelState>(value);
    //     } else {
    //       this->current_state_ = ACP_STATE_DISARMED;
    //     }
    //     break;
    //   }
    // }
    // this->desired_state_ = this->current_state_;
}

void CrowRunnerAlarmControlPanel::loop() {

}

// void CrowRunnerAlarmControlPanel::dump_config(){
//     ESP_LOGCONFIG(TAG, "Empty SPI sensor");
// }

}  // namespace empty_spi_sensor
}  // namespace esphome
