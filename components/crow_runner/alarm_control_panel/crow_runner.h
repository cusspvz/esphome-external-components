#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"

#include "esphome/components/alarm_control_panel/alarm_control_panel.h"

namespace esphome {
namespace crow_runner {

class CrowRunnerAlarmControlPanel : public alarm_control_panel::AlarmControlPanel, public Component {
    public:
        CrowRunnerAlarmControlPanel();
        void setup() override;
        void loop() override;
        void dump_config() override;
        uint32_t get_supported_features() const override;

        bool get_requires_code() const override { return true; }
        bool get_requires_code_to_arm() const override { return false; }

        /** add a code
        *
        * @param code The code
        */
        void add_code(const std::string &code) { this->codes_.push_back(code); }

    protected:
        void control(const alarm_control_panel::AlarmControlPanelCall &call) override;
        bool is_code_valid_(optional<std::string> code);
        void arm_(optional<std::string> code, alarm_control_panel::AlarmControlPanelState state, uint32_t delay);

        std::vector<std::string> codes_;
};

}  // namespace crow_runner
}  // namespace esphome
