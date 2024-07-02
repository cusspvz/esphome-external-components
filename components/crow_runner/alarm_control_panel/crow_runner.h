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

        /** add a code
        *
        * @param code The code
        */
        void add_code(const std::string &code) { this->codes_.push_back(code); }

        /** set requires a code to arm
        *
        * @param code_to_arm The requires code to arm
        */
        void set_requires_code_to_arm(bool code_to_arm) { this->requires_code_to_arm_ = code_to_arm; }


    protected:
        bool requires_code_to_arm_ = false;

        // a list of codes
        std::vector<std::string> codes_;
};

}  // namespace crow_runner
}  // namespace esphome
