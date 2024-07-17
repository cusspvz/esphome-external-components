#pragma once
#include <bitset>
#include <deque>
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/defines.h"
#include "esphome/components/alarm_control_panel/alarm_control_panel.h"
#include "bit_vector.h"
#include "crow_runner_bus.h"

namespace esphome {
namespace crow_runner {

// Based on my reverse engineering, I've confirmed the following information:
// - All the messages have the boundary of 0b10000001 before and after the actual message
// - The message length can vary
// - Some messages are emitted by the alarm, others from the keypad - I don't know yet how to distinguish both
//
const uint8_t BOUNDARY = 0b01111110;
const uint8_t BOUNDARY_SIZE_IN_BITS = 8;

class CrowRunnerAlarmControlPanel : public alarm_control_panel::AlarmControlPanel, public Component {
    public:
        CrowRunnerAlarmControlPanel();
        void setup() override;
        void dump_config() override;
        void loop() override;

        uint32_t get_supported_features() const override;
        bool get_requires_code() const override { return true; }
        bool get_requires_code_to_arm() const override { return false; }

        void set_pin_clock(InternalGPIOPin *pin) { pin_clock_ = pin; }
        void set_pin_data(InternalGPIOPin *pin) { pin_data_ = pin; }
        void add_code(const std::string &code) { this->codes_.push_back(code); }

    protected:
        InternalGPIOPin *pin_clock_;
        InternalGPIOPin *pin_data_;
        CrowRunnerBus bus_;
        void control(const alarm_control_panel::AlarmControlPanelCall &call) override;
        bool is_code_valid_(optional<std::string> code);
        void arm_(optional<std::string> code, alarm_control_panel::AlarmControlPanelState state, uint32_t delay);

        std::vector<std::string> codes_;
};


}  // namespace crow_runner
}  // namespace esphome
