#pragma once
#include <bitset>
#include <deque>
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/defines.h"
#include "esphome/components/alarm_control_panel/alarm_control_panel.h"

namespace esphome {
namespace crow_runner {

enum class CrowRunnerBusMessageType {
    Unknown,
    StatusChange,
    ZoneReporting
};

struct CrowRunnerBusMessageReporting {
    bool extra_zones;
    bool zone_activated;
    bool alarm_triggered;
    std::bitset<8> active_zones;
    std::bitset<8> alarm_trigger;
};

struct CrowRunnerBusMessage {
    public:
        CrowRunnerBusMessage(std::bitset<72> *msg);

        CrowRunnerBusMessageType getType() const { return type; }
        CrowRunnerBusMessageReporting getReporting() const { return reporting; }

    private:
    CrowRunnerBusMessageType type = CrowRunnerBusMessageType::Unknown;
        CrowRunnerBusMessageReporting reporting;
};

enum class CrowRunnerBusState {
    Idle,
    WaitingForData,
    ReceivingMessage,
    SendingMessage
};


class CrowRunnerBus {
    public:
        void setup(InternalGPIOPin *pin_clock, InternalGPIOPin *pin_data);
        static void IRAM_ATTR waiting_for_data_interrupt(CrowRunnerBus *arg);
        static void IRAM_ATTR receiving_message_interrupt(CrowRunnerBus *arg);
        static void IRAM_ATTR sending_message_interrupt(CrowRunnerBus *arg);
        void send_message(CrowRunnerBusMessage *message);
        void set_state(CrowRunnerBusState state);

        void attach_receiver(void (*receiver)(CrowRunnerBusMessage* msg)) { this->receiver_ = receiver; }
        void detach_receiver() { this->receiver_ = nullptr; }

    protected:
        const uint8_t boundary_length = 8;
        void process_received_message_();

        CrowRunnerBusState state_ = CrowRunnerBusState::Idle;
        InternalGPIOPin *pin_clock_;
        InternalGPIOPin *pin_data_;
        ISRInternalGPIOPin pin_data_isr_; // It is faster to access through ISR

        // receiving vars
        std::deque<bool> receiving_buffer_;
        const int8_t receiving_buffer_max_size_ = 128;
        uint8_t receiving_consecutive_ones_ = 128;

        // data message receiver
        void (*receiver_)(CrowRunnerBusMessage* msg) = nullptr;

        // sending vars
        std::vector<std::vector<bool>> sending_buffers_queue_;
};

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
