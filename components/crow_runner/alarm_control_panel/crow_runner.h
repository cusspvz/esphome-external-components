#pragma once
#include <bitset>
#include <deque>
#include <vector>
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/defines.h"
#include "esphome/components/alarm_control_panel/alarm_control_panel.h"
#include "bit_vector.h"

namespace esphome {
namespace crow_runner {

// Based on my reverse engineering, I've confirmed the following information:
// - All the messages have the boundary of 0b10000001 before and after the actual message
// - The message length can vary
// - Some messages are emitted by the alarm, others from the keypad - I don't know yet how to distinguish both
//
const uint8_t BOUNDARY = 0b01111110;
const uint8_t BOUNDARY_SIZE_IN_BITS = 8;

enum class AlarmState {
    Disarmed = 0,
    ArmedAway = 1,
    ArmedHome = 2,
    Triggered = 3,
    Chime = 4,
    ArmingAway = 5,
    ArmingHome = 6
};

enum class MessageType {
    Unknown,
    StatusChange,
    ZoneReporting
};

struct MessageReporting {
    bool extra_zones;
    bool zone_activated;
    bool alarm_triggered;
    std::bitset<8> active_zones;
    std::bitset<8> alarm_trigger;
};

struct MessageStatus {
    AlarmState state;
    bool is_arming;
    bool is_armed;
    bool is_partial;
    bool is_triggered;
    bool is_chime;
};

class Message {
public:
    Message(const BitVector& msg);  // Fix constructor name

    MessageType get_type() const { return type_; }
    MessageReporting get_reporting() const { return reporting_; }
    MessageStatus get_status() const { return status_; }

    // // Add utility methods
    // static Message create_arm_away_message();
    // static Message create_arm_home_message();
    // static Message create_disarm_message(const std::string& code);

private:
    MessageType type_ = MessageType::Unknown;
    MessageReporting reporting_;
    MessageStatus status_;

    void parse_message_(const BitVector& msg);
};


enum class BusState {
    Idle,
    WaitingForData,
    ReceivingMessage,
    SendingMessage
};

class Bus {
    public:
        void setup(InternalGPIOPin *pin_clock, InternalGPIOPin *pin_data);
        void loop();
        static void IRAM_ATTR clock_falling_interrupt(Bus *arg);
        static void IRAM_ATTR clock_rising_interrupt(Bus *arg);
        void send_message(Message *message);
        void set_state(BusState state);

        void attach_receiver(void (*receiver)(Message* msg)) { this->receiver_ = receiver; }
        void detach_receiver() { this->receiver_ = nullptr; }

        void send_keypad_button(uint8_t button_code);
        void send_disarm_code(const std::string& code);
        bool is_busy() const { return state_ != BusState::Idle; }
        void set_debug_mode(bool enable) { debug_mode_ = enable; }

    protected:
        // data message receiver
        void (*receiver_)(Message* msg) = nullptr;
        // bool debug_mode_ = false;
        bool debug_mode_ = true;

        BusState state_ = BusState::Idle;
        InternalGPIOPin *pin_clock_;
        InternalGPIOPin *pin_data_;
        ISRInternalGPIOPin pin_data_isr_; // It is faster to access through ISR

        BitVector receiving_buffer_ = BitVector(128 + (BOUNDARY_SIZE_IN_BITS * 2));
        std::deque<BitVector> receiving_queue_;
        std::deque<BitVector> sending_queue_;
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

        void register_zone_callback(std::function<void(uint8_t zone, bool active)> callback);
        void set_report_zones(bool report) { report_zones_ = report; }

    protected:
        InternalGPIOPin *pin_clock_;
        InternalGPIOPin *pin_data_;
        Bus bus_;

        bool report_zones_ = true;
        std::function<void(uint8_t zone, bool active)> zone_callback_ = nullptr;

        void control(const alarm_control_panel::AlarmControlPanelCall &call) override;
        bool is_code_valid_(optional<std::string> code);
        void arm_(optional<std::string> code, alarm_control_panel::AlarmControlPanelState state, uint32_t delay);

        std::vector<std::string> codes_;
};


}  // namespace crow_runner
}  // namespace esphome
