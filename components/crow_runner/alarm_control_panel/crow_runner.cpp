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


std::string vector_to_hex_string(const std::vector<uint8_t>& data) {
    std::ostringstream oss;
    for (const auto& byte : data) {
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << ' ';
    }
    std::string hexString = oss.str();
    if (!hexString.empty())
        hexString.pop_back(); // Remove the trailing space
    return hexString;
}


// Message implementation
Message::Message(const BitVector& msg) {
    parse_message_(msg);
}


void Message::parse_message_(const BitVector& msg) {
    // Parse message based on its type
    if (msg.get_bit(63) == 0) {
        type_ = MessageType::ZoneReporting;

        reporting_.extra_zones = msg.get_bit(16);
        reporting_.zone_activated = false;
        reporting_.alarm_triggered = false;

        // check active zones
        for (unsigned int i = 0; i < 8; i++) {
            reporting_.active_zones[i] = msg.get_bit(i + 24);

            if (!reporting_.zone_activated && reporting_.active_zones[i] == 1){
                reporting_.zone_activated = true;
            }
        }

        // check alarm triggering zones
        for (unsigned int i = 0; i < 8; i++) {
            reporting_.alarm_trigger[i] = msg.get_bit(i + 32);

            if (!reporting_.alarm_triggered && reporting_.alarm_trigger[i] == 1){
                reporting_.alarm_triggered = true;
            }
        }
    } else {
        // Handle status change messages
        type_ = MessageType::StatusChange;

        // Example parsing for status message - update according to your protocol
        status_.is_arming = msg.get_bit(32);
        status_.is_armed = msg.get_bit(33);
        status_.is_partial = msg.get_bit(34);
        status_.is_triggered = msg.get_bit(35);
        status_.is_chime = msg.get_bit(36);

        // Determine the alarm state based on the parsed flags
        if (status_.is_triggered) {
            status_.state = AlarmState::Triggered;
        } else if (status_.is_armed && status_.is_partial) {
            status_.state = AlarmState::ArmedHome;
        } else if (status_.is_armed) {
            status_.state = AlarmState::ArmedAway;
        } else if (status_.is_arming && status_.is_partial) {
            status_.state = AlarmState::ArmingHome;
        } else if (status_.is_arming) {
            status_.state = AlarmState::ArmingAway;
        } else if (status_.is_chime) {
            status_.state = AlarmState::Chime;
        } else {
            status_.state = AlarmState::Disarmed;
        }
    }
}

// Message Message::create_arm_away_message() {
//     // Create a bitset for arming away
//     std::bitset<72> msg;
//     // Set the appropriate bits for arming away
//     // This is just a placeholder - update with actual protocol
//     msg.set(63);  // Mark as status message
//     msg.set(32);  // is_arming
//     return Message(msg);
// }

// Message Message::create_arm_home_message() {
//     std::bitset<72> msg;
//     // Set the appropriate bits for arming home
//     msg.set(63);  // Mark as status message
//     msg.set(32);  // is_arming
//     msg.set(34);  // is_partial
//     return Message(msg);
// }

// Message Message::create_disarm_message(const std::string& code) {
//     std::bitset<72> msg;
//     // Set the bits for disarming with the given code
//     msg.set(63);  // Mark as status message

//     // Encode the code into the message
//     // This is a placeholder - update with actual protocol
//     for (size_t i = 0; i < code.length() && i < 8; i++) {
//         int digit = code[i] - '0';
//         for (int bit = 0; bit < 4; bit++) {
//             if (digit & (1 << bit))
//                 msg.set(40 + (i * 4) + bit);
//         }
//     }

//     return Message(msg);
// }

///
// Bus implementation
///

void Bus::setup(InternalGPIOPin *pin_clock, InternalGPIOPin *pin_data) {
    ESP_LOGD(TAG, "Setting up Bus");

    pin_clock->setup();
    pin_data->setup();

    pin_clock->pin_mode(gpio::FLAG_INPUT);
    pin_data->pin_mode(gpio::FLAG_INPUT);

    // Save pins
    pin_clock_ = pin_clock;
    pin_data_ = pin_data;
    pin_data_isr_ = pin_data->to_isr();

    // Start in the WaitingForData state
    set_state(BusState::WaitingForData);
}

void Bus::loop() {
    // check any pending receiving queue
    if (!receiving_queue_.empty()) {
        BitVector msgRaw = receiving_queue_.front();
        receiving_queue_.pop_front();

        // Debugging
        if (debug_mode_) {
            ESP_LOGD(TAG, "Received New Message: %s", vector_to_hex_string(msgRaw.get_data()).c_str());
        }

        // Create a Message object
        Message message(msgRaw);

        // Call the receiver with the message if one is attached
        if (receiver_) {
            receiver_(&message);
        }
    }
}


float Bus::get_bitrate() const {
    return bitrate_;
}

const char* BusStateToString(BusState state) {
    switch (state) {
        case BusState::Idle:
            return "Idle";
        case BusState::WaitingForData:
            return "WaitingForData";
        case BusState::ReceivingMessage:
            return "ReceivingMessage";
        case BusState::SendingMessage:
            return "SendingMessage";
        default:
            return "Unknown";
    }
}

void Bus::set_state(BusState state) {
    if (debug_mode_) {
        ESP_LOGD(TAG, "Bus state changed from %s to %s", BusStateToString(state_), BusStateToString(state));
    }

    // Logic to dissassemble the previous state
    switch (state_) {
        case BusState::Idle:
            pin_clock_->detach_interrupt();
            break;
        case BusState::WaitingForData:
            pin_data_->detach_interrupt();
            break;
        case BusState::ReceivingMessage:
            break;
        case BusState::SendingMessage:
            pin_data_->pin_mode(gpio::FLAG_INPUT);
            pin_clock_->detach_interrupt();
            break;
    }

    // Set new state
    state_ = state;

    // Logic to setup the previous state
    switch (state) {
        case BusState::Idle:
            pin_clock_->detach_interrupt();
            break;
        case BusState::WaitingForData:
            pin_data_->attach_interrupt(Bus::data_falling_interrupt, this, gpio::INTERRUPT_FALLING_EDGE);
            break;
        case BusState::ReceivingMessage:
            receiving_buffer_.clear();
            pin_clock_->attach_interrupt(Bus::clock_falling_interrupt, this, gpio::INTERRUPT_FALLING_EDGE);
            break;
        case BusState::SendingMessage:
            pin_data_->pin_mode(gpio::FLAG_OUTPUT);
            pin_clock_->detach_interrupt();
            pin_clock_->attach_interrupt(Bus::clock_rising_interrupt, this, gpio::INTERRUPT_RISING_EDGE);
            break;
    }
}

// Compute bitrate
void Bus::tick_bitrate() {
    bitrate_ticks_++;

    uint32_t now = millis();

    // determine if we need to measure and update the bitrate value
    if (now - bitrate_last_measurement_time_ >= 1000) {
        bitrate_ = bitrate_ticks_ / ((now - bitrate_last_measurement_time_) / 1000.0f);
        bitrate_ticks_ = 0;
        bitrate_last_measurement_time_ = now;

        ESP_LOGD(TAG, "Bitrate %f", bitrate_);
    }
}

// Detect whenever we're receiving a message
void Bus::data_falling_interrupt(Bus *arg) {
    bool data_bit = false;

    // first 0 got in, changing the state and
    arg->set_state(BusState::ReceivingMessage);

    // write bit to buffer
    arg->receiving_buffer_.write_bit(data_bit);
}

// When the clock is falling, we READ data from the data pin
void Bus::clock_falling_interrupt(Bus *arg) {
    // Read data pin state
    bool data_bit = arg->pin_data_isr_.digital_read();

    arg->tick_bitrate();

    // Check if we're out of bounderies before writing bit to buffer
    if (!arg->receiving_buffer_.is_writeable()) {
        if (arg->debug_mode_) {
            ESP_LOGD(TAG, "No valid message has been found...");
            ESP_LOGD(TAG, "Debugging buffer data: %s", vector_to_hex_string(arg->receiving_buffer_.get_data()).c_str());
        }
        arg->set_state(BusState::WaitingForData);
        return;
    }

    // write bit to buffer
    arg->receiving_buffer_.write_bit(data_bit);

    if (arg->receiving_buffer_.written_bits_so_far() % 8 == 0) {
        ESP_LOGD(TAG, "checking");

        // Check if theres a valid message
        size_t written_bytes = arg->receiving_buffer_.written_bytes_so_far();

        // Don't allow to proceed in case the first byte is not a boundary
        if (written_bytes == 1) {
            uint8_t first_byte = arg->receiving_buffer_.get_byte(0);

            if (first_byte != BOUNDARY) {
                arg->set_state(BusState::WaitingForData);
            }

            return; // continue to receive the message
        } else if (written_bytes < 3) {
            return; // continue to receive the message
        } else {
            // more than 3 bytes
            uint8_t last_byte = arg->receiving_buffer_.get_byte(written_bytes - 1);

            if (last_byte != BOUNDARY) {
                return; // continue to receive the message
            }
        }

        //
        // Potential message found (within boundaries)
        //

        // Copy the message into a new buffer and add it to the receiving queue
        BitVector binary_message = arg->receiving_buffer_.clone(8, (written_bytes - 1) * 8);
        arg->receiving_queue_.push_back(binary_message);

        // set the state back to waiting for data
        arg->set_state(BusState::WaitingForData);
    }
}

// When the clock is rising, we WRITE data from the data pin
void Bus::clock_rising_interrupt(Bus *arg) {
    if (arg->state_ != BusState::SendingMessage) {
        return;
    }

    // // Check if there's anything to send
    // if (arg->sending_buffers_queue_.empty()) {
    //     // No messages to send, go back to waiting for data
    //     arg->set_state(BusState::WaitingForData);
    //     return;
    // }

    // // Get the current message to send
    // std::vector<bool> &current_message = arg->sending_buffers_queue_.front();

    // // Send the current bit
    // bool bit_to_send = current_message.front();
    // arg->pin_data_isr_.digital_write(bit_to_send);

    // // Remove the sent bit from the current message
    // current_message.erase(current_message.begin());


    // // Check if the whole message has been sent
    //  if (current_message.empty()) {
    //      // Remove the sent message from the queue
    //     arg->sending_buffers_queue_.erase(arg->sending_buffers_queue_.begin());

    //     ESP_LOGD(TAG, "Message sent. Remaining messages in queue: %s", String(arg->sending_buffers_queue_.size()));

    //     // If there are no more messages to send, go back to waiting for data
    //     if (arg->sending_buffers_queue_.empty()) {
    //         arg->set_state(BusState::WaitingForData);
    //     }
    // }
}


void Bus::send_message(Message *message) {
    // Implement message sending
    // This would queue up a message to be sent
}

void Bus::send_keypad_button(uint8_t button_code) {
    // Implement keypad button sending
}

void Bus::send_disarm_code(const std::string& code) {
    // Implement disarm code sending
}

///
// CrowRunnerAlarmControlPanel
///

CrowRunnerAlarmControlPanel::CrowRunnerAlarmControlPanel() {
    // noop
}

void CrowRunnerAlarmControlPanel::setup() {
    ESP_LOGCONFIG(TAG, "Setting up Crow Runner AlarmControlPanel '%s'...", name_.c_str());
    bus_.setup(pin_clock_, pin_data_);
}

void CrowRunnerAlarmControlPanel::loop() {
    bus_.loop();
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

// bool CrowRunnerAlarmControlPanel::is_code_valid_(optional<std::string> code) {
//     if (!codes_.empty()) {
//         if (code.has_value()) {
//             ESP_LOGVV(TAG, "Checking code: %s", code.value().c_str());
//             return (std::count(codes_.begin(), codes_.end(), code.value()) == 1);
//         }
//         ESP_LOGD(TAG, "No code provided");
//         return false;
//     }
//     return true;
// }

void CrowRunnerAlarmControlPanel::control(const AlarmControlPanelCall &call) {
    // if (call.get_state()) {
    //     if (call.get_state() == ACP_STATE_ARMED_AWAY) {
    //         arm_(call.get_code(), ACP_STATE_ARMED_AWAY, 0);
    //     } else if (call.get_state() == ACP_STATE_DISARMED) {
    //         if (!is_code_valid_(call.get_code())) {
    //             ESP_LOGW(TAG, "Not disarming code doesn't match");
    //             return;
    //         }
    //         desired_state_ = ACP_STATE_DISARMED;
    //         publish_state(ACP_STATE_DISARMED);
    //     } else if (call.get_state() == ACP_STATE_TRIGGERED) {
    //         publish_state(ACP_STATE_TRIGGERED);
    //     } else if (call.get_state() == ACP_STATE_PENDING) {
    //         publish_state(ACP_STATE_PENDING);
    //     } else {
    //         ESP_LOGE(TAG, "State not yet implemented: %s",
    //                LOG_STR_ARG(alarm_control_panel_state_to_string(*call.get_state())));
    //     }
    // }
}

void CrowRunnerAlarmControlPanel::arm_(optional<std::string> code, AlarmControlPanelState state, uint32_t delay) {
    // if (current_state_ != ACP_STATE_DISARMED) {
    //     ESP_LOGW(TAG, "Cannot arm when not disarmed");
    //     return;
    // }
    // // if (!is_code_valid_(std::move(code))) {
    // //     ESP_LOGW(TAG, "Not arming code doesn't match");
    // //     return;
    // // }
    // desired_state_ = state;
    // if (delay > 0) {
    //     publish_state(ACP_STATE_ARMING);
    // } else {
    //     publish_state(state);
    // }
}


}  // namespace crow_runner
}  // namespace esphome
