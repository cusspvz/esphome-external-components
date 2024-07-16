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
    hexString.pop_back(); // Remove the trailing space
    return hexString;
}

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

///
// CrowRunnerBus
///

void CrowRunnerBus::setup(InternalGPIOPin *pin_clock, InternalGPIOPin *pin_data) {
    ESP_LOGD(TAG, "Setting up CrowRunnerBus");

    pin_clock->setup();
    pin_data->setup();

    pin_clock->pin_mode(gpio::FLAG_INPUT);
    pin_data->pin_mode(gpio::FLAG_INPUT);

    // Save pins
    pin_clock_ = pin_clock;
    pin_data_ = pin_data;
    pin_data_isr_ = pin_data->to_isr();

    // Start in the WaitingForData state
    set_state(CrowRunnerBusState::WaitingForData);

    // Attach clock interrupts
    // pin_clock_->attach_interrupt(CrowRunnerBus::clock_rising_interrupt, this, gpio::INTERRUPT_RISING_EDGE);
    pin_clock_->attach_interrupt(CrowRunnerBus::clock_falling_interrupt, this, gpio::INTERRUPT_FALLING_EDGE);
}


const char* CrowRunnerBusStateToString(CrowRunnerBusState state) {
    switch (state) {
        case CrowRunnerBusState::Idle:
            return "Idle";
        case CrowRunnerBusState::WaitingForData:
            return "WaitingForData";
        case CrowRunnerBusState::ReceivingMessage:
            return "ReceivingMessage";
        case CrowRunnerBusState::SendingMessage:
            return "SendingMessage";
        default:
            return "Unknown";
    }
}

void CrowRunnerBus::set_state(CrowRunnerBusState state) {
    // ESP_LOGD(TAG, "CrowRunnerBus state changed from %s to %s", CrowRunnerBusStateToString(state_), CrowRunnerBusStateToString(state));

    // Logic to dissassemble the previous state
    switch (state_) {
        case CrowRunnerBusState::Idle:
            break;
        case CrowRunnerBusState::WaitingForData:
            break;
        case CrowRunnerBusState::ReceivingMessage:
            receiving_buffer_.clear();
            break;
        case CrowRunnerBusState::SendingMessage:
            pin_data_->pin_mode(gpio::FLAG_INPUT);
            break;

    }

    // Set new state
    state_ = state;

    // Logic to setup the previous state
    switch (state) {
        case CrowRunnerBusState::Idle:
            break;
        case CrowRunnerBusState::WaitingForData:
            break;
        case CrowRunnerBusState::ReceivingMessage:
            break;
        case CrowRunnerBusState::SendingMessage:
            pin_data_->pin_mode(gpio::FLAG_OUTPUT);
            break;
    }
}

void CrowRunnerBus::process_receiving_buffer_() {

    // Check if theres a valid message
    size_t written_bytes = receiving_buffer_.written_bytes_so_far();

    // Don't allow to proceed in case the first byte is not a boundary
    if (written_bytes == 1) {
        uint8_t first_byte = receiving_buffer_.get_byte(0);

        ESP_LOGD(TAG, "pos 0 FIRST BYTE: %i", first_byte);

        if (first_byte != BOUNDARY) {
            set_state(CrowRunnerBusState::WaitingForData);
        }

        return; // continue to receive the message
    } else if (written_bytes < 3) {
        return; // continue to receive the message
    } else {
        // more than 3 bytes
        uint8_t last_byte = receiving_buffer_.get_byte(written_bytes - 1);

        ESP_LOGD(TAG, "pos %i LAST BYTE: %i", written_bytes, last_byte);

        if (last_byte != BOUNDARY) {
            return; // continue to receive the message
        }
    }

    //
    // Potential message found (within boundaries)
    //

    // Copy the message into a new buffer
    BitVector binary_message = receiving_buffer_.clone(8, (written_bytes - 1) * 8);

    // Debugging
    ESP_LOGD(TAG, "NEW MESSAGE: %s", vector_to_hex_string(binary_message.get_data()).c_str());

    // set the state back to waiting for data
    set_state(CrowRunnerBusState::WaitingForData);

    // TODO: pass it on to the message parser and then to the receiver

    // if (receiver_) {
    //     CrowRunnerBusMessage parsed_message = CrowRunnerBusMessage(&binary_message);

    //     // send it to the message receiver
    //     receiver_(&parsed_message);
    // }
}

// When the clock is falling, we READ data from the data pin
void CrowRunnerBus::clock_falling_interrupt(CrowRunnerBus *arg) {
    // Read data pin state
    bool data_bit = arg->pin_data_isr_.digital_read();

    // Initialization logic
    if (arg->state_ != CrowRunnerBusState::ReceivingMessage) {
        if (data_bit == 1) return;
        arg->set_state(CrowRunnerBusState::ReceivingMessage);
    }

    // write bit to buffer
    arg->receiving_buffer_.write_bit(data_bit);

    // Check if we're out of bounderies
    if (!arg->receiving_buffer_.is_writeable()) {
        ESP_LOGD(TAG, "No valid message has been found...");

        // Debugging
        ESP_LOGD(TAG, "Debugging buffer data: %s", vector_to_hex_string(arg->receiving_buffer_.get_data()).c_str());

        arg->set_state(CrowRunnerBusState::WaitingForData);
        return;
    }

    if (arg->receiving_buffer_.written_bits_so_far() % 8 == 0) {
        arg->process_receiving_buffer_();
    }
}

// When the clock is rising, we WRITE data from the data pin
void CrowRunnerBus::clock_rising_interrupt(CrowRunnerBus *arg) {
    if (arg->state_ != CrowRunnerBusState::SendingMessage) {
        return;
    }

    // // Check if there's anything to send
    // if (arg->sending_buffers_queue_.empty()) {
    //     // No messages to send, go back to waiting for data
    //     arg->set_state(CrowRunnerBusState::WaitingForData);
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
    //         arg->set_state(CrowRunnerBusState::WaitingForData);
    //     }
    // }
}

CrowRunnerBusMessage::CrowRunnerBusMessage(std::bitset<72> *msg) {
    // parse message
    if (msg->test(63) == 0) {
        type = CrowRunnerBusMessageType::ZoneReporting;
        CrowRunnerBusMessageReporting reporting {};

        reporting.extra_zones = msg->test(16);

        // check active zones
        for (unsigned int i = 0; i < 8; i++) {
            reporting.active_zones[i] = msg->test(i + 24);

            if (!reporting.zone_activated && reporting.active_zones[i] == 1){
                reporting.zone_activated = 1;
            }
        }

        // check alarm triggering zones
        for (unsigned int i = 0; i < 8; i++) {
            reporting.alarm_trigger[i] = msg->test(i + 32);

            if (!reporting.alarm_triggered && reporting.alarm_trigger[i] == 1){
                reporting.alarm_triggered = 1;
            }
        }

        reporting = reporting;
    }
}

}  // namespace empty_spi_sensor
}  // namespace esphome
