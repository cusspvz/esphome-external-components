#include "crow_runner.h"
#include <utility>
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
    ESP_LOGCONFIG(TAG, "Setting up Crow Runner AlarmControlPanel '%s'...", this->name_.c_str());

    this->bus_.setup(this->pin_clock_, this->pin_data_);

    // switch (this->restore_mode_) {
    //   case ALARM_CONTROL_PANEL_ALWAYS_DISARMED:
    //     this->current_state_ = ACP_STATE_DISARMED;
    //     break;
    //   case ALARM_CONTROL_PANEL_RESTORE_DEFAULT_DISARMED: {
    //     uint8_t value;
    //     this->pref_ = global_preferences->make_preference<uint8_t>(this->get_object_id_hash());
    //     if (this->pref_.load(&value)) {
    //       this->current_state_ = static_cast<AlarmControlPanelState>(value);
    //     } else {
    //       this->current_state_ = ACP_STATE_DISARMED;
    //     }
    //     break;
    //   }
    // }
    // this->desired_state_ = this->current_state_;
}
void CrowRunnerAlarmControlPanel::loop() {
    // noop
}


void CrowRunnerAlarmControlPanel::dump_config() {
  ESP_LOGCONFIG(TAG, "CrowRunnerAlarmControlPanel:");
  ESP_LOGCONFIG(TAG, "  Current State: %s", LOG_STR_ARG(alarm_control_panel_state_to_string(this->current_state_)));
  ESP_LOGCONFIG(TAG, "  Number of Codes: %u", this->codes_.size());

  LOG_PIN("  Clock Pin: ", this->pin_clock_);
  LOG_PIN("  Data Pin: ", this->pin_data_);

  ESP_LOGCONFIG(TAG, "  Supported Features: %" PRIu32, this->get_supported_features());
}


uint32_t CrowRunnerAlarmControlPanel::get_supported_features() const {
  return ACP_FEAT_ARM_AWAY | ACP_FEAT_TRIGGER;
}


bool CrowRunnerAlarmControlPanel::is_code_valid_(optional<std::string> code) {
  if (!this->codes_.empty()) {
    if (code.has_value()) {
      ESP_LOGVV(TAG, "Checking code: %s", code.value().c_str());
      return (std::count(this->codes_.begin(), this->codes_.end(), code.value()) == 1);
    }
    ESP_LOGD(TAG, "No code provided");
    return false;
  }
  return true;
}


void CrowRunnerAlarmControlPanel::control(const AlarmControlPanelCall &call) {
  if (call.get_state()) {
    if (call.get_state() == ACP_STATE_ARMED_AWAY) {
      this->arm_(call.get_code(), ACP_STATE_ARMED_AWAY, 0);
    } else if (call.get_state() == ACP_STATE_DISARMED) {
      if (!this->is_code_valid_(call.get_code())) {
        ESP_LOGW(TAG, "Not disarming code doesn't match");
        return;
      }
      this->desired_state_ = ACP_STATE_DISARMED;
      this->publish_state(ACP_STATE_DISARMED);
    } else if (call.get_state() == ACP_STATE_TRIGGERED) {
      this->publish_state(ACP_STATE_TRIGGERED);
    } else if (call.get_state() == ACP_STATE_PENDING) {
      this->publish_state(ACP_STATE_PENDING);
    } else {
      ESP_LOGE(TAG, "State not yet implemented: %s",
               LOG_STR_ARG(alarm_control_panel_state_to_string(*call.get_state())));
    }
  }
}


void CrowRunnerAlarmControlPanel::arm_(optional<std::string> code, AlarmControlPanelState state,
                                     uint32_t delay) {
  if (this->current_state_ != ACP_STATE_DISARMED) {
    ESP_LOGW(TAG, "Cannot arm when not disarmed");
    return;
  }
  if (!this->is_code_valid_(std::move(code))) {
    ESP_LOGW(TAG, "Not arming code doesn't match");
    return;
  }
  this->desired_state_ = state;
  if (delay > 0) {
    this->publish_state(ACP_STATE_ARMING);
  } else {
    this->publish_state(state);
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
    this->pin_clock_ = pin_clock;
    this->pin_data_ = pin_data;
    this->pin_data_isr_ = pin_data->to_isr();

    // Start in the WaitingForData state
    this->set_state(CrowRunnerBusState::WaitingForData);
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
    ESP_LOGD(TAG, "CrowRunnerBus state changed from %s to %s", CrowRunnerBusStateToString(this->state_), CrowRunnerBusStateToString(state));

    // Logic to dissassemble the previous state
    switch (state_) {
        case CrowRunnerBusState::Idle:
            // noop
            break;
        case CrowRunnerBusState::WaitingForData:
            this->pin_data_->detach_interrupt();
            break;
        case CrowRunnerBusState::ReceivingMessage:
            this->pin_clock_->detach_interrupt();
            break;
        case CrowRunnerBusState::SendingMessage:
            this->pin_clock_->detach_interrupt();
            this->pin_data_->pin_mode(gpio::FLAG_INPUT);
            break;

    }

    // Set new state
    this->state_ = state;

    // Logic to setup the previous state
    switch (state) {
        case CrowRunnerBusState::Idle:
            // noop
            break;
        case CrowRunnerBusState::WaitingForData:
            this->pin_data_->attach_interrupt(CrowRunnerBus::waiting_for_data_interrupt, this, gpio::INTERRUPT_FALLING_EDGE);
          break;
        case CrowRunnerBusState::ReceivingMessage:
            this->pin_clock_->attach_interrupt(CrowRunnerBus::receiving_message_interrupt, this, gpio::INTERRUPT_FALLING_EDGE);
          break;
        case CrowRunnerBusState::SendingMessage:
            this->pin_data_->pin_mode(gpio::FLAG_OUTPUT);
            this->pin_clock_->attach_interrupt(CrowRunnerBus::sending_message_interrupt, this, gpio::INTERRUPT_RISING_EDGE);
          break;
    }
}

void CrowRunnerBus::waiting_for_data_interrupt(CrowRunnerBus *arg) {
    // TODO: we might need to wipe the buffer and fill it with this first 0 bit

    // Transition from WaitingForData to ReceivingMessage
    arg->set_state(CrowRunnerBusState::ReceivingMessage);
}

void CrowRunnerBus::receiving_message_interrupt(CrowRunnerBus *arg) {

    // Read data pin state
    bool data_bit = arg->pin_data_isr_.digital_read();

    if (data_bit == 1) {
        arg->consecutive_ones_++;

        // don't let it increase too much
        if (arg->consecutive_ones_++ > 128) {
            arg->consecutive_ones_ = 128;
        }
    } else {
        arg->consecutive_ones_ = 0;
    }

    data_bit = !data_bit; // invert data_bit
    arg->receiving_buffer_.push_back(data_bit);

    if (arg->receiving_buffer_.size() == 72) {
        // End of message detected
        arg->process_received_message_();
        arg->set_state(CrowRunnerBusState::WaitingForData);
    }
}

void CrowRunnerBus::sending_message_interrupt(CrowRunnerBus *arg) {
    // ESP_LOGD(TAG, "Sending Message interrupt");

    // Check if there's anything to send
    if (arg->sending_buffers_queue_.empty()) {
        // No messages to send, go back to waiting for data
        arg->set_state(CrowRunnerBusState::WaitingForData);
        return;
    }

    // Get the current message to send
    std::vector<bool> &current_message = arg->sending_buffers_queue_.front();

    // Send the current bit
    bool bit_to_send = current_message.front();
    arg->pin_data_isr_.digital_write(bit_to_send);

    // Remove the sent bit from the current message
    current_message.erase(current_message.begin());


    // Check if the whole message has been sent
     if (current_message.empty()) {
         // Remove the sent message from the queue
        arg->sending_buffers_queue_.erase(arg->sending_buffers_queue_.begin());

        ESP_LOGD(TAG, "Message sent. Remaining messages in queue: %s", String(arg->sending_buffers_queue_.size()));

        // If there are no more messages to send, go back to waiting for data
        if (arg->sending_buffers_queue_.empty()) {
            arg->set_state(CrowRunnerBusState::WaitingForData);
        }
    }
}

std::vector<uint8_t> vector_bool_to_uint8(std::vector<bool> *bit_buffer) {
    // Convert the vector of bools into bytes
    std::vector<uint8_t> byte_buffer((bit_buffer->size() + 7) / 8, 0);

    for (size_t i = 0; i < bit_buffer->size(); ++i) {
        byte_buffer[i / 8] |= (bit_buffer->at(i) << (7 - (i % 8)));
    }

    return byte_buffer;
}

std::string vector_uint8_t_to_hex_string(std::vector<uint8_t> *byte_buffer) {
    // Log the received message in hex
    std::string hex_string;
    for (size_t i = 0; i < byte_buffer->size(); ++i) {
        char buf[3];  // two characters for the byte and one for the null terminator
        snprintf(buf, sizeof(buf), "%02X", byte_buffer->at(i));
        hex_string += buf;

        // if (i < byte_buffer->size() - 1) {
        //     hex_string += " ";  // separate bytes with a space
        // }
    }

    return hex_string;
}

void CrowRunnerBus::process_received_message_() {
    std::vector<uint8_t> buffer = vector_bool_to_uint8(&this->receiving_buffer_);

    // clear bit buffer
    this->receiving_buffer_.clear();

    ESP_LOGD(TAG, "New message: %s", vector_uint8_t_to_hex_string(&buffer).c_str());

    if (this->receiver_) {
        CrowRunnerBusMessage new_message = CrowRunnerBusMessage(&buffer);

        // send it to the message receiver
        this->receiver_(&new_message);
    }
}

CrowRunnerBusMessage::CrowRunnerBusMessage(std::vector<uint8_t> *buffer) {
    // TODO: parse message
    // no-op
}

}  // namespace empty_spi_sensor
}  // namespace esphome
