#include "crow_runner_bus.h"
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

        if (first_byte != BOUNDARY) {
            set_state(CrowRunnerBusState::WaitingForData);
        }

        return; // continue to receive the message
    } else if (written_bytes < 3) {
        return; // continue to receive the message
    } else {
        // more than 3 bytes
        uint8_t last_byte = receiving_buffer_.get_byte(written_bytes - 1);

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
    ESP_LOGD(TAG, "RECEIVED NEW MESSAGE: %s", vector_to_hex_string(binary_message.get_data()).c_str());

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



}
}
