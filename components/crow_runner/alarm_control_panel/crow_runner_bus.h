#pragma once
#include <bitset>
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/defines.h"
#include "esphome/components/alarm_control_panel/alarm_control_panel.h"
#include "bit_vector.h"
#include "crow_runner_message.h"


namespace esphome {
namespace crow_runner {

enum class CrowRunnerBusState {
    Idle,
    WaitingForData,
    ReceivingMessage,
    SendingMessage
};

class CrowRunnerBus {
    public:
        void setup(InternalGPIOPin *pin_clock, InternalGPIOPin *pin_data);
        static void IRAM_ATTR clock_falling_interrupt(CrowRunnerBus *arg);
        static void IRAM_ATTR clock_rising_interrupt(CrowRunnerBus *arg);
        void send_message(CrowRunnerMessage *message);
        void set_state(CrowRunnerBusState state);

        void attach_receiver(void (*receiver)(CrowRunnerMessage* msg)) { this->receiver_ = receiver; }
        void detach_receiver() { this->receiver_ = nullptr; }

    protected:
        // data message receiver
        void (*receiver_)(CrowRunnerMessage* msg) = nullptr;

        CrowRunnerBusState state_ = CrowRunnerBusState::Idle;
        InternalGPIOPin *pin_clock_;
        InternalGPIOPin *pin_data_;
        ISRInternalGPIOPin pin_data_isr_; // It is faster to access through ISR

        BitVector receiving_buffer_ = BitVector(128 + (BOUNDARY_SIZE_IN_BITS * 2));
        std::vector<BitVector> sending_buffers_queue_;

        void process_receiving_buffer_();
};

}
}
