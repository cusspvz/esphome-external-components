#include "crow_runner_message.h"
#include <utility>
#include <iostream>
#include <sstream>
#include <iomanip>


namespace esphome {
namespace crow_runner {

CrowRunnerMessage::CrowRunnerMessage(std::bitset<0> *msg) {
    // parse message
    if (msg->test(63) == 0) {
        type = CrowRunnerMessageType::ZoneReporting;
        CrowRunnerMessageReporting reporting {};

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

}
}
