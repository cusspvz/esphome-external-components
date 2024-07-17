#pragma once
#include <bitset>


namespace esphome {
namespace crow_runner {

enum class CrowRunnerMessageType {
    Unknown,
    StatusChange,
    ZoneReporting
};

struct CrowRunnerMessageReporting {
    bool extra_zones;
    bool zone_activated;
    bool alarm_triggered;
    std::bitset<8> active_zones;
    std::bitset<8> alarm_trigger;
};


struct CrowRunnerMessage {
    public:
        CrowRunnerBusMessage(std::bitset<72> *msg);

        CrowRunnerMessageType getType() const { return type; }
        CrowRunnerMessageReporting getReporting() const { return reporting; }

    private:
        CrowRunnerMessageType type = CrowRunnerMessageType::Unknown;
        CrowRunnerMessageReporting reporting;
};

}
}
