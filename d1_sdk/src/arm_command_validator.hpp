#pragma once

#include <string>

namespace d1_bridge {

struct ValidationResult {
    bool ok;
    std::string error;
};

// Validate the bounded JSON command subset accepted by the UDP-to-DDS bridge.
ValidationResult ValidateArmCommand(const std::string &payload);

}  // namespace d1_bridge
