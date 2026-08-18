#include "arm_command_validator.hpp"

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include <map>
#include <string>

namespace d1_bridge {
namespace {

struct CommandDocument {
    std::map<std::string, double> top;
    std::map<std::string, double> data;
    bool has_data = false;
};

// IDs 0-5 use padded URDF limits. ID 6 is the gripper: this project treats
// 0 degrees as closed and permits a conservative 0-90 degree opening range;
// the shipped UI defaults (closed=0, open=60) remain inside that contract.
constexpr std::array<double, 7> kServoMinimumDegrees = {
    -136.0, -91.0, -91.0, -136.0, -91.0, -136.0, 0.0
};
constexpr std::array<double, 7> kServoMaximumDegrees = {
    136.0, 91.0, 91.0, 136.0, 91.0, 136.0, 90.0
};

class Parser {
public:
    explicit Parser(const std::string &input) : input_(input) {}

    bool Parse(CommandDocument &document)
    {
        if (!Consume('{')) {
            return Fail("command must be a JSON object");
        }
        SkipWhitespace();
        if (Consume('}')) {
            return Finish();
        }

        while (true) {
            std::string key;
            if (!ParseKey(key) || !Require(':')) {
                return false;
            }
            if (key == "data") {
                if (document.has_data) {
                    return Fail("duplicate key 'data'");
                }
                document.has_data = true;
                if (!ParseNumericObject(document.data)) {
                    return false;
                }
            } else {
                if (document.top.count(key) != 0) {
                    return Fail("duplicate key '" + key + "'");
                }
                double value = 0.0;
                if (!ParseNumber(value)) {
                    return false;
                }
                document.top.emplace(key, value);
            }

            SkipWhitespace();
            if (Consume('}')) {
                return Finish();
            }
            if (!Require(',')) {
                return false;
            }
        }
    }

    const std::string &error() const { return error_; }

private:
    bool ParseNumericObject(std::map<std::string, double> &values)
    {
        if (!Require('{')) {
            return false;
        }
        SkipWhitespace();
        if (Consume('}')) {
            return true;
        }

        while (true) {
            std::string key;
            if (!ParseKey(key) || !Require(':')) {
                return false;
            }
            if (values.count(key) != 0) {
                return Fail("duplicate data key '" + key + "'");
            }
            double value = 0.0;
            if (!ParseNumber(value)) {
                return false;
            }
            values.emplace(key, value);

            SkipWhitespace();
            if (Consume('}')) {
                return true;
            }
            if (!Require(',')) {
                return false;
            }
        }
    }

    bool ParseKey(std::string &key)
    {
        SkipWhitespace();
        if (position_ >= input_.size() || input_[position_] != '"') {
            return Fail("expected an object key");
        }
        ++position_;
        while (position_ < input_.size()) {
            const unsigned char character = static_cast<unsigned char>(input_[position_++]);
            if (character == '"') {
                return key.empty() ? Fail("object keys must not be empty") : true;
            }
            if (character == '\\') {
                return Fail("escaped object keys are not supported");
            }
            const bool allowed =
                (character >= 'a' && character <= 'z') ||
                (character >= 'A' && character <= 'Z') ||
                (character >= '0' && character <= '9') ||
                character == '_';
            if (!allowed || key.size() >= 31) {
                return Fail("object keys must be short ASCII identifiers");
            }
            key.push_back(static_cast<char>(character));
        }
        return Fail("unterminated object key");
    }

    bool ParseNumber(double &value)
    {
        SkipWhitespace();
        const std::size_t start = position_;
        if (position_ < input_.size() && input_[position_] == '-') {
            ++position_;
        }
        if (position_ >= input_.size()) {
            return Fail("expected a finite JSON number");
        }
        if (input_[position_] == '0') {
            ++position_;
            if (position_ < input_.size() && IsDigit(input_[position_])) {
                return Fail("JSON numbers must not contain leading zeros");
            }
        } else if (input_[position_] >= '1' && input_[position_] <= '9') {
            while (position_ < input_.size() && IsDigit(input_[position_])) {
                ++position_;
            }
        } else {
            return Fail("expected a finite JSON number");
        }

        if (position_ < input_.size() && input_[position_] == '.') {
            ++position_;
            const std::size_t fraction_start = position_;
            while (position_ < input_.size() && IsDigit(input_[position_])) {
                ++position_;
            }
            if (position_ == fraction_start) {
                return Fail("JSON fraction requires at least one digit");
            }
        }

        if (position_ < input_.size() && (input_[position_] == 'e' || input_[position_] == 'E')) {
            ++position_;
            if (position_ < input_.size() && (input_[position_] == '+' || input_[position_] == '-')) {
                ++position_;
            }
            const std::size_t exponent_start = position_;
            while (position_ < input_.size() && IsDigit(input_[position_])) {
                ++position_;
            }
            if (position_ == exponent_start) {
                return Fail("JSON exponent requires at least one digit");
            }
        }

        const std::string token = input_.substr(start, position_ - start);
        char *end = nullptr;
        errno = 0;
        value = std::strtod(token.c_str(), &end);
        if (errno != 0 || end == token.c_str() || *end != '\0' || !std::isfinite(value)) {
            return Fail("expected a finite JSON number");
        }
        return true;
    }

    static bool IsDigit(char character)
    {
        return character >= '0' && character <= '9';
    }

    void SkipWhitespace()
    {
        while (position_ < input_.size()) {
            const char character = input_[position_];
            if (character != ' ' && character != '\t' && character != '\r' && character != '\n') {
                break;
            }
            ++position_;
        }
    }

    bool Consume(char expected)
    {
        SkipWhitespace();
        if (position_ >= input_.size() || input_[position_] != expected) {
            return false;
        }
        ++position_;
        return true;
    }

    bool Require(char expected)
    {
        if (Consume(expected)) {
            return true;
        }
        return Fail(std::string("expected '") + expected + "'");
    }

    bool Finish()
    {
        SkipWhitespace();
        return position_ == input_.size() || Fail("unexpected trailing data");
    }

    bool Fail(const std::string &message)
    {
        if (error_.empty()) {
            error_ = message + " at byte " + std::to_string(position_);
        }
        return false;
    }

    const std::string &input_;
    std::size_t position_ = 0;
    std::string error_;
};

ValidationResult Reject(const std::string &message)
{
    return {false, message};
}

bool HasExactKeys(
    const std::map<std::string, double> &values,
    std::initializer_list<const char *> expected
)
{
    if (values.size() != expected.size()) {
        return false;
    }
    for (const char *key : expected) {
        if (values.count(key) == 0) {
            return false;
        }
    }
    return true;
}

bool IsIntegerInRange(double value, long minimum, long maximum)
{
    return value >= static_cast<double>(minimum) &&
        value <= static_cast<double>(maximum) &&
        std::trunc(value) == value;
}

ValidationResult ValidateCommon(const CommandDocument &document, int &funcode)
{
    const auto seq = document.top.find("seq");
    const auto address = document.top.find("address");
    const auto function = document.top.find("funcode");
    if (seq == document.top.end() || address == document.top.end() || function == document.top.end()) {
        return Reject("command requires numeric seq, address, and funcode fields");
    }
    if (!IsIntegerInRange(seq->second, 0, 2147483647L)) {
        return Reject("seq must be an integer between 0 and 2147483647");
    }
    if (address->second != 1.0) {
        return Reject("address must equal 1");
    }
    if (!IsIntegerInRange(function->second, 0, 255)) {
        return Reject("funcode must be an integer between 0 and 255");
    }
    funcode = static_cast<int>(function->second);
    return {true, ""};
}

ValidationResult ValidateServoCommand(const CommandDocument &document)
{
    if (!document.has_data || !HasExactKeys(document.top, {"seq", "address", "funcode"}) ||
        !HasExactKeys(document.data, {"id", "angle", "delay_ms"})) {
        return Reject("funcode 1 requires exactly data.id, data.angle, and data.delay_ms");
    }
    if (!IsIntegerInRange(document.data.at("id"), 0, 6)) {
        return Reject("data.id must be an integer between 0 and 6");
    }
    const std::size_t servo_id = static_cast<std::size_t>(document.data.at("id"));
    const double angle = document.data.at("angle");
    if (angle < kServoMinimumDegrees[servo_id] || angle > kServoMaximumDegrees[servo_id]) {
        return Reject("data.angle is outside the supported range for servo " + std::to_string(servo_id));
    }
    if (!IsIntegerInRange(document.data.at("delay_ms"), 0, 60000)) {
        return Reject("data.delay_ms must be an integer between 0 and 60000");
    }
    return {true, ""};
}

ValidationResult ValidateMultiJointCommand(const CommandDocument &document)
{
    if (!document.has_data || !HasExactKeys(document.top, {"seq", "address", "funcode"}) ||
        !HasExactKeys(
            document.data,
            {"mode", "angle0", "angle1", "angle2", "angle3", "angle4", "angle5", "angle6"}
        )) {
        return Reject("funcode 2 requires mode and exactly seven joint angles");
    }
    if (document.data.at("mode") != 1.0) {
        return Reject("data.mode must equal 1 for funcode 2");
    }

    for (int index = 0; index < 7; ++index) {
        const std::string key = "angle" + std::to_string(index);
        const double angle = document.data.at(key);
        const std::size_t servo_id = static_cast<std::size_t>(index);
        if (angle < kServoMinimumDegrees[servo_id] || angle > kServoMaximumDegrees[servo_id]) {
            return Reject(key + " is outside the supported joint range");
        }
    }
    return {true, ""};
}

ValidationResult ValidateModeCommand(const CommandDocument &document)
{
    if (!document.has_data || !HasExactKeys(document.top, {"seq", "address", "funcode"}) ||
        !HasExactKeys(document.data, {"mode"})) {
        return Reject("funcode 5 requires exactly data.mode");
    }
    if (!IsIntegerInRange(document.data.at("mode"), 0, 1)) {
        return Reject("data.mode must equal 0 or 1 for funcode 5");
    }
    return {true, ""};
}

}  // namespace

ValidationResult ValidateArmCommand(const std::string &payload)
{
    CommandDocument document;
    Parser parser(payload);
    if (!parser.Parse(document)) {
        return Reject("invalid command JSON: " + parser.error());
    }

    int funcode = 0;
    const ValidationResult common = ValidateCommon(document, funcode);
    if (!common.ok) {
        return common;
    }

    switch (funcode) {
        case 1:
            return ValidateServoCommand(document);
        case 2:
            return ValidateMultiJointCommand(document);
        case 5:
            return ValidateModeCommand(document);
        case 7:
            if (document.has_data || !HasExactKeys(document.top, {"seq", "address", "funcode"})) {
                return Reject("funcode 7 accepts only seq, address, and funcode");
            }
            return {true, ""};
        default:
            return Reject("unsupported funcode; bridge accepts only 1, 2, 5, and 7");
    }
}

}  // namespace d1_bridge
