#include "arm_command_validator.hpp"

#include <iostream>
#include <string>

namespace {

int failures = 0;

void Expect(const std::string &name, const std::string &payload, bool expected)
{
    const d1_bridge::ValidationResult result = d1_bridge::ValidateArmCommand(payload);
    if (result.ok != expected) {
        std::cerr << name << ": expected ok=" << expected << ", got ok=" << result.ok
                  << " error=\"" << result.error << "\"\n";
        ++failures;
    }
}

}  // namespace

int main()
{
    const std::string valid_move =
        R"({"seq":1,"address":1,"funcode":2,"data":{"mode":1,"angle0":0,"angle1":-20.5,"angle2":30,"angle3":0,"angle4":1e1,"angle5":0,"angle6":60}})";

    Expect("valid multi-joint move", valid_move, true);
    Expect(
        "valid servo move",
        R"({"seq":2,"address":1,"funcode":1,"data":{"id":6,"angle":60,"delay_ms":0}})",
        true
    );
    Expect(
        "valid negative arm servo move",
        R"({"seq":2,"address":1,"funcode":1,"data":{"id":1,"angle":-90,"delay_ms":0}})",
        true
    );
    Expect(
        "valid arm servo boundary",
        R"({"seq":2,"address":1,"funcode":1,"data":{"id":5,"angle":136,"delay_ms":60000}})",
        true
    );
    Expect(
        "valid gripper boundary",
        R"({"seq":2,"address":1,"funcode":1,"data":{"id":6,"angle":90,"delay_ms":0}})",
        true
    );
    Expect("valid zero", R"({"seq":3,"address":1,"funcode":7})", true);
    Expect("valid mode", R"({"seq":4,"address":1,"funcode":5,"data":{"mode":0}})", true);

    Expect("empty", "", false);
    Expect("not JSON", "hello", false);
    Expect("shell text", "{}; /bin/sh", false);
    Expect("string value", R"({"seq":"1","address":1,"funcode":7})", false);
    Expect("boolean value", R"({"seq":true,"address":1,"funcode":7})", false);
    Expect("nonfinite", R"({"seq":1e999,"address":1,"funcode":7})", false);
    Expect("leading zero", R"({"seq":01,"address":1,"funcode":7})", false);
    Expect("duplicate key", R"({"seq":1,"seq":2,"address":1,"funcode":7})", false);
    Expect("unknown key", R"({"seq":1,"address":1,"funcode":7,"extra":0})", false);
    Expect("wrong address", R"({"seq":1,"address":2,"funcode":7})", false);
    Expect("negative sequence", R"({"seq":-1,"address":1,"funcode":7})", false);
    Expect("unsupported function", R"({"seq":1,"address":1,"funcode":99})", false);
    Expect(
        "missing joint",
        R"({"seq":1,"address":1,"funcode":2,"data":{"mode":1,"angle0":0}})",
        false
    );
    Expect(
        "joint out of range",
        R"({"seq":1,"address":1,"funcode":2,"data":{"mode":1,"angle0":0,"angle1":100,"angle2":0,"angle3":0,"angle4":0,"angle5":0,"angle6":0}})",
        false
    );
    Expect(
        "invalid servo id",
        R"({"seq":1,"address":1,"funcode":1,"data":{"id":7,"angle":0,"delay_ms":0}})",
        false
    );
    Expect(
        "single arm servo above joint limit",
        R"({"seq":1,"address":1,"funcode":1,"data":{"id":1,"angle":92,"delay_ms":0}})",
        false
    );
    Expect(
        "single arm servo below joint limit",
        R"({"seq":1,"address":1,"funcode":1,"data":{"id":3,"angle":-137,"delay_ms":0}})",
        false
    );
    Expect(
        "negative gripper angle",
        R"({"seq":1,"address":1,"funcode":1,"data":{"id":6,"angle":-1,"delay_ms":0}})",
        false
    );
    Expect(
        "gripper above safe range",
        R"({"seq":1,"address":1,"funcode":1,"data":{"id":6,"angle":91,"delay_ms":0}})",
        false
    );
    Expect(
        "multi-joint gripper above safe range",
        R"({"seq":1,"address":1,"funcode":2,"data":{"mode":1,"angle0":0,"angle1":0,"angle2":0,"angle3":0,"angle4":0,"angle5":0,"angle6":91}})",
        false
    );
    Expect(
        "duplicate data key",
        R"({"seq":1,"address":1,"funcode":5,"data":{"mode":0,"mode":1}})",
        false
    );

    std::string nul_suffix = valid_move;
    nul_suffix.push_back('\0');
    nul_suffix += "ignored";
    Expect("embedded NUL suffix", nul_suffix, false);

    if (failures != 0) {
        std::cerr << failures << " validator expectation(s) failed.\n";
        return 1;
    }
    return 0;
}
