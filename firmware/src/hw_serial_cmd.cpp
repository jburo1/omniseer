#include "hw_serial_cmd.hpp"
#include <Arduino.h>
#include "omniseer_core/cmd_protocol.hpp"

namespace {

constexpr size_t BUF_SIZE = 64;
char   buf[BUF_SIZE];
size_t idx = 0;

}

namespace hw_serial_cmd {

bool poll(CmdVel& out_cmd) {
    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());
        if (c == '\n' || c == '\r') {
            if (idx == 0) continue;
            buf[idx] = '\0';
            idx = 0;

            CmdVel cmd;
            if (omniseer_core::parse_cmd_vel_line(buf, cmd)) {
                out_cmd = cmd;
                return true;
            }  // Run control at fixed period

        } else if (idx < BUF_SIZE - 1) {
            buf[idx++] = c;
        }
    }
    return false;
}

} // namespace hw_serial_cmd
