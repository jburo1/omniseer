#pragma once
#include "omniseer_core/types.hpp"

namespace hw_serial_cmd
{

  using namespace omniseer_core;

  // Call from loop(); returns true if a new CmdVel was parsed.
  bool poll(CmdVel& out_cmd);

} // namespace hw_serial_cmd
