#pragma once
#include <Arduino.h>
#include <Wire.h>

inline bool writeReg(TwoWire& wire, uint8_t addr, uint8_t reg) {
  wire.beginTransmission(addr);
  wire.write(reg);
  uint8_t err = wire.endTransmission();
  return (err == 0);
}

inline bool writeRegData(TwoWire& wire, uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) {
  wire.beginTransmission(addr);
  wire.write(reg);
  wire.write(data, len);
  uint8_t err = wire.endTransmission();
  return (err == 0);
}

inline bool readRegData(TwoWire& wire, uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
  if (!writeReg(wire, addr, reg)) {
    return false;
  }

  uint8_t n = wire.requestFrom(addr, static_cast<uint8_t>(len));
  if (n != len) {
    return false;
  }

  for (size_t i = 0; i < len; ++i) {
    if (!wire.available()) return false;
    data[i] = wire.read();
  }
  return true;
}
