# Electrical System

The ROCK 5B+ provides high-level compute, ROS 2, perception, and autonomy. The
Teensy 4.1 provides low-level motor and sensor I/O. This schematic documents the
robot's power distribution and physical embedded interfaces.

## Electrical Schematic

<object
  data="../assets/hardware/robot_electrical.svg"
  type="image/svg+xml"
  aria-label="Omniseer electrical schematic"
  width="100%"
  style="display: block; width: 100%; aspect-ratio: 297.0022 / 210.0072;">
</object>

[Editable KiCad source](https://github.com/jburo1/omniseer/blob/master/assets/robot_electrical.kicad_sch) · [Exported SVG in the repository](https://github.com/jburo1/omniseer/blob/master/assets/images/robot_electrical.svg)

## Power Architecture

The schematic shows a 2S / 7.4 V battery motor-power path protected by a 15 A
main fuse. A fused LM2596 supply provides 5 V logic power, while the ROCK 5B+
is powered through its USB-C path. The relevant motor and logic power domains
share a common ground.

## Embedded I/O

The Teensy connects to the motor driver and four encoder channels, BNO055 IMU,
and HC-SR04 ultrasonic sensor. It connects to the ROCK 5B+ over USB.

## Hardware / Firmware Boundary

The ROCK 5B+ owns high-level ROS 2, perception, and autonomy. The Teensy owns
deterministic hardware I/O and motor and sensor interaction; its behavior and
ROS interfaces are documented in the [firmware overview](../firmware/overview.md).
See also the [Bill of Materials](bill-of-materials.md).
