# Firmware

This directory is reserved for the firmware that will run on next year's robot
microchips.

There is no active firmware implementation here yet. The current robot stack
still talks to the existing Teensy-compatible controller over USB HID from the
Jetson Rust application in `../tf_jetsoncode`.

When firmware development starts, document the board, toolchain, flashing
steps, packet format, calibration procedure, and hardware safety assumptions in
this directory.
