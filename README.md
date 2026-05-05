# Team Faabs Robot Code (2027 - now)

Reimplementation of the complete robot code for Team Faabs.
The code for the Jetson Nano is written in Rust and the code for
the Teensy 4.1 is written in C++. 

The Jetson handles all the high level logic and the vision processing,
while the Teensy is only there to control the motors.

# Building and Running



# Code structure
Crates
 - Tokio
 - anyhow

### Thread 0
Main Code

### Thread 1
Get UDP packets from CrashPilot

### Thread 2
Get onboard vision data from Unix Socket (Maybe implement own vision)

### Thread 3
Send Data to Teensy via Raw HID

### Thread 4
ORCA Path Planning