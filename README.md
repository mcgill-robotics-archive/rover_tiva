# Rover Tiva Microcontroller Code Submodule

## Setup and Running Instructions

- Copy `71-rover-tiva.rules` to `/etc/udev/rules.d/`
- Edit `launch/tiva.launch` to reflect the number of Tiva's connected
- `roscd` and run `roslaunch rover_tiva tiva.launch`
- Subscribe/publish to appropriate topics

## Flashing Instructions

- Connect a Tiva Launchpad's JTAG pins (`TCK`, `TMS`, `TDI`, `TDO`, `GND`) to the custom Tiva's header. Be sure to disconnect the `VDD` jumper on the Launchpad.
- `roscd` and run `catkin_make rover_tiva_arm_shoulder_flash`, or similar for the other joints
- Note that the backplane needs to be powered on for this to work (All green and red LEDs should be on)
