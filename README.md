# Rover Tiva Microcontroller Code Submodule

## Setup Instructions

- Copy `71-rover-tiva.rules` to `/etc/udev/rules.d/` and refresh the udev rules (you could do this by rebooting)

## Running Instructions 
- When you plug in the backplane, `/dev` should contain `arm_shoulder`, `arm_elbow`, and `arm_wrist` symlinks
- `roscd` and run `roslaunch rover_tiva arm.launch`
- Subscribe/publish to appropriate topics
  - `motor_shoulder_a`: Publish motor commands to these topics (velocities from -4000 to 4000)
  - `inc_shoulder_a`: Subscribe to incremental encoder readings from these topics
  - `abs_shoulder_a`: Subscribe to absolute encoder readings from these topics

## Flashing Instructions

- Connect a Tiva Launchpad's JTAG pins (`TCK`, `TMS`, `TDI`, `TDO`, `GND`) to the custom Tiva's header. Be sure to disconnect the `VDD` jumper on the Launchpad.
- Power on the backplane by plugging it into a computer. All green and red LEDs should be on
- `roscd` and run `catkin_make rover_tiva_arm_shoulder_flash`, or similar for the other joints

# Maintenance instructions

Let's say you want to add a completely new Tiva to the system. You would need to:

- Update the top level and inner CMakeLists to add its code to the build
- Give it its own serial number in `usb_serial_structs.c` and symlink both `usb_serial_structs` files into the Tiva's directory
- Update/create a udev rule to allow it to enumerate
- Add it to the launch file, or create another launch file

You should be able to do all that by looking at the code for the other Tivas

# Notes

- The shoulder and elbow Tivas drive 2 motors each, but the wrist Tiva drives 3 motors (including the claw)
- Plugging the backplane into a DC power source is not sufficient to power it on. Its internal USB hub must handshake with a computer's USB port before it will turn on the Tivas
- Remember, all the files inside the `arm_shoulder`, `arm_elbow`, and `arm_wrist` directories are symlinked together. The only actual files in those directories are the `CMakeLists.txt`. Compile-time defines generate the individual firmware for each Tiva
- If you want to manually blank a Tiva, you can run `lm4flash /dev/null`
- If you look in your kernel logs (`dmesg -wH`) you'll see that one of the USB ports on the backplane is reporting an overcurrent condition. This is normal, I just forgot to pull down the overcurrent pin on thehub's unused USB port
- Tivas with no serial number specified will be given the serial number `00000000`. The last udev rule in `71-rover-tiva.rules` will give those Tivas the symlink `tiva%n` where `%n` is an integer chosen by your computer
- Relevant page about the Tivas' VID and PID. Hopefully we will get registered in Ubuntu's USB ID database: https://github.com/mcgill-robotics/electrical/wiki/McGill-Robotics-USB-PID-for-TM4C-Microcontrollers
