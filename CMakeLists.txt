cmake_minimum_required(VERSION 2.8.3)
project(rover_tiva)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rosserial_client
  rosserial_tivac
  std_msgs
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_tivac
  SCRIPT make_libraries_tiva
)

# Chatter tutorial
rosserial_configure_client(
  DIRECTORY arm_shoulder arm_elbow arm_wrist drive_left drive_middle drive_right
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(arm_shoulder arm_shoulder.axf ALL)
rosserial_add_client_target(arm_shoulder flash)
rosserial_add_client_target(arm_shoulder size)
rosserial_add_client_target(arm_shoulder dump)
rosserial_add_client_target(arm_elbow arm_elbow.axf ALL)
rosserial_add_client_target(arm_elbow flash)
rosserial_add_client_target(arm_elbow size)
rosserial_add_client_target(arm_elbow dump)
rosserial_add_client_target(arm_wrist arm_wrist.axf ALL)
rosserial_add_client_target(arm_wrist flash)
rosserial_add_client_target(arm_wrist size)
rosserial_add_client_target(arm_wrist dump)
rosserial_add_client_target(drive_left drive_left.axf ALL)
rosserial_add_client_target(drive_left flash)
rosserial_add_client_target(drive_left size)
rosserial_add_client_target(drive_left dump)
rosserial_add_client_target(drive_middle drive_middle.axf ALL)
rosserial_add_client_target(drive_middle flash)
rosserial_add_client_target(drive_middle size)
rosserial_add_client_target(drive_middle dump)
rosserial_add_client_target(drive_right drive_right.axf ALL)
rosserial_add_client_target(drive_right flash)
rosserial_add_client_target(drive_right size)
rosserial_add_client_target(drive_right dump)
rosserial_add_client_target(sampling sampling.axf ALL)
rosserial_add_client_target(sampling flash)
rosserial_add_client_target(sampling size)
rosserial_add_client_target(sampling dump)
rosserial_add_client_target(power power.axf ALL)
rosserial_add_client_target(power flash)
rosserial_add_client_target(power size)
rosserial_add_client_target(power dump)
