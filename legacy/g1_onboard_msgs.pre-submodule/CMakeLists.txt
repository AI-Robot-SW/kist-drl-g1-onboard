cmake_minimum_required(VERSION 3.8)
project(g1_onboard_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Interface files. Order does not matter — rosidl resolves inter-message references.
# Removed historical: ValidatedCmd (2026-05-15), ActionChunk (2026-05-16); see git log.
# Re-added 2026-05-26: JointCmdChunk (wire reversal, chunk-as-wire).
set(msg_files
  "msg/AudioPCM.msg"
  "msg/JointCmd.msg"
  "msg/JointCmdChunk.msg"
  "msg/LocoCommand.msg"
  "msg/BufState.msg"
  "msg/SpeakerState.msg"
  "msg/EstopFlag.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
