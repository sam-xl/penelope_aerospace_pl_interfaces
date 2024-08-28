cmake_minimum_required(VERSION 3.0.2)
project(penelope_aerospace_pl_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  ModuleState.msg
  ResultCodes.msg
)

add_action_files(
  FILES
  ExampleAction.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    std_msgs
    message_runtime
)
