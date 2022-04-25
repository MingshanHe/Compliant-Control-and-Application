cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(mir_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
    BatteryCurrents.msg
    BMSData.msg
    BrakeState.msg
    ChargingState.msg
    Device.msg
    Devices.msg
    Encoders.msg
    EncoderTestEntry.msg
    Error.msg
    Event.msg
    Events.msg
    ExternalRobot.msg
    ExternalRobots.msg
    Gpio.msg
    GripperState.msg
    HeightState.msg
    HookExtendedStatus.msg
    HookStatus.msg
    IOs.msg
    JoystickVel.msg
    LocalMapStat.msg
    MirExtra.msg
    MissionCtrlCommand.msg
    MissionCtrlState.msg
    PalletLifterStatus.msg
    Path.msg
    Pendant.msg
    PlanSegment.msg
    PlanSegments.msg
    Pose2D.msg
    PowerBoardMotorStatus.msg
    PrecisionDockingStatus.msg
    Proximity.msg
    ResourcesState.msg
    ResourceState.msg
    RobotMode.msg
    RobotState.msg
    RobotStatus.msg
    SafetyStatus.msg
    Serial.msg
    SoundEvent.msg
    StampedEncoders.msg
    Trolley.msg
    Twist2D.msg
    UserPrompt.msg
    WebPath.msg
    WorldMap.msg
    WorldModel.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
)
