cmake_minimum_required(VERSION 2.8.3)
project(sandshark_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
Accelerometer.msg
ActuatorStatus.msg
AltitudeCommands.msg
Altitude.msg
ApplicationAbort.msg
ApplicationHealthStatus.msg
ApplicationRestartCommand.msg
ApplicationStatus.msg
BoardHealthStatus.msg
CurrentObjective.msg
CurrentSense.msg
Depth.msg
DynamicControlCommand.msg
DynamicControlGains.msg
DynamicRequestsStatus.msg
ElapsedMissionTime.msg
Environmental.msg
GasGauge.msg
GPS.msg
GravityAcceleration.msg
Gyro.msg
HealthValues.msg
LastAbort.msg
LeakDetect.msg
LEDCommand.msg
LEDStatus.msg
LinearAcceleration.msg
Magnetometer.msg
MavlinkObjectiveCommand.msg
MavlinkPolygon.msg
MavlinkWaypointList.msg
MotionCorrected.msg
MotionRaw.msg
Navigation.msg
ObjectiveCommand.msg
ObjectiveControlStatus.msg
OrientationDebug.msg
OrientationEuler.msg
OrientationQuaternion.msg
PayloadStatus.msg
PID.msg
PowerUsage.msg
Shutdown.msg
Speed.msg
StartMissionLog.msg
StopMissionLog.msg
TailconePowerUsage.msg
TailconeRaw.msg
TailconeState.msg
TailconeVersion.msg
Velocities.msg
adapt_payload_data.msg
)

add_service_files(
  FILES
AccelCalib.srv
MagCalib.srv
UpdateDCGain.srv
UpdateHealthVal.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)
