/*
 * sharklogger_node.cpp
 *
 *  Created on: Oct 28, 2015
 *      Author: brenden
 */

#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>
#include <sandshark_msgs/ObjectiveControlStatus.h>
#include <sandshark_msgs/StartMissionLog.h>
#include <sandshark_msgs/StopMissionLog.h>
#include <sandshark_msgs/Navigation.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <rosbag/bag.h>
#include <rosbag/stream.h>
#include <rosbag/macros.h>
#include "startstoprecorder.h"

#include <boost/thread.hpp>
#include <boost/thread/xtime.hpp>
#include <boost/date_time/local_time/local_time.hpp>
#include <pthread.h>
#include <signal.h>

using namespace std;
namespace bluefin {
namespace sandshark {

class SharkLoggerApp: public TaskBase {
  private:
    ros::Subscriber _objectiveStatusSub;
    ros::Subscriber _startMissionLogSub;
    ros::Subscriber _stopMissionLogSub;

    std::string _logDirectory;
    ros::Duration _missionTimeout;
    ros::Time _lastOCMTime;
    bool _inMission;
    bool _lastInMission;
    bool _missionIsLogging;
    bool _persistentIsLogging;

    rosbag::StartStopRecorder _missionRecorder;
    rosbag::StartStopRecorder _persistentRecorder;

    rosbag::StartStopRecorderOptions _mrOptions;
    rosbag::StartStopRecorderOptions _prOptions;

    boost::shared_ptr<boost::thread> _missionThread;
    boost::shared_ptr<boost::thread> _persistentThread;

    void startMissionLogging();
    void stopMissionLogging();
    void startPersistentLogging();
    void stopPersistentLogging();

    void startr();
    void stopr();

    void persistentLoggingThread();
    void missionLoggingThread();

    ros::Rate * _rate;
    void cleanup();

  protected:
    void startupInitCallback();
    bool doInitialize();
    bool doRun();
  public:
    SharkLoggerApp() : TaskBase("SharkLogger", "sharklogger") {}
    void handleSleep();
    void msgObjectiveStatusCallback(const sandshark_msgs::ObjectiveControlStatus::ConstPtr & msg);
    void logMissionStartCallback(const sandshark_msgs::StartMissionLog::ConstPtr & msg);
    void logMissionStopCallback(const sandshark_msgs::StopMissionLog::ConstPtr & msg);
};

void SharkLoggerApp::msgObjectiveStatusCallback(const sandshark_msgs::ObjectiveControlStatus::ConstPtr & msg) {
  //ROS_ERROR("IN CALLBACK  Mission Running: %s",_inMission? "true":"false");
  /*
  //update the last time
  _lastOCMTime = ros::Time::now();
  _inMission = msg->mission_running;
  ROS_ERROR("IN CALLBACK  Mission Running: %s",_inMission? "true":"false");
  */
}

void SharkLoggerApp::logMissionStartCallback(const sandshark_msgs::StartMissionLog::ConstPtr & msg) {
  startMissionLogging();
}

void SharkLoggerApp::logMissionStopCallback(const sandshark_msgs::StopMissionLog::ConstPtr & msg) {
  stopMissionLogging();
}

void SharkLoggerApp::startupInitCallback() {
  _objectiveStatusSub = _publicNode->subscribe("/objectivecontrol/objective_status", 1, &SharkLoggerApp::msgObjectiveStatusCallback, this);
  _startMissionLogSub = _publicNode->subscribe("/sharklogger/start_mission_log", 1, &SharkLoggerApp::logMissionStartCallback, this);
  _stopMissionLogSub = _publicNode->subscribe("/sharklogger/stop_mission_log", 1, &SharkLoggerApp::logMissionStopCallback, this);

  _privateNode->param("logDirectory", _logDirectory, std::string("/mnt/sdcard/"));

  _prOptions.record_all = true;
  _prOptions.prefix = _logDirectory + "p";
  _prOptions.name = "entireBrah";
  _prOptions.min_space = 4;

  _mrOptions.record_all = true;
  _mrOptions.prefix = _logDirectory + "m";
  _mrOptions.name = "missionBrah";
  _mrOptions.min_space = 4;

  //if no objective control messages for 10 seconds, we aren't in a mission anymore
  _missionTimeout = ros::Duration(10.0);

  _missionIsLogging = false;
  _persistentIsLogging = false;

  _rate = new ros::Rate(10);
}

void SharkLoggerApp::cleanup() {
  stopPersistentLogging();
  stopMissionLogging();
}

bool SharkLoggerApp::doInitialize() {
  cleanup();
  //start ten seconds ago
  _lastOCMTime = ros::Time::now();// - _missionTimeout;

  _inMission = false;
  _lastInMission = false;
  _missionIsLogging = false;
  _persistentIsLogging = false;

  _missionRecorder.setOptions(_mrOptions);
  _persistentRecorder.setOptions(_prOptions);

  //kick off the persistent logger, which will run until the node stops
  startPersistentLogging();
  return true;
}

bool SharkLoggerApp::doRun() {

  /*
  //have we transitioned to running a mission?
  ROS_ERROR("inMission: %s       lastInMission %s !!\n", _inMission? "true":"false", _lastInMission? "true":"false");
  if (!_lastInMission && _inMission) {
    ROS_ERROR("MISSION START DETECTED!!\n");
    //start mission logging
    startMissionLogging();
  }
  //have we transitioned out of a mission or have we timed out?
  if (ros::Time::now() - _lastOCMTime > _missionTimeout || (_lastInMission && !_inMission)) {
    ROS_ERROR("MISSION END DETECTED %s %s !!", (ros::Time::now() - _lastOCMTime > _missionTimeout)? "true":"false", (_lastInMission && !_inMission)? "true":"false");
    //stop the mission log
    stopMissionLogging();
  }

  _lastInMission = _inMission;
  */
  return true;
}

void SharkLoggerApp::handleSleep() {
  _rate->sleep();
}

//The actual thread that kicked off in startMissionLogging
void SharkLoggerApp::missionLoggingThread() {
  _missionRecorder.run();
}

//The actual thread that kicked off in startPersistentLogging
void SharkLoggerApp::persistentLoggingThread() {
  _persistentRecorder.run();
}

void SharkLoggerApp::startMissionLogging() {
  if (!_missionIsLogging) {
    boost::thread f(boost::bind(&SharkLoggerApp::missionLoggingThread, this));
    _missionRecorder._stopRunning = false;
    _missionRecorder._startRunning = true;
    _missionIsLogging = true;
  } else {
    ROS_ERROR("One mission logger is already active.  Cannot start a second");
  }
}

void SharkLoggerApp::startPersistentLogging() {
  if (!_persistentIsLogging) {
    boost::thread p(boost::bind(&SharkLoggerApp::persistentLoggingThread, this));
    _persistentRecorder._stopRunning = false;
    _persistentRecorder._startRunning = true;
    _persistentIsLogging = true;
  } else {
    ROS_ERROR("One persistent logger is already active.  Cannot start a second");
  }
}

void SharkLoggerApp::stopMissionLogging() {
  if (_missionIsLogging) {
    _missionRecorder.stopRecording();
    _missionIsLogging = false;
  } else {
    ROS_ERROR("No mission logger is active.  Cannot stop it...");
  }
}

void SharkLoggerApp::stopPersistentLogging() {
  if (_persistentIsLogging) {
    _persistentRecorder.stopRecording();
    _persistentIsLogging = false;
  } else {
    ROS_ERROR("No persistent logger is active.  Cannot stop it...");
  }
}

}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::SharkLoggerApp as;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) as, argc, argv);
}

