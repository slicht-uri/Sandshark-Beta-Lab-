/*
 * sharklogger_node.cpp
 *
 *  Created on: Oct 28, 2015
 *      Author: brenden
 */

#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>
#include <sandshark_msgs/ObjectiveControlStatus.h>
#include <sandshark_msgs/Navigation.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <rosbag/bag.h>
#include <rosbag/stream.h>
#include <rosbag/macros.h>
//#include <rosbag/recorder.h>
#include "actuallyusefulrecorder.h"

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/thread.hpp>
#include <boost/thread/xtime.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/local_time/local_time.hpp>

#include <pthread.h>
#include <signal.h>

using namespace std;
namespace bluefin {
namespace sandshark {

class SharkLoggerApp: public TaskBase {
  private:
    ros::Subscriber _objectiveStatusSub;

    std::string _logDirectory;
    ros::Duration _missionTimeout;
    ros::Time _lastOCMTime;
    bool _inMission;
    bool _lastInMission;
    bool _missionIsLogging;
    bool _persistentIsLogging;

    /*rosbag::Recorder *_missionRecorder;
    rosbag::Recorder *_persistentRecorder;
    rosbag::Recorder *_r;

    rosbag::RecorderOptions _mrOptions;
    rosbag::RecorderOptions _prOptions;*/

    rosbag::ActuallyUsefulRecorder *_missionRecorder;
    rosbag::ActuallyUsefulRecorder *_persistentRecorder;
    rosbag::ActuallyUsefulRecorder *_r;

    rosbag::ActuallyUsefulRecorderOptions _mrOptions;
    rosbag::ActuallyUsefulRecorderOptions _prOptions;

    boost::thread *_missionThread;
    boost::thread *_persistentThread;

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
    SharkLoggerApp() :
        TaskBase("SharkLogger", "sharklogger") {
    }
    void handleSleep();
    void msgObjectiveStatusCallback(const sandshark_msgs::ObjectiveControlStatus::ConstPtr & msg);
};

void SharkLoggerApp::msgObjectiveStatusCallback(const sandshark_msgs::ObjectiveControlStatus::ConstPtr & msg) {
  //update the last time
  _lastOCMTime = ros::Time::now();
  _inMission = msg->mission_running;
  ROS_ERROR("IN CALLBACK  Mission Running: %s",_inMission? "true":"false");
}

void SharkLoggerApp::startupInitCallback() {
  ROS_ERROR("SL: SINIT");
  _objectiveStatusSub = _publicNode->subscribe("/objectivecontrol/objective_status", 1,
      &SharkLoggerApp::msgObjectiveStatusCallback, this);
  _privateNode->param("logDirectory", _logDirectory, std::string("/home/brenden/Desktop/fahts"));//std::string("/mnt/sdcard/"));

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
  ROS_ERROR("SL: SINIT END");
}

void SharkLoggerApp::cleanup() {
  ROS_ERROR("SL: CLEANUP");
  stopPersistentLogging();
  stopMissionLogging();
}

bool SharkLoggerApp::doInitialize() {
  ROS_ERROR("SL: DOINIT");
  cleanup();
  //start ten seconds ago
  _lastOCMTime = ros::Time::now();// - _missionTimeout;

  _inMission = false;
  _lastInMission = false;
  _missionIsLogging = false;
  _persistentIsLogging = false;

  startPersistentLogging();
  ROS_ERROR("SL: END DOINIT");
  return true;
}

bool SharkLoggerApp::doRun() {
  ROS_ERROR("SL: Running, brah");
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
  return true;
}

void SharkLoggerApp::handleSleep() {
  _rate->sleep();
}

void SharkLoggerApp::missionLoggingThread() {
  _missionRecorder->run();
}

void SharkLoggerApp::persistentLoggingThread() {
  _persistentRecorder->run();
}

void SharkLoggerApp::startMissionLogging() {
  ROS_ERROR("-----------------SL: stahtml----------------");
  if (!_missionIsLogging) {
    _missionRecorder = new rosbag::ActuallyUsefulRecorder(_mrOptions);
    _missionThread = new boost::thread(boost::bind(&SharkLoggerApp::missionLoggingThread, this));
    //_missionThread->detach();
    _missionRecorder->_startRunning = true;
    _missionIsLogging = true;
  } else {
    ROS_ERROR("One mission logger is already active.  Cannot start a second");
  }
}

void SharkLoggerApp::startPersistentLogging() {
  ROS_ERROR("SL: stahtpl");
  if (!_persistentIsLogging) {
    _persistentRecorder = new rosbag::ActuallyUsefulRecorder(_prOptions);
    _persistentThread = new boost::thread(boost::bind(&SharkLoggerApp::persistentLoggingThread, this));
    //_persistentThread->detach();
    _persistentRecorder->_startRunning = true;
    _persistentIsLogging = true;
  } else {
    ROS_ERROR("One persistent logger is already active.  Cannot start a second");
  }
}

void SharkLoggerApp::stopMissionLogging() {
  ROS_ERROR("SL: stopml");
  if (_missionIsLogging) {
    ROS_ERROR("Mission is logging");
    //pthread_kill(_missionThread->native_handle(), 9);
    //pthread_cancel(_missionThread->native_handle());
    _missionRecorder->_stopRunning = true;
    ROS_ERROR("GONNA JOIN MIS");
    _missionThread->join();
    ROS_ERROR("MIS JOINED");
    //delete _missionRecorder;
    //delete _missionThread;
    //_missionRecorder = NULL;
    //_missionThread = NULL;
    _missionIsLogging = false;
  } else {
    ROS_ERROR("No mission logger is active.  Cannot stop it...");
  }
}

void SharkLoggerApp::stopPersistentLogging() {
  if (_persistentIsLogging) {
    _persistentRecorder->_stopRunning = true;
    _persistentThread->join();
    _persistentIsLogging = false;
  } else {
    ROS_ERROR("No persistent logger is active.  Cannot stop it...");
  }
}

void SharkLoggerApp::startr() {
  ROS_ERROR("SL: startr");
  if (!_missionIsLogging) {
    _r = new rosbag::ActuallyUsefulRecorder(_mrOptions);
    _r->run();
    //_r = new boost::thread(boost::bind(&SharkLoggerApp::missionLoggingThread, this));
    _missionIsLogging = true;
  } else {
    ROS_ERROR("One r logger is already active.  Cannot start a second");
  }
}

void SharkLoggerApp::stopr() {
  ROS_ERROR("SL: stopr");
  if (_missionIsLogging) {
    ROS_ERROR("r is logging");
   _r->~ActuallyUsefulRecorder();
   delete _r;
   //delete _missionThread;
   _missionIsLogging = false;
} else {
  ROS_ERROR("No r logger is active.  Cannot stop it...");
}
}



}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::SharkLoggerApp as;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) as, argc, argv);
}

