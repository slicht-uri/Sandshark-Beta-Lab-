/*
 * sharklogger_node.cpp
 *
 *  Created on: Oct 28, 2015
 *      Author: brenden
 */

/*
#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>
#include <sandshark_msgs/ObjectiveControlStatus.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <rosbag/bag.h>
#include <rosbag/stream.h>
#include <rosbag/macros.h>
#include <rosbag/recorder.h>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/thread.hpp>
#include <boost/thread/xtime.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/local_time/local_time.hpp>



using namespace std;
namespace bluefin {
namespace sandshark {

class SharkLoggerApp: public TaskBase {
  private:
    ros::Rate * _rate;
    void cleanup();


    rosbag::Recorder _missionRecorder;
    rosbag::Recorder _onRecorder;
  protected:
    void startupInitCallback();
    bool doInitialize();
    bool doRun();
  public:
    SharkLoggerApp() : TaskBase("SharkLogger", "sharklogger") {}
    void handleSleep();
};

     void SharkLoggerApp::startupInitCallback() {
       _rate = new ros::Rate(20);
     }

     void SharkLoggerApp::cleanup() {
     }

     bool SharkLoggerApp::doInitialize() {
       cleanup();

       last_buffer_warn_ = ros::Time();
       queue_ = new std::queue<OutgoingMessage>;



       return true;
     }

     bool SharkLoggerApp::doRun() {

       rosbag::RecorderOptions opts;
       rosbag::Recorder recorder(opts);
       int result = recorder.run();

       ROS_ERROR("LOGGER DORUN");
       ros::master::V_TopicInfo master_topics;
       ros::master::getTopics(master_topics);

       for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
         const ros::master::TopicInfo& info = *it;
         ROS_ERROR("Topic_ %s: %s", info.name.c_str(), info.datatype.c_str());
       }
       return true;
     }

     void SharkLoggerApp::handleSleep() {
       _rate->sleep();
     }

}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::SharkLoggerApp as;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) as, argc, argv);
}
*/
