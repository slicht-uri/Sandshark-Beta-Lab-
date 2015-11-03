/*
 * sharklogger_node.cpp
 *
 *  Created on: Oct 28, 2015
 *      Author: brenden
 */


#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/network.h>
#include <ros/xmlrpc_manager.h>

#include <rosbag/bag.h>
#include <rosbag/stream.h>
#include <rosbag/macros.h>
#include <rosbag/recorder.h>
#include <std_msgs/Empty.h>
#include <topic_tools/shape_shifter.h>

#include "XmlRpc.h"

#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <sys/statvfs.h>

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

class OutgoingMessage {
 public:
     OutgoingMessage(std::string const& _topic, topic_tools::ShapeShifter::ConstPtr _msg, boost::shared_ptr<ros::M_string> _connection_header, ros::Time _time);
     std::string                         topic;
     topic_tools::ShapeShifter::ConstPtr msg;
     boost::shared_ptr<ros::M_string>    connection_header;
     ros::Time                           time;
};
class OutgoingQueue {
 public:
     OutgoingQueue(std::string const& _filename, std::queue<OutgoingMessage>* _queue, ros::Time _time);
     std::string                  filename;
     std::queue<OutgoingMessage>* queue;
     ros::Time                    time;
};
struct RecorderOptions
{
     RecorderOptions();
     bool            trigger;
     bool            record_all;
     bool            regex;
     bool            do_exclude;
     bool            quiet;
     bool            append_date;
     bool            snapshot;
     bool            verbose;
     rosbag::CompressionType compression;
     std::string     prefix;
     std::string     name;
     boost::regex    exclude_regex;
     uint32_t        buffer_size;
     uint32_t        chunk_size;
     uint32_t        limit;
     bool            split;
     uint32_t        max_size;
     ros::Duration   max_duration;
     std::string     node;
     std::vector<std::string> topics;
};

 // OutgoingMessage
OutgoingMessage::OutgoingMessage(string const& _topic, topic_tools::ShapeShifter::ConstPtr _msg, boost::shared_ptr<ros::M_string> _connection_header, ros::Time _time) :
     topic(_topic), msg(_msg), connection_header(_connection_header), time(_time){}

 // OutgoingQueue
OutgoingQueue::OutgoingQueue(string const& _filename, std::queue<OutgoingMessage>* _queue, ros::Time _time) :
     filename(_filename), queue(_queue), time(_time){}

 // RecorderOptions
 RecorderOptions::RecorderOptions() :
     trigger(false),
     record_all(true),
     regex(false),
     do_exclude(false),
     quiet(false),
     append_date(true),
     snapshot(false),
     verbose(false),
     compression(rosbag::compression::Uncompressed),
     prefix(""),
     name(""),
     exclude_regex(),
     buffer_size(1048576 * 256),
     limit(0),
     split(false),
     max_size(0),
     max_duration(-1.0),
     node(""){}

class SharkLoggerApp: public TaskBase {



  private:
    ros::Rate * _rate;
    void updateFilenames();
    void startWriting();
    void stopWriting();
    bool checkLogging();
    bool scheduledCheckDisk();
    bool checkDisk();
    void snapshotTrigger(std_msgs::Empty::ConstPtr trigger);
    void doQueue(ros::MessageEvent<topic_tools::ShapeShifter const> msg_event, string const& topic, boost::shared_ptr<ros::Subscriber> subscriber, boost::shared_ptr<int> count);
    void doRecord();
    bool checkSize();
    bool checkDuration(const ros::Time&);
    void doRecordSnapshotter();
    void doCheckMaster(ros::TimerEvent const& e, ros::NodeHandle& node_handle);
    bool shouldSubscribeToTopic(std::string const& topic, bool from_node = false);
    template<class T>
    static std::string timeToStr(T ros_t);
    void cleanup();

    RecorderOptions               options_;
    rosbag::Bag                   bag_;
    std::string                   target_filename_;
    std::string                   write_filename_;
    std::set<std::string>         currently_recording_;
    int                           num_subscribers_;
    int                           exit_code_;
    boost::condition_variable_any queue_condition_;
    boost::mutex                  queue_mutex_;
    std::queue<OutgoingMessage>*  queue_;
    uint64_t                      queue_size_;
    uint64_t                      max_queue_size_;
    uint64_t                      split_count_;
    std::queue<OutgoingQueue>     queue_queue_;
    ros::Time                     last_buffer_warn_;
    ros::Time                     start_time_;
    bool                          writing_enabled_;
    boost::mutex                  check_disk_mutex_;
    ros::WallTime                 check_disk_next_;
    ros::WallTime                 warn_next_;

  protected:
    void startupInitCallback();
    bool doInitialize();
    bool doRun();
  public:
    SharkLoggerApp() : TaskBase("SharkLogger", "sharklogger") {}
    void handleSleep();

    void doTrigger();
    bool isSubscribed(std::string const& topic) const;
    boost::shared_ptr<ros::Subscriber> subscribe(string const& topic);
    int run2();
};


boost::shared_ptr<ros::Subscriber> SharkLoggerApp::subscribe(string const& topic) {
    ROS_INFO("Subscribing to %s", topic.c_str());

    ros::NodeHandle nh;
    boost::shared_ptr<int> count(new int(options_.limit));
    boost::shared_ptr<ros::Subscriber> sub(new ros::Subscriber);
    *sub = _privateNode->subscribe<topic_tools::ShapeShifter>(topic, 100, boost::bind(&SharkLoggerApp::doQueue, this, _1, topic, sub, count));
    currently_recording_.insert(topic);
    num_subscribers_++;

    return sub;
}

bool SharkLoggerApp::isSubscribed(string const& topic) const {
     return currently_recording_.find(topic) != currently_recording_.end();
}

bool SharkLoggerApp::shouldSubscribeToTopic(std::string const& topic, bool from_node) {
     // ignore already known topics
     if (isSubscribed(topic)) {
         return false;
     }

     // subtract exclusion regex, if any
     if(options_.do_exclude && boost::regex_match(topic, options_.exclude_regex)) {
         return false;
     }

     if(options_.record_all || from_node) {
         return true;
     }

     if (options_.regex) {
         // Treat the topics as regular expressions
         BOOST_FOREACH(string const& regex_str, options_.topics) {
             boost::regex e(regex_str);
             boost::smatch what;
             if (boost::regex_match(topic, what, e, boost::match_extra))
                 return true;
         }
     } else {
         BOOST_FOREACH(string const& t, options_.topics)
             if (t == topic)
                 return true;
     }

     return false;
}


template<class T>
std::string SharkLoggerApp::timeToStr(T ros_t) {
     char buf[1024] = "";
     time_t t = ros_t.sec;
     struct tm* tms = localtime(&t);
     strftime(buf, 1024, "%Y-%m-%d-%H-%M-%S", tms);
     return string(buf);
}


void SharkLoggerApp::doQueue(ros::MessageEvent<topic_tools::ShapeShifter const> msg_event, string const& topic, boost::shared_ptr<ros::Subscriber> subscriber, boost::shared_ptr<int> count) {
     ros::Time rectime = ros::Time::now();

     if (options_.verbose)
         cout << "Received message on topic " << subscriber->getTopic() << endl;

     OutgoingMessage out(topic, msg_event.getMessage(), msg_event.getConnectionHeaderPtr(), rectime);

     {
         boost::mutex::scoped_lock lock(queue_mutex_);

         queue_->push(out);
         queue_size_ += out.msg->size();

         // Check to see if buffer has been exceeded
         while (options_.buffer_size > 0 && queue_size_ > options_.buffer_size) {
             OutgoingMessage drop = queue_->front();
             queue_->pop();
             queue_size_ -= drop.msg->size();

             if (!options_.snapshot) {
                 ros::Time now = ros::Time::now();
                 if (now > last_buffer_warn_ + ros::Duration(5.0)) {
                     ROS_WARN("rosbag record buffer exceeded.  Dropping oldest queued message.");
                     last_buffer_warn_ = now;
                 }
             }
         }
     }

     if (!options_.snapshot)
         queue_condition_.notify_all();

     // If we are book-keeping count, decrement and possibly shutdown
     if ((*count) > 0) {
         (*count)--;
         if ((*count) == 0) {
             subscriber->shutdown();

             num_subscribers_--;

             if (num_subscribers_ == 0)
                 ros::shutdown();
         }
     }
 }

void SharkLoggerApp::updateFilenames() {
     vector<string> parts;

     std::string prefix = options_.prefix;
     uint32_t ind = prefix.rfind(".bag");

     if (ind == prefix.size() - 4)
     {
       prefix.erase(ind);
       ind = prefix.rfind(".bag");
     }

     if (prefix.length() > 0)
         parts.push_back(prefix);
     if (options_.append_date)
         parts.push_back(timeToStr(ros::WallTime::now()));
     if (options_.split)
         parts.push_back(boost::lexical_cast<string>(split_count_));

     target_filename_ = parts[0];
     for (unsigned int i = 1; i < parts.size(); i++)
         target_filename_ += string("_") + parts[i];

     target_filename_ += string(".bag");
     write_filename_ = target_filename_ + string(".active");
 }

 void SharkLoggerApp::snapshotTrigger(std_msgs::Empty::ConstPtr trigger) {
     updateFilenames();

     ROS_INFO("Triggered snapshot recording with name %s.", target_filename_.c_str());

     {
         boost::mutex::scoped_lock lock(queue_mutex_);
         queue_queue_.push(OutgoingQueue(target_filename_, queue_, ros::Time::now()));
         queue_      = new std::queue<OutgoingMessage>;
         queue_size_ = 0;
     }

     queue_condition_.notify_all();
 }

 void SharkLoggerApp::startWriting() {
      bag_.setCompression(options_.compression);

      updateFilenames();
      try {
          bag_.open(write_filename_, rosbag::bagmode::Write);
      }
      catch (rosbag::BagException e) {
          ROS_ERROR("Error writing: %s", e.what());
          exit_code_ = 1;
          ros::shutdown();
      }
      ROS_INFO("Recording to %s.", target_filename_.c_str());
  }

  void SharkLoggerApp::stopWriting() {
      ROS_INFO("Closing %s.", target_filename_.c_str());
      bag_.close();
      rename(write_filename_.c_str(), target_filename_.c_str());
  }

  bool SharkLoggerApp::checkSize()
  {
      if (options_.max_size > 0)
      {
          if (bag_.getSize() > options_.max_size)
          {
              if (options_.split)
              {
                  stopWriting();
                  split_count_++;
                  startWriting();
              } else {
                  ros::shutdown();
                  return true;
              }
          }
      }
      return false;
  }

  bool SharkLoggerApp::checkDuration(const ros::Time& t)
  {
      if (options_.max_duration > ros::Duration(0))
      {
          if (t - start_time_ > options_.max_duration)
          {
              if (options_.split)
              {
                  while (start_time_ + options_.max_duration < t)
                  {
                      stopWriting();
                      split_count_++;
                      start_time_ += options_.max_duration;
                      startWriting();
                  }
              } else {
                  ros::shutdown();
                  return true;
              }
          }
      }
      return false;
  }


   void SharkLoggerApp::doRecord() {
       // Open bag file for writing
       startWriting();

       // Schedule the disk space check
       warn_next_ = ros::WallTime();
       checkDisk();
       check_disk_next_ = ros::WallTime::now() + ros::WallDuration().fromSec(20.0);

       // Technically the queue_mutex_ should be locked while checking empty.
       // Except it should only get checked if the node is not ok, and thus
       // it shouldn't be in contention.
       ros::NodeHandle nh;
       while (nh.ok() || !queue_->empty()) {
           boost::unique_lock<boost::mutex> lock(queue_mutex_);

           bool finished = false;
           while (queue_->empty()) {
               if (!nh.ok()) {
                   lock.release()->unlock();
                   finished = true;
                   break;
               }
               boost::xtime xt;
               boost::xtime_get(&xt, boost::TIME_UTC_);
               xt.nsec += 250000000;
               queue_condition_.timed_wait(lock, xt);
               if (checkDuration(ros::Time::now()))
               {
                   finished = true;
                   break;
               }
           }
           if (finished)
               break;

           OutgoingMessage out = queue_->front();
           queue_->pop();
           queue_size_ -= out.msg->size();

           lock.release()->unlock();

           if (checkSize())
               break;

           if (checkDuration(out.time))
               break;

           if (scheduledCheckDisk() && checkLogging())
               bag_.write(out.topic, out.time, *out.msg, out.connection_header);
       }

       stopWriting();
   }

   void SharkLoggerApp::doRecordSnapshotter() {
       ros::NodeHandle nh;

       while (nh.ok() || !queue_queue_.empty()) {
           boost::unique_lock<boost::mutex> lock(queue_mutex_);
           while (queue_queue_.empty()) {
               if (!nh.ok())
                   return;
               queue_condition_.wait(lock);
           }

           OutgoingQueue out_queue = queue_queue_.front();
           queue_queue_.pop();

           lock.release()->unlock();

           string target_filename = out_queue.filename;
           string write_filename  = target_filename + string(".active");

           try {
               bag_.open(write_filename, rosbag::bagmode::Write);
           }
           catch (rosbag::BagException ex) {
               ROS_ERROR("Error writing: %s", ex.what());
               return;
           }

           while (!out_queue.queue->empty()) {
               OutgoingMessage out = out_queue.queue->front();
               out_queue.queue->pop();

               bag_.write(out.topic, out.time, *out.msg);
           }

           stopWriting();
       }
   }

   void SharkLoggerApp::doCheckMaster(ros::TimerEvent const& e, ros::NodeHandle& node_handle) {
       ros::master::V_TopicInfo topics;
       if (ros::master::getTopics(topics)) {
                   BOOST_FOREACH(ros::master::TopicInfo const& t, topics) {
                           if (shouldSubscribeToTopic(t.name))
                                   subscribe(t.name);
                   }
       }

       if (options_.node != std::string(""))
       {

         XmlRpc::XmlRpcValue req;
         req[0] = ros::this_node::getName();
         req[1] = options_.node;
         XmlRpc::XmlRpcValue resp;
         XmlRpc::XmlRpcValue payload;

         if (ros::master::execute("lookupNode", req, resp, payload, true))
         {
           std::string peer_host;
           uint32_t peer_port;

           if (!ros::network::splitURI(static_cast<std::string>(resp[2]), peer_host, peer_port))
           {
             ROS_ERROR("Bad xml-rpc URI trying to inspect node at: [%s]", static_cast<std::string>(resp[2]).c_str());
           } else {

             XmlRpc::XmlRpcClient c(peer_host.c_str(), peer_port, "/");
             XmlRpc::XmlRpcValue req;
             XmlRpc::XmlRpcValue resp;
             req[0] = ros::this_node::getName();
             c.execute("getSubscriptions", req, resp);

             if (!c.isFault() && resp.size() > 0 && static_cast<int>(resp[0]) == 1)
             {
               for(int i = 0; i < resp[2].size(); i++)
               {
                 if (shouldSubscribeToTopic(resp[2][i][0], true))
                   subscribe(resp[2][i][0]);
               }
             } else {
               ROS_ERROR("Node at: [%s] failed to return subscriptions.", static_cast<std::string>(resp[2]).c_str());
             }
           }
         }
       }
   }

   void SharkLoggerApp::doTrigger() {
       ros::NodeHandle nh;
       ros::Publisher pub = nh.advertise<std_msgs::Empty>("snapshot_trigger", 1, true);
       pub.publish(std_msgs::Empty());

       ros::Timer terminate_timer = nh.createTimer(ros::Duration(1.0), boost::bind(&ros::shutdown));
       ros::spin();
   }

   bool SharkLoggerApp::scheduledCheckDisk() {
       boost::mutex::scoped_lock lock(check_disk_mutex_);

       if (ros::WallTime::now() < check_disk_next_)
           return true;

       check_disk_next_ += ros::WallDuration().fromSec(20.0);
       return checkDisk();
   }

   bool SharkLoggerApp::checkDisk() {
       struct statvfs fiData;
       if ((statvfs(bag_.getFileName().c_str(), &fiData)) < 0) {
           ROS_WARN("Failed to check filesystem stats.");
           return true;
       }

       unsigned long long free_space = 0;
       free_space = (unsigned long long) (fiData.f_bsize) * (unsigned long long) (fiData.f_bavail);
       if (free_space < 1073741824ull) {
           ROS_ERROR("Less than 1GB of space free on disk with %s.  Disabling recording.", bag_.getFileName().c_str());
           writing_enabled_ = false;
           return false;
       }
       else if (free_space < 5368709120ull) {
           ROS_WARN("Less than 5GB of space free on disk with %s.", bag_.getFileName().c_str());
       }
       else {
           writing_enabled_ = true;
       }

       return true;
   }

   bool SharkLoggerApp::checkLogging() {
       if (writing_enabled_)
           return true;

       ros::WallTime now = ros::WallTime::now();
       if (now >= warn_next_) {
           warn_next_ = now + ros::WallDuration().fromSec(5.0);
           ROS_WARN("Not logging message because logging disabled.  Most likely cause is a full disk.");
       }
       return false;
   }











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
