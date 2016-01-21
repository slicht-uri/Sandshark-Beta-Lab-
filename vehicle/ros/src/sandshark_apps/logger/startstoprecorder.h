/*
 * startstoprecorder.h
 *
 *  Created on: Nov 2, 2015
 *      Author: brenden
 */

#ifndef STARTSTOPRECORDER_H_
#define STARTSTOPRECORDER_H_

#include <sys/stat.h>
#if !defined(_MSC_VER)
  #include <termios.h>
  #include <unistd.h>
#endif
#include <time.h>

#include <queue>
#include <string>
#include <vector>

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/regex.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Empty.h>
#include <topic_tools/shape_shifter.h>

#include "rosbag/bag.h"
#include "rosbag/stream.h"
#include "rosbag/macros.h"

namespace rosbag {

class OutgoingMessage
{
public:
    OutgoingMessage(std::string const& _topic, topic_tools::ShapeShifter::ConstPtr _msg, boost::shared_ptr<ros::M_string> _connection_header, ros::Time _time);

    std::string                         topic;
    topic_tools::ShapeShifter::ConstPtr msg;
    boost::shared_ptr<ros::M_string>    connection_header;
    ros::Time                           time;
};

class OutgoingQueue
{
public:
    OutgoingQueue(std::string const& _filename, std::queue<OutgoingMessage>* _queue, ros::Time _time);

    std::string                  filename;
    std::queue<OutgoingMessage>* queue;
    ros::Time                    time;
};

struct StartStopRecorderOptions
{
    StartStopRecorderOptions();

    bool            trigger;
    bool            record_all;
    bool            regex;
    bool            do_exclude;
    bool            quiet;
    bool            append_date;
    bool            snapshot;
    bool            verbose;
    CompressionType compression;
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
    unsigned long long min_space;
    std::string min_space_str;

    std::vector<std::string> topics;
};

class StartStopRecorder
{
public:
    StartStopRecorder(StartStopRecorderOptions const& options);
    StartStopRecorder();

    void setOptions(StartStopRecorderOptions const& options);

    void doTrigger();

    bool isSubscribed(std::string const& topic) const;

    boost::shared_ptr<ros::Subscriber> subscribe(std::string const& topic);

    int run();

    void startRecording();
    void stopRecording();

    //state variables to control when the record thread should actually be writing or not
    //they are also used in thread stop detection and shutdown
    bool _stopRunning;
    bool _startRunning;
private:
    void printUsage();

    void updateFilenames();
    void startWriting();
    void stopWriting();

    bool checkLogging();
    bool scheduledCheckDisk();
    bool checkDisk();

    void snapshotTrigger(std_msgs::Empty::ConstPtr trigger);
    void doQueue(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event, std::string const& topic, boost::shared_ptr<ros::Subscriber> subscriber, boost::shared_ptr<int> count);
    void doRecord();
    bool checkSize();
    bool checkDuration(const ros::Time&);
    void doRecordSnapshotter();
    void doCheckMaster(ros::TimerEvent const& e, ros::NodeHandle& node_handle);

    bool shouldSubscribeToTopic(std::string const& topic, bool from_node = false);

    template<class T>
    static std::string timeToStr(T ros_t);

private:
    StartStopRecorderOptions               options_;

    Bag                           bag_;

    std::string                   target_filename_;
    std::string                   write_filename_;

    std::set<std::string>         currently_recording_;  //!< set of currenly recording topics
    int                           num_subscribers_;      //!< used for book-keeping of our number of subscribers

    int                           exit_code_;            //!< eventual exit code

    boost::condition_variable_any queue_condition_;      //!< conditional variable for queue
    boost::mutex                  queue_mutex_;          //!< mutex for queue
    std::queue<OutgoingMessage>*  queue_;                //!< queue for storing
    uint64_t                      queue_size_;           //!< queue size
    uint64_t                      max_queue_size_;       //!< max queue size

    uint64_t                      split_count_;          //!< split count

    std::queue<OutgoingQueue>     queue_queue_;          //!< queue of queues to be used by the snapshot recorders

    ros::Time                     last_buffer_warn_;

    ros::Time                     start_time_;

    bool                          writing_enabled_;
    boost::mutex                  check_disk_mutex_;
    ros::WallTime                 check_disk_next_;
    ros::WallTime                 warn_next_;

    //Pointer to the recordthread
    boost::thread * _recordThreadPtr;
};

} // namespace rosbag


#endif /* STARTSTOPRECORDER_H_ */
