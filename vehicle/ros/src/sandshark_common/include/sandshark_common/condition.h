#ifndef _BLUEFIN__SANDSHARK__SANDSHARK_COMMON__CONDITION_H_
#define _BLUEFIN__SANDSHARK__SANDSHARK_COMMON__CONDITION_H_

#include <string>
#include <map>
#include <vector>
#include <set>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

namespace bluefin {
namespace sandshark {
class Condition {
  protected:
    std::string _topicName;
    std::string _paramName;
    int _priority;
    bool _isTriggered;
    ros::NodeHandle *_prNode;

  public:
    Condition() {
      _priority = 0;
      _isTriggered = false;
      _paramName = "";
    }

    std::string getTopicName() const {
      return _topicName;
    }

    std::string getParamName() const {
      return _paramName;
    }

    void reset() {
      _isTriggered = false;
    }

    bool isTriggered() const {
      return _isTriggered;
    }
    int getPriority() const {
      return _priority;
    }
    virtual std::string getReason() = 0;
    virtual void processMessage(const topic_tools::ShapeShifter::ConstPtr & msg) = 0;
    virtual ~Condition() {
    }
};

template<typename MsgType, typename InType, typename CompareOp>
class ConditionTemplate: public Condition {
  protected:
    boost::shared_ptr<MsgType> _msg;
    CompareOp _operation;
    InType _compareVal;

    std::string _opDescription;

    virtual void handleMessage() = 0;
  public:
    explicit ConditionTemplate(CompareOp op, InType val, std::string & desc) :
        _operation(op), _compareVal(val), _opDescription(desc) {
      reset();
    }

    void processMessage(const topic_tools::ShapeShifter::ConstPtr & msg) {
      _msg = msg->instantiate<MsgType>();
      handleMessage();
    }

    void setCompareVal(InType c) {
      _compareVal = c;
    }

    InType getCompareVal() {
      return _compareVal;
    }

    virtual ~ConditionTemplate() {
    }
};

class Action {
  public:
    virtual void createPublisher(ros::NodeHandle *node) = 0;
    virtual void execute(std::string reason) = 0;
    virtual ~Action() {
    }
};

class AbortAction: public Action {
  private:
    ros::Publisher _abortPub;
  public:
    void createPublisher(ros::NodeHandle *node);
    void execute(std::string reason);

};

class SolidRedLEDAction: public Action {
  private:
    ros::Publisher _redLEDPub;
  public:
    void createPublisher(ros::NodeHandle *node);
    void execute(std::string reason);

};

class SolidAllLEDAction: public Action {
  private:
    ros::Publisher _redLEDPub;
    ros::Publisher _amberLEDPub;
    ros::Publisher _greenLEDPub;
  public:
    void createPublisher(ros::NodeHandle *node);
    void execute(std::string reason);

};

class ConditionManager {
  private:
    static ConditionManager *_instance;
    ~ConditionManager() {
    }
    explicit ConditionManager() {
      _abortAction = new AbortAction();
      _solidRedLEDAction = new SolidRedLEDAction();
      _solidAllLEDAction = new SolidAllLEDAction();
    }

    Action *_abortAction;
    Action *_solidRedLEDAction;
    Action *_solidAllLEDAction;

    std::map<std::string, ros::Subscriber> _topicToSub;
    std::map<std::string, std::vector<Condition *> > _topicToConditions;
    std::map<Condition *, std::set<Action *> > _conditionToActions;

    struct ConditionCompare {
        bool operator()(const Condition * lcond, const Condition * rcond) {
          return lcond->getPriority() < rcond->getPriority();
        }
    };

    std::vector<Condition *> _orderedConditions;

  public:
    static ConditionManager *instance() {
      if (!_instance) {
        _instance = new ConditionManager();
      }
      return _instance;
    }
    void updateAbortAction(ros::NodeHandle *node);
    void updateSolidRedLEDAction(ros::NodeHandle *node);
    void updateSolidAllLEDAction(ros::NodeHandle *node);

    bool addCondition(ros::NodeHandle *node, Condition * cond);
    void linkConditionToAction(Condition * cond, Action *act);
    void msgCallback(const topic_tools::ShapeShifter::ConstPtr & msg, const std::string & topic);

    void executeTriggeredConditions();

    Action *getAbortAction() {
      return _abortAction;
    }
    Action *getSolidRedLEDAction() {
      return _solidRedLEDAction;
    }
    Action *getSolidAllLEDAction() {
      return _solidAllLEDAction;
    }
};
}
}
#endif

