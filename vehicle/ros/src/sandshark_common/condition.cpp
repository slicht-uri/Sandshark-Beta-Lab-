#include <sandshark_common/condition.h>
#include <cstddef>

#include <sandshark_msgs/ApplicationAbort.h>
#include <sandshark_msgs/LEDCommand.h>

namespace bluefin {
namespace sandshark {

ConditionManager *ConditionManager::_instance = NULL;

void AbortAction::createPublisher(ros::NodeHandle *node) {
  _abortPub = node->advertise<sandshark_msgs::ApplicationAbort>("/objectivecontrol/abort", 1);
}

void AbortAction::execute(std::string reason) {
  ROS_ERROR("AbortAction activated! Reason = %s", reason.c_str());
  sandshark_msgs::ApplicationAbort amsg;
  amsg.abort = true;
  amsg.reason = reason;
  _abortPub.publish(amsg);
}

void SolidRedLEDAction::createPublisher(ros::NodeHandle *node) {
  _redLEDPub = node->advertise<sandshark_msgs::LEDCommand>("/ledcontrol/red", 1);
}

void SolidRedLEDAction::execute(std::string reason) {
  sandshark_msgs::LEDCommand msg;
  msg.command = sandshark_msgs::LEDCommand::COMMAND_ON;
  _redLEDPub.publish(msg);
}

void SolidAllLEDAction::createPublisher(ros::NodeHandle *node) {
  _redLEDPub = node->advertise<sandshark_msgs::LEDCommand>("/ledcontrol/red", 1);
  _greenLEDPub = node->advertise<sandshark_msgs::LEDCommand>("/ledcontrol/green", 1);
  _amberLEDPub = node->advertise<sandshark_msgs::LEDCommand>("/ledcontrol/amber", 1);
}

void SolidAllLEDAction::execute(std::string reason) {
  sandshark_msgs::LEDCommand msg;
  msg.command = sandshark_msgs::LEDCommand::COMMAND_ON;
  _redLEDPub.publish(msg);
  _greenLEDPub.publish(msg);
  _amberLEDPub.publish(msg);
}

void ConditionManager::executeTriggeredConditions() {
  for (std::vector<Condition *>::iterator iter = _orderedConditions.begin(); iter != _orderedConditions.end(); ++iter) {
    if ((*iter)->isTriggered()) {
      std::map<Condition *, std::set<Action *> >::iterator actIter = _conditionToActions.find(*iter);
      if (actIter != _conditionToActions.end()) {
        for (std::set<Action *>::iterator actSet = actIter->second.begin(); actSet != actIter->second.end(); ++actSet) {
          (*actSet)->execute(actIter->first->getReason());
        }
      } else {
        ROS_WARN("No Action for triggered condition");
      }
    }
  }
}

void ConditionManager::updateAbortAction(ros::NodeHandle * node) {
  _abortAction->createPublisher(node);
}

void ConditionManager::updateSolidRedLEDAction(ros::NodeHandle * node) {
  _solidRedLEDAction->createPublisher(node);
}

void ConditionManager::updateSolidAllLEDAction(ros::NodeHandle * node) {
  _solidAllLEDAction->createPublisher(node);
}

bool ConditionManager::addCondition(ros::NodeHandle * node, Condition * cond) {
  std::string topic = cond->getTopicName();
  std::map<std::string, std::vector<Condition *> >::iterator iter = _topicToConditions.find(topic);
  if (iter == _topicToConditions.end()) {
    _topicToSub[topic] = node->subscribe<topic_tools::ShapeShifter>(topic, 1,
        boost::bind(&ConditionManager::msgCallback, this, _1, topic));
  }

  //Check for same condition insertion?
  _topicToConditions[topic].push_back(cond);

  _orderedConditions.push_back(cond);
  std::sort(_orderedConditions.begin(), _orderedConditions.end(), ConditionCompare());
  return false;
}

void ConditionManager::msgCallback(const topic_tools::ShapeShifter::ConstPtr & msg, const std::string & topic) {
  std::map<std::string, std::vector<Condition *> >::iterator iter = _topicToConditions.find(topic);
  if (iter != _topicToConditions.end()) {
    for (std::vector<Condition *>::iterator viter = _topicToConditions[topic].begin();
        viter != _topicToConditions[topic].end(); ++viter) {
      (*viter)->processMessage(msg);
    }
  }
}

void ConditionManager::linkConditionToAction(Condition *cond, Action *act) {
  _conditionToActions[cond].insert(act);
}

}
}
