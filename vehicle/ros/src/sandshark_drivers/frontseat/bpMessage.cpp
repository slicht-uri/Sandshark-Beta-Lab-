#include "bpMessage.h"
#include <sstream>
#include "boost/date_time/local_time/local_time.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/gregorian/gregorian.hpp"
#include "boost/date_time/date_formatting.hpp"
#include "boost/date_time/gregorian/greg_month.hpp"
#include "boost/algorithm/string/split.hpp"
#include "boost/algorithm/string/classification.hpp"
#include "boost/algorithm/string.hpp"
#include "boost/tokenizer.hpp"
#include "boost/foreach.hpp"

//Parse all common beginning bpMessage fields, up to and including the timestamp
//returns false on failures and false if parseRest returns false
bool bpMessage::parse(std::string in) {
  printf("BP: parse\n");
  //basic check to see if the message is long enough to parse
  //the min length here assumes no fields
  if(in.length() < (16 + _checksumRequired? 3 : 0)) {
    ROS_ERROR("bpMessage parse detected too short of a message to parse");
    return false;
  }

  //get rid of stuff that if left in the message will
  //massacre boost lexical case
  boost::algorithm::erase_all(in, "\r");
  boost::algorithm::erase_all(in, "\n");

  // Do we have a checksum in the string?
  if (boost::algorithm::contains(in, "*")) {
    //if we need one and it is wrong, return false
    if (_checksumRequired && !verifyChecksum(in)) {
      ROS_ERROR("bpMessage parse detected an incorrect checksum");
      return false;
    }
    //get rid of the checksum
    in.erase(in.length()-3, std::string::npos);
  } else { //checksum not present
    if (_checksumRequired) {
      ROS_ERROR("bpMessage parse did not detect required checksum");
      return false;
    }
  }

  //parse the direction, always len 2
  _direction = in.substr(1, 2);
  in.erase(0, 3);
  ROS_ERROR("BP: Direction erased: remaining %s\n", in.c_str());
  //then the header, always len 3
  _header = in.substr(0,3);
  in.erase(0, 4);
  ROS_ERROR("BP: Header erased: remaining %s\n", in.c_str());

  if(_header.compare("LOG") == 0) {
    ROS_ERROR("----- LOG detected, ignoring time -----\n");

  } else {

    //and time, could be variable length
    std::size_t timeEnd = in.find(",");
    if (timeEnd == std::string::npos) {
      //what the smoke has happened?  better return false!!
      ROS_ERROR("bpMessage parse detected an improperly formed message");
      return false;
    }
    _time = in.substr(0,timeEnd);
    in.erase(0, timeEnd+1);
  }

  ROS_ERROR("BP: Going to rest: remaining %s\n", in.c_str());


  //subclasses may want to do weird stuff from here on out
  return parseRest(in);
}

//very basic for now, just split into strings at commas
bool bpMessage::parseRest(std::string in) {
  //TODO deal with exceptions here, but it doesn't seem like it will make any
  boost::split(_body, in, boost::is_any_of(","));
  return true;
}

/*****************Rmb Message********************/
float rmbMessage::getHorizontal() {
  printf("0 casting %s\n", _body[0].c_str());
  return boost::lexical_cast<float>(_body[0]);
}

float rmbMessage::getVertical() {
  printf("1 casting %s\n", _body[1].c_str());
  return boost::lexical_cast<float>(_body[1]);
}

float rmbMessage::getTranslational() {
  printf("3 casting %s\n", _body[3].c_str());
  return boost::lexical_cast<float>(_body[3]);
}

int rmbMessage::getHorizontalMode() {
  printf("5 casting %s\n", _body[5].c_str());
  return boost::lexical_cast<int>(_body[5]);
}

int rmbMessage::getVerticalMode() {
  printf("2 casting %s\n", _body[2].c_str());
  return boost::lexical_cast<int>(_body[2]);
}

int rmbMessage::getTranslationalMode() {
  printf("4 casting %s\n", _body[4].c_str());
  return boost::lexical_cast<int>(_body[4]);
}
/*****************Rmb Message********************/

/*****************Log Message********************/
std::string logMessage::getMsgHeader() {
  return _body[0];
}

std::string logMessage::getOnOff() {
  return _body[1];
}
/*****************Log Message********************/

/*****************Sts Message********************/
int stsMessage::getStatusFlag() {
  return boost::lexical_cast<int>(_body[0]);
}

std::string stsMessage::getStatusText() {
  return _body[1];
}
/*****************Sts Message********************/

/*****************Abt Message********************/
std::string abtMessage::getAbortMessage() {
  return _body[0];
}

int abtMessage::getAbortCode() {
  return boost::lexical_cast<int>(_body[1]);
}
/*****************Abt Message********************/

/*****************Ver Message********************/
std::string verMessageP::getVersionString() {
  return _body[0];
}
/*****************Ver Message********************/
