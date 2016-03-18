#include "bfMessage.h"
#include <sstream>
#include "boost/date_time/local_time/local_time.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/gregorian/gregorian.hpp"
#include "boost/date_time/date_formatting.hpp"
#include "boost/date_time/gregorian/greg_month.hpp"

void bfMessage::unparse(std::string &out) {
  std::string delimiterString = ",";
  //Form the header
  out.append("$");
  out.append(_direction);
  out.append(_header);
  out.append(delimiterString);
  //Add timestamp (current time should suffice)
  out.append(getBFTimestring(ros::Time::now()));
  out.append(delimiterString);
  // Put all the bodies into the message, separated by delimeter string
  for (std::vector<std::string>::const_iterator iterator = _body.begin(), end = _body.end(); iterator != end;
      ++iterator) {
    out.append(*iterator);
    out.append(delimiterString);
  }
  //replace the last delimiter with an *
  out.erase(out.length() - 1, 1);
  out.append("*");
  // We'll always send a checksum, whether it is optional or not
  // Finally compute and append the checksum
  out.append(getChecksum(out));
  out.append("\r\n");
}

/******************Ctl Message*****************/
void ctlMessage::setControlState(int c) {
  _body[0] = str(boost::format("%d") % c);
}
/******************Ctl Message*****************/

/*****************Nvg Message*********************/
void nvgMessage::setLatitudeAndHemisphere(float l) {
  _body[1] = "N";
  if (l < 0) {
    _body[1] = "S";
    l *= -1.0;
  }
  float degrees = (float)((int)l);
  _body[0] = str(boost::format("%02d") % degrees);
  float minutes = (l - degrees) * 60.0;
  _body[0] = _body[0] + str(boost::format("%04.6f") % minutes);
}

void nvgMessage::setLongitudeAndHemisphere(float l) {
  _body[3] = "E";
  if (l < 0) {
    _body[3] = "W";
    l *= -1.0;
  }
  float degrees = (float)((int)l);
  _body[2] = str(boost::format("%03d") % degrees);
  float minutes = (l - degrees) * 60.0;
  _body[2] = _body[2] + str(boost::format("%04.6f") % minutes);
}

void nvgMessage::setPositionQuality(int q) {
  _body[4] = str(boost::format("%d") % q);
}

void nvgMessage::setAltitude(float a) {
  _body[5] = str(boost::format("%.2f") % a);
}

void nvgMessage::setDepth(float d) {
  _body[6] = str(boost::format("%f") % d);
}

void nvgMessage::setHeading(float h) {
  _body[7] = str(boost::format("%f") % h);
}

void nvgMessage::setRoll(float r) {
  _body[8] = str(boost::format("%f") % r);
}

void nvgMessage::setPitch(float p) {
  _body[9] = str(boost::format("%f") % p);
}

void nvgMessage::setComputedTime(ros::Time t) {
  _body[10] = getBFTimestring(t);
}
/*****************Nvg Message*********************/

/*****************Nvr Message********************/
void nvrMessage::setEastVelocity(float e) {
  _body[0] = str(boost::format("%.2f") % e);
}

void nvrMessage::setWestVelocity(float w) {
  _body[1] = str(boost::format("%.2f") % w);
}

void nvrMessage::setDownVelocity(float d) {
  _body[2] = str(boost::format("%.2f") % d);
}

void nvrMessage::setPitchRate(float p) {
  _body[3] = str(boost::format("%.2f") % p);
}

void nvrMessage::setRollRate(float r) {
  _body[4] = str(boost::format("%.2f") % r);
}

void nvrMessage::setHeadingRate(float h) {
  _body[5] = str(boost::format("%.2f") % h);
}
/*****************Nvr Message********************/

/*****************Sht Message********************/
//sht has no fields
/*****************Sht Message********************/

/*****************Mis Message********************/
void misMessage::setDiveFile(std::string d) {
  _body[0] = d;
}

void misMessage::setMissionStatus(std::string m) {
  _body[1] = m;
}

void misMessage::setAdditionalStatus(std::string a) {
  _body[2] = a;
}
/*****************Mis Message********************/

/*****************Ack Message********************/
void ackMessage::setAckHeader(std::string a) {
  _body[0] = a;
}

void ackMessage::setReceivedTimestamp(std::string t) {
  _body[1] = t;
}

void ackMessage::setCommandID(int i) {
  //_body[2] = str( boost::format("%04d") % i);
  //will always be blank for now
  _body[2] = "";
}

void ackMessage::setAckStatusCode(int s) {
  _body[3] = str(boost::format("%d") % s);
}

void ackMessage::setDescription(std::string s) {
  _body[5] = s;
}
/*****************Ack Message********************/

/*****************Ver Message********************/
void verMessageF::setVersionString(std::string s) {
  _body[0] = s;
}
/*****************Ver Message********************/
