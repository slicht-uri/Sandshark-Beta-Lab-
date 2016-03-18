#include "fnmea.h"
#include <sstream>
#include "boost/date_time/local_time/local_time.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/gregorian/gregorian.hpp"
#include "boost/date_time/date_formatting.hpp"
#include "boost/date_time/gregorian/greg_month.hpp"

/**************************** Getters ***************************/
std::string fMessage::getHeader(void) {
  return _header;
}
std::string fMessage::getTime(void) {
  return _time;
}
std::vector<std::string> fMessage::getBody(void) {
  return _body;
}
/**************************** Getters ***************************/

/**************************** Setters ***************************/
void fMessage::setHeader(std::string h) {
  _header = h;
}
void fMessage::setTime(std::string t) {
  _time = t;
}
void fMessage::addBody(std::string b) {
  _body.push_back(b);
}
/**************************** Setters ***************************/

/**************************** CHECKSUM CODE *********************/
bool fMessage::verifyChecksum(std::string message) {
  std::string receivedChecksum = message.substr(message.length() - 2, 2);
  message.erase(message.length() - 2, 2);
  std::string calculatedChecksum = getChecksum(message);
  return (calculatedChecksum.compare(receivedChecksum) == 0 ? true : false);
}

std::string fMessage::getChecksum(std::string message) {
  //we get the message with the leading $ and trailing *, which aren't used in the checksum calculation
  message.erase(0, 1);
  message.erase(message.length() - 1, 1);

  unsigned char checksum = 0;
  for (unsigned int i = 0; i < message.length(); i++) {
    checksum ^= message.at(i);
  }
  char tmp[4];
  sprintf(tmp, "%02X", checksum);
  return std::string(tmp);
}
/**************************** CHECKSUM CODE *********************/

/**************************** UTILITIES *************************/
std::string fMessage::getBFTimestring(ros::Time t) {
  //This is pretty silly, but the boost facet way was not working
  //Call me baker's chocolate, because I am so bitter about this
  boost::posix_time::ptime tNow;
  tNow = t.toBoost();

  std::string timeString = "";
  timeString.append(str(boost::format("%02d") % tNow.time_of_day().hours()));
  timeString.append(str(boost::format("%02d") % tNow.time_of_day().minutes()));
  timeString.append(str(boost::format("%02d") % tNow.time_of_day().seconds()));
  timeString.append(".");
  timeString.append(
      str(
          boost::format("%02d")
              % ((tNow.time_of_day().total_milliseconds() - (tNow.time_of_day().total_seconds() * 1000)) / 10)));

  return timeString;
  //All of this should work but it only ever prints in the standard format....
  //I'm angrily and bitterly leaving this in here so that one day, I can come back
  //and serve up some white hot boost justice in these timestamps faces.
  /*
   printf("Time0 String is %s\n", boost::gregorian::to_iso_string(tNow.c_str());

   boost::local_time::local_time_facet *time_facet = new boost::local_time::local_time_facet("%H%M%s%.2f");

   std::stringstream ss;

   ss.imbue(std::locale(ss.getloc(), time_facet));
   time_facet->format(boost::local_time::local_time_facet::iso_time_format_specifier);
   ss.str("");
   ss << tNow;
   printf("Time String is %s\n", ss.str().c_str());
   return ss.str();
   //shouldn't need a delete here according to boost docs, the ss will own the pointer
   */
}
/**************************** UTILITIES *************************/
