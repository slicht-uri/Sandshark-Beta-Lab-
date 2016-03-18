#ifndef FNMEA_H
#define FNMEA_H

#include <string>
#include <vector>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ros/ros.h"

//Base class for representing a nmea Bluefin Standard Payload Interface Message
class fMessage {
  protected:
    std::string _header; //NVG, RMB, etc
    std::string _time; //string representation of the message timestamp
    std::string _direction; //B->P "BF" P->B "BP"
    std::vector<std::string> _body; //variable number of actual body fields as strings

    int _numBodyFields;
    bool _checksumRequired;

    // returns a bluefin format representation of a passed ros time
    std::string getBFTimestring(ros::Time t);

    // return whether or not the passes message string's checksum is correct
    bool verifyChecksum(std::string message);
    // return an ascii representation of the NMEA checksum of the passed string
    std::string getChecksum(std::string message);
  public:
    // return the header string of the message
    std::string getHeader();
    // return std::string representation of when the message was received
    std::string getTime();
    // return the body vector of strings of the message
    std::vector<std::string> getBody();

    // set the header string of the message
    void setHeader(std::string);
    // set the time string of the message
    void setTime(std::string);
    // add a body string to the message, order matters
    void addBody(std::string);

    fMessage(std::string h, std::string d, int n, bool c) {
      _header = h;
      _direction = d;
      _numBodyFields = n;
      _checksumRequired = c;
      //make all the fields initially empty strings
      for (int i = 0; i < _numBodyFields; i++) {
        _body.push_back("");
      }
    }
};

#endif
