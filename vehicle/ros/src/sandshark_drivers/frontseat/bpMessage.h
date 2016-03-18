#ifndef BPMESSAGE_H
#define BPMESSAGE_H

#include "fnmea.h"
#include "ros/ros.h"
#include <string>
#include <vector>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Base Class for payload->bluefin messages
class bpMessage: public fMessage {
  public:
    // Parse an incoming message, return false if a failure occurrs or
    // a required checksum is absent or invalid
    bool parse(std::string in);
    // Parse the body part of a received string representation of the message
    virtual bool parseRest(std::string in);
    // Constructor - first 2 characters are always BP
    bpMessage(std::string h, int n, bool c) :
        fMessage(h, "BP", n, c) {
    }
    virtual ~bpMessage() {
    }
};

// Class for the Modify Current Behavior message 
// (Section 7.12 in Bluefin Standard Interface V1.12a)
class rmbMessage: public bpMessage {
  public:
    float getHorizontal();
    float getVertical();
    float getTranslational();
    int getHorizontalMode();
    int getVerticalMode();
    int getTranslationalMode();

    rmbMessage() :
        bpMessage("RMB", 6, false) {
    }
};

// Class for the Logging Control message 
// (Section 7.1 in Bluefin Standard Interface V1.12a)
class logMessage: public bpMessage {
  public:
    std::string getMsgHeader();
    std::string getOnOff();

    logMessage() :
        bpMessage("LOG", 2, false) {
    }
};

// Class for the Payload Status message 
// (Section 7.2 in Bluefin Standard Interface V1.12a)
class stsMessage: public bpMessage {
  public:
    int getStatusFlag();
    std::string getStatusText();

    stsMessage() :
        bpMessage("STS", 2, false) {
    }
};

// Class for the Abort Mission message 
// (Section 7.16 in Bluefin Standard Interface V1.12a)
class abtMessage: public bpMessage {
  public:
    std::string getAbortMessage();
    int getAbortCode();

    abtMessage() :
        bpMessage("ABT", 2, false) {
    }
};

// Class for the Vehicle Interface Version message 
// (Section 5.7 in Bluefin Standard Interface V1.12a)
class verMessageP: public bpMessage {
  public:
    std::string getVersionString();

    verMessageP() :
        bpMessage("VER", 1, false) {
    }
};

#endif
