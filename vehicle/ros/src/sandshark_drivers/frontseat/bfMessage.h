#ifndef BFMESSAGE_H
#define BFMESSAGE_H

#include "fnmea.h"
#include "ros/ros.h"
#include <string>
#include <vector>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//Base for all bluefin->payload messages
class bfMessage: public fMessage {
  public:
    // Format the data contained within the message into a valid NMEA message
    void unparse(std::string &out);
    // Constructor - first 2 characters are always BF, and we always send a checksum
    bfMessage(std::string h, int n) :
        fMessage(h, "BF", n, true) {
    }
    virtual ~bfMessage() {
    }
};

// Class for the Backseat Control message 
// (Section 6.24 in Bluefin Standard Interface V1.12a)
class ctlMessage: public bfMessage {
  public:
    void setControlState(int c);

    ctlMessage() :
        bfMessage("CTL", 1) {
    }
};

// Class for the Navigation Update message 
// (Section 6.1 in Bluefin Standard Interface V1.12a)
class nvgMessage: public bfMessage {
  public:
    void setLatitudeAndHemisphere(float l);
    void setLongitudeAndHemisphere(float l);
    void setPositionQuality(int q);
    void setAltitude(float a);
    void setDepth(float d);
    void setHeading(float h);
    void setRoll(float r);
    void setPitch(float p);
    void setComputedTime(ros::Time t);

    nvgMessage() :
        bfMessage("NVG", 11) {
    }
};

// Class for the Velocity and Rate Update message 
// (Section 6.2 in Bluefin Standard Interface V1.12a)
class nvrMessage: public bfMessage {
  public:
    void setEastVelocity(float e);
    void setWestVelocity(float w);
    void setDownVelocity(float d);
    void setPitchRate(float p);
    void setRollRate(float r);
    void setHeadingRate(float h);

    nvrMessage() :
        bfMessage("NVR", 6) {
    }
};

// Class for the Payload Shutdown message 
// (Section 5.2 in Bluefin Standard Interface V1.12a)
class shtMessage: public bfMessage {
  public:
    //sht message has no fields
    shtMessage() :
        bfMessage("SHT", 0) {
    }
};

// Class for the Mission Status message 
// (Section 6.11 in Bluefin Standard Interface V1.12a)
class misMessage: public bfMessage {
  public:
    void setDiveFile(std::string d);
    void setMissionStatus(std::string m);
    void setAdditionalStatus(std::string a);

    misMessage() :
        bfMessage("MIS", 3) {
    }
};

// Class for the Message Acknowledgement message 
// (Section 6.21 in Bluefin Standard Interface V1.12a)
class ackMessage: public bfMessage {
  public:
    void setAckHeader(std::string a);
    void setReceivedTimestamp(std::string t);
    void setCommandID(int i);
    void setAckStatusCode(int s);
    void setDescription(std::string s);

    ackMessage() :
        bfMessage("ACK", 6) {
      _body[4] = "0"; //field 5 is reserved for now and fixed at "0"
    }
};

// Class for the Vehicle Interface Version message 
// (Section 5.7 in Bluefin Standard Interface V1.12a)
class verMessageF: public bfMessage {
  public:
    void setVersionString(std::string v);

    verMessageF() :
        bfMessage("VER", 1) {
    }
};
#endif
