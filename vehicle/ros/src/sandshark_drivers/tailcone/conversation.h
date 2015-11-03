#ifndef TC_CONVERSATION_H
#define TC_CONVERSATION_H
#include <string>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

namespace bluefin {
namespace sandshark {

class TCResponse {

  public:
    TCResponse(const std::string& commandName) :
        invalid(false), error(""), _commandName(commandName) {
    }

    bool parseResponse(std::string response) {
      // check unsupported or blank first

      boost::replace_all(response, "\r", "");
      boost::replace_all(response, "\n", "");

      boost::char_separator<char> sep(" ");
      boost::tokenizer<boost::char_separator<char> > tokens(response, sep);

      std::vector<std::string> vals;
      BOOST_FOREACH(const std::string& t, tokens) {
        vals.push_back(t);
      }

      if (vals.size() == 0) {
        error = "Blank return value";
        invalid = true;
        return false;
      } else if (vals.size() == 1 && vals[0] == "U") {
        error = "Command " + _commandName + " unsupported";
        invalid = true;
        return false;
      } else if (vals.size() == 1 && vals[0] == "E") {
        error = "Command " + _commandName + " reported error";
        invalid = true;
        return false;
      }

      //now parse the rest of the command
      return parse(vals);
    }

    bool invalid;
    std::string error;

  protected:
    std::string _commandName;

    virtual bool parse(std::vector<std::string> tokens) = 0;

};

template<typename T> class TCValueResponse: public TCResponse {
  public:
    T value;

    TCValueResponse(const std::string& commandName) :
        TCResponse(commandName) {
    }
  protected:
    bool parse(std::vector<std::string> tokens) {
      if (tokens.size() == 1) {
        try {
          value = boost::lexical_cast<T>(tokens[0]);
        } catch (boost::bad_lexical_cast e) {
          invalid = true;
          error = "Bad value for " + _commandName + ": '" + tokens[0] + "'";
          return false;
        }
      } else {
        invalid = true;
        error = "Bad number of values received for command " + _commandName;
        return false;
      }

      return true;
    }
};

template<typename T> class TCChannelValueResponse: public TCResponse {
  private:
    std::string _channel;

  public:

    T value;

    TCChannelValueResponse(const std::string& commandName, const std::string& channel) :
        TCResponse(commandName), _channel(channel) {
    }

  protected:
    bool parse(std::vector<std::string> tokens) {
      if (tokens.size() == 2) {
        if (tokens[0] == _channel) {
          try {
            value = boost::lexical_cast<T>(tokens[1]);
          } catch (boost::bad_lexical_cast e) {
            invalid = true;
            error = "Bad value for " + _commandName + ": '" + tokens[1] + "'";
            return false;
          }
        } else {
          invalid = true;
          error = "Wrong channel in response for command " + _commandName + "Expected '" + _channel + "' but got '"
              + tokens[0] + "'";
          return false;
        }
      } else {
        invalid = true;
        error = "Bad number of values received for command " + _commandName;
        return false;
      }

      return true;
    }
};
}
}
#endif
