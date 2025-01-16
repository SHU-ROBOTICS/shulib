#pragma once

#include "pros/rtos.hpp"
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <iomanip>


namespace shulib {

class Logger {
public:
  static Logger &getInstance() {
    static Logger instance;
    return instance;
  }

  // Update a telemetry value
  template <typename T>
  void updateTelemetry(const std::string &key, const T &value) {
    mutex.take();
    telemetryData[key] = std::to_string(value);
    mutex.give();
  }

  // Specialization for string types
  void updateTelemetry(const std::string &key, const std::string &value) {
    mutex.take();
    telemetryData[key] = value;
    mutex.give();
  }

  void updateTelemetry(const std::string &key, const char* value) {
    mutex.take();
    telemetryData[key] = value;
    mutex.give();
  }

  // Add a debug message
  void debug(const std::string &message) {
    mutex.take();
    debugMessages.push_back(message);
    mutex.give();
  }

  // Set how often telemetry should be sent (in ms)
  void setTelemetryInterval(int ms) { telemetryInterval = ms; }

  // Call this periodically to send data
  void update();

  // Message type enum
  enum class MessageType { LOG, ERROR, WARNING, SUCCESS, ANNOUNCE };

  // Different message type methods
  void error(const std::string &message) { debug("[ERROR] " + message); }

  void warning(const std::string &message) { debug("[WARNING] " + message); }

  void success(const std::string &message) { debug("[SUCCESS] " + message); }

  void announce(const std::string &message) { debug("[ANNOUNCE] " + message); }

  void log(const std::string &message) { debug("[LOG] " + message); }

  // Variadic template versions for multiple arguments
  template <typename... Args> void error(const Args &...args) {
    std::stringstream ss;
    (ss << ... << args);
    error(ss.str());
  }

  template <typename... Args> void warning(const Args &...args) {
    std::stringstream ss;
    (ss << ... << args);
    warning(ss.str());
  }

  template <typename... Args> void success(const Args &...args) {
    std::stringstream ss;
    (ss << ... << args);
    success(ss.str());
  }

  template <typename... Args> void announce(const Args &...args) {
    std::stringstream ss;
    (ss << ... << args);
    announce(ss.str());
  }

  template <typename... Args> void log(const Args &...args) {
    std::stringstream ss;
    (ss << ... << args);
    log(ss.str());
  }

private:
  Logger() : lastTelemetryTime(0), telemetryInterval(200) {}
  Logger(const Logger &) = delete;
  Logger &operator=(const Logger &) = delete;

  // Helper function to format JSON strings
  std::string formatJSON(const std::unordered_map<std::string, std::string>& data) {
    std::stringstream ss;
    ss << "{";
    bool first = true;
    for (const auto& pair : data) {
      if (!first) ss << ",";
      ss << "\"" << pair.first << "\":";
      
      // Check if the value looks like a JSON object
      if (pair.second.front() == '{' && pair.second.back() == '}') {
        ss << pair.second;  // Don't quote JSON objects
      } else {
        ss << "\"" << pair.second << "\"";  // Quote normal strings
      }
      first = false;
    }
    ss << "}";
    return ss.str();
  }

  // Helper function to escape JSON strings
  std::string escapeJSONString(const std::string& input) {
    std::stringstream ss;
    for (char c : input) {
      switch (c) {
        case '"': ss << "\\\""; break;
        case '\\': ss << "\\\\"; break;
        case '\b': ss << "\\b"; break;
        case '\f': ss << "\\f"; break;
        case '\n': ss << "\\n"; break;
        case '\r': ss << "\\r"; break;
        case '\t': ss << "\\t"; break;
        default:
          if ('\x00' <= c && c <= '\x1f') {
            ss << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c);
          } else {
            ss << c;
          }
      }
    }
    return ss.str();
  }

  std::unordered_map<std::string, std::string> telemetryData;
  std::unordered_map<std::string, std::string> previousTelemetryData;
  std::vector<std::string> debugMessages;
  pros::Mutex mutex;
  uint32_t lastTelemetryTime;
  int telemetryInterval;

  void sendTelemetry();
  void sendDebugMessages();
};

// Global accessor function
inline Logger &logger() { return Logger::getInstance(); }

} // namespace shulib