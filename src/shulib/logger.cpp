#include "shulib/logger.hpp"
#include "pros/rtos.hpp"
#include <iostream>

namespace shulib {

void Logger::update() {
  uint32_t currentTime = pros::millis();

  if (currentTime - lastTelemetryTime >= telemetryInterval) {
    sendTelemetry();
    lastTelemetryTime = currentTime;
  }

  sendDebugMessages();
}

void Logger::sendTelemetry() {
  mutex.take();

  if (!debugMessages.empty()) {
    std::string messages = "[";
    for (const auto &msg : debugMessages) {
      messages += "{ \"message\": \"" + msg.first + "\", \"type\": \"" + 
        (msg.second == MessageType::ERROR ? "error" :
         msg.second == MessageType::WARNING ? "warning" :
         msg.second == MessageType::SUCCESS ? "success" :
         msg.second == MessageType::ANNOUNCE ? "announce" :
         msg.second == MessageType::DEBUG ? "debug" : "log") + "\" }";

      if (&msg != &debugMessages.back()) {
        messages += ", ";
      }
    }
    messages += "]";
    telemetryData["messages"] = messages;
    debugMessages.clear();
  }

  if (!telemetryData.empty()) {
    bool first = true;
    std::string telemetry = "{";
    for (const auto &pair : telemetryData) {
      auto prevIt = previousTelemetryData.find(pair.first);
      if (prevIt == previousTelemetryData.end() || prevIt->second != pair.second) {
        if (!first) telemetry += ", ";
        telemetry += "\"" + pair.first + "\": " + pair.second;
        first = false;
        previousTelemetryData[pair.first] = pair.second;
      }
    }
    telemetry += "}";
    if (telemetry != "{}") {
      printf("%s\n", telemetry.c_str());
    }
  }
  mutex.give();
}

void Logger::init() {
  printf("Initializing logger...\n");
  if (telemetryTask == nullptr) {
    telemetryTask = new pros::Task([this] {
      while (true) {
        this->sendTelemetry();
        pros::delay(this->telemetryInterval);
      }
    });
  }
  success("Logger initialized!");
}

} // namespace shulib