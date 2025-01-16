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
  if (!telemetryData.empty()) {
    for (const auto &pair : telemetryData) {
      auto prevIt = previousTelemetryData.find(pair.first);
      if (prevIt == previousTelemetryData.end() || prevIt->second != pair.second) {
        std::cout << "{ \"" << pair.first << "\": " << pair.second << " }" << std::endl;
        previousTelemetryData[pair.first] = pair.second;
      }
    }
  }
  mutex.give();
}

void Logger::sendDebugMessages() {
  mutex.take();
  for (const auto &msg : debugMessages) {
    std::cout << "[DEBUG] " << msg << std::endl;
  }
  debugMessages.clear();
  mutex.give();
}

} // namespace shulib