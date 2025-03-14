#include "shulib/logger.hpp"
#include "pros/rtos.hpp"
#include <iostream>
#include <sstream>
#include <queue>

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
    const size_t maxSize = 900;
    std::queue<std::pair<std::string, MessageType>> messageQueue(std::deque<std::pair<std::string, MessageType>>(debugMessages.begin(), debugMessages.end()));
    debugMessages.clear();

    while (!messageQueue.empty()) {
      std::vector<std::pair<std::string, MessageType>> currentChunk;
      std::stringstream ss;
      ss << "{\"messages\": [";
      size_t currentSize = 15; // Initial size for {"messages": []}

      while (!messageQueue.empty()) {
        const auto &msg = messageQueue.front();
        std::string msgJson = "{ \"message\": \"" + msg.first + "\", \"type\": \"" + 
          (msg.second == MessageType::ERROR ? "error" :
           msg.second == MessageType::WARNING ? "warning" :
           msg.second == MessageType::SUCCESS ? "success" :
           msg.second == MessageType::ANNOUNCE ? "announce" :
           msg.second == MessageType::DEBUG ? "debug" : "log") + "\" }";

        size_t newSize = currentSize + msgJson.length() + (currentChunk.empty() ? 0 : 2); // +2 for ", "
        if (newSize > maxSize && !currentChunk.empty()) {
          break;
        }

        currentChunk.push_back(msg);
        if (!currentChunk.empty() && currentChunk.size() > 1) {
          ss << ", ";
        }
        ss << msgJson;
        currentSize = newSize;
        messageQueue.pop();
      }

      ss << "]}";
      printf("%s\n", ss.str().c_str());
      pros::delay(50);
    }
  }

  if (!telemetryData.empty()) {
    const size_t targetChunkSize = 900;
    std::vector<std::string> chunks;
    std::stringstream chunk;
    chunk << "{";
    bool isFirst = true;

    for (const auto &pair : telemetryData) {
      auto prevIt = previousTelemetryData.find(pair.first);
      if (prevIt == previousTelemetryData.end() || prevIt->second != pair.second) {
        std::string entry = "\"" + pair.first + "\": " + pair.second;
        
        if (!isFirst) {
          if (chunk.str().length() + entry.length() + 2 > targetChunkSize) {
            chunk << "}";
            chunks.push_back(chunk.str());
            chunk.str("");
            chunk << "{" << entry;
          } else {
            chunk << ", " << entry;
          }
        } else {
          chunk << entry;
          isFirst = false;
        }
        previousTelemetryData[pair.first] = pair.second;
      }
    }

    if (chunk.str().length() > 1) {
      chunk << "}";
      chunks.push_back(chunk.str());
    }

    for (size_t i = 0; i < chunks.size(); i++) {
      printf("%s\n", chunks[i].c_str());
      if (i < chunks.size() - 1) {
        pros::delay(5);
      }
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

template<>
void shulib::Logger::updateTelemetry(const std::string &key, const std::map<std::string, double> &value) {
    std::stringstream ss;
    ss << "{";
    bool first = true;
    for (const auto &pair : value) {
        if (!first) ss << ",";
        ss << "\"" << pair.first << "\":" << pair.second;
        first = false;
    }
    ss << "}";
    updateTelemetry(key, ss.str());
}

} // namespace shulib