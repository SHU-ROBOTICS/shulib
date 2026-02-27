#include "shulib/logger.hpp"
#include "pros/rtos.hpp"
#include <iostream>
#include <sstream>
#include <queue>
#include <iomanip>

namespace shulib {

// Helper to format a timestamp from pros::millis() as MM:SS.mmm
static std::string getTimestamp() {
    uint32_t ms = pros::millis();
    uint32_t totalSeconds = ms / 1000;
    uint32_t minutes = totalSeconds / 60;
    uint32_t seconds = totalSeconds % 60;
    uint32_t millis = ms % 1000;

    std::stringstream ss;
    ss << std::setfill('0') << std::setw(2) << minutes << ":"
       << std::setfill('0') << std::setw(2) << seconds << "."
       << std::setfill('0') << std::setw(3) << millis;
    return ss.str();
}

// Helper to convert MessageType to a fixed-width label
static std::string getTypeLabel(Logger::MessageType type) {
    switch (type) {
        case Logger::MessageType::ERROR:    return "ERROR";
        case Logger::MessageType::WARNING:  return "WARN ";
        case Logger::MessageType::SUCCESS:  return "OK   ";
        case Logger::MessageType::ANNOUNCE: return "ANNC ";
        case Logger::MessageType::DEBUG:    return "DEBUG";
        case Logger::MessageType::LOG:
        default:                            return "LOG  ";
    }
}

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

    // --- HUMAN-READABLE LOG MESSAGES ---
    // Output each message as a clean, readable line:
    //   [MM:SS.mmm] [TYPE ] message text here
    if (!debugMessages.empty()) {
        for (const auto &msg : debugMessages) {
            std::stringstream line;
            line << "[" << getTimestamp() << "] "
                 << "[" << getTypeLabel(msg.second) << "] "
                 << msg.first;
            printf("%s\n", line.str().c_str());
        }
        debugMessages.clear();
    }

    // --- TELEMETRY JSON (unchanged format, prefixed for easy filtering) ---
    // Only sends values that changed since last send.
    // Prefixed with [TELEM] so you can visually skip it or grep for it.
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
            printf("[TELEM] %s\n", chunks[i].c_str());
            if (i < chunks.size() - 1) {
                pros::delay(5);
            }
        }
    }

    mutex.give();
}

void Logger::sendDebugMessages() {
    // Messages are now handled in sendTelemetry() to keep output ordering sane.
    // This function exists to satisfy the header declaration.
}

void Logger::init() {
    printf("\n");
    printf("========================================\n");
    printf("  shulib logger starting\n");
    printf("  timestamp format: [MM:SS.mmm]\n");
    printf("  telemetry lines prefixed with [TELEM]\n");
    printf("========================================\n");
    printf("\n");

    if (telemetryTask == nullptr) {
        telemetryTask = new pros::Task([this] {
            while (true) {
                this->sendTelemetry();
                pros::delay(100);
            }
        });
    }
    success("Logger initialized!");
}

template<>
void Logger::updateTelemetry(const std::string &key, const std::map<std::string, double> &value) {
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