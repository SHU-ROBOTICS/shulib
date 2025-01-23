#include "shulib/config.hpp"
#include <fstream>
#include <iostream>
#include <string>

namespace shulib {

Config& Config::getInstance() {
    static Config instance;
    return instance;
}

bool Config::loadFromFile(const std::string& filename) {
    std::string path = "/usd/" + filename;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Failed to open config file: " << path << std::endl;
        return false;
    }

    values.clear();
    std::string line;
    while (std::getline(file, line)) {
        size_t delimPos = line.find('=');
        if (delimPos == std::string::npos) continue;

        std::string key = line.substr(0, delimPos);
        std::string value = line.substr(delimPos + 1);

        // Try to parse as different types
        try {
            // Try bool
            if (value == "true") {
                values[key] = true;
                continue;
            }
            if (value == "false") {
                values[key] = false;
                continue;
            }

            // Try integer
            size_t pos;
            int intVal = std::stoi(value, &pos);
            if (pos == value.length()) {
                values[key] = intVal;
                continue;
            }

            // Try double
            double doubleVal = std::stod(value, &pos);
            if (pos == value.length()) {
                values[key] = doubleVal;
                continue;
            }

            // Default to string
            values[key] = value;
        } catch (...) {
            // If parsing fails, store as string
            values[key] = value;
        }
    }

    return true;
}

bool Config::saveToFile(const std::string& filename) const {
    std::string path = "/usd/" + filename;
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "Failed to open config file for writing: " << path << std::endl;
        return false;
    }

    for (const auto& [key, value] : values) {
        file << key << "=";
        std::visit([&file](const auto& v) {
            file << v;
        }, value);
        file << "\n";
    }

    return true;
}

} 