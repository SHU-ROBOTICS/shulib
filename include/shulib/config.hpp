#pragma once

#include <string>
#include <map>
#include <variant>
#include <vector>

namespace shulib {

using ConfigValue = std::variant<int, double, bool, std::string>;
using ConfigMap = std::map<std::string, ConfigValue>;

class Config {
public:
    static Config& getInstance();

    bool loadFromFile(const std::string& filename);
    bool saveToFile(const std::string& filename) const;

    template<typename T>
    T get(const std::string& key, const T& defaultValue = T()) const {
        auto it = values.find(key);
        if (it == values.end()) return defaultValue;
        try {
            return std::get<T>(it->second);
        } catch (const std::bad_variant_access&) {
            return defaultValue;
        }
    }

    template<typename T>
    void set(const std::string& key, const T& value) {
        values[key] = value;
    }

private:
    Config() = default;
    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;

    ConfigMap values;
}; 