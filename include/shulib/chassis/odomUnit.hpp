#pragma once

#include "pros/rotation.hpp"
namespace shulib {

class OdomUnit {
public:
  /**
   * @brief Create a new OdomUnit
   *
   * @param sensor The rotation sensor to use for tracking
   * @param diameter The diameter of the tracking wheel in inches
   * @param offset The distance from the tracking center to the tracking wheel
   * in inches
   */
  OdomUnit(pros::Rotation *sensor, float diameter, float offset);

  /**
   * @brief Reset the tracking wheel's position to 0
   */
  void reset();

  /**
   * @brief Get the distance traveled by the tracking wheel since the last reset
   *
   * @return float distance traveled in inches
   */
  double get_travel();

  /**
   * @brief Get the change in travel distance since the last call to this function
   *
   * @return float change in travel distance in inches
   */
  double get_travel_delta();

  /**
   * @brief Get the offset of the tracking wheel from the tracking center
   *
   * @return float offset in inches
   */
  double get_offset();

private:
  pros::Rotation *sensor = nullptr;
  float diameter;
  float offset;
  float lastPosition;
};

} // namespace shulib