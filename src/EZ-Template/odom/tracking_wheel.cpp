#include "EZ-Template/odom/tracking_wheel.hpp"

/**
 * @brief initialize new tracking wheel
 *
 * @param tracker rotation sensor for dead wheel
 * @param diameter diameter of dead wheel
 * @param distance offset distance to tracking center in inches
*/
gheese::TrackingWheel::TrackingWheel (pros::Rotation* tracker, float diameter, float distance) {
    this->rotation = tracker;
    this->diameter = diameter;
    this->distance = distance;
}

/**
 * @brief resets the rotation sensor to 0
*/
void gheese::TrackingWheel::reset() {
    this->rotation->reset_position();
}

/**
 * @brief get the distance traveled by the tracking wheel
 *
 * @return float distance traveled in inches
*/
float gheese::TrackingWheel::get_distance_traveled() { return (float(this->rotation->get_position()) * this->diameter * M_PI / 36000); }