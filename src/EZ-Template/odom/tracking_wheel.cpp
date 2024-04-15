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
    if (this->rotation != nullptr) this->rotation->reset_position();
}

/**
 * @brief get the distance traveled by the tracking wheel
 *
 * @return float distance traveled in inches
*/
float gheese::TrackingWheel::get_distance_traveled() { 
    if (this->rotation != nullptr) 
    return (float(this->rotation->get_position()) * this->diameter * M_PI / 36000); 
}

/**
 * @brief get offset of wheel from tracking center
 *
 * @return distance
*/
float gheese::TrackingWheel::get_offset() { return this-> distance; }