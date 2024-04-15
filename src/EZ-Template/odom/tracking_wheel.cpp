#include "EZ-Template/odom/tracking_wheel.hpp"
#include <math.h>
#include "EZ-Template/util.hpp"


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
* @brief initialize new tracking wheel w/ motor groups
*
* @param motors motor group
* @param diameter diameter of wheels on chassis
* @param distance half of trackwidth distance in inches
* @param rpm rpm of chassis
*/
gheese::TrackingWheel::TrackingWheel (pros::Motor_Group* motors, float diameter, float distance, float rpm) {
    this->motors = motors;
    this->motors->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    this->diameter = diameter;
    this->distance = distance;
    this->rpm = rpm;
}

/**
 * @brief resets the rotation sensor to 0
*/
void gheese::TrackingWheel::reset() {
    if (this->rotation != nullptr) this->rotation->reset_position();
    if (this->motors != nullptr) this->motors->tare_position();
}

/**
 * @brief get the distance traveled by the tracking wheel
 *
 * @return float distance traveled in inches
*/
float gheese::TrackingWheel::get_distance_traveled() { 
    if (this->rotation != nullptr) return (float(this->rotation->get_position()) * this->diameter * M_PI / 36000); 
    if (this->motors != nullptr){
        std::vector<pros::motor_gearset_e_t> gearsets = this->motors->get_gearing();
        std::vector<double> positions = this->motors->get_positions();
        std::vector<float> distances;
        for (int i = 0; i < this->motors->size(); i++) {
            float in;
            switch (gearsets[i]) {
                case pros::E_MOTOR_GEARSET_36: in = 100; break;
                case pros::E_MOTOR_GEARSET_18: in = 200; break;
                case pros::E_MOTOR_GEARSET_06: in = 600; break;
                default: in = 200; break;
            }
            distances.push_back(positions[i] * (diameter * M_PI) * (rpm / in));
        }
        return gheese::avg(distances);
    }
}

/**
 * @brief get offset of wheel from tracking center
 *
 * @return distance
*/
float gheese::TrackingWheel::get_offset() { return this-> distance; }