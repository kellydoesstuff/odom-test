#pragma once

#include "pros/rotation.hpp"
#include "pros/motors.hpp"
#include <numbers>
#include <math.h>

namespace gheese {

class TrackingWheel {

    private:
        float diameter;
        float distance;
        float rpm;
        pros::Rotation* rotation = nullptr;
        pros::Motor_Group* motors = nullptr;


    public:
        /**
         * @brief initialize new tracking wheel
         *
         * @param tracker rotation sensor for dead wheel
         * @param diameter diameter of dead wheel
         * @param distance offset distance to tracking center in inches
         */
        TrackingWheel (pros::Rotation* tracker, float diameter, float distance);
        /**
         * @brief initialize new tracking wheel w/ motor groups
         *
         * @param motors motor group
         * @param diameter diameter of wheels on chassis
         * @param distance half of trackwidth distance in inches
         * @param rpm rpm of chassis
         */
        TrackingWheel (pros::Motor_Group* motors, float diameter, float distance, float rpm);

        /**
         * @brief resets the rotation sensor to 0
        */
        void reset();

        /**
         * @brief get the distance traveled by the tracking wheel
         *
         * @return float distance traveled in inches
         */
        float get_distance_traveled();

        /**
         * @brief get offset of wheel from tracking center
         *
         * @return distance
         */
         float get_offset();

};

}