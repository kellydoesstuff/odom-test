#pragma once

#include "pros/rotation.hpp"
#include <numbers>
#include <math.h>

namespace gheese {

class TrackingWheel {

    private:
        float diameter;
        float distance;
        pros::Rotation* rotation = nullptr;

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
         * @brief resets the rotation sensor to 0
        */
        void reset();

        /**
         * @brief get the distance traveled by the tracking wheel
         *
         * @return float distance traveled in inches
         */
        float get_distance_traveled();


};

}