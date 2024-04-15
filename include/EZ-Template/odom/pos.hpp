#pragma once
#include <math.h>

namespace gheese {
class Pos {
    public:
        float x{0};
        float y{0};
        float theta{0};
        /**
         * @brief initialize new pos
         *
         * @param x coord
         * @param y coord
         * @param theta heading. default to 0
         */
        Pos (float x, float y, float theta = 0);
        /**
         * @brief add to pos together
         *
         * @param other pos being added
         * @return new pos
         */
         Pos add (const Pos& other);
        /**
         * @brief get the distance between two poses
         *
         * @param other the other pose
         * @return float
         */
         float distance(Pos other);
        /**
         * @brief get the angle between two poses
         *
         * @param other the other pose
         * @return float in radians
         */
        float angle(Pos other);
        
        

};
}