#include "pos.hpp"

/**
 * @brief initialize new pos
 *
 * @param x coord
 * @param y coord
 * @param theta heading. default to 0
*/
gheese::Pos::Pos (float x, float y, float theta) {
    this -> x = x;
    this -> y = y;
    this -> theta = theta;
}

/**
 * @brief add to pos together
 *
 * @param other pos being added
 * @return new pos
*/
gheese::Pos gheese::Pos::add (const Pos& other) {return gheese::Pos(this->x + other.x, this->y + other.y, this->theta);}

/**
 * @brief get the distance between two poses
 *
 * @param other the other pose
 * @return float
*/
float gheese::Pos::distance (Pos other) {return std::hypot(this->x - other.x, this->y - other.y);}

/**
 * @brief get the angle between two poses
 *
 * @param other the other pose
 * @return float in radians
*/
float gheese::Pos::angle (Pos other) {return std::atan2(other.y - this->y, other.x - this->x);}