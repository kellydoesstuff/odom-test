#pragma once
// #include "EZ-Template/drive/drive.hpp"
#include "pos.hpp"
#include "tracking_wheel.hpp"
#include "pros/imu.hpp"

namespace gheese {

/**
* @brief Struct containing all the sensors used for odometry
*
*/
struct OdomSensors {
        /**
        *
        * odom sensors are in a struct for easy initialiation in chassis ez's chassis class
        * @param vertical pointer to the first vertical tracking wheel
        * @param horizontal pointer to the first horizontal tracking wheel
        * @param imu pointer to the IMU
        */
        OdomSensors(gheese::TrackingWheel* vertical, gheese::TrackingWheel* horizontal, pros::Imu* imu);
        TrackingWheel* vertical;
        TrackingWheel* horizontal;
        pros::Imu* imu;
};

/**
* @brief set sensors for odometry
*
*/
void set_sensors(gheese::OdomSensors sensors);
/**
* @brief initialize the odometry system
*
*/
// void init();
/**
* @brief get position of robot
*
* @param radians true for theta in radians, false for degrees. false by default
* @return pos
*/
Pos get_pos (bool radians = false);
/**
* @brief Set the Pose of the robot
*
* @param pose the new pose
* @param radians true if theta is in radians, false if in degrees. False by default
*/
void set_pos(Pos pos, bool radians = false);
/**
* @brief Update the pose of the robot
*
*/
void update();
/**
* @brief print position of the robot through brain
*
*/
void print_pos ();


}