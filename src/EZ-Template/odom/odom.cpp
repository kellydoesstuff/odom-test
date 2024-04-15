#include "odom.hpp"
#include "pos.hpp"
#include "pros/llemu.hpp"
#include "tracking_wheel.hpp"
#include "EZ-Template/util.hpp"
#include <math.h>
#include "pros/rtos.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "main.h"
#include "pros/motors.hpp"
/**
 * @brief Struct containing all the sensors used for odometry
 *
 */


// task that updates pos
// pros::Task* tracking_task = nullptr;


// global vars
gheese::Pos odom_pos(0,0,0); // position of robot
pros::Imu imu(1);
// pros::Rotation horizontal_track(2, false);
// 3" offset, behind tracking center (negative)
pros::Motor left_motor (9, pros::E_MOTOR_GEARSET_18);
pros::Motor right_motor (-10, pros::E_MOTOR_GEARSET_18);
pros::Motor_Group leftm_group ({left_motor});
pros::MotorGroup rightm_group({right_motor});
// gheese::TrackingWheel horizontal (&horizontal_track, 2.75, -3);
// pros::Rotation vertical_track(3, true);
// 3" offset, infront tracking center 
gheese::TrackingWheel vertical (&leftm_group, 4.0, -5.25, 200.0);
gheese::OdomSensors odom_sensors (&vertical, nullptr, &imu);
// gheese::OdomSensors odom_sensors(nullptr, nullptr, nullptr);



float prev_vert {0};
float prev_horiz {0};
float prev_imu {0};


/**
* @brief set sensors for odometry
*
*/
void set_sensors(gheese::OdomSensors sensors) {
    odom_sensors = sensors;
}
/**
* @brief get position of robot
*
* @param radians true for theta in radians, false for degrees. false by default
* @return pos
*/
gheese::Pos gheese::get_pos (bool radians) {
    if (radians) return odom_pos;
    else return gheese::Pos(odom_pos.x, odom_pos.y, gheese::rad_to_deg(odom_pos.theta));
}

/**
* @brief Set the Pose of the robot
*
* @param pose the new pose
* @param radians true if theta is in radians, false if in degrees. False by default
*/
void gheese::set_pos(Pos pos, bool radians) {
    if (radians) odom_pos = pos;
    else odom_pos = gheese::Pos(pos.x, pos.y, deg_to_rad((pos.theta)));
}

/**
* @brief print position of the robot through terminal
*
*/
void gheese::print_pos () {
    pros::lcd::initialize();
    pros::delay(2000);
    while (true) {
        pros::lcd::print(0, "X: %f", gheese::get_pos().x); // x
        pros::lcd::print(1, "Y: %f", gheese::get_pos().y); // y
        pros::lcd::print(2, "Theta: %f", gheese::get_pos().theta); // heading
        pros::delay(ez::util::DELAY_TIME);
    }
}

/**
* @brief Update the pose of the robot
*
*/
void gheese::update() {
    pros::delay(2000);
    float vert_raw{0};
    float horiz_raw{0};
    float imu_raw{0};
    // odom_sensors.horizontal->reset();
    odom_sensors.vertical->reset();
    odom_sensors.imu->reset(true);
    // odom_sensors.imu->set_heading(90);
    
    while (true) {
        if (odom_sensors.vertical != nullptr) vert_raw = odom_sensors.vertical->get_distance_traveled();
        if (odom_sensors.horizontal != nullptr) horiz_raw = odom_sensors.horizontal->get_distance_traveled();
        if (odom_sensors.imu != nullptr) imu_raw = gheese::deg_to_rad(odom_sensors.imu->get_rotation());
        // if (odom_sensors.imu != nullptr) imu_raw = imu.get_rotation();

        // // calc change in sensors
        float delta_vert = vert_raw - prev_vert;
        float delta_horiz = horiz_raw - prev_horiz;
        float delta_imu = imu_raw - prev_imu;

        // // update prev sensors
        prev_vert = vert_raw;
        prev_horiz = horiz_raw;
        prev_imu = imu_raw;

        // // calculate heading
        float heading = odom_pos.theta;
        if (odom_sensors.imu != nullptr) heading += delta_imu;
        float delta_heading = heading - odom_pos.theta;
        float avg_heading = odom_pos.theta + delta_heading /2;

        // // calculate change in x and y
        float delta_x = 0;
        float delta_y = 0;
        if (odom_sensors.vertical != nullptr) delta_y = vert_raw - prev_vert;
        if (odom_sensors.horizontal != nullptr) delta_x = horiz_raw - prev_horiz;
        prev_vert = vert_raw;
        prev_horiz = horiz_raw;

        float horiz_offset = 0;
        if (odom_sensors.horizontal != nullptr) horiz_offset = odom_sensors.horizontal->get_offset();

        // // calculate local x and y
        float local_x = 0;
        float local_y = 0;
        if (delta_heading == 0) {
            local_x = delta_horiz;
            local_y = delta_vert;
        } else {
            local_x = 2 * sin(delta_heading /2) * (delta_x / delta_heading + horiz_offset);
            local_y = 2 * sin(delta_heading /2) * (delta_y / delta_heading + odom_sensors.vertical->get_offset());
        }

        // // save old pos
        gheese::Pos prev_pos = odom_pos;

        // // calcualte global x and y
        odom_pos.x += local_y * sin(avg_heading);
        odom_pos.y += local_y * cos(avg_heading);
        odom_pos.x += local_x * -cos(avg_heading);
        odom_pos.y += local_x * sin(avg_heading);
        odom_pos.theta = heading;
        pros::lcd::print(0, "X: %f", odom_pos.x); // x
        pros::lcd::print(1, "Y: %f", odom_pos.y); // y
        pros::lcd::print(2, "Theta: %f", gheese::rad_to_deg(odom_pos.theta)); // heading
        // pros::lcd::print(0, "X: %f", vert_raw); // x
        // pros::lcd::print(1, "Y: %f", horiz_raw); // y
        // pros::lcd::print(2, "Theta: %f", imu.get_heading()); // heading
        pros::delay(ez::util::DELAY_TIME);
    }
}

/**
* @brief initialize the odometry system
*
*/
// void gheese::init() {
//     if (tracking_task == nullptr) {
//         tracking_task = new pros::Task {[=] {
//             while (true) {
//                 update();
//                 pros::delay(10);
//             }
//         }};
//     }
// }
