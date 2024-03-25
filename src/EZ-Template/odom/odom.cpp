#include "odom.hpp"
#include "pos.hpp"
#include "tracking_wheel.hpp"
#include "EZ-Template/util.hpp"
#include <math.h>
#include "pros/rtos.hpp"

/**
 * @brief Struct containing all the sensors used for odometry
 *
 */
gheese::OdomSensors::OdomSensors (gheese::TrackingWheel* vertical, gheese::TrackingWheel* horizontal, pros::Imu* imu)
        : vertical(vertical),
          horizontal(horizontal),
          imu(imu){}

// task that updates pos
pros::Task* tracking_task = nullptr;


// global vars
gheese::Pos odom_pos(0,0,0); // position of robot
gheese::OdomSensors odom_sensors(nullptr, nullptr, nullptr);


float prev_vert {0};
float prev_horiz {0};
float prev_imu {0};

void gheese::update() {
    float vert_raw{};
    float horiz_raw{};
    float imu_raw{};
    
    vert_raw = odom_sensors.vertical->get_distance_traveled();
    horiz_raw = odom_sensors.horizontal->get_distance_traveled();
    imu_raw = gheese::deg_to_rad(odom_sensors.imu->get_rotation());

    // calc change in sensors
    float delta_vert {vert_raw - prev_vert};
    float delta_horiz {horiz_raw - prev_horiz};
    float delta_imu {imu_raw - prev_imu};

    // update prev sensors
    prev_vert = vert_raw;
    prev_horiz = horiz_raw;
    prev_imu = imu_raw;

    // calculate heading
    float heading {odom_pos.theta};
    heading += delta_imu;
    float delta_heading {heading - odom_pos.theta};
    float avg_heading {(odom_pos.theta + delta_heading)/2};

    // calculate local x and y
    float local_x {0};
    float local_y {0};
    if (delta_heading == 0) {
        local_x = delta_horiz;
        local_y = delta_vert;
    } else {
        local_x = 2 * sin(delta_heading /2) * (delta_horiz / delta_heading + odom_sensors.horizontal->get_offset());
        local_y = 2 * sin(delta_heading /2) * (delta_vert / delta_heading + odom_sensors.vertical->get_offset());
    }

    // save old pos
    gheese::Pos prev_pos = odom_pos;

    // calcualte global x and y
    odom_pos.x += local_y * sin(avg_heading);
    odom_pos.y += local_y * cos(avg_heading);
    odom_pos.x += local_x * -cos(avg_heading);
    odom_pos.y += local_x * sin(avg_heading);
    odom_pos.theta = heading;
     
}

void gheese::init() {
    if (tracking_task == nullptr) {
        tracking_task = new pros::Task {[=] {
            while (true) {
                update();
                pros::delay(10);
            }
        }};
    }
}