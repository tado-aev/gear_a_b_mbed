#ifndef GAB_CONTROLLER_H_
#define GAB_CONTROLLER_H_

#include "BaseController.hpp"

#include "GearController.h"
#include "AccelController.h"
#include "BrakeController.h"

#include <mbed.h>

#include <ros.h>
#include <coms_msgs/ComsStatus.h>

/**
 * Class for controlling the gear, accelerator, and the brake.
 */

class GabController : public BaseController {
public:
    /* Constructors, Destructor, and Assignment operators {{{ */
    // Default constructor
    GabController();

    // Copy constructor
    // Deleted because Serial does not support copying
    GabController(const GabController& other) = delete;

    // Move constructor
    // Deleted because Serial does not support moving
    GabController(GabController&& other) = delete;

    // Destructor
    ~GabController();

    // Assignment operator
    // Deleted because Serial does not support copying
    GabController&
    operator=(const GabController& other) = delete;

    // Move assignment operator
    // Deleted because Serial does not support moving
    GabController&
    operator=(GabController&& other) = delete;
    /* }}} */

    inline
    AccelController&
    accel() {
        return accel_controller;
    }

    inline
    BrakeController&
    brake() {
        return brake_controller;
    }

    inline
    GearController&
    gear() {
        return gear_controller;
    }

    Thread&
    begin_publishing(ros::NodeHandle& nh, const float hz);

    void
    end_publishing();

    void
    publish_status();

private:
    GearController gear_controller;
    AccelController accel_controller;
    BrakeController brake_controller;

    unsigned seq;
    bool stop_publishing;
    float status_rate;
    ros::NodeHandle nh;
    coms_msgs::ComsStatus status_msg;
    ros::Publisher status_pub;
    Thread publisher_thread;

    void
    keep_publishing();
};

#endif /* end of include guard */
