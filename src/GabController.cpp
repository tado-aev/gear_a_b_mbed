#include "GabController.h"

/* Constructors, Destructor, and Assignment operators {{{ */
// Default constructor
GabController::GabController()
    : BaseController{}
    , gear_controller{p22, p23, p25, p26, p24}
    , accel_controller{p20, p5, p6, p7, p8}
    , brake_controller{p28, p27, p15}
    , seq{0}
    , stop_publishing{false}
    , status_rate{10}
    , nh{}
    , status_pub{nullptr}
    , status_msg{nullptr}
{
}

// Destructor
GabController::~GabController()
{
}
/* }}} */

void
GabController::begin_publishing(ros::NodeHandle* nh,
                                ros::Publisher* status_pub,
                                coms_msgs::ComsStatus* status_msg,
                                const float hz,
                                Thread& publisher_thread) {
    this->nh = nh;
    status_rate = hz;
    this->status_pub = status_pub;
    this->status_msg = status_msg;
    stop_publishing = false;
    publisher_thread.start(this, &GabController::keep_publishing);
}

void
GabController::end_publishing() {
    stop_publishing = true;
}

void
GabController::publish_status() {
    if (!nh->connected()) {
        return;
    }
    if (status_msg == nullptr) {
        return;
    }
    if (status_pub == nullptr) {
        return;
    }

    status_msg->header.stamp = nh->now();
    status_msg->header.seq = seq++;
    status_msg->header.frame_id = "0";
    // TODO: Read from main.cpp
    status_msg->gab.program_mode = false;
    switch (gear_controller.get_gear()) {
        case GearController::REVERSE:
            status_msg->gab.gear = "r";
            break;
        case GearController::NEUTRAL:
            status_msg->gab.gear = "n";
            break;
        case GearController::DRIVE:
            status_msg->gab.gear = "d";
            break;
    }
    status_msg->gab.accel = accel_controller.get_percentage();
    status_msg->gab.accel = accel_controller.get_percentage();
    status_msg->gab.brake = brake_controller.get_percentage();
    status_msg->brake_potentiometer = brake_controller.get_percentage_potentiometer();

    status_pub->publish(status_msg);
}

void
GabController::keep_publishing() {
    while (nh->connected() && !stop_publishing) {
        publish_status();
        wait_ms(1000 / status_rate);
    }
}

