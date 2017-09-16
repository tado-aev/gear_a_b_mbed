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
    , status_msg{}
    , status_pub{"coms_status", &status_msg}
    , publisher_thread{}
{
}

// Destructor
GabController::~GabController()
{
}
/* }}} */

Thread&
GabController::begin_publishing(ros::NodeHandle& nh, const float hz) {
    this->nh = nh;
    nh.advertise(status_pub);
    status_rate = hz;
    stop_publishing = false;
    publisher_thread.start(this, &GabController::keep_publishing);
    return publisher_thread;
}

void
GabController::end_publishing() {
    stop_publishing = true;
    publisher_thread.join();
}

void
GabController::publish_status() {
    status_msg.header.stamp = ros::Time::now();
    status_msg.header.seq = seq++;
    status_msg.header.frame_id = "0";
    switch (gear_controller.get_gear()) {
        case GearController::REVERSE:
            status_msg.gab.gear = "r";
            break;
        case GearController::NEUTRAL:
            status_msg.gab.gear = "n";
            break;
        case GearController::DRIVE:
            status_msg.gab.gear = "d";
            break;
    }
    status_msg.gab.accel = accel_controller.get_percentage();
    status_msg.gab.accel = accel_controller.get_percentage();
    status_msg.gab.brake = brake_controller.get_percentage();
    status_msg.brake_potentiometer = brake_controller.get_percentage_potentiometer();

    status_pub.publish(&status_msg);
}

void
GabController::keep_publishing() {
    if (status_rate == 0) {
        return;
    }

    while (!stop_publishing) {
        publish_status();
        wait_ms(1 / status_rate);
    }
}

