#include <mbed.h>

#include <ros.h>
#include <coms_msgs/ComsGAB.h>

#include "GabController.h"

static const unsigned COOL_MUSCLE_BAUDRATE = 38400;

GabController controller;

ros::NodeHandle nh;

void
turn_on() {
    nh.loginfo("Turning gear, accelerator, and brake ON");

    controller.gear().on();
    controller.accel().on();
    // Turned on because of the brake follower
    // controller.brake().on();
}

void
turn_off() {
    controller.gear().off();
    controller.accel().off();
    // Never turned off because brake follower is used
    // BrakeController::end_brake_follower handles it
    // controller.brake().off();
}

void
callback(const coms_msgs::ComsGAB& msg) {
    controller.gear().set(msg.gear);
    controller.accel().set(msg.accel);
    controller.brake().set(msg.brake);
}

void
run_node() {
    ros::Subscriber<coms_msgs::ComsGAB> command_sub{"cmd_gab", &callback};
    coms_msgs::ComsStatus status_msg;
    auto status_pub = ros::Publisher{"coms_status", &status_msg};

    // Blocked here until ros_serial connects to the mbed (via roslaunch)
    nh.initNode();
    nh.subscribe(command_sub);
    nh.advertise(status_pub);

    while (!nh.connected()) {
       nh.spinOnce();
       wait_ms(10);
    }

    //// Get parameters
    nh.loginfo("Getting parameters");
    int cool_muscle_baudrate;
    int signal_overlap_ms;
    float ecu_voltage_a[2];
    float ecu_voltage_b[2];
    float status_rate;
    nh.getParam("~cool_muscle_baudrate", &cool_muscle_baudrate);
    nh.getParam("~signal_overlap_ms", &signal_overlap_ms);
    nh.getParam("~ecu_voltage_a", ecu_voltage_a, 2);
    nh.getParam("~ecu_voltage_b", ecu_voltage_b, 2);
    nh.getParam("~status_rate", &status_rate);

    controller.gear().set_signal_overlap(signal_overlap_ms);
    controller.brake().set_cool_muscle_baudrate(cool_muscle_baudrate);
    controller.accel().set_ecu_dac_slope_intercept(ecu_voltage_a[0],
                                                   ecu_voltage_a[1],
                                                   ecu_voltage_b[0],
                                                   ecu_voltage_b[1]);

    nh.loginfo("Starting init");
    controller.gear().init();
    controller.accel().init();
    // Already enabled for the brake follower
    // controller.brake().init();

    controller.brake().set_brake_follower(false);

    turn_on();

    nh.loginfo("Publishing status");
    Thread publisher_thread;
    controller.begin_publishing(&nh,
                                &status_pub,
                                &status_msg,
                                status_rate,
                                publisher_thread);

    // We're ready to control!
    nh.loginfo("Ready to receive messages!");

    // Spin
    while (nh.connected()) {
        nh.spinOnce();
        wait_ms(1);
    }

    controller.end_publishing();

    // Turn everything off except for the brake
    turn_off();

    controller.brake().set_brake_follower(true);
}

int
main() {
    // Start up the brake follower independent of ROS commands
    controller.led_output(100);
    wait_ms(1000);
    controller.brake().set_cool_muscle_baudrate(COOL_MUSCLE_BAUDRATE);
    controller.brake().init();
    controller.brake().begin_brake_follower();

    while (true) {
        run_node();
    }

    controller.brake().end_brake_follower();

    return 0;
}
