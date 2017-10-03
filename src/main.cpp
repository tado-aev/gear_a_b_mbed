#include <mbed.h>

#include <ros.h>
#include <coms_msgs/ComsGAB.h>

#include "GabController.h"

static constexpr double BRAKE_FOLLOWER_RATE = 20;
// Offset from the potentiometer to the actual brake percentage
static constexpr double BRAKE_FOLLOWER_OFFSET = -10;

bool is_g_a_turned_on = false;

bool program_mode = false;

GabController controller;

void
turn_on() {
    ros::NodeHandle nh;
    nh.loginfo("Turning gearr, accelerator, and brake ON");
    controller.gear().on();
    controller.accel().on();
    controller.brake().on();
}

void
turn_off() {
    controller.gear().off();
    controller.accel().off();
    controller.brake().off();
}

void
brake_follower() {
    ros::NodeHandle nh;

    controller.brake().on();

    while (nh.connected()) {
        wait_ms(1000.0 / BRAKE_FOLLOWER_RATE);

        auto potentiometer = controller.brake().get_percentage_potentiometer();
        controller.brake().set(potentiometer + BRAKE_FOLLOWER_OFFSET);
    }

    controller.brake().off();
}

void
callback(const coms_msgs::ComsGAB& msg) {
    if (!program_mode) {
        return;
    }

    controller.gear().set(msg.gear);
    controller.accel().set(msg.accel);
    controller.brake().set(msg.brake);
}

void
run_node() {
    ros::NodeHandle nh;
    ros::Subscriber<coms_msgs::ComsGAB> command_sub{"cmd_gab", &callback};
    coms_msgs::ComsStatus status_msg;
    auto status_pub = ros::Publisher{"coms_status", &status_msg};

    nh.initNode();
    nh.subscribe(command_sub);
    nh.advertise(status_pub);

    while (!nh.connected()) {
       nh.spinOnce();
       wait_ms(10);
    }

    //// Get parameters
    nh.loginfo("Getting parameters");
    bool use_brake_follower;
    int use_brake_follower_int;
    int cool_muscle_baudrate;
    int signal_overlap_ms;
    float ecu_voltage_a[2];
    float ecu_voltage_b[2];
    float status_rate;
    nh.getParam("~use_brake_follower", &use_brake_follower_int);
    use_brake_follower = use_brake_follower_int == 1;
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
    controller.brake().init();

    if (use_brake_follower) {
        program_mode = false;
        // Blocks
        brake_follower();
        return;
    }

    program_mode = true;

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

    turn_off();
}

int
main() {
    while (true) {
        run_node();
    }

    return 0;
}
