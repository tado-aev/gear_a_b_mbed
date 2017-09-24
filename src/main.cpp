#include <mbed.h>

#include <ros.h>
#include <coms_msgs/ComsGAB.h>

#include <thread>

#include "GabController.h"

static constexpr double BRAKE_FOLLOWER_RATE = 20;
// Offset from the potentiometer to the actual brake percentage
static constexpr double BRAKE_FOLLOWER_OFFSET = -35;

bool is_g_a_turned_on = false;

bool program_mode = false;
Mutex program_mode_mutex;

GabController controller;

ros::NodeHandle nh;

void
gear_accelerator_on() {
    nh.loginfo("Turning gear and accelerator ON");
    controller.gear().on();
    controller.accel().on();

    is_g_a_turned_on = true;
}

void
gear_accelerator_off() {
    controller.gear().off();
    controller.accel().off();

    is_g_a_turned_on = false;
}

void
brake_follower() {
    controller.brake().on();

    while (nh.connected()) {
        wait_ms(1000.0 / BRAKE_FOLLOWER_RATE);

        program_mode_mutex.lock();
        auto program_mode_flag = program_mode;
        program_mode_mutex.unlock();
        if (program_mode_flag) {
            continue;
        }

        auto potentiometer = controller.brake().get_percentage_potentiometer();
        controller.brake().set(potentiometer + BRAKE_FOLLOWER_OFFSET);
    }

    controller.brake().off();
}

void
callback(const coms_msgs::ComsGAB& msg) {
    program_mode_mutex.lock();
    program_mode = msg.program_mode;
    program_mode_mutex.unlock();

    if (msg.program_mode) {
        if (!is_g_a_turned_on) {
            gear_accelerator_on();
        }

        controller.gear().set(msg.gear);
        controller.accel().set(msg.accel);
        controller.brake().set(msg.brake);
    }
    else if (is_g_a_turned_on) {
        // Brake following mode
        gear_accelerator_off();
    }
}

void
run_node() {
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
    controller.brake().init();

    // Brake-following when ComsGAB::program_mode is false
    // Brake is turned on in brake_follower
    Thread brake_follower_thread;
    brake_follower_thread.start(brake_follower);

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

    gear_accelerator_off();
    // Brake is turned off in brake_follower

    program_mode = false;
}

int
main() {
    while (true) {
        run_node();
    }

    return 0;
}
