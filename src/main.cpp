#include <mbed.h>

#include <ros.h>
#include <coms_msgs/ComsGAB.h>

#include "GabController.h"

bool is_ready = false;

GabController controller;

void
callback(const coms_msgs::ComsGAB& msg) {
    if (!is_ready) {
        return;
    }

    controller.gear().set(msg.gear);
    controller.accel().set(msg.accel);
    controller.brake().set(msg.brake);
}

int
main() {
    ros::NodeHandle nh;
    ros::Subscriber<coms_msgs::ComsGAB> command_sub{"cmd_gab", &callback};

    nh.initNode();
    nh.subscribe(command_sub);

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
    nh.getParam("~cool_muscle_baudrate", &cool_muscle_baudrate);
    nh.getParam("~signal_overlap_ms", &signal_overlap_ms);
    nh.getParam("~ecu_voltage_a", ecu_voltage_a, 2);
    nh.getParam("~ecu_voltage_b", ecu_voltage_b, 2);

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

    nh.loginfo("Turning gear, accelerator, and brake ON");
    controller.gear().on();
    controller.accel().on();
    controller.brake().on();

    // We're ready to control!
    is_ready = true;
    nh.loginfo("Ready to receive messages!");

    // Test brake
    /*
    controller.brake().set(25);
    controller.write_to_pc(std::to_string(static_cast<int>(controller.brake().get_percentage())));
    wait_ms(1000);
    controller.brake().set(50);
    controller.write_to_pc(std::to_string(static_cast<int>(controller.brake().get_percentage())));
    wait_ms(1000);
    controller.brake().set(100);
    controller.write_to_pc(std::to_string(static_cast<int>(controller.brake().get_percentage())));
    wait_ms(1000);
    */

    // Test accel
    /*
    controller.gear().set('d');
    controller.accel().set(10);
    wait_ms(3000);
    controller.accel().set(25);
    wait_ms(2000);
    controller.accel().set(50);
    wait_ms(2000);
    controller.accel().set(100);
    wait_ms(2000);
    controller.accel().set(0);
    */

    // Spin
    while (nh.connected()) {
        nh.spinOnce();
        wait_ms(1);
    }

    controller.gear().off();
    controller.accel().off();
    controller.brake().off();

    return 0;
}
