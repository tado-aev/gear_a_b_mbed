#include "AccelController.h"

/* Constructors, Destructor, and Assignment operators {{{ */
AccelController::AccelController(PinName enable,
                                 PinName mosi,
                                 PinName miso,
                                 PinName sclk,
                                 PinName cs)
    : BaseController{}
    , enable{enable}
    , to_ecu_dac{mosi, miso, sclk}
    , to_ecu_dac_cs{cs}
{
}

// Destructor
AccelController::~AccelController()
{
}
/* }}} */

void
AccelController::init() {
    to_ecu_dac.format(12, 0);
    to_ecu_dac.frequency(100e3);

    to_ecu_dac_cs = 0;
    to_ecu_dac.write(voltage_b_intercept);
    to_ecu_dac.write(voltage_a_intercept);
    to_ecu_dac_cs = 1;
}

void
AccelController::on() {
    enable = 1;
}

void
AccelController::off() {
    enable = 0;
}

void
AccelController::set(const double percentage) {
    led_output(percentage);
    auto p = limit_range(percentage, 0.0, 100.0);

    int a_out = static_cast<int>(voltage_a_slope * p + voltage_a_intercept);
    int b_out = static_cast<int>(voltage_b_slope * p + voltage_b_intercept);

    to_ecu_dac_cs = 0;
    to_ecu_dac.write(b_out);
    to_ecu_dac.write(a_out);
    to_ecu_dac_cs = 1;
}

double
AccelController::get_percentage() {
    return current_percentage;
}

void
AccelController::set_ecu_dac_slope_intercept(const float voltage_a_slope,
                                             const float voltage_a_intercept,
                                             const float voltage_b_slope,
                                             const float voltage_b_intercept) {
    this->voltage_a_slope = voltage_a_slope;
    this->voltage_a_intercept = voltage_a_intercept;
    this->voltage_b_slope = voltage_b_slope;
    this->voltage_b_intercept = voltage_b_intercept;
}
