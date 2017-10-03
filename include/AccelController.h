#ifndef ACCEL_CONTROLLER_H_
#define ACCEL_CONTROLLER_H_

#include <mbed.h>

#include "BaseController.hpp"

class AccelController : public BaseController {
public:
    /* Constructors, Destructor, and Assignment operators {{{ */
    AccelController(PinName enable,
                    PinName mosi,
                    PinName miso,
                    PinName sclk,
                    PinName cs);

    // Copy constructor
    AccelController(const AccelController& other) = delete;

    // Move constructor
    AccelController(AccelController&& other) = delete;

    // Destructor
    ~AccelController();

    // Assignment operator
    AccelController&
    operator=(const AccelController& other) = delete;

    // Move assignment operator
    AccelController&
    operator=(AccelController&& other) = delete;
    /* }}} */

    void
    init();

    void
    on();

    void
    off();

    /**
     * Sets the percentage of the brake pedal
     *
     * 0% is when the pedal is fully pulled out (on brake applied), and 100%
     * is when the brake is fully applied.
     *
     * \param[in] percentage a number in the range [0, 100]
     */
    void
    set(const double percentage);

    double
    get_percentage();

    /**
     * Sets the parameters for calculating values to write to the DAC
     */
    void
    set_ecu_dac_slope_intercept(const float voltage_a_slope,
                                const float voltage_a_intercept,
                                const float voltage_b_slope,
                                const float voltage_b_intercept);

private:
    Mutex com_mutex;

    // Enable accelerator control
    DigitalOut enable;
    // DAC that commands voltage to the ECU
    SPI to_ecu_dac;
    DigitalOut to_ecu_dac_cs;
    double current_percentage;

    /**
     * The linear relationship between the stepping in of the accelerator and
     * the voltage going into the ECU is determined empirically. The voltage
     * which the DAC should send to the ECU is calculated as follows:
     *
     * voltage = slope * percentage + intercept
     *
     * where percentage is a value in the range [0, 100]. Also, the ECU
     * requires two inputs of voltage for redundancy. Therefore, we have two
     * voltage commands, namely a and b, for commanding to the accelerator.
     */
    float voltage_a_slope;
    float voltage_a_intercept;
    float voltage_b_slope;
    float voltage_b_intercept;
};

#endif /* end of include guard */
