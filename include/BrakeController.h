#ifndef BRAKE_CONTROLLER_H_
#define BRAKE_CONTROLLER_H_

#include <mbed.h>

#include "BaseController.hpp"

#include <cstdio>
#include <string>

class BrakeController : public BaseController {
public:
    using pulse_t = long long int;

    // Default speed for Cool Muscle
    static const int DEFAULT_S = 10;
    // Default acceleration for Cool Muscle
    static const int DEFAULT_A = 10;
    // Default torque for Cool Muscle
    static const int DEFAULT_M = 20;

    // See the comment in BrakeController::init
    static const int brake_slack_pulse_front = 0;
    static const int brake_slack_pulse_back = -340;

    /* Constructors, Destructor, and Assignment operators {{{ */
    BrakeController(PinName tx, PinName rx, PinName potentiometer);

    // Copy constructor
    BrakeController(const BrakeController& other) = delete;

    // Move constructor
    BrakeController(BrakeController&& other) = delete;

    // Destructor
    ~BrakeController();

    // Assignment operator
    BrakeController&
    operator=(const BrakeController& other) = delete;

    // Move assignment operator
    BrakeController&
    operator=(BrakeController&& other) = delete;
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

    /**
     * Sends the emergency stop command to the actuator
     */
    void
    emergency();

    /**
     * Sends the release emergency command to the actuator
     */
    void
    release_emergency();

    /**
     * Returns the raw pulse count obtained from the actuator
     */
    pulse_t
    get_pulse_count();

    /**
     * Returns the current percentage of the brake pedal
     *
     * The value is obtained from the Cool Muscle command ?96.
     */
    double
    get_percentage();

    /**
     * Returns the current percentage of the potentiometer
     *
     * Uses the potentiometer to calculate how much the brake pedal is being
     * stepped in.
     */
    double
    get_percentage_potentiometer();

    /**
     * Sets the baud rate for the Cool Muscle actuator
     */
    void
    set_cool_muscle_baudrate(const unsigned baudrate);

private:
    // Cool Muscle actuator for braking
    Serial to_cool_muscle;
    AnalogIn potentiometer;

    /*
     * Pulse count from the actuator at 0% brake
     * Note that the pulse count is larger at 0% brake than it is at 100%.
     * Thus, the min/max notion doesn't make sense in terms of pulse count,
     * but it's the pulse count at min/max brake percentage.
     */
    pulse_t min_cool_muscle;
    // Pulse count from the actuator at 100% brake
    pulse_t max_cool_muscle;

    // Value from the potentiometer at 0% brake
    unsigned short min_potentiometer;
    // Value from the potentiometer at 100% brake
    unsigned short max_potentiometer;

    /**
     * Reads one line from the cool muscle
     *
     * Note that a line cannot be longer than 511 characters long.
     *
     * \param[in] fmt format of the line. Same as what you give to scanf
     *
     * \return the line read
     */
    std::string
    readline(const std::string& fmt = "%s");

    /**
     * Reads one line from the cool muscle
     *
     * Given an argument, populates that variable according to the format.
     * Note that a line cannot be longer than 511 characters long.
     *
     * \param[in] fmt format of the line. Same as what you give to scanf
     *
     * \param[in] var variable to be populated
     *
     * \return true if var was populated, false otherwise
     */
    template<typename T>
    void
    readline(const std::string& fmt, T& var);
};

#endif /* end of include guard */
