#ifndef BRAKE_CONTROLLER_H_
#define BRAKE_CONTROLLER_H_

#include <mbed.h>

#include "BaseController.hpp"

#include <cstdio>
#include <string>

class BrakeController : public BaseController {
public:
    /* Note: LPC1768 (or Serial::scanf) doesn't support long long int? */
    //using pulse_t = long long int;
    using pulse_t = int;

    static const int RESPONSE_MOVING = 0;
    static const int RESPONSE_OVERFLOW = 1;
    static const int RESPONSE_OVERSPEED = 2;
    static const int RESPONSE_TORQUE_LIMIT = 4;
    static const int RESPONSE_OK = 8;
    static const int RESPONSE_MOTOR_OFF = 16;
    static const int RESPONSE_PRESSING = 32;
    static const int RESPONSE_PRESS_COMPLETE = 64;
    static const int RESPONSE_TEMPERATURE_ERROR = 128;
    static const int RESPONSE_PRESS_ERROR = 256;
    static const int RESPONSE_EMERGENCY = 512;

    // Default speed for Cool Muscle
    static const int DEFAULT_S = 10;
    // Default acceleration for Cool Muscle
    static const int DEFAULT_A = 10;
    // Default torque for Cool Muscle
    static const int DEFAULT_M = 120;

    // See the comment in BrakeController::init
    static const int brake_slack_pulse_front = 0;
    static const int brake_slack_pulse_back = -340;

    static constexpr double BRAKE_FOLLOWER_RATE = 20;
    // Offset from the potentiometer to the actual brake percentage
    static constexpr double BRAKE_FOLLOWER_OFFSET = -35;

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
     * Returns the current voltage from the potentiometer
     */
    double
    get_potentiometer_value();

    /**
     * Returns the current percentage of the potentiometer
     *
     * Uses the potentiometer to calculate how much the brake pedal is being
     * stepped in.
     */
    double
    get_percentage_potentiometer();

    /**
     * Returns the motor status
     */
    int
    get_status();

    /**
     * Sets the baud rate for the Cool Muscle actuator
     */
    void
    set_cool_muscle_baudrate(const unsigned baudrate);

    /**
     * Loop that reads the value from the potentiometer and moves the brake
     * actuator so that it follows the movement of the brake pedal.
     */
    void
    brake_follower();

    /**
     * Spins up the thread for the brake follower
     *
     * BrakeController::brake_follower enables the actuator torque (i.e. calls
     * BrakeController::on) so there is no need to on() before calling this
     * method. However, it is the required that the init() method be called
     * before calling this method.
     */
    void
    begin_brake_follower();

    /**
     * Changes the status of the brake follower
     *
     * \param[in] enable true to enable brake follower, false otherwise
     */
    void
    set_brake_follower(const bool enable);

    /**
     * Sets the flag to end the brake follower
     *
     * BrakeController::brake_follower disables the actuator torque (i.e.
     * calls BrakeController::off) so there is no need to off() after calling
     * this method.
     */
    void
    end_brake_follower();

private:
    Mutex serial_mutex;
    Mutex potentiometer_mutex;

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
    double min_potentiometer;
    // Value from the potentiometer at 100% brake
    double max_potentiometer;

    Thread brake_follower_thread;
    bool enable_brake_follower;
    bool stop_brake_follower;

    /**
     * Writes a line to the Cool Muscle actuator after adding a CRLF
     */
    void
    writeline(const std::string& line);

    /**
     * Writes a line to the Cool Muscle actuator after adding a CRLF
     *
     * The second parameter is concatenated to the first parameter.
     * The string written is in the following format: "{line}{var}\r\n"
     */
    template<typename T>
    void
    writeline(const std::string& line, const T var);

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
     * \param[in] allow_empty true if the format doesn't have to match the
     *                        line read. If true, exactly one line is read
     */
    template<typename T>
    void
    readline(const std::string& fmt, T& var, const bool allow_empty = false);
};

#endif /* end of include guard */
