#include "BrakeController.h"

using pulse_t = BrakeController::pulse_t;

/* Constructors, Destructor, and Assignment operators {{{ */
BrakeController::BrakeController(PinName tx,
                                 PinName rx,
                                 PinName potentiometer)
    : BaseController{}
    , to_cool_muscle{tx, rx}
    , potentiometer{potentiometer}
{
}

BrakeController::~BrakeController()
{
}
/* }}} */

void
BrakeController::init() {
    int response_code;

    /*
     * Search for origin
     *
     *   back of                  front of
     *   vehicle                  vehicle
     *
     * Pulse Count
     *   high <----> low
     * Percentage
     *      0                        100
     * |----|-------------------------|
     *    origin                    full
     *
     * First, search for the pulse count when the pedal is at full extension.
     * Ideally, this position should be 0% brake pedal, but due to the
     * mechanism of the brake actuator, there is some slack and goes past 0%.
     *
     * Thus, we move the actuator so that the pedal is actually at 0%. We set
     * this position as origin.
     *
     * Then, push the pedal in as far as possible until we hit the full
     * braking point and get an error for going over the torque limit. This is
     * set as 100% brake.
     */
    // Enable motor
    on();

    // Send in-position status, in/out change status, disable echo
    to_cool_muscle.printf("K23.1=%d\r\n", 0b01111);
    // Direction of search (in CW direction i.e. towards back of vehicle)
    to_cool_muscle.printf("K45.1=0\r\n");
    // Use torque threshold for detecting origin
    to_cool_muscle.printf("K46.1=0\r\n");
    // Set torque limit for detecting origin
    to_cool_muscle.printf("K47.1=110\r\n");

    // Initiate origin search
    to_cool_muscle.printf("|.1\r\n");
    // Wait for origin search to complete
    pulse_t dummy;
    readline("Origin.1=%lld\r\n", dummy);
    // Response should be 8
    readline("Ux.1=%d\r\n", response_code);

    // Move the actuator a bit towards the front
    // Position
    to_cool_muscle.printf("P.1=%d\r\n", brake_slack_pulse_back);
    // Speed
    to_cool_muscle.printf("S.1=%d\r\n", DEFAULT_S);
    // Acceleration
    to_cool_muscle.printf("A.1=%d\r\n", DEFAULT_A);
    // Torque limit
    to_cool_muscle.printf("M.1=%d\r\n", DEFAULT_M);
    // Go!
    to_cool_muscle.printf("^.1\r\n");
    // Status should be 8
    readline("Ux.1=%d\r\n", response_code);

    // Set current position as origin
    to_cool_muscle.printf("|2.1");
    min_cool_muscle = 0;

    // Now, find the pulse count at 100% brake
    // Hit the brake as far in as possible
    to_cool_muscle.printf("P.1=-10000\r\n");
    // Go! After this operation, motor is freed due to excessive torque
    to_cool_muscle.printf("^.1\r\n");
    // Response code should be 4 (torque limit)
    // TODO: check response code
    readline("Ux.1=%d\r\n", response_code);
    max_cool_muscle = get_pulse_count();
    // Re-enable the motor
    on();

    // Readjust the motor so that it's at 0% brake pedal
    to_cool_muscle.printf("|1.1\r\n");

    // Set soft limit (for the + side == pulling out the brake)
    to_cool_muscle.printf("K58=%d\r\n", min_cool_muscle);
    // Set soft limit (for the - side == pushing in the brake)
    to_cool_muscle.printf("K59=%d\r\n", max_cool_muscle);

    // Disable motor
    off();
    write_to_pc("Finished init for brake");
}

void
BrakeController::on() {
    to_cool_muscle.printf("(.1\r\n");
    int response_code;
    readline("Ux.1=%d\r\n", response_code);
    // TODO: check if response_code was 8
    write_to_pc("Finished on for brake");
}

void
BrakeController::off() {
    to_cool_muscle.printf(").1\r\n");
}

void
BrakeController::set(const double percentage) {
    pulse_t diff = max_cool_muscle - min_cool_muscle;
    pulse_t pulse = min_cool_muscle + static_cast<pulse_t>(diff * percentage);
    to_cool_muscle.printf("P.1=%lld\r\n", pulse);
    to_cool_muscle.printf("S.1=%d\r\n", DEFAULT_S);
    to_cool_muscle.printf("A.1=%d\r\n", DEFAULT_A);
    to_cool_muscle.printf("M.1=%d\r\n", DEFAULT_M);
    to_cool_muscle.printf("^.1\r\n");
    int response_code;
    readline("Ux.1=%d\r\n", response_code);
    write_to_pc("Finished set for brake");
}

void
BrakeController::emergency() {
    to_cool_muscle.printf("*\r\n");
}

void
BrakeController::release_emergency() {
    to_cool_muscle.printf("*1\r\n");
}

pulse_t
BrakeController::get_pulse_count() {
    to_cool_muscle.printf("?96.1\r\n");
    pulse_t pulse;
    readline("Px.1=%lld\r\n", pulse);
    return pulse;
}

double
BrakeController::get_percentage() {
    int pulse = get_pulse_count();
    return static_cast<double>(pulse) / (brake_slack_pulse_back - brake_slack_pulse_front);
}

double
BrakeController::get_percentage_potentiometer() {
    unsigned short raw_val = potentiometer.read_u16();
    int diff = max_potentiometer - min_potentiometer;
    return static_cast<double>(raw_val - min_potentiometer) / diff;
}

void
BrakeController::set_cool_muscle_baudrate(const unsigned baudrate) {
    to_cool_muscle.baud(baudrate);
}

std::string
BrakeController::readline(const std::string& fmt /* = "%s" */) {
    char buf[512];
    auto populated = to_cool_muscle.scanf(fmt.c_str(), buf);
    if (populated == 0) {
        return "";
    }

    return std::string{buf};
}

template<typename T>
void
BrakeController::readline(const std::string& fmt, T& var) {
    // Read a line, and then parse it
    char buf[512];
    to_cool_muscle.scanf("%s", buf);
    int populated = 0;
    while (populated == 0) {
        populated = sscanf(buf, fmt.c_str(), &var);
    }
}
