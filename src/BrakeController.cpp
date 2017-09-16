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
     *   Pulse count is 0 at origin
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

    // Send in-position status, in/out change status, disable echo
    writeline("K23.1=", 0b01111);
    // Direction of search (in CW direction i.e. towards back of vehicle)
    writeline("K45.1=0");
    // Use torque threshold for detecting origin
    writeline("K46.1=0");
    // Set torque limit for detecting origin
    writeline("K47.1=150");

    // Enable motor
    on();

    // Initiate origin search
    writeline("|.1");
    // Wait for origin search to complete
    pulse_t dummy;
    // Note: %lld crashes LPC1768
    //readline("Origin.1=%lld", dummy);
    readline("Origin.1=%d", dummy);
    // Response should be 8
    readline("Ux.1=%d", response_code);

    // Move the actuator a bit towards the front
    // Position
    writeline("P.1=", brake_slack_pulse_back);
    // Speed
    writeline("S.1=", DEFAULT_S);
    // Acceleration
    writeline("A.1=", DEFAULT_A);
    // Torque limit
    writeline("M.1=", DEFAULT_M);
    // Go!
    writeline("^.1");
    // Status should be 8
    readline("Ux.1=%d", response_code);

    // Set current position as origin
    writeline("|2.1");
    min_cool_muscle = get_pulse_count();
    min_potentiometer = get_potentiometer_value();

    // Now, find the pulse count at 100% brake
    // Hit the brake as far in as possible
    writeline("P.1=-10000");
    // Go! After this operation, motor is freed due to excessive torque
    writeline("^.1");
    // Response code should be 4 (torque limit)
    readline("Ux.1=%d", response_code);
    max_cool_muscle = get_pulse_count();
    max_cool_muscle += brake_slack_pulse_front;
    // Re-enable the motor
    on();
    max_potentiometer = get_potentiometer_value();
    led_output(80);

    // Readjust the motor so that it's at 0% brake pedal
    writeline("|1.1");
    // Response code should be 8
    readline("Ux.1=%d", response_code);
    led_output(0);

    // Set soft limit (for the + side == pulling out the brake)
    writeline("K58=", min_cool_muscle);
    // Set soft limit (for the - side == pushing in the brake)
    writeline("K59=", max_cool_muscle);

    // Disable motor
    off();
}

void
BrakeController::on() {
    int response_code = RESPONSE_MOTOR_OFF;
    while (response_code != RESPONSE_OK) {
        writeline("(.1");
        readline("Ux.1=%d", response_code);
    }
}

void
BrakeController::off() {
    writeline(").1");
}

void
BrakeController::set(const double percentage) {
    int response_code = get_status();

    if (response_code == RESPONSE_MOTOR_OFF) {
        on();
    }

    pulse_t diff = max_cool_muscle - min_cool_muscle;
    pulse_t offset = static_cast<pulse_t>(diff * percentage / 100);
    pulse_t pulse = min_cool_muscle + offset;
    writeline("P.1=", pulse);
    writeline("S.1=", DEFAULT_S);
    writeline("A.1=", DEFAULT_A);
    writeline("M.1=", DEFAULT_M);
    writeline("^.1");
    readline("Ux.1=%d", response_code);
}

void
BrakeController::emergency() {
    writeline("*");
}

void
BrakeController::release_emergency() {
    writeline("*1");
}

pulse_t
BrakeController::get_pulse_count() {
    writeline("?96.1");
    pulse_t pulse;
    readline("Px.1=%d", pulse);
    // Note: %lld crashes LPC1768
    //readline("Px.1=%lld", pulse);
    return pulse;
}

double
BrakeController::get_percentage() {
    pulse_t pulse = get_pulse_count();
    pulse_t diff = max_cool_muscle - min_cool_muscle;
    return 100 * static_cast<double>(pulse - min_cool_muscle) / diff;
}

double
BrakeController::get_potentiometer_value() {
    return static_cast<double>(potentiometer);
}

double
BrakeController::get_percentage_potentiometer() {
    double raw_val = get_potentiometer_value();
    auto diff = max_potentiometer - min_potentiometer;
    return 100 * (raw_val - min_potentiometer) / diff;
}

int
BrakeController::get_status() {
    int response_code;
    writeline("?99.1");
    readline("Ux.1=%d", response_code);
    return response_code;
}

void
BrakeController::set_cool_muscle_baudrate(const unsigned baudrate) {
    to_cool_muscle.baud(baudrate);
}

void
BrakeController::writeline(const std::string& line) {
    to_cool_muscle.printf("%s\r\n", line.c_str());
}

template<typename T>
void
BrakeController::writeline(const std::string& line, const T var) {
    writeline(line + std::to_string(var));
}

template<typename T>
void
BrakeController::readline(const std::string& fmt,
                          T& var,
                          const bool allow_empty /* = false */) {
    int populated = 0;
    while (populated == 0) {
        std::string line = "";

        while (line.empty()) {
            char buf[512];
            to_cool_muscle.scanf("%s", buf);
            line = std::string{buf};

            if (allow_empty) {
                break;
            }
        }

        while (line.back() == '\r' || line.back() == '\n') {
            line.pop_back();
        }

        populated = sscanf(line.c_str(), fmt.c_str(), &var);

        if (allow_empty) {
            return;
        }
    }
}
