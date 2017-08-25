#include "GearController.h"

/* Constructors, Destructor, and Assignment operators {{{ */
GearController::GearController(PinName mode_shift_program,
                               PinName mode_shift_manual,
                               PinName shift_r,
                               PinName shift_n,
                               PinName shift_d)
    : BaseController{}
    // Shifting from a program
    , mode_shift_program{mode_shift_program}
    // Shifting manually with the gear selector
    , mode_shift_manual{mode_shift_manual}
    // Reverse, Neutral, and Drive gears
    , shift_r{shift_r}
    , shift_n{shift_n}
    , shift_d{shift_d}
{
}

// Move constructor
GearController::GearController(GearController&& other)
    : mode_shift_program{std::move(other.mode_shift_program)}
    , mode_shift_manual{std::move(other.mode_shift_manual)}
    , shift_r{std::move(other.shift_r)}
    , shift_n{std::move(other.shift_n)}
    , shift_d{std::move(other.shift_d)}
{
}

// Destructor
GearController::~GearController()
{
}

// Move assignment operator
GearController&
GearController::operator=(GearController&& other) {
    mode_shift_program = std::move(other.mode_shift_program);
    mode_shift_manual = std::move(mode_shift_manual);
    shift_r = std::move(shift_r);
    shift_n = std::move(shift_n);
    shift_d = std::move(shift_d);
    return *this;
}
/* }}} */

void
GearController::init() {
    mode_manual();
    set_n();
}

void
GearController::on() {
    mode_program();
}

void
GearController::off() {
    set_n();
    mode_manual();
}

void
GearController::mode_program() {
    mode_shift_program = 1;
    wait_ms(MODE_SWITCH_PULSE_MS);
    mode_shift_program = 0;

    current_mode = ControlMode::PROGRAM;
}

void
GearController::mode_manual() {
    mode_shift_manual = 1;
    wait_ms(MODE_SWITCH_PULSE_MS);
    mode_shift_manual = 0;

    current_mode = ControlMode::MANUAL;
}

void
GearController::set(const std::string& gear) {
    if (gear.size() == 0) {
        set_n();
        return;
    }

    set(gear[0]);
}

void
GearController::set(const char g) {
    switch (g) {
        case 'r':
        case 'R':
            set_r();
            break;
        case 'n':
        case 'N':
            set_n();
            break;
        case 'd':
        case 'D':
            set_d();
            break;
        default:
            set_n();
            break;
    }
}

void
GearController::set_r() {
    shift_r = 1;
    shift_n = 0;
    shift_d = 0;

    current_gear = Gear::REVERSE;
}

void
GearController::set_n() {
    shift_r = 0;
    shift_n = 1;
    shift_d = 0;

    current_gear = Gear::NEUTRAL;
}

void
GearController::set_d() {
    shift_r = 0;
    shift_n = 0;
    shift_d = 1;

    current_gear = Gear::DRIVE;
}

GearController::ControlMode
GearController::get_mode() const {
    return current_mode;
}

GearController::Gear
GearController::get_gear() const {
    return current_gear;
}

void
GearController::set_signal_overlap(const unsigned overlap_duration_ms) {
    this->signal_overlap_ms = overlap_duration_ms;
}
