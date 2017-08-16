#include "GabController.h"

/* Constructors, Destructor, and Assignment operators {{{ */
// Default constructor
GabController::GabController()
    : BaseController{}
    , gear_controller{p22, p23, p25, p26, p24}
    , accel_controller{p20, p5, p6, p7, p8}
    , brake_controller{p28, p27, p15}
{
}

// Destructor
GabController::~GabController()
{
}
/* }}} */
