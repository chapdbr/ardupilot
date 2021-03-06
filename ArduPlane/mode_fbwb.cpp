#include "mode.h"
#include "Plane.h"

bool ModeFBWB::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;

#if SOARING_ENABLED == ENABLED
    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();
#endif

    plane.set_target_altitude_current();
    plane.epr2Throttle.ini();

    return true;
}

void ModeFBWB::update()
{
    // Thanks to Yury MonZon for the altitude limit code!
    plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();
    plane.update_fbwb_speed_height();
    plane.epr2Throttle.update_speed_target();

}

