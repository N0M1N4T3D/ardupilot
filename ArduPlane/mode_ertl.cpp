#include "mode.h"
#include "Plane.h"

bool ModeERTL::_enter()
{
    plane.do_loiter_at_location();
    plane.setup_terrain_target_alt(plane.next_WP_loc);

    if (plane.stick_mixing_enabled() && (plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL))) {
        plane.set_target_altitude_current();
    }

    plane.loiter_angle_reset();

    return true;
}

void ModeERTL::update()
{
    plane.calc_nav_roll();
    if (plane.stick_mixing_enabled() && plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL)) {
        plane.update_fbwb_speed_height();
    } else {
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }

#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        // while a trick is running we reset altitude
        plane.set_target_altitude_current();
        plane.next_WP_loc.set_alt_cm(plane.target_altitude.amsl_cm, Location::AltFrame::ABSOLUTE);
    }
#endif
}

void ModeERTL::navigate()
{
    if (plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL)) {
        // update the WP alt from the global target adjusted by update_fbwb_speed_height
        plane.next_WP_loc.set_alt_cm(plane.target_altitude.amsl_cm, Location::AltFrame::ABSOLUTE);
    }

#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        // don't try to navigate while running trick
        return;
    }
#endif

    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}