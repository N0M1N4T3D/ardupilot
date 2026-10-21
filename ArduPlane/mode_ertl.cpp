#include "mode.h"
#include "Plane.h"

bool ModeERTL::_enter()
{
    const int16_t fs_gcs_wp = plane.g.fs_gcs_wp.get();
    const float rtl_alt = plane.g.RTL_altitude.get();

    float current_alt_m;
    if (!plane.current_loc.get_alt_m(Location::AltFrame::ABSOLUTE, current_alt_m)) {
        return false;
    }
    const float target_alt_m = current_alt_m + (rtl_alt >= 0 ? rtl_alt : 0.0f);


    if (fs_gcs_wp > 0 && fs_gcs_wp < plane.mission.num_commands()) {
        AP_Mission::Mission_Command cmd{};

        if (!plane.mission.read_cmd_from_storage(fs_gcs_wp, cmd) || cmd.id != MAV_CMD_NAV_WAYPOINT) {
            return false;
        }

        Location target = cmd.content.location;
        target.set_alt_m(target_alt_m, Location::AltFrame::ABSOLUTE);
            
        plane.set_next_WP(target);
    } 
    else if (fs_gcs_wp == -1) {
        plane.do_loiter_at_location();

        plane.next_WP_loc.set_alt_m(target_alt_m, Location::AltFrame::ABSOLUTE);
        plane.setup_terrain_target_alt(plane.next_WP_loc);
        plane.set_target_altitude_location(plane.next_WP_loc);
        plane.loiter_angle_reset();
    }
    else {
        return false;
    }

    return true;
}

void ModeERTL::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

void ModeERTL::navigate()
{
    const int16_t fs_gcs_wp = plane.g.fs_gcs_wp.get();
    
    if (fs_gcs_wp > 0) {
        AP_Mission::Mission_Command cmd{};
        if (!plane.mission.read_cmd_from_storage(fs_gcs_wp, cmd) || cmd.id != MAV_CMD_NAV_WAYPOINT) {
            return;
        }

        if (plane.verify_nav_wp(cmd)) {
            const uint16_t next_wp = fs_gcs_wp + 1;

            if (next_wp >= plane.mission.num_commands()) {
                //последняя точка
                return;
            }

            if (!plane.mission.set_current_cmd(next_wp)) {
                return;
            }

            plane.mission.set_force_resume(true); //auto не может начать миссию с начала

            if (!plane.set_mode(plane.mode_auto, ModeReason::GCS_FAILSAFE)) {
                plane.mission.set_force_resume(false);
            }
        }
    } 
    else {
        const int16_t radius = plane.g.rtl_radius.get();

        if (radius != 0) {
            plane.loiter.direction = (radius < 0) ? -1 : 1;
        }

        plane.update_loiter(abs(radius));
    }
}
