#include "mode.h"
#include "Plane.h"

bool ModeERTL::_enter()
{
    //сброс таймеров в случае повторного захода
    lt_started = false;
    rt_started = false;
    rt_finished = false;
    lt_start_ms = 0;
    rt_start_ms = 0;
    enter_heading_course = plane.ahrs.yaw_sensor;

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
        const int32_t fs_gcs_lt = plane.g.fs_gcs_lt.get();

        if (radius != 0) {
            plane.loiter.direction = (radius < 0) ? -1 : 1;
        }

        if (rt_finished) {
            plane.update_loiter(abs(radius));
            return;
        }


        if (plane.loiter.reached_target_alt) {
            if (!lt_started) {
                lt_start_ms = AP_HAL::millis();
                lt_started = true;
            }

            if (fs_gcs_lt <= 0 || (AP_HAL::millis() - lt_start_ms) / 1000U >= static_cast<uint32_t>(fs_gcs_lt)) {
                const int32_t fs_gcs_rt = plane.g.fs_gcs_rt.get();
                const int32_t fs_gcs_cour = plane.g.fs_gcs_cour.get();

                if (fs_gcs_rt > 0) {
                    if (fs_gcs_cour == -1) {
                        const int32_t heading_cd = wrap_360_cd(int32_t((18000 + enter_heading_course)));
                        plane.nav_controller->update_heading_hold(heading_cd);
                    }
                    else if (fs_gcs_cour >= 0 && fs_gcs_cour < 360) {
                        const float declination_deg = degrees(AP::compass().get_declination());
                        const int32_t heading_cd = wrap_360_cd(int32_t((fs_gcs_cour + declination_deg) * 100.0f));
                        plane.nav_controller->update_heading_hold(heading_cd);
                    }
                    else {
                        if (!rt_finished) {
                            plane.do_loiter_at_location();
                            plane.setup_terrain_target_alt(plane.next_WP_loc);
                            plane.set_target_altitude_location(plane.next_WP_loc);
                            plane.loiter_angle_reset();
                            rt_finished = true;
                        }
                        plane.update_loiter(abs(radius));
                        return;
                    }

                    if (!rt_started && abs(plane.nav_controller->bearing_error_cd()) <= 100) {
                        rt_start_ms = AP_HAL::millis();
                        rt_started = true;
                    }


                    if (rt_started && (AP_HAL::millis() - rt_start_ms) / 1000U >= static_cast<uint32_t>(fs_gcs_rt)) {
                        if (!rt_finished) {
                            plane.do_loiter_at_location();
                            plane.setup_terrain_target_alt(plane.next_WP_loc);
                            plane.set_target_altitude_location(plane.next_WP_loc);
                            plane.loiter_angle_reset();
                            rt_finished = true;
                        }
                        plane.update_loiter(abs(radius));
                    }
                }
                else {
                    plane.update_loiter(abs(radius));
                }

                return;
            }
        }

        plane.update_loiter(abs(radius));
    }
}
