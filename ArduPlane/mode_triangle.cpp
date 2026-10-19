#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS.h>

#include "mode.h"
#include "Plane.h"


bool ModeTriangle::_enter()
{
    // plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);

    _start_point_loc = plane.current_loc;
    plane.prev_WP_loc = _start_point_loc;

    _middle_point_loc = _start_point_loc;
    _middle_point_loc.offset_bearing(60.0f, _side_m);

    _last_point_loc = _middle_point_loc;
    _last_point_loc.offset_bearing(180.0f, _side_m);

    _next_point_loc = _middle_point_loc;
    _next_point_number = 1;
    // Initialise a fresh waypoint leg from the current position.
    // plane.auto_state.next_wp_crosstrack = false;
    plane.set_next_WP(_next_point_loc);

    gcs().send_text(MAV_SEVERITY_INFO, "TRIANGLE: Entering into the mode");

    return true;
}


// mode_triangle.cpp
void ModeTriangle::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

void ModeTriangle::navigate()
{
    // Update the normal waypoint navigation controller.
    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);

    constexpr float acceptance_radius_m = 50.0f;

    if (plane.current_loc.get_distance(_next_point_loc) > acceptance_radius_m) {
        return;
    }

    switch (_next_point_number) {
    case 1: // reached middle
        _next_point_loc = _last_point_loc;
        _next_point_number = 2;
        break;
    case 2: // reached last
        _next_point_loc = _start_point_loc;
        _next_point_number = 0;
        break;
    default: // reached start; repeat
        _next_point_loc = _middle_point_loc;
        _next_point_number = 1;
        break;
    }

    plane.set_next_WP(_next_point_loc);
    gcs().send_text(MAV_SEVERITY_INFO, "TRIANGLE: waypoint %u",
                    _next_point_number);
}

void ModeTriangle::run()
{
    Mode::run();
}
