#include "Copter.h"

//#if MODE_SQUARE_ENABLED == ENABLED

// init - initialise guided controller
bool ModeSquare::init(bool ignore_checks)
{
   wp_control_start();
}

// run - runs the guided controller
// should be called at 100hz or more
void ModeSquare::run()
{
    // run waypoint controller
    wp_control_run();

    if (send_notification && wp_nav->reached_wp_destination()) {
        send_notification = false;
        gcs().send_mission_item_reached_message(0);
    }
}

bool ModeSquare::allows_arming(AP_Arming::Method method) const
{
    // always allow arming from the ground station
    if (method == AP_Arming::Method::MAVLINK) {
        return true;
    }

    // optionally allow arming from the transmitter
    return (copter.g2.guided_options & (uint32_t)Options::AllowArmingFromTX) != 0;
};

// initialise guided mode's waypoint navigation controller
void ModeGuided::wp_control_start()
{
    // set to position control mode
    guided_mode = SubMode::WP;

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // initialise wpnav to stopping point
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(stopping_point, false);

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

// run guided mode's waypoint navigation controller
void ModeGuided::wp_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(wp_nav->get_thrust_vector(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_thrust_vector_rate_heading(wp_nav->get_thrust_vector(), auto_yaw.rate_cds());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_thrust_vector_heading(wp_nav->get_thrust_vector(), auto_yaw.yaw());
    }
}

//#endif
