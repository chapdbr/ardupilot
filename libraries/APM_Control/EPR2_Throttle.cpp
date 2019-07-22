/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//	Code by Bruno Chapdelaine
//

#include <AP_HAL/AP_HAL.h>
#include "EPR2_Throttle.h"
#include <GCS_MAVLink/GCS.h>


extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo EPR2_Throttle::var_info[] = {
	// @Param: P
	// @DisplayName: Proportional Gain
	// @Description: Proportional gain from roll angle demands to ailerons. Higher values allow more servo response but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 4
	// @Increment: 0.0001
	// @User: User
	AP_GROUPINFO("P",        0, EPR2_Throttle, _kp,        2.9506f),

	// @Param: I
	// @DisplayName: Integrator Gain
	// @Description: Integrator gain from long-term roll angle offsets to ailerons. Higher values "trim" out offsets faster but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 2
	// @Increment: 0.0001
	// @User: User
	AP_GROUPINFO("I",        1, EPR2_Throttle, _ki,        1.2622f),

	// @Param: D
	// @DisplayName: Damping Gain
	// @Description: Damping gain from roll acceleration to ailerons. Higher values reduce rolling in turbulence, but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 0.01
	// @Increment: 0.0001
	// @User: User
	AP_GROUPINFO("D",        2, EPR2_Throttle, _kd,        0.0033f),

	// @Param: IMAX
	// @DisplayName: Integrator limit
	// @Description: Limit of roll integrator gain in centi-degrees of servo travel. Servos are assumed to have +/- 4500 centi-degrees of travel, so a value of 3000 allows trim of up to 2/3 of servo travel range.
	// @Range: 0 4500
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("IMAX",      3, EPR2_Throttle, _imax,        100),

	// @Param: ASPD
	// @DisplayName: Airspeed target
	// @Description: Airspeed target.
	// @Units: m/s
	// @Range: 10 30
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("ASPD",      4, EPR2_Throttle, _aspd,       15),

	// @Param: MAX_THRUST
	// @DisplayName: Maximum thrust
	// @Description: Maximum thrust.
	// @Units: %
	// @Range: 0 1
	// @Increment: 0.1
	// @User: Advanced
	AP_GROUPINFO("MAX",      5, EPR2_Throttle, _max_thrust,        1.0f),

	// @Param: TRACKING
	// @DisplayName: Trajectory tracking
	// @Description: Trajectory tracking.
	// @Range: 0:Disable,1:Enable
	// @User: Advanced
	AP_GROUPINFO("TRACK",      6, EPR2_Throttle, _use_tracking,        0),

	// @Param: TARGET
	// @DisplayName: Groundspeed target
	// @Description: Groundspeed target.
	// @Units: m/s
	// @Range: 10 30
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("GSPD",      7, EPR2_Throttle, _grndspd,       15),

	// @Param: TARGET
	// @DisplayName: Minimum airspeed
	// @Description: Minimum airspeed.
	// @Units: m/s
	// @Range: 10 30
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("MIN_ASPD",      8, EPR2_Throttle, _min_aspd,       15),

	// @Param: TAU
	// @DisplayName: Time constant
	// @Description: Time constant.
	// @Units: s
	// @Range: 0 10
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("TAU",      9, EPR2_Throttle, _tau,       3),

	// @Param: RADIUS
	// @DisplayName: Time constant
	// @Description: Time constant.
	// @Units: m
	// @Range: 0 100
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("RADIUS",      10, EPR2_Throttle, _radius,       15),


	AP_GROUPEND
};


/*
  internal azimuth controller, called by plane::calc_throttle()
*/
int32_t EPR2_Throttle::get_servo_out(void)
{
	// Calculate delta time
	uint32_t tnow = AP_HAL::millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;
	// Get time delta in s
	float delta_time    = (float)dt * 0.001f;
	float spd_error;
	// Get an airspeed estimate - default to 15 if none available
	float aspeed;
	if (!_ahrs.airspeed_estimate(&aspeed)) {
        aspeed = 15.0f;
    }
	if (_use_tracking == 1) {
		//Calculate groundspeed
		Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();
		float grndspeed = _groundspeed_vector.length();
		// Calculate the grndspd error
		spd_error = (_speed_target - grndspeed);
	} else {
		// Calculate the aspd error
		spd_error = (_aspd - aspeed);
	}
	// Compute proportional component
	_pid_info.P = spd_error * _kp;

	if (isnan(_last_error)) {
		// we've just done a reset, last_error = 0
		_last_error = 0;
	}
	// Compute derivative component if time has elapsed
	if ((fabsf(_kd) > 0) && (dt > 0)) {
		float derivative;

		if (isnan(_last_derivative)) {
			// we've just done a reset, suppress the first derivative
			// term as we don't want a sudden change in input to cause
			// a large D output change
			derivative = 0;
		} else {
			float tau = 1/_fCut;
			derivative = (2*tau-delta_time)/(2*tau+delta_time)*_last_derivative + 2/(2*tau+delta_time)*(spd_error - _last_error);
		}

		// add in derivative component
		_pid_info.D = derivative * _kd;
		// update last derivative
		_last_derivative    = derivative;
	}

	// Multiply roll error by gains.I and integrate

	// Don't integrate if in stabilise mode as the integrator will wind up against the pilots inputs
	if (_ki > 0) {
		//only integrate if gain and time step are positive and airspeed above min value.
		if (dt > 0 && aspeed > float(aparm.airspeed_min)) {
			// Trapeziodal integration
			float integrator_delta = (spd_error + _last_error) * delta_time * 0.5 * _ki;
			// prevent the integrator from increasing if surface defln demand is above the upper limit
			if (_last_out < -_max_thrust) {
				integrator_delta = MAX(integrator_delta , 0);
			} else if (_last_out > _max_thrust) {
				// prevent the integrator from decreasing if surface defln demand  is below the lower limit
				 integrator_delta = MIN(integrator_delta, 0);
			}
			_pid_info.I += integrator_delta;
		}
	} else {
		_pid_info.I = 0;
	}

	// Scale the integration limit
	float intLimScaled = _imax * 0.01f;

	// Constrain the integrator state
	_pid_info.I = constrain_float(_pid_info.I, -intLimScaled, intLimScaled);

	// Save desired, achieved angles and last error
	_pid_info.desired = _aspd;
	_pid_info.actual = aspeed;
	_last_error = spd_error;

	// Calculate the demanded control command (0-1)
	_last_out = _pid_info.P + _pid_info.I + _pid_info.D;
	// Command max throttle if airspeed is below minimum airspeed
	if (aspeed < float(aparm.airspeed_min)) {
		_last_out = 1;
	}
	// Convert to percentage and constrain (0-100)
	return constrain_float(_last_out * 100, 0, 100);
}

void EPR2_Throttle::reset_I()
{
	_pid_info.I = 0;
}

void EPR2_Throttle::update_speed_target()
{
	uint32_t tnow = AP_HAL::millis();
	tnow = (float)tnow * 0.001f;
	//if (tnow - _last_speed_update >= 0.1)
	if (tnow - _last_speed_update >= 1) // low rate for debug
	{
		// we run this loop at 10 Hz
		_last_speed_update = tnow;
		// Calculate delta time
		float telapsed = tnow - _tini;
		// Calculate azimuth target
		float alt_target = _epr2alt.get_target();
		float r_alt = _radius*sinf(acosf(alt_target/_radius));
		float c_alt = 2*M_PI*r_alt;
		float t_trim = c_alt/_grndspd;
		float omega_trim = 2*M_PI/t_trim;
		float azimuth_target = (telapsed)*omega_trim;

		// Location from home position
		Vector2f position;
		if (!_ahrs.get_relative_position_NE_home(position)) {
			// we have no idea where we are....
			return;
		}
		float azimuth = atan2f(position.y,position.x);
		float delta_azimuth = azimuth - _last_azimuth;
		float central_angle = acosf(cosf(delta_azimuth));

		if (delta_azimuth < 0){
			central_angle = 0; // bad data, aircraft shouldnt go backwards
		} else {
			_last_azimuth = azimuth;
		}

		_azimuth_sum = _azimuth_sum+central_angle;
		float azimuth_error = azimuth_target-_azimuth_sum;
		float distance_error = azimuth_error*r_alt;

		float speed_target = distance_error/_tau+_grndspd;
		_speed_target = constrain_float(speed_target,14,24);

		/*
		hal.console->printf("X=%f\t Y=%f\t az=%f\t az_sum=%f\t az_t=%f\n",
			                            position.x,
			                            position.y,
			                            azimuth,
										(float)_azimuth_sum,
										azimuth_target);
		*/
		gcs().send_text(MAV_SEVERITY_CRITICAL, "X=%3.2f\t Y=%3.2f\t az=%3.2f\t az_sum=%3.2f\t az_t=%3.2f\n",
                position.x,
                position.y,
                azimuth,
				(float)_azimuth_sum,
				azimuth_target);
	}
}

void EPR2_Throttle::ini()
{
	// Initialize function
	uint32_t tnow = AP_HAL::millis();
	tnow = (float)tnow * 0.001f;
	_tini = tnow;
	// get position as a 2D offset from ahrs home
	Vector2f position;
	if (!_ahrs.get_relative_position_NE_home(position)) {
		// we have no idea where we are....
		return;
	}
	float azimuth = atan2f(position.y,position.x);
	_last_azimuth = azimuth;
	_azimuth_sum = 0;
	/*
	hal.console->printf("Initialisation values: X=%f\t Y=%f\t az=%f\t az_sum=%f\n",
	                            position.x,
	                            position.y,
	                            (float)_azimuth_ini,
								(float)_azimuth_sum);
	*/
	gcs().send_text(MAV_SEVERITY_CRITICAL, "X=%3.2f\t Y=%3.2f\t az_ini=%3.2f\t az_sum=%3.2f\n",
	                position.x,
	                position.y,
					(float)_last_azimuth,
					(float)_azimuth_sum);
}
