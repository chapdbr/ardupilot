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
#include "EPR2_AltController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo EPR2_AltController::var_info[] = {
	// @Param: P
	// @DisplayName: Proportional Gain
	// @Description: Proportional gain from roll angle demands to ailerons. Higher values allow more servo response but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 20
	// @Increment: 0.0001
	// @User: User
	AP_GROUPINFO("P",        0, EPR2_AltController, _kp,        14.1582f),

	// @Param: I
	// @DisplayName: Integrator Gain
	// @Description: Integrator gain from long-term roll angle offsets to ailerons. Higher values "trim" out offsets faster but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 2
	// @Increment: 0.0001
	// @User: User
	AP_GROUPINFO("I",        1, EPR2_AltController, _ki,        1.0707f),

	// @Param: D
	// @DisplayName: Damping Gain
	// @Description: Damping gain from roll acceleration to ailerons. Higher values reduce rolling in turbulence, but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 2
	// @Increment: 0.0001
	// @User: User
	AP_GROUPINFO("D",        2, EPR2_AltController, _kd,        1.7248f),

	// @Param: IMAX
	// @DisplayName: Integrator limit
	// @Description: Limit of roll integrator gain in centi-degrees of servo travel. Servos are assumed to have +/- 4500 centi-degrees of travel, so a value of 3000 allows trim of up to 2/3 of servo travel range.
	// @Range: 0 4500
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("IMAX",      3, EPR2_AltController, _imax,        1000),

	// @Param: SCALER
	// @DisplayName: Command scaler
	// @Description: Scale the command to degrees according to the range of motion.
	// @Range: 0 45
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("SCALER",      4, EPR2_AltController, _scaler,        20),

	// @Param: TARGET
	// @DisplayName: Altitude target
	// @Description: Altitude target.
	// @Units: m
	// @Range: 0 100
	// @Increment: 0.5
	// @User: Advanced
	AP_GROUPINFO("TARGET",      5, EPR2_AltController, _target,        4.0f),

	// @Param: MAX_ANGLE
	// @DisplayName: Maximum angle
	// @Description: Maximum angle.
	// @Units: deg
	// @Range: 0 45
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("MAX",      6, EPR2_AltController, _max_angle,        20),

	// @Param: LOADCELL
	// @DisplayName: Using load cell
	// @Description: Using load cell.
	// @Range: 0:Disable,1:Enable
	// @User: Advanced
	AP_GROUPINFO("LOADCL",      7, EPR2_AltController, _loadcell,        0),

	AP_GROUPEND
};


/*
  internal bank angle controller, called by stabilize
*/
void EPR2_AltController::calc_desired_pitch(void)
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

	// Get altitude
	// return a Down position relative to home in meters
	// if EKF is unavailable will return the baro altitude

	if (_loadcell == 0)
	{
	_ahrs.get_relative_position_D_home(_height);
	_height *= -1.0f;
	} else	{
	_height = _sensor_alt;
	}

	// Calculate the altitude error (m)
	float alt_error = (_target - _height);
	
	// Get an airspeed estimate - default to 15 if none available
	float aspeed;
	if (!_ahrs.airspeed_estimate(&aspeed)) {
        aspeed = 15.0f;
    }
	// Compute proportional component
	_pid_info.P = alt_error * _kp;
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
			derivative = (2*tau-delta_time)/(2*tau+delta_time)*_last_derivative + 2/(2*tau+delta_time)*(alt_error - _last_error);
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
		    float integrator_delta = (alt_error + _last_error) * delta_time * 0.5 * _ki;

			// prevent the integrator from increasing if surface defln demand is above the upper limit
			if (_last_out < -_max_angle) {
                integrator_delta = MAX(integrator_delta , 0);
            } else if (_last_out > _max_angle) {
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
	
	// Save desired, achieved angles and update last error
    _pid_info.desired  = _target;
    _pid_info.actual   = _height;
	_last_error        = alt_error;


    // Calculate the demanded control surface deflection (degrees) with the scaler
	_last_out = _pid_info.P + _pid_info.I + _pid_info.D;
	//_last_out = _last_out * _scaler;
	_pitch_dem = constrain_float(_last_out, -_max_angle, _max_angle);

	if (tnow - _last_height_update >= 1) // low rate for debug
	{
	_last_height_update = tnow;
	gcs().send_text(MAV_SEVERITY_CRITICAL, "height=%2.2f\n",
					_height);
	}
}

void EPR2_AltController::reset_I()
{
	_pid_info.I = 0;
}
