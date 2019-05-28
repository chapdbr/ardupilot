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
#include "EPR2_RollController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo EPR2_RollController::var_info[] = {
	// @Param: P
	// @DisplayName: Proportional Gain
	// @Description: Proportional gain from roll angle demands to ailerons. Higher values allow more servo response but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0.1 4.0
	// @Increment: 0.1
	// @User: User
	AP_GROUPINFO("P",        0, EPR2_RollController, _kp,        1.0f),

	// @Param: I
	// @DisplayName: Integrator Gain
	// @Description: Integrator gain from long-term roll angle offsets to ailerons. Higher values "trim" out offsets faster but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 1.0
	// @Increment: 0.05
	// @User: User
	AP_GROUPINFO("I",        1, EPR2_RollController, _ki,        0.3f),

	// @Param: D
	// @DisplayName: Damping Gain
	// @Description: Damping gain from roll acceleration to ailerons. Higher values reduce rolling in turbulence, but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 0.2
	// @Increment: 0.01
	// @User: User
	AP_GROUPINFO("D",        2, EPR2_RollController, _kd,        0.08f),

	// @Param: IMAX
	// @DisplayName: Integrator limit
	// @Description: Limit of roll integrator gain in centi-degrees of servo travel. Servos are assumed to have +/- 4500 centi-degrees of travel, so a value of 3000 allows trim of up to 2/3 of servo travel range.
	// @Range: 0 4500
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("IMAX",      3, EPR2_RollController, _imax,        3000),

	// @Param: SCALER
	// @DisplayName: Command scaler
	// @Description: Scale the command to degrees according to the range of motion.
	// @Range: 0 45
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("SCALER",      4, EPR2_RollController, _scaler,        45),


	AP_GROUPEND
};


/*
  internal bank angle controller, called by stabilize
*/
int32_t EPR2_RollController::get_servo_out(float desired_angle)
{

	// Calculate delta time
	uint32_t tnow = AP_HAL::millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;
	float delta_time    = (float)dt * 0.001f;
    // Get body angle .roll_sensor (centi-degrees) ou .roll (radians)
	float achieved_angle = _ahrs.roll_sensor;
	// Get body rate (degrees)
	float achieved_rate = _ahrs.get_gyro().x;
	// Calculate the angle error (deg)
	float angle_error = (desired_angle - achieved_angle);
	
	// Get an airspeed estimate - default to zero if none available
	float aspeed;
	if (!_ahrs.airspeed_estimate(&aspeed)) {
        aspeed = 0.0f;
    }
	// Compute proportional component
	_pid_info.P = angle_error * _kp;

	// Compute derivative component if time has elapsed
	if ((fabsf(_kd) > 0) && (dt > 0)) {
		float derivative;

		if (isnan(_last_derivative)) {
			// we've just done a reset, suppress the first derivative
			// term as we don't want a sudden change in input to cause
			// a large D output change
			derivative = 0;
			_last_derivative = 0;
		} else {
			derivative = (angle_error - _last_error) / delta_time;
		}

		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy
		float RC = 1/(2*M_PI*_fCut);
		derivative = _last_derivative +
					 ((delta_time / (RC + delta_time)) *
					  (derivative - _last_derivative));

		// update state
		_last_error             = angle_error;
		_last_derivative    = derivative;

		// add in derivative component
		_pid_info.D = derivative * _kd;
	}

	// Multiply roll error by gains.I and integrate

	// Don't integrate if in stabilise mode as the integrator will wind up against the pilots inputs
	if (_ki > 0) {
		//only integrate if gain and time step are positive and airspeed above min value.
		if (dt > 0 && aspeed > float(aparm.airspeed_min)) {
		    float integrator_delta = angle_error * _ki * delta_time;
			// prevent the integrator from increasing if surface defln demand is above the upper limit
			if (_last_out < -45) {
                integrator_delta = MAX(integrator_delta , 0);
            } else if (_last_out > 45) {
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
	
	// Save desired and achieved angles
    _pid_info.desired = desired_angle;
    _pid_info.actual = achieved_angle;

    // Calculate the demanded control surface deflection (degrees) with the scaler
	_last_out = _pid_info.P + _pid_info.I + _pid_info.D;
	_last_out = _last_out * _scaler;
	
	// Convert to centi-degrees and constrain
	return constrain_float(_last_out * 100, -4500, 4500);
}

void AP_RollController::reset_I()
{
	_pid_info.I = 0;
}
