#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include "AP_AutoTune.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>

#include <APM_Control/EPR2_AltController.h>

class EPR2_Throttle {
public:
	EPR2_Throttle(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms, EPR2_AltController &epr2altcontroller)
        : aparm(parms)
        , _ahrs(ahrs)
		, _epr2alt(epr2altcontroller)

    {
        AP_Param::setup_object_defaults(this, var_info);
        // set _last_derivative as invalid when we startup
		_last_derivative = NAN;
		_last_error = NAN;
    }

    /* Do not allow copies */
	EPR2_Throttle(const EPR2_Throttle &other) = delete;
	EPR2_Throttle &operator=(const EPR2_Throttle&) = delete;

	int32_t get_servo_out(void);

	void reset_I();

	void ini();

	void update_speed_target();

    const       AP_Logger::PID_Info& get_pid_info(void) const { return _pid_info; }

	static const struct AP_Param::GroupInfo var_info[];

	float get_radius(void) { return _radius; }

	float get_tracking_status(void) { return _use_tracking; }

private:
    const AP_Vehicle::FixedWing &aparm;

    AP_Float        _kp;
	AP_Float        _ki;
	AP_Float        _kd;
	AP_Int16        _imax;
	AP_Int16		_aspd_target;
	AP_Float		_max_thrust;

	AP_Int8			_use_tracking;
	AP_Int8			_rate_limiter;
	AP_Int16		_grndspd;
	AP_Int16		_min_aspd;
	AP_Int16		_tau;
	AP_Float		_radius;
	AP_Float		_last_azimuth;
	AP_Float		_azimuth_sum;
	AP_Float		_speed_target;
	AP_Float		_tini;
	AP_Float		_last_speed_update;
	AP_Float		_max_rate;

	uint32_t _last_t;
	float _last_out;
	float _out;
	float _integrator;///< integrator value
	float _last_error;///< last error for derivative
	float _last_derivative;///< last derivative for low-pass filter

    AP_Logger::PID_Info _pid_info;

	AP_AHRS &_ahrs;
	EPR2_AltController &_epr2alt;

	/// Low pass filter cut frequency for derivative calculation in Hz.
	static const uint8_t        _fCut = 40;

};
