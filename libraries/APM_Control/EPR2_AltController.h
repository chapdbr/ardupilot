#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include "AP_AutoTune.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>

class EPR2_AltController {
public:
	EPR2_AltController(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms)
        : aparm(parms)
        , _ahrs(ahrs)
    {
        AP_Param::setup_object_defaults(this, var_info);
        // set _last_derivative as invalid when we startup
		_last_derivative = NAN;
		_last_error = NAN;
    }

    /* Do not allow copies */
	EPR2_AltController(const EPR2_AltController &other) = delete;
	EPR2_AltController &operator=(const EPR2_AltController&) = delete;

	void calc_desired_pitch(void);

	// demanded pitch angle in centi-degrees
	// should return between -9000 to +9000
	int32_t get_pitch_demand(void){
		return int32_t(_pitch_dem);
	}

	void reset_I();

    const       AP_Logger::PID_Info& get_pid_info(void) const { return _pid_info; }

	static const struct AP_Param::GroupInfo var_info[];

	float get_target(void) { return _target; }

	void write_alt(float alt) { _sensor_alt = alt; }

private:
    const AP_Vehicle::FixedWing &aparm;

    AP_Float        _kp;
	AP_Float        _ki;
	AP_Float        _kd;
	AP_Int16        _imax;
	AP_Int16		_scaler;
	AP_Float		_target;
	AP_Int16		_max_angle;
	AP_Int32		_pitch_dem;
	AP_Int8			_loadcell;

	uint32_t _last_t;
	float _last_out;
	float _integrator;///< integrator value
	float _last_error;///< last error for derivative
	float _last_derivative;///< last derivative for low-pass filter
	float _height;/// current height estimate (above field elevation)
	float _sensor_alt;/// altitude from load cell positive up (m)
	float _last_height_update;
    AP_Logger::PID_Info _pid_info;

	AP_AHRS &_ahrs;

	/// Low pass filter cut frequency for derivative calculation in Hz.
	static const uint8_t        _fCut = 20;

};
