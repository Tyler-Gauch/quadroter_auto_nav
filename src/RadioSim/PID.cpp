// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	PID.cpp
/// @brief	Generic PID algorithm

#include "PID.h"

float PID::get_p(float error) const
{
    return (float)error * _kp;
}

float PID::get_i(float error, float dt)
{
    if((_ki != 0) && (dt != 0)) {
        _integrator += ((float)error * _ki) * dt;
        if (_integrator < -_imax) {
            _integrator = -_imax;
        } else if (_integrator > _imax) {
            _integrator = _imax;
        }
        return _integrator;
    }
    return 0;
}

float PID::get_d(float input, float dt)
{
    if ((_kd != 0) && (dt != 0)) {
        float derivative;
		if (isnan(_last_derivative)) {
			// we've just done a reset, suppress the first derivative
			// term as we don't want a sudden change in input to cause
			// a large D output change			
			derivative = 0;
			_last_derivative = 0;
		} else {
			// calculate instantaneous derivative
			derivative = (input - _last_input) / dt;
		}

        // discrete low pass filter, cuts out the
        // high frequency noise that can drive the controller crazy
        derivative = _last_derivative + _d_lpf_alpha * (derivative - _last_derivative);

        // update state
        _last_input             = input;
        _last_derivative    = derivative;

        // add in derivative component
        return _kd * derivative;
    }
    return 0;
}

float PID::get_pi(float error, float dt)
{
    return get_p(error) + get_i(error, dt);
}


float PID::get_pid(float error, float dt)
{
    return get_p(error) + get_i(error, dt) + get_d(error, dt);
}

void PID::reset_I()
{
    _integrator = 0;
	// mark derivative as invalid
    _last_derivative = NAN;
}

void PID::set_d_lpf_alpha(short int cutoff_frequency, float time_step)
{    
    // calculate alpha
    float rc = 1/(2*PI*cutoff_frequency);
    _d_lpf_alpha = time_step / (time_step + rc);
}
