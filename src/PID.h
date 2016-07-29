// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	PID.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#ifndef __PID_H__
#define __PID_H__

#include <stdlib.h>
#include <math.h>               // for fabs()

#define PI 3.14159265359

// Examples for _filter:
// f_cut = 10 Hz -> _alpha = 0.385869
// f_cut = 15 Hz -> _alpha = 0.485194
// f_cut = 20 Hz -> _alpha = 0.556864
// f_cut = 25 Hz -> _alpha = 0.611015
// f_cut = 30 Hz -> _alpha = 0.653373
#define PID_D_TERM_FILTER 0.556864f    // Default 100Hz Filter Rate with 20Hz Cutoff Frequency

/// @class	PID
/// @brief	Object managing one PID control
class PID {
public:

    /// Constructor for PID that saves its settings to EEPROM
    ///
    /// @note	PIDs must be named to avoid either multiple parameters with the
    ///			same name, or an overly complex constructor.
    ///
    /// @param  initial_p       Initial value for the P term.
    /// @param  initial_i       Initial value for the I term.
    /// @param  initial_d       Initial value for the D term.
    /// @param  initial_imax    Initial value for the imax term.4
    ///
    PID(
        const float &   initial_p = 0.0,
        const float &   initial_i = 0.0,
        const float &   initial_d = 0.0,
        const short int & initial_imax = 0.0):
        _integrator(0),
        _last_input(0),
        _last_derivative(0),
        _d_lpf_alpha(PID_D_TERM_FILTER)
    {
        _kp = initial_p;
        _ki = initial_i;
        _kd = initial_d;
        _imax = abs(initial_imax);

		// derivative is invalid on startup
		_last_derivative = NAN;
    }

    /// Iterate the PID, return the new control value
    ///
    /// Positive error produces positive output.
    ///
    /// @param error	The measured error value
    /// @param dt		The time delta in milliseconds (note
    ///					that update interval cannot be more
    ///					than 65.535 seconds due to limited range
    ///					of the data type).
    /// @param scaler	An arbitrary scale factor
    ///
    /// @returns		The updated control output.
    ///
    float       get_pid(float error, float dt);
    float       get_pi(float error, float dt);
    float       get_p(float error) const;
    float       get_i(float error, float dt);
    float       get_d(float error, float dt);

    /// Reset the PID integrator
    ///
    void        reset_I();
    
    /// Sets filter Alpha for D-term LPF
    void        set_d_lpf_alpha(short int cutoff_frequency, float time_step);

    /// @name	parameter accessors
    //@{

    /// Overload the function call operator to permit relatively easy initialisation
    void operator        () (const float    p,
                             const float    i,
                             const float    d,
                             const short int  imaxval) {
        _kp = p; _ki = i; _kd = d; _imax = abs(imaxval);
    }

    // accessors
    float       kP() const { return _kp; }
    float       kI() const { return _ki; }
    float       kD() const { return _kd; }
    short int   imax() const { return _imax; }
    void        kP(const float v) { _kp = v; }
    void        kI(const float v) { _ki = v; }
    void        kD(const float v) { _kd = v; }
    void        imax(const short int v) { _imax = abs(v); }
    float       get_integrator() const { return _integrator; }
    void        set_integrator(float i) { _integrator = i; }

protected:
    float                   _kp;
    float                   _ki;
    float                   _kd;
    unsigned short int      _imax;

    float           _integrator;                                ///< integrator value
    float           _last_input;                                ///< last input for derivative
    float           _last_derivative;                           ///< last derivative for low-pass filter
    float           _d_lpf_alpha;                               ///< alpha used in D-term LPF
};

#endif // __PID_H__
