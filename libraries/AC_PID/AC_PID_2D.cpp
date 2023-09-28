/// @file	AC_PID_2D.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PID_2D.h"

#define AC_PID_2D_FILT_D_HZ_MIN      0.005f   // minimum input filter frequency

const AP_Param::GroupInfo AC_PID_2D::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P",    0, AC_PID_2D, _kp, default_kp),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("I",    1, AC_PID_2D, _ki, default_ki),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("IMAX", 2, AC_PID_2D, _kimax, default_kimax),

    // @Param: FLTE
    // @DisplayName: PID Input filter frequency in Hz
    // @Description: Input filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTE", 3, AC_PID_2D, _filt_E_hz, default_filt_E_hz),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D",    4, AC_PID_2D, _kd, default_kd),

    // @Param: FLTD
    // @DisplayName: D term filter frequency in Hz
    // @Description: D term filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTD", 5, AC_PID_2D, _filt_D_hz, default_filt_D_hz),

    // @Param: FF
    // @DisplayName: PID Feed Forward Gain
    // @Description: FF Gain which produces an output that is proportional to the magnitude of the target
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FF",    6, AC_PID_2D, _kff, default_kff),

    AP_GROUPEND
};

// Constructor
AC_PID_2D::AC_PID_2D(float initial_kP, float initial_kI, float initial_kD, float initial_kFF, float initial_imax, float initial_filt_E_hz, float initial_filt_D_hz) :
    default_kp(initial_kP),
    default_ki(initial_kI),
    default_kd(initial_kD),
    default_kff(initial_kFF),
    default_kimax(initial_imax),
    default_filt_E_hz(initial_filt_E_hz),
    default_filt_D_hz(initial_filt_D_hz)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    // reset input filter to first value received
    _reset_filter = true;
}

//  update_all - set target and measured inputs to PID controller and calculate outputs
//  target and error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated if it does not increase in the direction of the limit vector
Vector2f AC_PID_2D::update_all(const Vector2f &target, const Vector2f &measurement, float dt, const Vector2f &limit)
{
    // don't process inf or NaN
    if (target.is_nan() || target.is_inf() ||
        measurement.is_nan() || measurement.is_inf()) {
        return Vector2f{};
    }

    _target = target;

    // reset input filter to value received
    if (_reset_filter) {
        _reset_filter = false;
        _error = _target - measurement;
        _derivative.zero();
    } else {
        Vector2f error_last{_error};
        _error += ((_target - measurement) - _error) * get_filt_E_alpha(dt);

        // calculate and filter derivative
        if (is_positive(dt)) {
            const Vector2f derivative{(_error - error_last) / dt};
            _derivative += (derivative - _derivative) * get_filt_D_alpha(dt);
        }
    }

    // update I term
    update_i(dt, limit);

    _pid_info_x.target = _target.x;
    _pid_info_x.actual = measurement.x;
    _pid_info_x.error = _error.x;
    _pid_info_x.P = _error.x * _kp;
    _pid_info_x.I = _integrator.x;
    _pid_info_x.D = _derivative.x * _kd;
    _pid_info_x.FF = _target.x * _kff;

    _pid_info_y.target = _target.y;
    _pid_info_y.actual = measurement.y;
    _pid_info_y.error = _error.y;
    _pid_info_y.P = _error.y * _kp;
    _pid_info_y.I = _integrator.y;
    _pid_info_y.D = _derivative.y * _kd;
    _pid_info_y.FF = _target.y * _kff;

    return _error * _kp + _integrator + _derivative * _kd + _target * _kff;
}

Vector2f AC_PID_2D::update_all3(const Vector3f &target, const Vector3f &measurement, float dt, const Vector3f &limit)
{
    return update_all(Vector2f{target.x, target.y}, Vector2f{measurement.x, measurement.y}, dt, Vector2f{limit.x, limit.y});
}

//  update_i - update the integral
//  If the limit is set the integral is only allowed to reduce in the direction of the limit
void AC_PID_2D::update_i(float dt, const Vector2f &limit)
{
    _pid_info_x.limit = false;
    _pid_info_y.limit = false;

    Vector2f delta_integrator = (_error * _ki) * dt;
    float integrator_length = _integrator.length();
    _integrator += delta_integrator;
    // do not let integrator increase in length if delta_integrator is in the direction of limit
    if (is_positive(delta_integrator * limit) && _integrator.limit_length(integrator_length)) {
        _pid_info_x.limit = true;
        _pid_info_y.limit = true;
    }

    _integrator.limit_length(_kimax);
}

Vector2f AC_PID_2D::get_p() const
{
    return _error * _kp;
}

const Vector2f& AC_PID_2D::get_i() const
{
    return _integrator;
}

Vector2f AC_PID_2D::get_d() const
{
    return _derivative * _kd;
}


/* calculate saturation value to avoid chattering************
 */
float AC_PID_2D::saturation(float x)
{
	if (x > 1.0f) {
		return 1.0f;
	} else if (x < -1.0f) {
		return -1.0f;
	} else {
		return x;
	}
	return x;
}

/*
  Get throttle adaptive robust rule term
 */
float AC_PID_2D::_update_adaptive_robust_rule_x(float pid, float error, float error_dot, float error_int)
{
	// Calculate the adaptive_robust_rule to better deal with the uncertainties.
	// tau = s + rho*sign(s); rho = K0 + K1*||xi|| + K2*||xi||^2; K_i_dot = ||s|| * ||xi||^i - alfa * K_i, i = 0,1,2;
	// xi = [error, error_dot, error_int];
	

	float s = pid;
	//float norm_xi = sqrt(error*error +  error_dot*error_dot +  error_int*error_int);
	//float norm_s  = fabs(s);
	
	float sign_s = 1;
 	float rho = _intK0_x + _intK1_x * error + _intK2_x * error_dot + _intK3_x * error_int;
	float adaptive_term = 0;
//printf("_intK0_x %f, _intK1_x  %f, _intK2_x  %f,  _integrator_scaler %f \n" , _intK0_x, _intK1_x,  _intK2_x, _integrator_scaler);
	if (!is_zero(_dt))
	{
		float intK0_delta =  (_eta*s - _alpha * _intK0_x) * _dt;
		float intK1_delta =  (_eta*s * error - _alpha * _intK1_x) * _dt;
		float intK2_delta =  (_eta*s * error_dot - _alpha * _intK2_x) * _dt;
        float intK3_delta =  (_eta*s * error_int - _alpha * _intK3_x) * _dt;
		_intK0_x += intK0_delta;  
		_intK1_x += intK1_delta;
		_intK2_x += intK2_delta;
        _intK3_x += intK3_delta;
		
		_intK0_x *= _integrator_scaler;
		_intK1_x *= _integrator_scaler;
		_intK2_x *= _integrator_scaler;
        _intK3_x *= _integrator_scaler;
		
	}
	
/*	if (_intK0_x < 0) {
		_intK0_x = 0;
	}
	if (_intK1_x < 0) {
		_intK1_x = 0;
	}
	if (_intK2_x < 0) {
		_intK2_x = 0;
	}
    if (_intK3_x < 0) {
		_intK3_x = 0;
	}
*/
	adaptive_term = rho*sign_s;
		
//	printf("pid %f, e %f, e_dot %f,  e_int %f,   rho  %f, _intK0_x  %f,   _intK1_x %f,  _intK2_x %f,  sat_s %f,  xi %f,   adaptive_term %f\n" , 
//		(double)pid, (double)error, (double)error_dot, (double)error_int ,  rho ,  _intK0_x , _intK1_x ,  _intK2_x, sign_s, (double)norm_xi, (double)adaptive_term);

	return adaptive_term;
}

float AC_PID_2D::_update_adaptive_robust_rule_y(float pid, float error, float error_dot, float error_int)
{
	// Calculate the adaptive_robust_rule to better deal with the uncertainties.
	// tau = s + rho*sign(s); rho = K0 + K1*||xi|| + K2*||xi||^2; K_i_dot = ||s|| * ||xi||^i - alfa * K_i, i = 0,1,2;
	// xi = [error, error_dot, error_int];
	

	float s = pid;
	//float norm_xi = sqrt(error*error +  error_dot*error_dot +  error_int*error_int);
	//float norm_s  = fabs(s);
	
	float sign_s = 1;
 	float rho = _intK0_y + _intK1_y * error + _intK2_y * error_dot + _intK3_y * error_int;
	float adaptive_term = 0;
//printf("_intK0_y %f, _intK1_y  %f, _intK2_y  %f,  _integrator_scaler %f \n" , _intK0_y, _intK1_y,  _intK2_y, _integrator_scaler);
	if (!is_zero(_dt))
	{
		float intK0_delta =  (_eta*s - _alpha * _intK0_y) * _dt;
		float intK1_delta =  (_eta*s * error - _alpha * _intK1_y) * _dt;
		float intK2_delta =  (_eta*s * error_dot - _alpha * _intK2_y) * _dt;
        float intK3_delta =  (_eta*s * error_int - _alpha * _intK3_y) * _dt;
		_intK0_y += intK0_delta;  
		_intK1_y += intK1_delta;
		_intK2_y += intK2_delta;
        _intK3_y += intK3_delta;
//		printf("y      intK0_delta %f, intK1_delta %f, intK2_delta %f\n", intK0_delta, intK1_delta, intK2_delta);
		_intK0_y *= _integrator_scaler;
		_intK1_y *= _integrator_scaler;
		_intK2_y *= _integrator_scaler;
        _intK3_y *= _integrator_scaler;
		
	}
	// if (_intK0_y < 0) {
	// 	_intK0_y = 0;
	// }
	// if (_intK1_y < 0) {
	// 	_intK1_y = 0;
	// }
	// if (_intK2_y < 0) {
	// 	_intK2_y = 0;
	// }
    // if (_intK3_y < 0) {
	// 	_intK3_y = 0;
	// }

	adaptive_term = rho*sign_s;
		
//	printf("pid %f, e %f, e_dot %f,  e_int %f,   rho  %f, _intK0_y  %f,   _intK1_y %f,  _intK2_y %f,  sat_s %f,  xi %f,   adaptive_term %f\n" , 
//		(double)pid, (double)error, (double)error_dot, (double)error_int ,  rho ,  _intK0_y , _intK1_y ,  _intK2_y, sign_s, (double)norm_xi, (double)adaptive_term);

	return adaptive_term;
}

Vector2f AC_PID_2D::get_adaptive_term(float scaler)
{
	float x = 0;
	float y = 0;
	x = _update_adaptive_robust_rule_x(get_pid().x*scaler, _kp * _error.x * scaler, _kd * _derivative.x * scaler, _integrator.x * scaler);
	y = _update_adaptive_robust_rule_y(get_pid().y*scaler, _kp * _error.y * scaler, _kd * _derivative.y * scaler, _integrator.y * scaler);
}

Vector2f AC_PID_2D::get_pid()  //*************
{
    return get_p() + get_i() + get_d();  //****************
}


Vector2f AC_PID_2D::get_ff()
{
    _pid_info_x.FF = _target.x * _kff;
    _pid_info_y.FF = _target.y * _kff;
    return _target * _kff;
}

void AC_PID_2D::reset_I()
{
    _integrator.zero(); 
}

// save_gains - save gains to eeprom
void AC_PID_2D::save_gains()
{
    _kp.save();
    _ki.save();
    _kd.save();
    _kff.save();
    _kimax.save();
    _filt_E_hz.save();
    _filt_D_hz.save();
}

// get the target filter alpha
float AC_PID_2D::get_filt_E_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_E_hz);
}

// get the derivative filter alpha
float AC_PID_2D::get_filt_D_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_D_hz);
}

void AC_PID_2D::set_integrator(const Vector2f& target, const Vector2f& measurement, const Vector2f& i)
{
    set_integrator(target - measurement, i);
}

void AC_PID_2D::set_integrator(const Vector2f& error, const Vector2f& i)
{
    set_integrator(i - error * _kp);
}

void AC_PID_2D::set_integrator(const Vector2f& i)
{
    _integrator = i;
    _integrator.limit_length(_kimax);
}

