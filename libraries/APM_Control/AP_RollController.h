#pragma once

#include <AP_Common/AP_Common.h>
#include "AP_AutoTune.h"
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PID.h>
#include <AP_Vehicle/AP_Vehicle.h>

class AP_RollController
{
public:
    AP_RollController(const AP_FixedWing &parms);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_RollController);

    float get_rate_out(float desired_rate, float scaler);
    float get_servo_out(int32_t angle_err, float scaler, bool disable_integrator, bool ground_mode);

    void reset_I();

    /*
      reduce the integrator, used when we have a low scale factor in a quadplane hover
    */
    void decay_I()
    {
        // this reduces integrator by 95% over 2s
        _pid_info.I *= 0.995f;
        rate_pid.set_integrator(rate_pid.get_i() * 0.995);
    }

    void autotune_start(void);
    void autotune_restore(void);

    float saturation(float x);
	float _update_roll_adaptive_robust_rule(float pid_sum,float FF, float error, float error_dot, float error_int, float delta_time);
	float sign(float x);

    const AP_PIDInfo& get_pid_info(void) const
    {
        return _pid_info;
    }

    static const struct AP_Param::GroupInfo var_info[];


    // tuning accessors
    AP_Float &kP(void) { return rate_pid.kP(); }
    AP_Float &kI(void) { return rate_pid.kI(); }
    AP_Float &kD(void) { return rate_pid.kD(); }
    AP_Float &kFF(void) { return rate_pid.ff(); }
    AP_Float &tau(void) { return gains.tau; }

    void convert_pid();

private:
    const AP_FixedWing &aparm;
    AP_AutoTune::ATGains gains;
    AP_AutoTune *autotune;
    bool failed_autotune_alloc;
    uint32_t _last_t;    //****************add
    float _last_out;
    AC_PID rate_pid{0.08, 0.15, 0, 0.345, 0.666, 3, 0, 12, 150, 1};
    float angle_err_deg;

//*********************
    float _last_sumPid = 0;
	float _satRoll=1000.0f;
	float _asmc_alfa=158.298;//    1000.0;// 1000.0;// 1000.0;//  79.746391;//               5.169132;//0.326;//170.75;//170.751556;//0.01f;//0.00284f;
	float _sat_eps= 1000.0;//1000.0;// 88.044846;//   100;// 100.0;       //100.0;//30.018;//4.125;//4.125474;//1.0f;
	float _eta = 1;// 1.01;             //0.000003;//   0.01;//0.000004;//               0.00000;//000001;//0.000001;//0.00000001;//0.000066f;

	
/*	float _asmc_alfa=0.01f;//0.00284f;
	float _sat_eps=1.0f;
	float _eta = 0.00000001;//0.000066f;*/
	
	float _lambda3Roll =   0.001;//                       0.002512;
	float _gammaRoll =    0.001;//                          0.002492;         //no optimizatin 0.5
    float _varepsRoll =   10.0;//                            0.048459;        //no optimizatin 0.5
	float _upsilonRoll =   1000.0;//                         2.545184;
	float _betaRoll =     158.298;//                            11.014763;
	float _vRoll =     0.99;//                                0.999;             //no optimizatin  0.07
	float _sigmaRoll =    0.001;//                            1.849223;
	

	// roll ASMC controller integraor parameter, the upper saturation limit can be tuned by _satRoll, the lower limit is 0
	float _intK0Roll = 0.00001;
	float _intK1Roll = 0.00001;
	float _intK2Roll = 0.00001;
    float _intK3Roll = 0.00001;
//***************************

    AP_PIDInfo _pid_info;

    float _get_rate_out(float desired_rate, float scaler, bool disable_integrator, bool ground_mode);
};
