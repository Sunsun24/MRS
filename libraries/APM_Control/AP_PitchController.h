#pragma once

#include <AP_Common/AP_Common.h>
#include "AP_AutoTune.h"
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PID.h>
#include <AP_Vehicle/AP_Vehicle.h>

class AP_PitchController
{
public:
    AP_PitchController(const AP_FixedWing &parms);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_PitchController);

    float get_rate_out(float desired_rate, float scaler);
    float get_servo_out(int32_t angle_err, float scaler, bool disable_integrator, bool ground_mode);

    float saturation(float x);
	float sign(float x);
	float _update_pitch_adaptive_robust_rule(float pid_sum, float FF, float error, float error_dot, float error_int, float delta_time);
    
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

    const AP_PIDInfo& get_pid_info(void) const
    {
        return _pid_info;
    }

    static const struct AP_Param::GroupInfo var_info[];

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
    AP_Int16 _max_rate_neg;
    AP_Float _roll_ff;
    uint32_t _last_t;          //*******************
    float _last_out;
    AC_PID rate_pid{0.04, 0.15, 0, 0.345, 0.666, 3, 0, 12, 150, 1};
    float angle_err_deg;

//**********************************
    float _last_sumPid = 0;
	float _satPitch=1000.0f;
	float _asmc_alfa=10.147;//     0.555513;// 6.697521;// 201.728607;//            0.00001;//1000;//0.004;//0.003987;//670.0f;//671.64;//577.50f;//577.496704f;
	float _sat_eps= 420.185;// 1000.0;//  88.044846;//  100;// 100.0;     //100.0;//30.018;//4.125;//4.125474;//1.0f;
	float _eta =  1;//  0.01;             //0.003421;// 0.01;//0.000007;//             0.000005;//0046;//0.01f;//0.0008f;//0.000765f;

	
/*	float _asmc_alfa=670.0f;//671.64;//577.50f;//577.496704f;
	float _sat_eps=1.0f;
	float _eta = 0.01f;//0.0008f;//0.000765f;*/
	
	float _lambda3Pitch =  0.001;//  0.001;//                                  0.002512;
	float _gammaPitch =   0.001; // 0.001;                                  0.002492;         //no optimizatin 0.5
    float _varepsPitch =    10.0;//  10.0                                 0.048459;        //no optimizatin 0.5
	float _upsilonPitch =    420.185;//                                   2.545184;
	float _betaPitch =       10.147;//                                   11.014763;
	float _vPitch =   0.8;//0.7;//    0.99;//                                        0.999;             //no optimizatin  0.07
	float _sigmaPitch =  3.6;//  0.966444;//                                   1.849223;
	

	// pitch ASMC controller integraor parameter, the upper saturation limit can be tuned by _satPitch, the lower limit is 0
	float _intK0Pitch = 0.00001;
    float _intK1Pitch = 0.00001;
    float _intK2Pitch = 0.00001;
    float _intK3Pitch = 0.00001;
//********************************


    AP_PIDInfo _pid_info;

    float _get_rate_out(float desired_rate, float scaler, bool disable_integrator, float aspeed, bool ground_mode);
    float _get_coordination_rate_offset(float &aspeed, bool &inverted) const;
};
