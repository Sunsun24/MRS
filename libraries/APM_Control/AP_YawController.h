#pragma once

#include <AP_Common/AP_Common.h>
#include <AC_PID/AC_PID.h>
#include "AP_AutoTune.h"
#include <AP_Vehicle/AP_Vehicle.h>

class AP_YawController
{
public:
    AP_YawController(const AP_FixedWing &parms);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_YawController);

    // return true if rate control or damping is enabled
    bool enabled() const { return rate_control_enabled() || (_K_D > 0.0); } 

    // return true if rate control is enabled
    bool rate_control_enabled(void) const { return _rate_enable != 0; }

    // get actuator output for sideslip and yaw damping control
    int32_t get_servo_out(float scaler, bool disable_integrator);

    // get actuator output for direct rate control
    // desired_rate is in deg/sec. scaler is the surface speed scaler
    float get_rate_out(float desired_rate, float scaler, bool disable_integrator);

    void reset_I();

    void reset_rate_PID();

    /*
      reduce the integrator, used when we have a low scale factor in a quadplane hover
    */
    void decay_I()
    {
        // this reduces integrator by 95% over 2s
        _pid_info.I *= 0.995f;
    }

    float saturation(float x);
	float sign(float x);
    //float _update_yaw_adaptive_robust_rule(float pid_sum, float error, float error_dot, float delta_time);
    float _update_yaw_adaptive_robust_rule(float pid_sum, float FF, float error, float error_dot, float error_int, float delta_time);
    const AP_PIDInfo& get_pid_info(void) const
    {
        return _pid_info;
    }

    // start/stop auto tuner
    void autotune_start(void);
    void autotune_restore(void);
    

    static const struct AP_Param::GroupInfo var_info[];

private:
    const AP_FixedWing &aparm;
    AP_Float _K_A;
    AP_Float _K_I;
    AP_Float _K_D;
    AP_Float _K_FF;
    AP_Int16 _imax;
    AP_Int8  _rate_enable;
    AC_PID rate_pid{0.04, 0.15, 0, 0.15, 0.666, 3, 0, 12, 150, 1};

    uint32_t _last_t;
    float _last_out;
    float _last_rate_hp_out;
    float _last_rate_hp_in;
    float _K_D_last;

    //**********************************
	float _satYaw=1000.0f;
	float _asmc_alfa=  0.001;//0.021514;//   0.000108;//0.005431;//0.002019;//1.138;//220.0f;//221.96;//0.007f;//0.006779f;
	float _sat_eps=0.001;//0.000102;//0.0001;// 0.003361;//30.018;//4.125;//4.125474;//1.0f;
	float _eta = 1;//0.0001;//0.03f;//85.607f;//85.607552f;
	float _last_sumPid = 0;
	
/*	float _asmc_alfa=220.0f;//221.96;//0.007f;//0.006779f;
	float _sat_eps=1.0f;
	float _eta = 0.03f;//85.607f;//85.607552f;*/
	
	float _lambda3Yaw =   0.001;//                          0.002512;
	float _gammaYaw =   0.001;//                             0.002492;         //no optimizatin 0.5
    float _varepsYaw =   10.0;//                              0.048459;        //no optimizatin 0.5
	float _upsilonYaw =   0.001;//                                   2.545184;
	float _betaYaw =    0.001;//                            11.014763;
	float _vYaw =   0.472179;//                               0.999;             //no optimizatin  0.07
	float _sigmaYaw =  1.4;//  14.872527;//                     1.849223;
	
	// yaw ASMC controller integraor parameter, the upper saturation limit can be tuned by _satYaw, the lower limit is 0
	float _intK0Yaw = 0.00001;
	float _intK1Yaw = 0.00001;
	float _intK2Yaw = 0.00001;
    float _intK3Yaw = 0.00001;
//********************************************

    float _integrator;

    AP_AutoTune::ATGains gains;
    AP_AutoTune *autotune;
    bool failed_autotune_alloc;
    
    AP_PIDInfo _pid_info;
};
