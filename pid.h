/*
* pid.h
*
* Created: 12/5/2017 8:48:05 AM
*  Author: akudlacek
*/


#ifndef PID_H_
#define PID_H_


#include <stdint.h>


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
typedef enum
{
	MANUAL_MODE,
	SUCCESS,
	MIN_GREATER_EQUAL_MAX,
	NULL_POINTER,
	DT_ZERO,
	OUTPUT_ERR              //floating point error on output
} pid_return_t;

typedef enum
{
	MANUAL,
	AUTOMATIC
} pid_mode_t;

typedef struct
{
	//PID gain values
	float kp;                     //Proportional gain
	float ki;                     //Integral gain
	float kd;                     //Derivative gain, derivative on measurement not filtered
	float kt;                     //Saturation tracking gain: suggested kt = sqrt(1/ki*1/kd)
	
	//PID component limits
	float p_max;
	float p_min;
	float i_max;
	float i_min;
	float d_max;
	float d_min;
	float out_min;
	float out_max;
	
	pid_mode_t pid_mode;
	
	//Data pointers
	float *input;
	float *output;
	float *setpoint;
	uint32_t *tick_ms;
} pid_conf_t;

typedef struct
{
	//PID configuration
	pid_conf_t conf;
	
	//Dynamic data
	float last_input;
	float p_component;
	float i_component;
	float d_component;
	float error;
	uint32_t last_tick_ms;
	float last_output;
	float last_output_sat;
} pid_inst_t;


/**************************************************************************************************
*                                            PROTOTYPES
*************************************************^************************************************/
void pid_get_config_defaults(pid_conf_t *pid_conf);
pid_return_t pid_init(pid_inst_t *pid, pid_conf_t pid_conf);
pid_return_t pid_task(pid_inst_t *pid);

void pid_set_kp(pid_inst_t *pid, float kp);
void pid_set_ki(pid_inst_t *pid, float ki);
void pid_set_kd(pid_inst_t *pid, float kd);
void pid_set_kt(pid_inst_t *pid, float kt);
pid_return_t pid_set_output_limits(pid_inst_t *pid, float min, float max);
void pid_set_mode(pid_inst_t *pid, pid_mode_t pid_mode);

float pid_get_kp(pid_inst_t pid);
float pid_get_ki(pid_inst_t pid);
float pid_get_kd(pid_inst_t pid);
float pid_get_kt(pid_inst_t pid);
float pid_get_out_min(pid_inst_t pid);
float pid_get_out_max(pid_inst_t pid);
pid_mode_t pid_get_mode(pid_inst_t pid);
float pid_get_p_component(pid_inst_t pid);
float pid_get_i_component(pid_inst_t pid);
float pid_get_d_component(pid_inst_t pid);
float pid_get_error(pid_inst_t pid);


#endif /* PID_H_ */