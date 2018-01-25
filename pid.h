/*
* pid.h
*
* Created: 12/5/2017 8:48:05 AM
*  Author: akudlacek
*/


#ifndef PID_H_
#define PID_H_


#include <stdint.h>


/******************************************************************************
* Defines
******************************************************************************/
typedef enum
{
	MANUAL_MODE,
	SUCCESS,
	MIN_GREATER_EQUAL_MAX,
	NULL_POINTER,
	NEGATIVE_PARAM
} pid_return_t;

typedef enum
{
	MANUAL,
	AUTOMATIC
} pid_mode_t;

typedef enum
{
	DIRECT,
	REVERSE
} pid_direction_t;

typedef struct
{
	//PID parameters entered
	float disp_kp;
	float disp_ki;
	float disp_kd;
	
	//PID parameters corrected for sample time
	float kp;
	float ki;
	float kd;
	
	//Configuration
	pid_direction_t direction;
	uint32_t sample_time_ms;
	float out_min;
	float out_max;
	pid_mode_t pid_mode;
	
	//Data pointers
	float *input;
	float *output;
	float *setpoint;
	uint32_t *current_time_ms;
	
	//Dynamic data
	uint32_t last_time_ms;
	float last_input;
	float p_component;
	float i_component;
	float d_component;
} pid_inst_t;

typedef struct
{
	//PID parameters
	float kp;
	float ki;
	float kd;
	
	//Configuration
	pid_direction_t direction;
	uint32_t sample_time_ms;
	float out_min;
	float out_max;
	
	//Data pointers
	float *input;
	float *output;
	float *setpoint;
	uint32_t *current_time_ms;
} pid_conf_t;


/******************************************************************************
* Prototypes
******************************************************************************/
pid_return_t pid_init(pid_inst_t *pid, pid_conf_t pid_settings);
pid_return_t pid_task(pid_inst_t *pid);
pid_return_t pid_set_tuning(pid_inst_t *pid, float kp, float ki, float kd);
void pid_set_sample_time_ms(pid_inst_t *pid, uint32_t sample_time_ms);
pid_return_t pid_set_output_limits(pid_inst_t *pid, float min, float max);
void pid_set_mode(pid_inst_t *pid, pid_mode_t pid_mode);
void pid_set_direction(pid_inst_t *pid, pid_direction_t direction);


#endif /* PID_H_ */