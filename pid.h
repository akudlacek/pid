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
typedef enum pid_return_t
{
	MANUAL_MODE,
	SUCCESS,
	MIN_GREATER_EQUAL_MAX,
	NULL_POINTER,
	DT_ZERO,
	OUTPUT_ERR              //floating point error on output
} pid_return_t;

typedef enum pid_mode_t
{
	MANUAL,
	AUTOMATIC
} pid_mode_t;

typedef struct pid_conf_t
{
	//PID gain values
	float kp;                     //Proportional gain
	float ki;                     //Integral gain
	float kd;                     //Derivative gain, derivative on measurement not filtered
	//todo: Document kt a little better. Double check the suggested kt. Doesn't seem right
	float kt;                     //Saturation tracking gain: suggested kt = sqrt(1/ki*1/kd)

	//Filter
	float d_filter;               //0 no filtering, >0 low pass filtering, larger the lower

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
	volatile const float    * fb_in_ptr;    //feedback input
	volatile       float    * output_ptr;   //output
	volatile const float    * setpoint_ptr; //setpoint input
	volatile const uint32_t * tick_ptr;     //time tick input
} pid_conf_t;

typedef struct pid_inst_t
{
	//PID configuration
	pid_conf_t conf;

	//Dynamic data
	float last_fb_in;
	float d_fb_in;       //derivative of the feedback input
	float p_component;
	float i_component;
	float d_component;
	float error;
	uint32_t last_tick;
	float last_output;
	float last_output_sat;
} pid_inst_t;


/**************************************************************************************************
*                                            PROTOTYPES
*************************************************^************************************************/
void         pid_get_config_defaults(pid_conf_t * const pid_conf);
pid_return_t pid_init               (pid_inst_t * const pid, const pid_conf_t pid_conf);
pid_return_t pid_task               (pid_inst_t * const pid);

void         pid_set_kp             (pid_inst_t * const pid, const float kp);
void         pid_set_ki             (pid_inst_t * const pid, const float ki);
void         pid_set_kd             (pid_inst_t * const pid, const float kd);
void         pid_set_kt             (pid_inst_t * const pid, const float kt);
void         pid_set_d_filter       (pid_inst_t * const pid, const float d_filter);
pid_return_t pid_set_output_limits  (pid_inst_t * const pid, const float min, const float max);
void         pid_set_mode           (pid_inst_t * const pid, const pid_mode_t pid_mode);

float        pid_get_kp             (const pid_inst_t pid);
float        pid_get_ki             (const pid_inst_t pid);
float        pid_get_kd             (const pid_inst_t pid);
float        pid_get_kt             (const pid_inst_t pid);
float        pid_get_d_filter       (const pid_inst_t pid);
float        pid_get_d_fb_in        (const pid_inst_t pid);
float        pid_get_out_min        (const pid_inst_t pid);
float        pid_get_out_max        (const pid_inst_t pid);
pid_mode_t   pid_get_mode           (const pid_inst_t pid);
float        pid_get_p_component    (const pid_inst_t pid);
float        pid_get_i_component    (const pid_inst_t pid);
float        pid_get_d_component    (const pid_inst_t pid);
float        pid_get_error          (const pid_inst_t pid);


#endif /* PID_H_ */
