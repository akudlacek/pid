/*
* pid.c
*
* Created: 12/5/2017 8:47:56 AM
*  Author: akudlacek
*
*/


#include "pid.h"

#include <math.h>


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
#define PID_LIM(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt))) //keeps within high and low range


/**************************************************************************************************
*                                         LOCAL PROTOTYPES
*************************************************^************************************************/


/**************************************************************************************************
*                                            FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief PID get config defaults
*
*  \note
******************************************************************************/
void pid_get_config_defaults(pid_conf_t * const pid_conf)
{
	pid_conf->kp = 0;
	pid_conf->ki = 0;
	pid_conf->kd = 0;
	pid_conf->kt = 0;

	pid_conf->d_filter = 0;

	pid_conf->p_max = 0;
	pid_conf->p_min = 0;
	pid_conf->i_max = 0;
	pid_conf->i_min = 0;
	pid_conf->d_max = 0;
	pid_conf->d_min = 0;

	pid_conf->out_min   = 0;
	pid_conf->out_max   = 0;
	pid_conf->pid_mode  = MANUAL;

	pid_conf->fb_in_ptr    = 0;
	pid_conf->output_ptr   = 0;
	pid_conf->setpoint_ptr = 0;
	pid_conf->tick_ptr     = 0;
}

/******************************************************************************
*  \brief PID Init
*
*  \note
******************************************************************************/
pid_return_t pid_init(pid_inst_t * const pid, const pid_conf_t pid_conf)
{
	pid_return_t pid_return;

	/*NULL pointer check*/
	if(pid_conf.fb_in_ptr == 0 || pid_conf.output_ptr == 0 || pid_conf.setpoint_ptr == 0 || pid_conf.tick_ptr == 0)
	{
		return NULL_POINTER;
	}

	/*Conf*/
	pid->conf = pid_conf;

	/*Inst*/
	pid->last_fb_in      = 0;
	pid->d_fb_in         = 0;
	pid->p_component     = 0;
	pid->i_component     = 0;
	pid->d_component     = 0;
	pid->error           = 0;
	pid->last_tick       = *pid->conf.tick_ptr;
	pid->last_output     = 0;
	pid->last_output_sat = 0;

	/*Set gains*/
	pid_set_kp(pid, pid_conf.kp);
	pid_set_ki(pid, pid_conf.ki);
	pid_set_kd(pid, pid_conf.kd);
	pid_set_kt(pid, pid_conf.kt);

	/*Set filter*/
	pid_set_d_filter(pid, pid_conf.d_filter);

	/*Set output limits*/
	pid_return = pid_set_output_limits(pid, pid_conf.out_min, pid_conf.out_max);

	if(pid_return == SUCCESS)
	{
		/*Turn on PID*/
		pid_set_mode(pid, AUTOMATIC);
	}

	return pid_return;
}

/******************************************************************************
*  \brief PID task
*
*  \note Returns SUCCESS for new output or MANUAL_MODE for nothing done
*        Derivative on Measurement used
******************************************************************************/
pid_return_t pid_task(pid_inst_t * const pid)
{
	pid_return_t pid_return = MANUAL_MODE;
	float fb_in             = 0;
	float output            = 0;
	uint32_t tick           = *pid->conf.tick_ptr;
	float saturation_error  = 0;
	
	pid->dt = (tick - pid->last_tick);

	/*Check for nonzero dt*/
	if(pid->dt == 0)
	{
		pid_return = DT_ZERO;
	}

	/*MANUAL mode returns without doing anything*/
	else if(pid->conf.pid_mode == AUTOMATIC)
	{
		/*Compute all the working error variables*/
		fb_in             = *pid->conf.fb_in_ptr;
		pid->error        = *pid->conf.setpoint_ptr - fb_in;

		/****Calculate the filtered derivitive of the feedback input****/
		if(pid->conf.d_filter > 0.0)
		{
			pid->d_fb_in      -= pid->d_fb_in / (pid->conf.d_filter + 1.0);
			pid->d_fb_in      += ((fb_in - pid->last_fb_in) / (float)pid->dt) / (pid->conf.d_filter + 1.0);
		}
		else
		{
			pid->d_fb_in      = ((fb_in - pid->last_fb_in) / (float)pid->dt);
		}


		/****Calculate P component****/
		pid->p_component = pid->conf.kp * pid->error;

		/****Calculate I component****/
		/*Remove residual sum from integrator for ki of zero*/
		if(pid->conf.ki == 0.0)
		{
			pid->i_component = 0.0;
		}
		else
		{
			saturation_error = pid->last_output_sat - pid->last_output;
			pid->i_component += ((pid->conf.ki * pid->error) + (pid->conf.kt * saturation_error)) * (float)pid->dt;
		}

		/****Calculate D component****/
		pid->d_component = -1 * (pid->conf.kd * pid->d_fb_in);

		/*Component Limits*/
		pid->p_component = PID_LIM(pid->p_component, pid->conf.p_min, pid->conf.p_max);
		pid->i_component = PID_LIM(pid->i_component, pid->conf.i_min, pid->conf.i_max);
		pid->d_component = PID_LIM(pid->d_component, pid->conf.d_min, pid->conf.d_max);

		/*Sum PID components*/
		output = pid->p_component + pid->i_component + pid->d_component;

		/*Set output with limit*/
		*pid->conf.output_ptr = PID_LIM(output, pid->conf.out_min, pid->conf.out_max);

		/*Remember some variables for next time*/
		pid->last_fb_in      = fb_in;
		pid->last_tick       = tick;
		pid->last_output     = output;
		pid->last_output_sat = *pid->conf.output_ptr;

		pid_return = SUCCESS;

		/*Check for float errors*/
		if(!isnormal(*pid->conf.output_ptr) && *pid->conf.output_ptr != 0.0)
		{
			pid_return = OUTPUT_ERR;
		}
	}

	return pid_return;
}

/******************************************************************************
*  \brief Set kp
*
*  \note
******************************************************************************/
void pid_set_kp(pid_inst_t * const pid, const float kp)
{
	pid->conf.kp = kp;
}

/******************************************************************************
*  \brief Set ki
*
*  \note
******************************************************************************/
void pid_set_ki(pid_inst_t * const pid, const float ki)
{
	pid->conf.ki = ki;
}

/******************************************************************************
*  \brief Set kd
*
*  \note
******************************************************************************/
void pid_set_kd(pid_inst_t * const pid, const float kd)
{
	pid->conf.kd = kd;
}

/******************************************************************************
*  \brief Set kt
*
*  \note
******************************************************************************/
void pid_set_kt(pid_inst_t * const pid, const float kt)
{
	pid->conf.kt = kt;
}

/******************************************************************************
*  \brief Set derivative filter level
*
*  \note
******************************************************************************/
void pid_set_d_filter(pid_inst_t * const pid, const float d_filter)
{
	pid->conf.d_filter = d_filter;
}

/******************************************************************************
*  \brief Set Output Limits
*
*  \note
******************************************************************************/
pid_return_t pid_set_output_limits(pid_inst_t * const pid, const float min, const float max)
{
	if(min >= max)
	{
		return MIN_GREATER_EQUAL_MAX;
	}

	else
	{
		pid->conf.out_min = min;
		pid->conf.out_max = max;

		*pid->conf.output_ptr = PID_LIM(*pid->conf.output_ptr, pid->conf.out_min, pid->conf.out_max);
	}

	return SUCCESS;
}

/******************************************************************************
*  \brief Sets PID in AUTOMATIC or MANUAL mode
*
*  \note AUTOMATIC = PID controls output, MANUAL = user controls output
******************************************************************************/
void pid_set_mode(pid_inst_t * const pid, const pid_mode_t pid_mode)
{
	/*If going from MANUAL to AUTOMATIC*/
	if((pid->conf.pid_mode == MANUAL) && (pid_mode == AUTOMATIC))
	{
		pid->i_component      = *pid->conf.output_ptr;
		pid->last_fb_in       = *pid->conf.fb_in_ptr;
		pid->last_tick        = *pid->conf.tick_ptr;
		pid->d_fb_in          = 0;
	}

	pid->conf.pid_mode = pid_mode;
}

/******************************************************************************
*  \brief Get kp
*
*  \note
******************************************************************************/
float pid_get_kp(const pid_inst_t pid)
{
	return pid.conf.kp;
}

/******************************************************************************
*  \brief Get ki
*
*  \note
******************************************************************************/
float pid_get_ki(const pid_inst_t pid)
{
	return pid.conf.ki;
}

/******************************************************************************
*  \brief Get kd
*
*  \note
******************************************************************************/
float pid_get_kd(const pid_inst_t pid)
{
	return pid.conf.kd;
}

/******************************************************************************
*  \brief Get kt
*
*  \note
******************************************************************************/
float pid_get_kt(const pid_inst_t pid)
{
	return pid.conf.kt;
}

/******************************************************************************
*  \brief Get d filter level
*
*  \note
******************************************************************************/
float pid_get_d_filter(const pid_inst_t pid)
{
	return pid.conf.d_filter;
}

/******************************************************************************
*  \brief Get d feedback input
*
*  \note this is the filtered or not filtered feedback input
******************************************************************************/
float pid_get_d_fb_in(const pid_inst_t pid)
{
	return pid.d_fb_in;
}

/******************************************************************************
*  \brief Get out min
*
*  \note
******************************************************************************/
float pid_get_out_min(const pid_inst_t pid)
{
	return pid.conf.out_min;
}

/******************************************************************************
*  \brief Get out max
*
*  \note
******************************************************************************/
float pid_get_out_max(const pid_inst_t pid)
{
	return pid.conf.out_max;
}

/******************************************************************************
*  \brief Get mode
*
*  \note
******************************************************************************/
pid_mode_t pid_get_mode(const pid_inst_t pid)
{
	return pid.conf.pid_mode;
}

/******************************************************************************
*  \brief Get p component
*
*  \note
******************************************************************************/
float pid_get_p_component(const pid_inst_t pid)
{
	return pid.p_component;
}

/******************************************************************************
*  \brief Get i component
*
*  \note
******************************************************************************/
float pid_get_i_component(const pid_inst_t pid)
{
	return pid.i_component;
}

/******************************************************************************
*  \brief Get d component
*
*  \note
******************************************************************************/
float pid_get_d_component(const pid_inst_t pid)
{
	return pid.d_component;
}

/******************************************************************************
*  \brief Get error
*
*  \note
******************************************************************************/
float pid_get_error(const pid_inst_t pid)
{
	return pid.error;
}

/******************************************************************************
*  \brief Get dt
*
*  \note Time between PID runs, in ticks
******************************************************************************/
uint32_t pid_get_dt(const pid_inst_t pid)
{
	return pid.dt;
}


/**************************************************************************************************
*                                         LOCAL FUNCTIONS
*************************************************^************************************************/

