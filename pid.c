/*
* pid.c
*
* Created: 12/5/2017 8:47:56 AM
*  Author: akudlacek
*
* Based off of https://github.com/br3ttb/Arduino-PID-Library
*/


#include "pid.h"


/******************************************************************************
*  Local Prototypes
******************************************************************************/
static inline void clamp_and_windup(float min, float max, float *i_component, float *output);


/******************************************************************************
*  \brief PID Init
*
*  \note
******************************************************************************/
pid_return_t pid_init(pid_inst_t *pid, pid_conf_t pid_settings)
{
	pid_return_t pid_return;
	
	/*Set data pointers*/
	if(pid_settings.output == 0 || pid_settings.input == 0 || pid_settings.setpoint == 0 || pid_settings.current_time_ms == 0)
	{
		return NULL_POINTER;
	}
	else
	{
		pid->output = pid_settings.output;
		pid->input = pid_settings.input;
		pid->setpoint = pid_settings.setpoint;
		pid->current_time_ms = pid_settings.current_time_ms;
	}
	
	/*Set configuration*/
	pid->sample_time_ms = pid_settings.sample_time_ms;
	pid->direction = pid_settings.direction;
	
	pid_return = pid_set_tuning(pid, pid_settings.kp, pid_settings.ki, pid_settings.kd);
	if(pid_return != SUCCESS)
	{
		return pid_return;
	}
	
	pid_return = pid_set_output_limits(pid, pid_settings.out_min, pid_settings.out_max);
	if(pid_return != SUCCESS)
	{
		return pid_return;
	}
	
	/*Turn on PID*/
	pid->pid_mode = AUTOMATIC;
	
	/*Record time*/
	pid->last_time_ms = *pid->current_time_ms;
	
	return SUCCESS;
}

/******************************************************************************
*  \brief PID task
*
*  \note needs to be called faster then sample time
*        Returns SUCCESS for new output or MANUAL_MODE for nothing done
*        Derivative on Measurement used
******************************************************************************/
pid_return_t pid_task(pid_inst_t *pid)
{
	uint32_t time_ms   = 0;
	float input        = 0;
	float error        = 0;
	float d_input      = 0;
	float output       = 0;
	
	/*MANUAL mode returns without doing anything*/
	if(pid->pid_mode == AUTOMATIC)
	{
		/*Get current time*/
		time_ms = *pid->current_time_ms;
		
		/*Check if it is time to calculate PID*/
		if((time_ms - pid->last_time_ms) >= pid->sample_time_ms)
		{
			/*Compute all the working error variables*/
			input        = *pid->input;
			error        = *pid->setpoint - input;
			d_input = input - pid->last_input;
			
			/****Calculate P component****/
			pid->p_component = pid->kp * error;
			
			/****Calculate I component****/
			pid->i_component += pid->ki * error;
			
			/****Calculate D component****/
			pid->d_component = pid->kd * d_input;
			
			/*Sum PID components*/
			output = pid->p_component + pid->i_component - pid->d_component;
			
			/*Clamp output to limits and windup protection*/
			clamp_and_windup(pid->out_min, pid->out_max, &pid->i_component, &output);
			
			/*Set output*/
			*pid->output = output;
			
			/*Remember some variables for next time*/
			pid->last_input = input;
			pid->last_time_ms = time_ms;

			return SUCCESS;
		}
	}
	
	return MANUAL_MODE;
}


/******************************************************************************
*  \brief Set Tuning
*
*  \note
******************************************************************************/
pid_return_t pid_set_tuning(pid_inst_t *pid, float kp, float ki, float kd)
{
	float sample_time_s;
	
	/*Keep only positive PID parameters*/
	if(kp < 0 || ki < 0 || kd < 0)
	{
		return NEGATIVE_PARAM;
	}
	
	/*Remove residual sum from integrator on ki of zero*/
	if(ki <= 0.0)
	{
		pid->i_component = 0.0;
	}
	
	/*Set display PID parameters*/
	pid->disp_kp = kp;
	pid->disp_ki = ki;
	pid->disp_kd = kd;
	
	/*Rescale PID parameters for new sample time in mS*/
	sample_time_s = (float)pid->sample_time_ms / 1000.0;
	pid->kp = kp;
	pid->ki = ki * sample_time_s;
	pid->kd = kd / sample_time_s;
	
	/*Set PID direction*/
	if(pid->direction == REVERSE)
	{
		pid->kp = -pid->kp;
		pid->ki = -pid->ki;
		pid->kd = -pid->kd;
	}
	
	return SUCCESS;
}

/******************************************************************************
*  \brief Set sample time
*
*  \note sets the period, in Milliseconds, at which the calculation is performed
******************************************************************************/
void pid_set_sample_time_ms(pid_inst_t *pid, uint32_t sample_time_ms)
{
	float ratio;
	
	ratio  = (float)sample_time_ms / (float)pid->sample_time_ms;
	
	pid->ki *= ratio;
	pid->kd /= ratio;
	pid->sample_time_ms = sample_time_ms;
}


/******************************************************************************
*  \brief Set Output Limits
*
*  \note
******************************************************************************/
pid_return_t pid_set_output_limits(pid_inst_t *pid, float min, float max)
{
	if(min >= max)
	{
		return MIN_GREATER_EQUAL_MAX;
	}
	
	pid->out_min = min;
	pid->out_max = max;
	
	clamp_and_windup(min, max, &pid->i_component, pid->output);
	
	return SUCCESS;
}

/******************************************************************************
*  \brief Sets PID in AUTOMATIC or MANUAL mode
*
*  \note AUTOMATIC = PID controls output, MANUAL = user controls output
******************************************************************************/
void pid_set_mode(pid_inst_t *pid, pid_mode_t pid_mode)
{
	/*If going from MANUAL to AUTOMATIC*/
	if((pid->pid_mode == MANUAL) && (pid_mode == AUTOMATIC))
	{
		pid->i_component = *pid->output;
		pid->last_input = *pid->input;
		
		clamp_and_windup(pid->out_min, pid->out_max, &pid->i_component, pid->output);
	}
	
	pid->pid_mode = pid_mode;
}

/******************************************************************************
*  \brief Set PID direction
*
*  \note
******************************************************************************/
void pid_set_direction(pid_inst_t *pid, pid_direction_t direction)
{
	if(pid->direction != direction)
	{
		pid->kp = -pid->kp;
		pid->ki = -pid->ki;
		pid->kd = -pid->kd;
		
		pid->direction = direction;
	}
}

/******************************************************************************
*  \brief Clamp and windup protection
*
*  \note
******************************************************************************/
static inline void clamp_and_windup(float min, float max, float *i_component, float *output)
{
	if(*output > max)
	{
		*i_component -= *output - max; //Integral windup protection
		*output = max;                 //Clamp output
	}
	else if(*output < min)
	{
		*i_component += min - *output; //Integral windup protection
		*output = min;                 //Clamp output
	}
}
