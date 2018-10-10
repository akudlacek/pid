/*
* pid.c
*
* Created: 12/5/2017 8:47:56 AM
*  Author: akudlacek
*
* Based off of https://github.com/br3ttb/Arduino-PID-Library
*/


#include "pid.h"


/**************************************************************************************************
*                                         LOCAL PROTOTYPES
*************************************************^************************************************/
static inline void clamp_and_windup(float min, float max, float *i_component, float *output);


/**************************************************************************************************
*                                            FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief PID get config defaults
*
*  \note
******************************************************************************/
void pid_get_config_defaults(pid_conf_t *pid_conf)
{
	pid_conf->kp = 0;
	pid_conf->ki = 0;
	pid_conf->kd = 0;
	
	pid_conf->direction = 0;
	pid_conf->out_min = 0;
	pid_conf->out_max = 0;
	
	pid_conf->input = 0;
	pid_conf->output = 0;
	pid_conf->setpoint = 0;
}

/******************************************************************************
*  \brief PID Init
*
*  \note
******************************************************************************/
pid_return_t pid_init(pid_inst_t *pid, pid_conf_t pid_conf)
{
	pid_return_t pid_return;
	
	/*NULL pointer check*/
	if(pid_conf.output == 0 || pid_conf.input == 0 || pid_conf.setpoint == 0)
	{
		pid_return = NULL_POINTER;
	}
	
	else
	{
		/*Set data pointers*/
		pid->output = pid_conf.output;
		pid->input = pid_conf.input;
		pid->setpoint = pid_conf.setpoint;
		
		/*Set direction*/
		pid->direction = pid_conf.direction;
		
		/*Set gain*/
		pid_return = pid_set_tuning(pid, pid_conf.kp, pid_conf.ki, pid_conf.kd);
		
		if(pid_return == SUCCESS)
		{
			/*Set output limits*/
			pid_return = pid_set_output_limits(pid, pid_conf.out_min, pid_conf.out_max);
			
			if(pid_return == SUCCESS)
			{
				/*Turn on PID*/
				pid->pid_mode = AUTOMATIC;
			}
		}
	}
	
	return pid_return;
}

/******************************************************************************
*  \brief PID task
*
*  \note Returns SUCCESS for new output or MANUAL_MODE for nothing done
*        Derivative on Measurement used
******************************************************************************/
pid_return_t pid_task(pid_inst_t *pid)
{
	pid_return_t pid_return = MANUAL_MODE;
	float input        = 0;
	float error        = 0;
	float d_input      = 0;
	float output       = 0;
	
	/*MANUAL mode returns without doing anything*/
	if(pid->pid_mode == AUTOMATIC)
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

		pid_return = SUCCESS;
	}
	
	return pid_return;
}

/******************************************************************************
*  \brief Set Tuning
*
*  \note
******************************************************************************/
pid_return_t pid_set_tuning(pid_inst_t *pid, float kp, float ki, float kd)
{
	pid_return_t pid_return = SUCCESS;
	
	/*Keep only positive PID parameters*/
	if(kp < 0 || ki < 0 || kd < 0)
	{
		pid_return =  NEGATIVE_PARAM;
	}
	
	else
	{
		/*Remove residual sum from integrator on ki of zero*/
		if(ki <= 0.0)
		{
			pid->i_component = 0.0;
		}
		
		/*Set new PID gain values*/
		pid->kp = kp;
		pid->ki = ki;
		pid->kd = kd;
		
		/*Set PID direction*/
		if(pid->direction == REVERSE)
		{
			pid->kp = -pid->kp;
			pid->ki = -pid->ki;
			pid->kd = -pid->kd;
		}
	}
	
	return pid_return;
}

/******************************************************************************
*  \brief Set Output Limits
*
*  \note
******************************************************************************/
pid_return_t pid_set_output_limits(pid_inst_t *pid, float min, float max)
{
	pid_return_t pid_return = SUCCESS;
	
	if(min >= max)
	{
		pid_return = MIN_GREATER_EQUAL_MAX;
	}
	
	else
	{
		pid->out_min = min;
		pid->out_max = max;
		
		clamp_and_windup(min, max, &pid->i_component, pid->output);
	}
	
	return pid_return;
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


/**************************************************************************************************
*                                         LOCAL FUNCTIONS
*************************************************^************************************************/
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
