#include "dab_control.h"

float mode = 0.0f; // global variable to store the mode value
float k_aux = 0.0f;

float calc_control_loop(volatile control_t *control, volatile pi_controller_t *pi_controller, float i_meas)
{
	float phi_out = 0.0f;
	float i_ref_aux = clamp_magnitude(control->i_ref_usr, MIN_ABS_I_REF, MAX_ABS_I_REF);

	//  Apply rate limit to the reference current or not
	if(control->current_rate_limit_enable)
	{
		control->i_ref = rate_limitter(i_ref_aux, control->i_ref, CURRENT_RATE_LIMIT);
	}
	else
	{
		control->i_ref = i_ref_aux;
	}

	switch(control->control_mode_act)
		{
		  case open_loop:
			 /* phi_out = saturate(control->phi_ref, MIN_PHI_OPEN_LOOP, MAX_PHI_OPEN_LOOP);*/
			  control->phi_ref = clamp_magnitude(control->phi_ref_usr, MIN_PHI_OPEN_LOOP, MAX_PHI_OPEN_LOOP);
			  phi_out = control->phi_ref;

			  if(control->control_mode_ref == closed_loop)
			  {
				  // Calculate PI controller state variable for smooth transition to closed loop.
				  control->i_ref_usr = i_meas; // To get smooth transient to closed loop. Transient thus occurs at zero error.
				  control->i_ref = control->i_ref_usr;
				  pi_controller->e[1] = 0.0f;
				  pi_controller->u[1] = phi_out/180.0f;
				  update_pi_constant(pi_controller, pi_controller->t_s);
				  control->control_mode_act = closed_loop;
			  }
			  break;
		  case closed_loop:
			 // update_pi_constant(pi_controller, Ts); // The PI controller constant have to be changed according to actual switching/calculation frequency.
			  if(control->update_pi_constant)
			  {
				  update_pi_constant(pi_controller, pi_controller->t_s);
				  control->update_pi_constant = false;
			  }
			  // control->i_ref = clamp_magnitude(control->i_ref_usr, MIN_ABS_I_REF, MAX_ABS_I_REF);
			  calc_pi_controller(pi_controller, control, i_meas);
			  phi_out = pi_controller->u[0]*180.0f; // According to the old version of the code. The PI controller return phy in relative unit (-1, 1) I guess.

			  if(control->control_mode_ref == open_loop)
			  {
				  control->phi_ref_usr = phi_out; // Just for smooth transient from closed loop to open loop control.
				  // control->phi_ref = control->phi_ref_usr;
				  control->control_mode_act = open_loop;
			  }
			 
			  break;

		  default:
			  control->control_mode_act = open_loop;
			  control->phi_ref = 60; // It has to be changed.
			  break;
		}

	return phi_out;
}

float get_value()
{
    return mode;
}

float rate_limitter(float input, float output, float rate_limit)
{
	float diff = input - output;
	output += saturate(diff, -rate_limit, rate_limit);
	return output;
}

void calc_pi_controller(volatile pi_controller_t *pi_controller, volatile control_t *control, float i_meas)
{
	pi_controller->e[0] = i_meas - control->i_ref;
	pi_controller->u[0] = pi_controller->u[1] + pi_controller->b0 * pi_controller->e[0] + pi_controller->b1 * pi_controller->e[1];
	pi_controller->e[1] = pi_controller->e[0];

	// Saturate the output of the PI controller. There are two saturation bands for positive and negative phase shift.
	// The area around zero phase shift is forbidden due to ZVS.
    if(control->i_ref <= 0.0)
    {
        pi_controller->u[0] = saturate(pi_controller->u[0], PI_U_MIN, PI_U_MAX);
    }
    else
    {
        pi_controller->u[0] = saturate(pi_controller->u[0], -PI_U_MIN, -PI_U_MAX);
    }
	// pi_controller->u[0] = clamp_magnitude(pi_controller->u[0], PI_U_MIN, PI_U_MAX);

	pi_controller->u[1] = pi_controller->u[0];
}

void  update_pi_constant(volatile pi_controller_t *pi_controller, float Ts)
{
	pi_controller->b0 = pi_controller->k_p + pi_controller->k_i * Ts * 0.5f;
	pi_controller->b1 = -pi_controller->k_p + pi_controller->k_i * Ts * 0.5f;
}

void  update_pi_state_variables(volatile pi_controller_t *pi_controller, float u, float e)
{
	pi_controller->u[1] = u;
	pi_controller->e[1] = e;
}

float saturate(float ref_value, float min_value, float max_value)
{
	ref_value = (ref_value > max_value) ? max_value : ref_value;
	ref_value = (ref_value < min_value) ? min_value : ref_value;

	return ref_value;
}

float clamp_magnitude(float ref_value, float min_abs_value, float max_abs_value)
{
	float abs_value = fabsf(ref_value);

	abs_value = (abs_value > max_abs_value) ? max_abs_value : abs_value;
	abs_value = (abs_value < min_abs_value) ? min_abs_value : abs_value;

	return (ref_value >= 0.0f) ? abs_value: -abs_value;
}

