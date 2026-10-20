#ifndef DAB_CONTROL_H
#define DAB_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define max(a,b) ((a) >= (b) ? (a) : (b)) 
#define min(a,b) ((a) <= (b) ? (a) : (b))
#define abs(a) (((a) >= 0.0) ? (a) : (-1.0*(a))) 
#define sign(a) ((a) > 0 ? 1 : (((a) < 0) ? -1 : 0))
#define sat(a,min,max) ((a) >= (max) ? (max):((a) <= (min) ? (min) : a))

// Definition of constants
#define M_PI 3.14159265359
#define TWO_OVER_THREE 0.66666666667
#define ONE_OVER_THREE 0.33333333333
#define TWO_OVER_SQRT_THREE 1.15470053838
#define SQRT_THREE_OVER_TWO 0.86602540378
#define SQRT_THREE_OVER_THREE 0.5773502692
#define PI_OVER_SIX 0.5235987756
#define PI_OVER_THREE 1.0471975512
#define SQRT_THREE 1.73205080756887729353

// Define
#define PI_U_MIN 0.139f // 0.22f // Lower saturation limit for the output of PI controller in relative units.
#define PI_U_MAX 0.4f // 0.3f	// Upper saturation limit for the output of PI controller in relative units.
#define MIN_PHI_OPEN_LOOP 10.0f //25.0f	// Minimum phase shift for open loop control in ˚. It Has to be changed
#define MAX_PHI_OPEN_LOOP 80.0f	// Maximum phase shift for open loop control in ˚. It has to be changed
#define MIN_ABS_I_REF 0.0f //1.0f		// Minimum reference current
#define MAX_ABS_I_REF 15.0f		// Maximum reference current
#define I_LIM_30_KHZ 4.0f		// Maximum current value for switching frequency of 30 kHz
#define I_LIM_60_KHZ 2.0f		// Maximum current value for switching frequency of 60 kHz
#define I_LIM_120_KHZ 1.0f		// Maximum current value for switching frequency of 120 kHz
#define I_ZVS 1.4f				// Minimum current for ZVS, taken from the old code
#define PHI_ZVS	28.0f			// Minimum phase shift for ZVS, taken from the old code
#define TS_120 0.000008332f		// Period of switching frequency 120 kHz
#define TS_60 0.000016667f		// Period of switching frequency 60 kHz
#define TS_30 0.000033332f		// Period of switching frequency 30 kHz
#define CURRENT_RATE_LIMIT 0.000033f // Rate limit in A/Tcalc, where Tcalc is fixed to 1/30000

// Define new type
typedef struct{
    float d1;
    float d2;
    float phi;
}modulator_t;

typedef enum {
	closed_loop,	/**< The control is operating in open loop. The controlled variable is phase shif.t*/
	open_loop		/**< The control is operating in closed loop. The controlled variable is input current. */
}control_mode_t;

typedef struct
{
	float u[2];	// output, [0] -> k, [1] -> k-1
	float e[2];	// error, [0] -> k, [1] -> k-1
	float b0;	// constant given by k_p, k_i and t_s
	float b1;	// constant given by k_p, k_i and t_s
	float k_p;	// proportional constant of PI controller
	float k_i;	// integration constant of PI controller
	float t_s;	// sampling period
}pi_controller_t;

typedef struct
{
	control_mode_t control_mode_ref;
	control_mode_t control_mode_act;
	float i_ref;							// reference current for closed loop control
	float i_ref_usr;						// user reference current for closed loop control
	float phi_ref;							// reference phase shift for open loop control
	float phi_ref_usr;						// user reference phase shift for open loop control
	float deadtime_hb1_ns;					// Set deadtime of HB1 in ns
	float deadtime_hb2_ns;					// Set deadtime of HB2 in ns
	uint16_t fs_ref;						// Reference switching frequency from the user in the case of open_loop or from the controller in the case of closed_loop
	uint16_t fs_act;						// Actual switching frequency
	uint16_t bm_cmp;						// Set compare value of Burst Mode controller
	uint16_t bm_period;						// Set period of Burst Mode controller
	bool enable;
	bool update_pi_constant;
	bool update_deadtime;
	bool current_rate_limit_enable;			// enable or disable rate limiter
}control_t;

float calc_control_loop(volatile control_t *control, volatile pi_controller_t *pi_controller, float i_meas);
float rate_limitter(float input, float output, float rate_limit);
void calc_pi_controller(volatile pi_controller_t *pi_controller, volatile control_t *control, float i_meas);
float saturate(float ref_value, float min_value, float max_value);
float clamp_magnitude(float ref_value, float min_abs_value, float max_abs_value);
void update_pi_constant(volatile pi_controller_t *pi_controller, float Ts);
void update_pi_state_variables(volatile pi_controller_t *pi_controller, float u, float e);
float get_value();

#endif
