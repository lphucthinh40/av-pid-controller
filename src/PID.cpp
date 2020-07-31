#include "PID.h"
#include <cstdio>
#include <cstdlib>
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, bool enable_twiddle_, int n_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
	p_error = 0;
	i_error = 0;
	d_error = 0;
	
	K[0] = Kp_;
	K[1] = Ki_;
	K[2] = Kd_;

	enable_twiddle = enable_twiddle_;
	reset_simulator = false;

	if (enable_twiddle)
	{	
		// dK offset is set to be 10% of the corresponding K
		dK[0] = 0.1*Kp_;
		dK[1] = 0.1*Ki_;
		dK[2] = 0.1*Kd_;

		n = n_;
		n_warmup = 0;
		counter = 0;
		rotary_index = 0;
		sum_error = 0;
		average_error = 0;
		state = BEST_ERROR_INIT;		
	}	

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
	double prev_cte = p_error;
	
	p_error = cte;
	i_error += cte;
	d_error = cte - prev_cte; 
    
    if (enable_twiddle)
    	UpdateTwiddle(cte);
}

void PID::UpdateTwiddle(double cte)
{
	switch(state)
	{
		case BEST_ERROR_INIT:	{	// also skip the first two cycles like other states
									if (counter<2)	counter++; 
									else if (counter<2+n)
									{
										printf("state: init_twiddle | count: %d\n", counter-2);
										sum_error += std::abs(cte);
										counter++;
									}
									else
									{
										best_error = sum_error / n;
										state = TRY_ADDITION;
										counter = 0; 
										sum_error = 0;
									}
								}
								break;
		case TRY_ADDITION:		{	
									if (counter == 0) // one cycle for reseting simulator
									{
										reset_simulator = true;
										counter++;
									}
									else if (counter == 1) // one cycle for updating K
									{	
										K[rotary_index] += dK[rotary_index];
										counter++;
									}
									else if (counter <= n + 1) // n cycles for collecting errors
									{
										printf("state: addition (K_id:%d) | count: %d\n", rotary_index, counter-1);
										sum_error += std::abs(cte);
										counter++;
									}
									else
									{
										average_error = sum_error / n;
										printf("new_error: %f | best_error: %f\n", average_error, best_error);
										if (average_error < best_error)
										{	better_found = true;
											best_error = average_error;
											state = OFFICIAL_UPDATE;
										}
										else
										{	better_found = false;
											state = TRY_SUBTRACTION;
										}
										counter = 0;
										sum_error = 0;
									}

								}
								break;
		case TRY_SUBTRACTION:	{	
									if (counter == 0) // one cycle for reseting simulator
									{
										reset_simulator = true;
										counter++;
									}
									if (counter == 1) // one cycle for updating K
									{	
										K[rotary_index] -= 2*dK[rotary_index];
										counter++;
									}
									else if (counter <= n + 1) // n cycles for collecting errors
									{
										printf("state: subtraction (K_id:%d) | count: %d\n", rotary_index, counter-1);
										sum_error += std::abs(cte);
										counter++;
									}
									else
									{	
										average_error = sum_error / n;
										printf("new_error: %f | best_error: %f\n", average_error, best_error);
										if (average_error < best_error)
										{	better_found = true;
											best_error = average_error;
										}
										else
											better_found = false;
										state = OFFICIAL_UPDATE;
										counter = 0;	
										sum_error = 0;																
									}
								}
								break;
		case OFFICIAL_UPDATE:	{
									if (better_found)
									{
										dK[rotary_index] *= 1.1;
										better_found = false;
										printf("state: update positive | K_id: %d, K: %f, dK: %f\n", rotary_index, K[rotary_index], dK[rotary_index]);
									}
									else
									{
										K[rotary_index] += dK[rotary_index];
										dK[rotary_index] *= 0.9;
										printf("state: update negative | K_id: %d, K: %f, dK: %f\n", rotary_index, K[rotary_index], dK[rotary_index]);
									}
									printf("Kp: %f, Ki: %f, Kd: %f\n\n", K[0], K[1], K[2]);
									// rotary_index = (rotary_index+1) % 3;
									rotary_index = (rotary_index==0)? 2:0; // limit to only P & D
									state = TRY_ADDITION;
								}
								break;
	}
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return K[0] * p_error + K[1] * i_error + K[2] * d_error;  // TODO: Add your total error calc here!
}