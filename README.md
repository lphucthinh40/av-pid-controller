[//]: # (Image References)
[image_0]: ./pid.png "PID Diagram"
[image_1]: ./twiddle_sm.png "Twiddle State Machine"
[image_2]: ./twiddle_in_action.png "Twiddle Test"
[image_3]: ./p_only_controller.gif "P Only Controller"
[image_4]: ./pid_controller.gif "PID Controller"
[image_5]: ./demo.gif "PID Demo"

# Steering PID Controler Project
As a part of Udacity's Self-Driving Car Engineer Nanodegree Program

---

#### Objective
The goal of this project is to drive a vehicle autonomously on Udacity's Term 2 simulator, using only a PID-controller to control the steering wheel angle of the vehicle.

#### Build Instructions
1. To enable Twiddle, use `pid.Init(0.210815, 0.001000, 1.570833, true, 1500);` instead in main.cpp (enable_twiddle=true, number_of_sample=1500)
2. Create a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run: `./pid`. 

## PID Controller Overview

PID Controller is a control loop feedback mechanism to control process variables in a system. It uses closed-loop control feedbacks to keep the output of a process as close to the target output as possible. PID Controller is widely used in many industrial control systems and applications to regulate temperature, pressure, speed, and other process variables.

In this project, we are using PID Controller to control the steering angle of the vehicle. The input to PID controller is Cross Track Error (CTE), which is the displacement between the actual position of the vehicle and its projected position on the central longitudinal line of the road. As the vehicle runs, CTE is repeatedly calculated by the simulator and sent to the PID-controller as a direct input. As the result, the whole purpose of our PID controller in this project is to minimize cross track error as much as possible. The output of PID controller is called Total Error, which is calculated as follow: <br>
<p align="center">
E = Kp · e + Kd · de ∕ dt + Ki · ∫e(t)dt<br>
Steering = - E <br>
</p>

![alt text][image_0] <br>
<i>PID Control Feedback Loop (source: wikipedia)</i>

#### P - Proportional Component

The proportional component depends only on the error at the present. It is calculated simply by multiplying the error term with a proportional gain (Kp). P-component is often the driving force of the controller in minimizing errors. It changes the controller output proportionally to the error. In other words, the larger the error, the stronger control action P-Component will yield to correct such error. However, if the proportional gain is set too high, the controller loop will begin oscillating due to overshooting, making the system unstable.

#### D - Derivative Component

The derivative component depends on the rate of change of the error. In other words, while p-component is based on the error at the present, d-component is based on its future trend. With d-component, the system will produce more control action when the error changes at a faster rate. As a result, adding d-component to a p-only controller can help reduce oscillation and make the system more stable (settling at a certain point).

![alt text][image_3]  ![alt text][image_4] <br>
<i>Adding D-Component helps reduce overshooting (left: without D, right: with D)</i>

#### I - Integral Component

The integral component is propotional to the sums of the error term over time. As a result, the integral component will increase slowly over time even when the system has very small error term. The i-component ensures that certain control action will be exerted continually unless the error is zero. It can be very effective in driving the Steady-State error to zero (Steady-State error is the difference between the stable process variable and set point, due to system bias).

## Twiddle - Fine-Tuning PID Parameters

After manually selecting Kp, Ki, Kd parameters so that the vehicle can stay on track, I used Twiddle method to fine-tune these parameters. The idea of twiddling is very straight forward: applying subtle change to a parameter; if a smaller error is achieved, such change will be encouraged even further; otherwise, such change will be discourage and become more conservative. My implementation is largely based on Udacity's lecture on Twiddle for Parameter Optimization (https://youtu.be/2uQ2BSzDvXs). When Twiddle mode is enabled, the program will first initialize Best Error term by calculating the Mean Error over N time steps. Whenever new error is calculated, the vehicle will be reset to its starting position before trying a new set of parameters and calculating the corresponding error. To simplify Twiddling process and to make my code more visible, I implemented a state machine as follow:

![alt text][image_1]

- BEST_ERROR_INIT: calculating best error after collecting enough sample of CTE.
- TRY_ADDITION: Reseting vehicle to starting position, adding an offset to the K parameter and calculating new error after collecting enough sample of CTE.
- TRY_SUBTRACTION: Reseting vehicle to starting position, subtracting K parameter by an offset and calculating new error after collecting enought sample of CTE.
- OFFICIAL_UPDATE: If new error is smaller than the current best error, such change to the previous K parameter will become official, current offset will also increase by 10%. Otherwise, such change will be rejected. The previous value of K will be kept and the current offset will decrease by 10%.

![alt text][image_2]

<p align="center">
<i>Twiddle in action</i>
</p>

## Demo

![alt text][image_5]


