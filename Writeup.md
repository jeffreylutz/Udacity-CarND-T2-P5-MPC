# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
[![Alt Text](https://i.ytimg.com/vi/OHsSgVQxtc4/hqdefault.jpg)](https://www.youtube.com/watch?v=OHsSgVQxtc4&feature=youtu.be)

## Model definition

#### Actuators:
   * delta: steering angle
   * acceleration:  throttle and brake

#### Error Types:
   * cross track error
   * psi error

#### State:
   * x position: x coordinate
   * y position: y coordinate
   * psi angle: vehicle orientation
   * velocity:  vehicle velocity

## Timestep Length and Elapsed Duration

The delay of 100 ms causes issues with oscillations in the system.  So, taking
the delay into account by adjusting dt and N elimnates the unstability of the
system.  After trial and error I noted the following behavior with N and dt
as speed increased.
   * When the speed is low (30 - 40 mph), the following was adequate
      N = 15
      dt = 0.10
   * When the speed increased (65-82 mph), the following was required
      N = 8
      dt = 0.13

It's interesting to observe the behavior with increasing dt time and decreasing N
as speed increaed.  It's as if too many samples cause the function solution to over-fit
a solution.  And too small a dt at increased speed doesn't allow the function solution
to project out far enough for the speed.

## Polynomial Fitting and MPC Preprocessing

A polynomial order of three (3) works best for fitting x and y.  I attempted to use
values of 2 and 4 with dramatic stability issues.

## Model Predictive Control with Latency

To handle the latency the x and y position get scaled by the latency and using the
cos and sin of the last used steering angle to correct the position.
