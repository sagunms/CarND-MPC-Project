# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

[//]: # (Image References)

[overview]: ./images/overview.gif "Overview"
[mpc-model-equations]: ./images/mpc-model-equations.png "Model"

Overview
---

This project implements a Model Predictive Controller to control the steering, throttle and brake to drive a car around the simulation track. 

In a real car, due to factors such as actuator dynamics, there exists a realistic delay on the order of 100 milliseconds for the actuation command to propagate through the system. This is a problem called "latency" and it is very difficult for a controller such as PID to overcome. This is because PID controller can calculate the error with respect to present status, but the actuation will be performed when the vehicle is in a future (and likely different) state. An MPC controller however, can adapt quite well because we can model this latency in the system and explicitly take it into account as part of the prediction. 

![alt text][overview]

The yellow is a polynomial fitted to way points and the green line represents the x and y coordinates of the MPC trajectory. The cross track error is calculated and additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency. 

Discussion
---

### The Model

I calculated steering angle and throttle using MPC model and the predicted trajectory was displayed by a green line with points in reference to the vehicular coordinate system. Similarly, a third order polynomial was solved to display the yellow way points/reference line. 

The MPC uses a simple kinematic model where the equations are as follows:

![alt text][mpc-model-equations]

* `x, y` - Vehicle coordinates
* `psi` - Vehicle rotation angle
* `v` - Velocity magnitude
* `delta` - Steering angle. Range [-25, 25] degrees
* `a` - Acceleration (positive) or Brake (negative). Range: [-1, 1]
* `Lf` - Distance between the front of the vehicle and its center of gravity

My cost function is as follows:
```
Cost = 4000*(cte)^2 + 5000*(epsi)^2 + 
        (v - velocity_desired)^2 + 
        5*(delta)^2 + 5*(a)^2 + 
        200*(delta - delta_prev)^2 + 10*(a - a_prev)^2
```

* `4000*(cte)^2` - Cross track error has a high weight on the cost function 
* `5000*(epsi)^2` - Error of vehicle orientation has a high weight on the cost function. It is difference between current psi and desired psi in the current point.
* `(v - v_desired)^2` - Drive with desired velocity and penalise stopping
* `5*delta^2` and `5*a^2` - Penalise use of big actuators values without reason
* `200 * (delta - delta_prev)^2` - Smooth steering commands between iterations
* `10 * (a - a_prev)^2` - Smooth acceleration/break commands between iterations

### Timestep Length (N) and Elapsed Duration (dt)

Initially, I ran the simulation with the values (N=25, dt=0.05) but the car went through extreme turns went out of the track. Also the downside of using large N and small dt was that it leads to excessive computation. After some experimentation and tweaks, (N=10, dt=0.1) seem to work quite well with my final cost function mentioned earlier. It also uses computation because of smaller N and larger dt.

### Polynomial Fitting and MPC Preprocessing

The objective is to drive with the highest speed possible at the center of the lane. This is represented by a number of points in global coordinate system and is discrete. In order to calculate `cte` and `epsi` in any desired point on the road, we need to fit a curve to fill the gaps between points. The lane points are transformed from global to vehicle coordinate system and then the curve representing the centre of the lane is fit with a third order polynomial (yellow line). 

### Model Predictive Control with Latency

We need to handle the latency which is the delay between moment we obtain actuators values and its application by the vehicle. The latency used in this project is 100 ms or 0.1 s which was added as part of the velocity calculation. In order to handle the overall latency, I changed the weights of the cost functions until the vehicle in the simulation drove quite well on the track. 

Simulation
---
As shown in the video demonstration, my implementation performs reasonably well. No tire leaves the drivable portion of the track surface and drives safe enough if a human were to be in the vehicle.

[![Simulation Video](https://img.youtube.com/vi/KzWeLVJ4gc0/0.jpg)](https://www.youtube.com/watch?v=KzWeLVJ4gc0)