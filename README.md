# CarND: Model Predictive Control

[//]: # (Image References)

[overview]: ./images/overview.gif "Overview"
[mpc-model-equations]: ./images/mpc-model-equations.png "Model"

Overview
---

This project implements a Model Predictive Controller to control the steering, throttle and brake to drive a car around a simulation track. 

In a real car, due to factors such as actuator dynamics, there exists a realistic delay on the order of 100 milliseconds for the actuation command to propagate through the system. This is a problem called "latency" and it is very difficult for a controller such as PID to overcome. This is because PID controller can calculate the error with respect to present status, but the actuation will be performed when the vehicle is in a future (and likely different) state. An MPC controller however, can adapt quite well because we can model this latency in the system and explicitly take it into account as part of the prediction. 

![alt text][overview]

The yellow is a polynomial fitted to way points and the green line represents the x and y coordinates of the MPC trajectory. The cross track error is calculated and additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency. 

Discussion
---

### Model

Steering angle and throttle was calculated using MPC model and the predicted trajectory was displayed by a green line with points in reference to the vehicular coordinate system. Similarly, a third order polynomial was solved to display the yellow way points/reference line. 

The MPC uses a simple kinematic model where the equations are as follows:

![alt text][mpc-model-equations]

Simulator state variables:

* `x, y` - Vehicle x, y coordinates
* `psi` - Vehicle rotation angle
* `v` - Velocity magnitude
* `delta` - Steering angle. Range [-25, 25] degrees
* `a` - Acceleration (positive) or Brake (negative). Range: [-1, 1]
* `Lf` - Distance between the front of the vehicle and its center of gravity

Actuator variables:

* `cte` - Cross track error
* `epsi` - Vehicle rotation angle error

My cost function is as follows:
```
Cost = (cte)^2 + (epsi)^2 + 
        (v - velocity_desired)^2 + 
        1*(delta)^2 + 10*(a)^2 + 
        700*(delta - delta_prev)^2 + 20*(a - a_prev)^2
```

* `(cte)^2` - Cross track error impacts cost function 
* `(epsi)^2` - Error of vehicle orientation impacts cost function. It is difference between current psi and desired psi in the current point.
* `(v - v_desired)^2` - Drive with desired velocity and penalise stopping
* `1 * (delta)^2` and `10 * (a)^2` - Penalise use of actuators values
* `700 * (delta - delta_prev)^2` - Smooth steering commands between iterations
* `20 * (a - a_prev)^2` - Smooth acceleration/break commands between iterations

### Timestep Length (N) and Elapsed Duration (dt)

The model works by creating a reference trajectory for T seconds ahead on the horizon of current vehicle position (T = N * dt). Larger T means longer horizon and smoother changes over time. However, it can also be computationally expensive and also can cause large deviations in tracking as  horizon can change a lot. On the contrary, smaller T means shorter horizons and less smooth changes over time. But is faster computationally with more accuracy. 

Initially, I ran the simulation with the values (N=25, dt=0.05) but the car went through extreme turns went out of the track.  After some experimentation and tweaks, (N=10, dt=0.1) seem to work quite well with my final cost function mentioned earlier. It also uses computation because of smaller N and larger dt.

### Polynomial Fitting and MPC Preprocessing

The objective is to drive with the highest speed possible at the center of the lane. This is represented by a number of points in global coordinate system and is discrete. In order to calculate `cte` and `epsi` in any desired point on the road, we need to fit a curve to fill the gaps between points. The lane points are transformed from global to vehicle coordinate system and then the curve representing the centre of the lane is fit with a third order polynomial (yellow line). 

### Model Predictive Control with Latency

We need to handle the latency which is the delay between moment we obtain actuators values and its application by the vehicle. The simulator has a latency of 100 ms so I added the constraints for the overall duration of latency. To make current actuations smooth and best trajectory is calculated from the time after the latency, the previous actuator states were stored and applied during the latency period. Therefore, I have implemented the latency compensation by predicting the vehicle state 100 ms into the future before passing it to the solver. 

Video Demonstration
---

As you can see in the video, my implementation performs reasonably well. No tire leaves the drivable portion of the track surface and drives safe enough if a human were to be in the vehicle.

[![Simulation Video](https://img.youtube.com/vi/idvprYhj9K4/0.jpg)](https://www.youtube.com/watch?v=idvprYhj9K4)

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.