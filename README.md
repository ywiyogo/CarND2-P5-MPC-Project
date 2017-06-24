# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Project Description
In this project describes how to use Model Predictive Control to drive the car around the track in the Udacity simulator. This time however the cross track error (CTE) is unknown, thus we have to calculate that ourself! Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.

The provided data from the simulator are:

* The waypoints of the road lane
* The global position of the vehicle
* The orientation of the vehicle
* The speed of the vehicle in mph
* The steering angle 
* The throttle

## Implementation
The step-by-step implementation of this project is:
1. Create the transformation matrix for mapping 2 CS 
2. Fit the polynomial to the given points from the simulator
3. Calculate CTE and orientation error (epsi)
4. Determine the vehicle model for MPC and set the initial state
5. Findout the best cost function and parameter values for `N` and `dt`
6. Include the 100 ms latency to the model
7. Call the MPC Solve return vectors of the predicted points, actuators delta, and acceleration
8. Assigning the actuator values to the simulator

As discussed in this forum [thread](https://discussions.udacity.com/t/here-is-some-advice-about-steering-values/276487), there is an inconsistency of the steering values between the input data to the simulator and the output data from the simulator:

> The simulator takes as input values in [-1, 1], where -1 is equivalent to 25 degrees to the left and 1 is equivalent to 25 degrees to the right.
The simulator returns values in [-0.46332, +0.46332] radians, where -0.46332 is 25 degrees to the left and 0.46332 is 25 degrees to the right. - pierluigi.ferrari.

The result video can be seen in this below animation or click to see in the original size.

<p align="center">
<a href="https://drive.google.com/open?id=0B2EMsm6nYzwWOTkyVE9CMFhTT0E
" target="_blank"><img src="./img/mpc_result_v50.gif" 
alt="MPC result with max speed 50 mph" border="4" /></a>
</p>

## Coordinate System Transformation
Two coordinate systems are defined in the simulator environment: the global or map coordinate system and the vehicle coordinate system. These coordinate systems can be visualized as below figure:

![image1]

In order to get a point in the vehicle coordinate system we can use a transformation matrix `T`. I defined the transformation matrix as 3x3 matrix that contains the rotation and the translation elements. The matrix takes the vehicle car `psi` and the vehicle position. Given a point in the map coordinate `M`, the transformation result of this point in the vehicle coordinate system `V` can be calculated as:

    V = T.inverse().M

## Calculating the Cross Track Error (CTE) and orientation error
The CTE can be calculated by finding the shortest distance between the given waypoints to the position of the vehicle. However, this method is rather complicated. Thus, we convert the polynomial in the vehicle coordinate system and calculate the distance by assigning 0 to the function. The y value of the `f_poly(0)` represents the distance as the above illustration.


## MPC

### Vehicle Model
The MPC uses the Constant Rate of Turn & Velocity model (CRTV) for the vehicle model. The state of this model is defined as:

    x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
    psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
    v_[t+1] = v[t] + a[t] * dt
    cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
    epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

where,
* x and y refers to the position of the vehicle,
* psi denotes the direction or orientation of the vehicle,
* v refers to the velocity of the vehicle,
* cte refers to the cross-track-error,
* epsi is the orientation error.

### Cost Function

The cost function is depends to the length `N`. It is defined as a sum of many parameters:

    cost = sum_N { (v[n]-v_ref[n])² + 
                   cte[n]² +
                   epsi[n]² +
                   delta[n]² +
                   w_a * a[n]² +
                   w_d * (delta[n+1]-delta[n])² +
                   (a[n+1]-a[n])²}

I include two weights (`w_a` and `w_d`) for the acceleration and the heading direction to emphasize these parameters. The `w2` helps me to deal with the sharp curve.

### Prediction Horizon, Timestep Length and Elapsed Duration
The predicted horizon is defined by the time `T = N*dt`, where `N` is the timestep length and `dt` is the elapsed duration. A short horizon leads the prediction to be less accurate or even instability. A long time of prediction provides a better prediction, but it reduces the performance. I apply `N=10` and `dt=0.1` for my MPC in order to deal with the maximal velocity of 50 mph. A higher value of `N` such as 12 or 14 increase the computational process and leads to a delay which results an bad trajectory on curve road. A lower value of `N`, such as 7 leads to a oscillation of the vehicle. `N=5` leads MPC to predict the wrong trajectory to the opposite direction of the waypoints.

### Latency
Due to the 100 millisecond latency between actuations commands on top of the connection latency, we need to incorporate this latency to our model. One way is to include this latency in the state before the state is passed to the `MPC::solve()`. The updated state is defined in the function `include_latency(Eigen::VectorXd& s, const double delta, const double acc, double latency = 0.1)` as:

    s[0] = x + v * cos(psi) * latency;
    s[1] = y + v * sin(psi) * latency;
    s[2] = psi + v / Lf * delta * latency;
    s[3] = v + acc * latency;

The above functions are based on the discussion thread in the [Udacity forum](https://discussions.udacity.com/t/how-to-incorporate-latency-into-the-model/257391). The `delta` variable is taken from the simulator and multiplied by -1 due to the inconsistency of the simulator, see [above](#implementation).

### Return Values
The MPC class return a structure called `MpcResult`, which contains the next predicted waypoints, the cross-track-error, the orientation error, steering, throttle, and cost values.

    struct MpcResult{
      vector<double> xpts;
      vector<double> ypts;
      vector<double> cte;
      vector<double> epsi;
      double delta;
      double a;
      double cost;
    };

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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## IDE

For this project, I use the codelite IDE. To import this project in codelite, please import the project file *CarND2-P5-MPC-Project.project*.


[//]: # (Image References)

[video1]: ./mpc_result_v50.mp4 "Result video with max speed 50 mph"
[image1]: ./img/cte_calc.png "Calculating CTE"
[image2]: ./img/thumbnail.png "Calculating CTE"