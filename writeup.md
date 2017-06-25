# Writeup

## Implementation
My step-by-step implementation of this project is:
1. Determine the vehicle model and write down the update equation on a paper
2. Create the transformation matrix for mapping 2 CS 
3. Fit the polynomial to the given points from the simulator
4. Calculate the CTE and the orientation error (epsi)
5. Set the initial state to MPC and try with arbitrary values of `N` and `dt`
6. Calling the MPC::solve and it returns vectors of the predicted points, actuators delta, and acceleration
7. Assigning the actuator values to the simulator, and do observations
8. Include the 100 ms latency to the model and do observation
9. Tuning the cost function and parameter values for `N` and `dt`


As discussed in this forum [thread](https://discussions.udacity.com/t/here-is-some-advice-about-steering-values/276487), there is an inconsistency of the steering values between the input data to the simulator and the output data from the simulator:

> The simulator takes as input values in [-1, 1], where -1 is equivalent to 25 degrees to the left and 1 is equivalent to 25 degrees to the right.
The simulator returns values in [-0.46332, +0.46332] radians, where -0.46332 is 25 degrees to the left and 0.46332 is 25 degrees to the right. - pierluigi.ferrari.



## Vehicle Model
I choose the constant rate of turn & velocity model (CRTV) for the vehicle model. The below figure from the previous lesson visualizes the vehichle model:

![image1]

The state vector of the model is `x = (px, py, psi, v)`.

The concept of MPC is to compare a reference trajectory with the current state. For this reason, I extend the above state vector by including the cross-track-error and the orientation error: `x = (px, py, psi, v, cte, epsi)`

The state vector depends on the two values for the actuators, because the vehicle needs input values for the control. These actuators are `delta` (steering wheel) and `a` (throttle and brake). Thus, the update equation can be defined as:

    x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
    psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
    v_[t+1] = v[t] + a[t] * dt
    cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
    epsi[t+1] = psi[t] - psi_des[t] + v[t] * delta[t] / Lf * dt

where,
* `x` and `y` refers to the position of the vehicle,
* `psi` denotes the direction or orientation of the vehicle,
* `v` refers to the velocity of the vehicle,
* `cte` refers to the cross-track-error,
* `epsi` is the orientation error.
* `f(x)` is a 3rd degree polynom function, thus `f(x[t])-y[t]` refers to `cte[t]`.
* `psi_des` is the desired orientation that can be calculated as the [tangential angle](https://en.wikipedia.org/wiki/Tangential_angle) of the polynomial `f(x)` evaluated at `x​[t]`.

## MPC Preprocessing
### Coordinate System Transformation
Two coordinate systems are defined in the simulator environment: the global or map coordinate system and the vehicle coordinate system. These coordinate systems can be visualized as below figure:

![image2]

In order to get a point in the vehicle coordinate system we can use a transformation matrix `T`. I defined the transformation matrix as 3x3 matrix that contains the rotation and the translation elements. The matrix takes the vehicle car `psi` and the vehicle position. Given a point in the map coordinate `M`, the transformation result of this point in the vehicle coordinate system `V` can be calculated as:

    V = T.inverse().M

### Calculating the Cross Track Error (CTE) and orientation error
The CTE can be calculated by finding the shortest distance between the given waypoints to the position of the vehicle. However, this method is rather complicated. Thus, we convert the polynomial in the vehicle coordinate system and calculate the distance by assigning 0 to the function. The y value of the `f_poly(0)` represents the distance as the above illustration.


## MPC

### Cost Function, Timestep Length and Elapsed Duration

As given in the lesson, the cost function consists on 7 elements. Based on my experiment, the important elements that influence the stability are:

1. the steering value for the actuator (`delta`), 
2. the steering value gap of the sequential actuator,
3. and the orientation error. 

The bellow figure shows why I assign a high weight to the steering value:

![image3]

Thus, my cost function is defined as:

    cost = sum_N { (v[n]-v_ref[n])² + 
                   cte[n]² +
                   w_e * epsi[n]² +
                   w_d * delta[n]² +
                   a[n]² +
                   w_ddiff * (delta[n+1]-delta[n])² +
                   (a[n+1]-a[n])²}

where `w_e`, `w_d`, and `w_ddif` are the corresponding weights of each parameter.



The predicted horizon is defined by the time `T = N*dt`, where `N` is the timestep length and `dt` is the elapsed duration. `dt` describes how oft the prediction is calculated. Both parameters affects the computational performance. I found out that applying a high velocity (`ref_v=55`) requires a smaller dt in order to deal with a sharp curve on the road. The below figure shows the comparison using `N=10`, and `ref_v=55`.

![image4]

The above figure shows that we need to apply another weight for the delta in order to make the vehicle more responsive to deal with the sharp curve. I set the value of `w_d` to 100 for `dt=0.1` and `w_d = 500` for `dt=0.05`. Here is the comparison:

![image5]

Moreover, a higher value of `N` such as 12 or 14 increase the computational process and leads to a additional computational delay. In my experiment using `dt=0.05` and velocity 55 mph, the vehicle cannot pass the sharp curve. A very low value of `N`, for example 4 or 6, cannot provide a good prediction and control. I compared the parameter `N` with value 8 and 10, using the same condition of `dt=0.05`:

![image6]

Based on the figure, we see that the CTE of `N=10` is better than `N=8`. 

By applying a different value of `N` and `dt`, the weight of the cost function has to be adapted in order to get a stable result. This video clip shows a zig-zag trajectory by applying no weight to the delta element (`w_d=1`). 

<p align="center">
<a href="https://drive.google.com/open?id=0B2EMsm6nYzwWOTkyVE9CMFhTT0E
" target="_blank"><img src="./img/mpc_result_v50.gif" 
alt="MPC result with zig-zag trajectory and max speed 50 mph" border="5" /></a>
</p>

As final result I chose these values:

* `N=10`, 
* `dt=0.05`
* `w_e=100`
* `w_d=500`
* `w_ddiff=500` 

in order to deal with the maximal velocity of 55 mph. 


### Latency
Due to the 100 millisecond latency between actuations commands on top of the connection latency, we need to incorporate this latency to our model. One way is to include this latency in the state before the state is passed to the `MPC::solve()`. The updated state is defined in the function `include_latency(Eigen::VectorXd& s, const double delta, const double acc, double latency = 0.1)` as:

    s[0] = x + v * cos(psi) * latency;
    s[1] = y + v * sin(psi) * latency;
    s[2] = psi + v / Lf * delta * latency;
    s[3] = v + acc * latency;
    s[4] = cte + v*sin(epsi) *latency;
    s[5] = epsi + v / Lf * delta * latency;

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

## Result video

The result video can be seen in this below animation or click to see in the original size.

<p align="center">
<a href="https://drive.google.com/open?id=0B2EMsm6nYzwWemVmVmx2dWlvak0
" target="_blank"><img src="./img/mpc_result_N10dt005.gif" 
alt="MPC final result" border="5" /></a>
</p>

[//]: # (Image References)

[image1]: ./img/vehicle_model.png "Vehicle Model"
[image2]: ./img/cte_calc.png "Calculating CTE"
[image3]: ./img/exp_N10_dt005_w_d100.png "Oscillation"
[image4]: ./img/exp_compare_dt_same_cost.png "dt comparison"
[image5]: ./img/exp_diff_dt_diff_wd.png "Optimal cost, dt and N"
[image6]: ./img/exp_compareN.png "N comparison"