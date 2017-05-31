# Student describes their model in detail. This includes the state, actuators and update equations.

First of all I found the correct parameters for kinematic car model, which means that all units and numbers more or less fits into the behavior of the car in the simulator. Or in other words, given previous position of the car (x, y, psi), elapsed delta T of measurements and actuators applied on the previous time step (steering, throttle) I need to predict the current car position and speed. (see commented lines main.cpp:210-260)

The resulting kinematic model looks like:

```apple js
      // Estimate current state
      double cpx = prev_state[0] + 1.60934 * 1000.0 * prev_state[3] * cos(prev_state[2]) * dt / 3600.0;
      double cpy = prev_state[1] + 1.60934 * 1000.0 * prev_state[3] * sin(prev_state[2]) * dt / 3600.0;
      double cpsi = prev_state[2] - 1000.0 * prev_state[3] * prev_steer_value / (Lf * 1.60934) * dt / 3600.0;
      double cv = prev_state[3] + 28000.0 * prev_throttle * dt / 3600.0;
```

where `prev_state` is {x, y, psi, v}, `prev_steer_value` - steering angle actuator [-1.0, 1.0], `prev_throttle` - throttle actuator [-1.0, 1.0]  

Couple things to note here:
1) Angle is rotating clockwise instead of counter clockwise as conventional.
2) Speed in mp/h and x, y in meters - so here is conversions
3) Throttle to acceleration converts with the help of constant `28000.0` that I found out experimentally for the current simulator (don't know why is this magic number has such value but without it the model is not correct) 

For linear optimization problem we unwrap the kinematic model for N steps in the future together with cross-track error and orientation error and corresponding actuators.
 
I've assumed that due to the latency we are unable to change our throttle and steer in the first time step so I've fixed them to constant together with the initial car location, orientation and speed.

# Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

## dt selection

My MacBook Pro 13 is not a fast computer for the Unity emulator so the time (dt) between sequential processing of telemetry is floating dramatically (together with introduced 100ms latency it can be somewhere from 0.11s to 0.25s). So I maintain 2 seconds history window and calculate average time between subsequent call to the `onMessage` method. I'm also using this average dt as an input to MPC solve.
   
## N - selection

I've started with an arbirtary N = 20 which was OK for low speeds. But for the high values of speed (>70mph) the predicting horizon reached more than 200 meters and became unstable especially when dt jumped to higher numbers. So I've dynamically calculate the number of steps that we need to reach 100m distance but no more than 20 steps given current speed(v) and dt.
  
```
  // We should be able to plan not father than 100m
  // so we are going to estimate the N step given the average dt
  double nd = 100.0 / (1.60934 * 1000.0 * v * (dt / 3600.0));
  size_t n = nd > 20 ? 20 : size_t(nd);
```

This approach added stability for the model on 80mp/h speeds. (though the model is not still safe for driving on such speeds:)

# If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

I'm translating global coordinate system into car coordinate system and then further calculations and optimizations is done in car coordinate system.
 
For translation I'm using the inverse of rotation and translation matrix which is calculating as following:

```
  // Find coeffs of reference line

  // Convert ptsx and ptsy to car coordinates
  // Transform matrix
  // [cos(psi), -sin(psi), px ]
  // [sin(psi),  cos(psi), py ]
  // [       0,         0,  1 ]

  // Inverse of transformation matrix
  // [ cos(psi), sin(psi), - (  px * cos(psi) + py * sin(psi)) ]    [ ptsx[0] ]
  // [-sin(psi), cos(psi), - (- px * sin(psi) + py * cos(psi)) ]  * [ ptsy[0] ]
  // [        0,        0,                                   1 ]    [       1 ]
  int pts_size = ptsx.size();
  for (int i = 0; i < pts_size; ++i) {
    double ptsxi = ptsx[i];
    double ptsyi = ptsy[i];
    ptsx[i] = ptsxi * cos(psi) + ptsyi * sin(psi)
              - (px * cos(psi) + py * sin(psi));
    ptsy[i] = - 1.0 *  ptsxi * sin(psi) + ptsyi * cos(psi)
              + px * sin(psi) - py * cos(psi);
  }

```

# Student provides details on how they deal with latency.

To deal with latency I'm fixing steering and throttle for the first time step and then using the second actuator values to the command. 

# Driving result available on YouTube

[![Driving Result MPC, throttle 0.65](https://img.youtube.com/vi/4ZvbeG0sU2E/0.jpg)](https://youtu.be/4ZvbeG0sU2E)


