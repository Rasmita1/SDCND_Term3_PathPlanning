# SDCND_Term3_PathPlanning
The goal of this project is to build a path planner that creates smooth, safe trajectories for the car to follow. The highway track has other vehicles, all going different speeds, but approximately obeying the 50 MPH speed limit.
The car transmits its location, along with its sensor fusion data, which estimates the location of all the vehicles on the same side of the road.

The car must not violate a set of motion constraints, namely maximum velocity, maximum acceleration, and maximum jerk, while also avoiding collisions with other vehicles, keeping to within a highway lane (aside from short periods of time while changing lanes), and changing lanes when doing so is necessary to maintain a speed near the posted speed limit.

# Implementation
1. A proportional velocity control
2. Occupied lane avoidance logic
3. Trajectory generation using a spline function

## 1. A proportional velocity control
The target velocity of the ego-vehicle is set depending on the state of the vehicle. If there is no vehicle ahead of the ego-vehicle, the target velocity is set to 49.5 mph, which is just below the speed limit. However, if the ego-vehicle is close to another vehicle ahead of it, it adjusts the target velocity to the vehicle ahead of it and keeps a safety distance between them. The acceleration of the ego-vehicle to achieved using a proportional controller, as in;

  vel_control.Init(0.005, 0.0, 0.0);
...
...
            // target velocity control
            if (ego_vehicle.too_close)
            {
              // keeping lane
              speed_limit = ego_vehicle.other_car_vel;
            } else {
              speed_limit = 49.5;
            }

            double vel_error = ref_vel - speed_limit;
            vel_control.UpdateError(vel_error);
            double new_vel = vel_control.TotalError();
            ref_vel += new_vel;
            
 
 ## 2. Occupied lane avoidance logic
 The ego-vehicle is able to drive safely by avoiding occupied lanes on the left and on the right. At every time step, the logic checks for; (i) other cars ahead of the ego-car, (ii) cars in adjacent lanes, and (iii) safety space ahead of and behind the ego-vehicle. Hence, the ego-vehicle only changes lane if there are no cars within defined safety ranges ahead of and behind the ego-vehicle. The logic is shown in the code snippet below:
// check which lanes are not free depending on the ego vehichle's lane
  if (lane==0)
  {
    left_is_free = false;
  }
  if (lane==2)
  {
    right_is_free = false;
  }
...
...
    // if car is too close
    if (too_close) 
    {
      if ((lane==0) && right_is_free)
      {
          lane = 1;
      }
      else if (lane==1)
      {
        if (left_is_free && right_is_free)
        {
          if (space_on_right > space_on_left)
          {
            lane += 1;
          } else {
            lane -= 1;
          }
        } else if (left_is_free) { 
          lane -= 1;
        } else if(right_is_free) {
          lane += 1;
        }
      }
      else if ((lane==2) && left_is_free)
      {
          lane = 1;
      }
    }
    
  
 
 ## 3. Trajectory generation using a spline function
 After a desired lane of the ego-vehicle has been set, We create (x, y) coordinate of waypoints ahead of the ego-vehicle using the corresponding spaced (s,d) Frenet coordinates. From these waypoints, we use a spline function to generate evenly spaced points for a smooth trajectory towards a desired horizon. The car is expected to visit each of these points every 0.02 secs, so the number of the points are calculated by N = double N = (target_dist/(0.02*ref_vel/2.24)) in order to travel at the desired reference velocity. In the implementation, 6 waypoints were created and a horizon of 30 m was chosen to generate the desired trajectory. More detail is shown in the code snippet below,
 
 // create a spline
            tk::spline s;

            //set (x,y) points to the spline
            s.set_points(ptsx, ptsy); // the anchor points/waypoints

...
...

            // fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
            for (int i = 1; i <= 60-previous_path_x.size(); i++)
            {
              double N = (target_dist/(0.02*ref_vel/2.24)); // 2.24: conversion to m/s
              double x_point = x_add_on+(target_x)/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // go back to global coordinate
              x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
              y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
            
      
# Basic Build Instructions
1. Clone this repo.
2. Make a build directory: mkdir build && cd build
3. Compile: cmake .. && make
4. Run it: ./path_planning

# Dependencies
* cmake >= 3.5
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
* [CppAD](https://www.coin-or.org/CppAD/)
  * Linux `sudo apt-get install cppad` or equivalent.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

