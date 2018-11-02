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
            
      
    
