# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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

## Method description

My approach to solve the issue in this chapter can be devided into two parts. First is to estimate in which state the car should be and the second is to plot smooth trajectory based on the estimated earlier goal velocity.

### Choosing states

I've defined three different states:

* Drive straight - the line is not changes, the max velocity is always 49 mph. Acceleration is 0.224. Possible state to which it can change: Drive straight, Turn, Prepare to turn
* Turn - When the state changes in to 'Turn' state, lane is changes to left or right lane, whichever is free and feasible. Acceleration is 0.124  Possible state to which it can change: Turn, Drive straight
* Prepare to turn - Slow down to speed of the car in front of you. It accelerates or decelerates with rate 0.224  Possible state to which it can change: Turn, Prepare to turn, Drive straight (only if there is no car in front)

The transition between the states is straight forward. If it's possible the preferred state is to drive straight whenever possible. If it's not possible next try is always to turn. If turning is not possible it changes to 'Prepare to turn' state.

Based on the state properties and current car velocity the goal car velocity is calculated. 

### Calculate trajectory

The trajectory is calculated based on the current trajectory, goal velocity and goal line.

To ensure smooth transitions it uses previously generated trajectory and build upon it. The controller is always sending next 50 waypoints to the controller. 

Steps to generate the trajectory:

1. Get 5 points that defines the road. First two are taken either from previous path or from current car position, next three are the closest waypoints to points that are 30, 60 and 90 m away from the car on the goal line.
2. Convert points to local car coordinates.
3. Calculate the spline base on these points.
4. Calculate the step for getting the next trajectory points. To do it we have to calculate the distance that has to be 
driven to get to the point that is 30m away in x direction (remember we are in car coordinates). The distance might be 
bigger than 30 m, because the car might also have to travel in y direction. To get the y travel distance the splice that
was calculated earlier can be used. The step will be the difference divided by 0.02 (time step) and car velocity in m/s
5. Calculate the next x position based on the step increase calculated in previous step. Next y position is calculated based on the spline.
6. Convert next x and y points back to map coordinates and store in the trajectory. The number of generated points depends on the steps that are left from previous trajectory. There should be in total 50 points (sum of not executed points from previous trajectory and new points)


## Possible improvements

The presented approach is quite basic. It's safe and feasible but not flexible and quite conservative. 
There are multiple improvements to this approach e.q.:
* Add check for safety of the maneuver during whole line change. Right now during the change line maneuver the speed of cars on the new lane is no longer monitored. It is not a problem in the simulation, since the car are not breaking the rules and the distances used to calculates if the maneuver is safe are big enough, but in real life the car in front might rapidly stop or the car behind might be driving 300 km/s.
* Introduce real state machine. Method currently used in the project is extremely simplified version of the state machine which says that it's always best to drive straight if the lane is not occupied. This might fail e.g. when we have a goal line where the car should be. 
* Try to always go back to right lane, to comply with the European drive rules. 
* Calculate multiple trajectories with slightly different acceleration when checking for feasibility. Right now the car might get stuck behind two cars that are driving with similar speed, because the change line maneuver is always calculated with big safety values and very smooth transition. 
* Refactor code. Move classes to different files, create functions from nested loops. 
