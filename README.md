# CarND-Path-Planning-Project

Self-Driving Car Engineer Nanodegree Program

## Goals

The goal for this project is to safely navigate the ego vehicle around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data will be provided. There is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all costs as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete one loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### The map of the highway is in data/highway_map.txt

Each waypoint in the list contains [x, y, s, dx, dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Visual Studio

The repo also includes Visual Studio solution and project files in the PathPlanning subdirectory.

## Simulator

You can download the Term3 Simulator which contains the Path Planning Project from the releases tab <https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2>.

To run the simulator on Mac/Linux, first make the binary file executable with the following command:

```shell
sudo chmod u+x {simulator_file_name}
```

### Data provided from the Simulator to the C++ Program:

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Details

1. The car uses a perfect controller and will visit every (x, y) point it recieves in the list every .02 seconds. The units for the (x, y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x, y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.)

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Solution

This project consists of essentially two separate components: path planning and trajectory generation. In the path planning step the ego vehicle position and speed are updated as necessary to accomplish our goals. In this case our goal is to safely navigate around the track with speed as close to the speed limit as possible while staying in our lane and avoiding collisions with other vehicles. Lane changes are allowed as long as these constraints are met (no collisions, no speeding). The trajectory generation step is where we take the updated ego vehicle position as determined by the path planner and generate points for the ego vehicle to follow to get from its current position to this new position.

I'll describe the various parts of the code in some detail below.

* ### _constants_h_

    Lots of constants related to track characteristics (length, number of lanes), speed limit, max acceleration, etc. There are also constants use for trajectory generation and prediction horizon calculations. Basically any value that ends up being hardcoded is added here to make it easier to find and change various parameter, e.g., the number of points to keep from a previous path.

* ### __main()__ in _main.cpp_

    1. Initialization

        * Read waypoints from _highway_map.txt_.
        * Create the ego vehicle object. For this I used the Vehicle class copied and adapted from the Behavior Planning lesson.

    1. Message handlers for interacting with the simulator.

        * __onConnection()__:

            This message handler is triggered each time the simulator begins a simulation run. I set initial values for the ego vehicle here so that I can start and stop the simulator without the need to stop and start my path planning application. Environment information is passed to the _ego_ __Vehicle__ object in this function, e.g., speed limit, number of lanes, max. allowable acceleration.

        * __onMessage()__:

            This message handler receives all updates from the simulator.  For each "telemetry" event the application receives information about the current car position, previous path information as well as sensor fusion data describing other vehicles on the road. There are essentially three steps involved in handling updated telemetry data:

            1. Parse the telemetry data into current vehicle position and previous path points. The current vehicle position is used to update our _ego_ object for each iteration. Also as part of this telemetry parsing process we translate sensor fusion data regarding the position and speed of other vehicles into a form usable by downstream functions. In addition predictions are made as to the future position (speed remaining constant) for other vehicles.

            1. The path planning step. This is described in more detail below. In summary this step uses the current vehicle position and speed as well as prediction data concerning other vehicles to determine a new position and speed for the ego vehicle. The new position and speed are implemented in the trajectory generation step.

            1. The trajectory generation step. In summary this step generates a list of (x, y) coordinates designed to take the ego vehicle from its current position and speed to the target position and speed as determined by the path planner. These updated (x, y) coordinates are passed back to the simulator. The code for this step may be found in the __PathPlannerTrajectory()__ function in _trajectory.cpp_ - see the description of this function below for more detail.

* ### __UpdateEgo()__ in _trajectory.cpp_

This function implements a very simple algorithm designed to attempt to accelerate the ego vehicle to the target speed, changing lanes as necessary and when possible, all while avoiding collisions. This implementation works but is not easily modifiable to implement additional or alternate goals. I discuss the path planning step in more detail below.

* ### __PathPlannerTrajectory()__ in _trajectory.cpp_

The trajectory-generation step is independent of the method used for the path planning step. One important note is that the _ego_ __Vehicle__ object used in this function is assumed to have been updated by the path planner. So that all references to current ego position or speed refer to values updated by the path planner.

This function implements the trajectory generation logic. The ultimate goal is to return a list of (x, y) coordinates to be passed back to the simulator as an updated path for the ego vehicle to follow.

Since any path points that were not used in a given iteration are returned in the next iteration as updated telemetry data, we first add a few of these previous path points to our new list. The intent of this is to provide a smooth transition from previous paths to the new path we are generating in this step.

The next step is to generate a spline that includes the current ego position as well as several waypoints along the road. To generate the spline we collect several points. The first two points are the last two points we used from the previous path points list. For the initial iteration when no previous path points are available we simply use the current ego position. The previous ego position is calculated by subtracting a bit from the current ego _s_ position, keeping _d_ constant so we're in the same lane. In either case these two points are used to generate an _angle_ that we'll use to translate spline points from world space to vehicle space and back again. As mentioned we also add a few waypoints to our spline point list by projecting the current ego vehicle _s_ position along the track a few steps while again keeping the _d_ position constant.

Once we have a list of points we'd like to use to generate a spline we must first translate from world space to vehicle space. To do this we use the _angle_ we calculated above and the delta between the point and our starting point. This will normalize our spline so that the initial position is (0, 0) with a neutral yaw angle. Now we can finally generate the spline.

The final step is to use the spline to generate (x, y) coordinates to pass back to the simulator.

1. Choose a target that is a bit in front of the current ego position and in the same lane to obtain the starting _x_ position. Use the spline to generate the starting _y_ position.

1. Determine the delta for the _x_ value by calculating the number of steps required to travel from the starting position to the target position while traveling at the current ego vehicle speed. Since we cannot set speed directly the speed is determined by the distance between subsequent points in the path that we pass back to the simulator.

1. Start at _x_ = 0 (because we've normalized our spline) we simply increment by the delta value until we reach the target. For each step we use the spline to generate the _y_ coordinate and then translate the resulting (_x_, _y_) point from vehicle space back to world space before adding to the list we'll pass back to the simulator.

* ### __Vehicle__ class in _vehicle.[h|cpp]_

The __Vehicle__ class code was copied and adapted from the Behavior Planning lesson. It stores vehicle position, speed, acceleration and state as well as a goal position. Position is stored as Frenet coordinates, (_s_, _d_), but instead of the actual _d_ value we use an integer lane number. The class also stores a few parameters related to the constraints imposed by the environment. Constraints consist of the number of available lanes, the target speed (i.e., the speed limit) and the maximum allowable acceleration.

This class implements path planning via use of cost functions. For each given vehicle state there are certain allowable successor states. For each of these potential successor states we generate a trajectory and then apply a cost calculation (see the next section for more detail on costs). The state with the lowest cost is chosen as the next state and the associated trajectory is applied to update the _ego_ __Vehicle__ object. A trajectory is represented as an initial __Vehicle__ object (which represents the current _ego_ state) and a new __Vehicle__ object which represents the results of path planning (i.e., the trajectory with the lowest cost).

* ### cost functions in _cost.[h|cpp]_

This code was also copied and adapted from the Behavior Planning lesson. The _`calculate_cost()`_ function is used to calcuate a cost for a given __Vehicle__ object with specified predictions and a trajectory generated by the path planner. The process is quite simple - apply each of a set of cost functions to the given data and calculate a weighted combination. The result is the cost for the given trajectory. The essence of the cost calculation step is in the various individual cost functions and the weights assigned to each.

The two cost functions from the Behavior Planning lesson were left more or less intact: _`goal_distance_cost()`_ and _`inefficiency_cost()`_. The _`goal_distance_cost()`_ function is designed to encourage the vehicle to stay in the goal lane as the it gets closer to the goal distance. In our case the goal lane was arbitrarily set to the rightmost lane and the goal distance was set to one lap around the track. The _`inefficiency_cost()`_ function is designed to encourage the vehicle to travel as fast as possible while remaining under the speed limit.

In addition to these two cost functions I added two more: _`goal_lane_cost()`_ and _`collision_cost()`_. The _`goal_lane_cost()`_ function is designed to encourage the vehicle to stay in the goal lane. In the end I abandoned this cost function - by setting the corresponding weight to 0.0 - due to issues resulting from changing lanes at low speeds. See discussion below under the Path Planning section.

The _`collision_cost()`_ function is designed to heavily penalize any proposed trajectory that may result in a collision. This would seem to be redudnant given that a trajectory that results in a collision is not generated by the path planner and will not even reach the cost calculation step. However, there are cases where the path planner did not "see" a potential collision for a lane change operation due to a large difference in speed between the ego vehicle and other traffic in the target lane.

### Path Planning

For the path planning step I initially implemented a very simple algorithm designed to attempt to accelerate the ego vehicle to the target speed, changing lanes as necessary and when possible, all while avoiding collisions. This algorithm is implementd in the __UpdateEgo()__ in _trajectory.cpp_. This function works but is not easily modifiable to implement additional or alternate goals.

As an alternate solution I used the path planning functions in the __Vehicle__ class that are based on a cost calculation. This method is much more amenable to modification by adding addition cost functions and/or adjusting cost function weights. In the end I could not get this method to work reliably so I used the __UpdateEgo()__ function.

I think part of the difficulty was an attempt to have the ego vehicle travel in the rightmost lane whenever possible. Since the simulator starts the vehicle in the middle lane this goal would result in an immediate lane change to the right. The difficulty is that there may be other vehicles in the right lane that are a significant distance behind the ego vehicle but are nevertheless prone to cause collisions due to the large difference in speed between the other vehicle and the ego vehicle at the start of the simulation. One way I attempted to deal with this issue was to adjust the prediction horizon for generating predictions for other vehicles derived from sensor fusion data. I determined empirically that a lane change takes about 3 seconds and used this number alone with the 20 ms update interval to generate enough prediction steps to reach 3 seconds into the future. This effort was not entirely successful and led to other issues possibly due to an excessibly large number of predictions for each vehicle. The prediction horizon was adjusted based on current ego vehicle speed but at slower speeds the prediction horizon might be in excess of 100 steps.

I believe another difficulty is that a lane change operation can take a relatively long time at slow speeds. The __Vehicle__ state transitions from Keep Lane to Prepare Lane Change Right to Lane Change Right and back to Keep Lane in just that many steps. So the ego vehicle goes through a lane change transition very quickly while the actual lane change in the simulator can take several seconds. This disconnect led to invalid path planning results (and a seriously unhappy simulator) because the __Vehicle__ class believe that the ego vehicle was in one lane while the simulator believed it was in a different lane. The resulting difference led to invalid cost function results due to a perceived collision in a lane other than the one the path planner was assuming.

A solution might be to disallow lane changes at lower speeds. Alternatively a method of keeping a vehicle in the Land Change [Right|Left] state util the lane change is actually accomplished. I believe this could be done but would require more tracking in the __Vehicle__ class and more interaction with current telemetry data (basically "are we there yet?").

## Further Implemnetation Details

### Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

### Dependencies

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

    ```shell
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
