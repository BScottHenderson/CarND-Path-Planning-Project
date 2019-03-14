
#include "trajectory.h"
#include <algorithm>
#include "constants.h"
#include "helpers.h"
#include "spline.h"
#include "JMT.h"

#define STAY_IN_CENTER_LANE 0


void StraightLineTrajectory(double car_x, double car_y, double car_yaw,
                            std::vector<double>& next_x_vals, std::vector<double>& next_y_vals) {

    double  angle = deg2rad(car_yaw);

    // Increment distance by 0.5m.
    // The car moves 50 times / second (every 20ms == 0.02s).
    //      0.5m * 50/s == 25m/s ~= 55.9 mph
    double  dist_inc = 0.5;
    for (int i = 0; i < 50; ++i) {
        // Move in a straight line in the direction the car is already facing.
        next_x_vals.push_back(car_x + (dist_inc * i) * cos(angle));
        next_y_vals.push_back(car_y + (dist_inc * i) * sin(angle));
    }
}


void CircleTrajectory(double car_x, double car_y, double car_yaw,
                      std::vector<double>& next_x_vals, std::vector<double>& next_y_vals) {

    double  angle = deg2rad(car_yaw);

    // Increment distance by 0.5m.
    // The car moves 50 times / second (every 20ms == 0.02s).
    //      0.5m * 50/s == 25m/s ~= 55.9 mph
    double  dist_inc = 0.5; // ~56 mph
    for (int i = 0; i < 50; ++i) {    
        car_x += (dist_inc) * cos(angle + (i+1) * (pi()/100));
        car_y += (dist_inc) * sin(angle + (i+1) * (pi()/100));
        next_x_vals.push_back(car_x);
        next_y_vals.push_back(car_y);
    }
}


void UpdateEgo(
    Vehicle& ego,
    std::vector<Vehicle>& traffic,
    std::vector<double> previous_path_x, std::vector<double> previous_path_y) {
    // Update (lane, v) for the ego vehicle.

    int     prev_path_size = std::min(PREVIOUS_POINTS_TO_KEEP, (int) previous_path_x.size());

    // Check other cars in traffic: Are there other cars ahead or to either side?
    bool    car_ahead = false;
    bool    car_left  = false;
    bool    car_right = false;
    for (auto car : traffic) {
        // Adjust the other vehicle 's' position by adding 'prev_path_size' steps.
        // We know that each step is 20ms (0.02s) and that the other vehicle
        // is moving at velocity 'car.v'.
        double  t = prev_path_size * 0.02;
        double  check_car_s = car.s + car.v * t;

        // Identify whether other vehicle is ahead, to the left, or to the right.
        switch (car.lane - ego.lane) {
        case 0:     // Car in our lane.
            // If the car is ahead of us and within the buffer distance, set the flag.
            if (ego.s < check_car_s &&
                        check_car_s < (ego.s + PREFERRED_BUFFER_LANE_CHANGE))
                car_ahead = true;
            break;
        case -1:    // Car to the left.
            // If car is within the buffer distance of our location, set the flag.
            if ((ego.s - PREFERRED_BUFFER_LANE_CHANGE) < check_car_s &&
                                                         check_car_s < (ego.s + PREFERRED_BUFFER_LANE_CHANGE))
                car_left = true;
            break;
        case 1:     // Car to the right.
            // If car is within the buffer distance of our location, set the flag.
            if ((ego.s - PREFERRED_BUFFER_LANE_CHANGE) < check_car_s &&
                                                         check_car_s < (ego.s + PREFERRED_BUFFER_LANE_CHANGE))
                car_right = true;
            break;
        }
    }

    // Adjust speed to avoid collisions. Change lanes if it is safe to do so.
    if (car_ahead) {
        // A car is ahead - change lanes if we can, else slow down.
        if (!car_left && ego.lane > 0)
            // No car to the left and we're not already in the left lane.
            ego.lane--;
        else if (!car_right && ego.lane < MAX_LANE)
            // No car to the right and we're not already in the right lane.
            ego.lane++;
        else
            // We're boxed in so just slow down.
            ego.v -= MAX_ACCELERATION;
    } else {
#if STAY_IN_CENTER_LANE
        // Stay in the center lane (lane 1) if possible.
        if (ego.lane == 0 && !car_right)
            ego.lane++;
        else if (ego.lane > 1 && !car_left)
            ego.lane--;
#else
        // Prefer the rightmost (MAX_LANE) lane in the absence of other goals.
        // Note: Make sure the vehicle is moving (velocity > 0) before changing
        // lanes. This will avoid excessive acceleration/jerk at initialization
        // since the vehicle does not start in the rightmost lane.
        if (ego.v > 0.0 && ego.lane < MAX_LANE && !car_right)
            ego.lane++;
#endif

        // If there is no car ahead and we're still below the speed limit, speed up!
        if (ego.v < SPEED_LIMIT)
            ego.v += MAX_ACCELERATION;
    }
}

void PathPlannerTrajectory(
    Vehicle& ego,
    std::vector<double>& map_waypoints_x, std::vector<double>& map_waypoints_y, std::vector<double>& map_waypoints_s,
    std::vector<double>  previous_path_x, std::vector<double>  previous_path_y,
    std::vector<double>& next_x_vals,     std::vector<double>& next_y_vals) {
    // Generate a smooth path for the ego vehicle.

    int     prev_path_size = std::min(PREVIOUS_POINTS_TO_KEEP, (int) previous_path_x.size());

    // Before adding new path points, add a few path points from the previous
    // cycle to maintain continuity.
    for (int i = 0; i < prev_path_size; ++i) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }


    // Calculate the duration for each path (this should always be 1.0s).
    double  t = PATH_LENGTH * UPDATE_INTERVAL;

    // Distance traveled for each interval.
    double  path_s = (ego.v > 0.0) ? ego.v * t : PATH_STEP;


    // Collect points for the path we want to follow.
    vector<double>  path_x;
    vector<double>  path_y;

    // Starting point & angle.
    double  start_x;
    double  start_y;
    double  angle;

    // If we don't have at least two previous path points to use,
    // just use the ego car's current location as a starting point.
    double  prev_x;
    double  prev_y;
    if (prev_path_size < 2) {
        double  lane_d = lane_to_d(ego.lane);

        // Start at the current ego car position.
        vector<double> xy = getXY(ego.s, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        start_x = xy[0];
        start_y = xy[1];

        // Find the assumed previous ego car position.
        // Bicycle model:
        //  x' = x + d cos(theta)
        //  y' = y + d sin(theta)
        // We're backing up by one meter so d == 1 and we subtract instead of add.
        //prev_x = start_x - cos(car_yaw);
        //prev_y = start_y - sin(car_yaw);

        // Rather than using the bicycle model to calculate a hypothetical previous
        // car position, just backup by one path step interval in the same lane -
        // this removes the need to know 'car_yaw' and should prevent backing up
        // out of the current lane.
        xy = getXY(ego.s - path_s, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        prev_x = xy[0];
        prev_y = xy[1];
    } else {
        // Start at the current end-of-path.
        start_x = previous_path_x[prev_path_size-1];
        start_y = previous_path_y[prev_path_size-1];

        // Penultimate path point.
        prev_x = previous_path_x[prev_path_size-2];
        prev_y = previous_path_y[prev_path_size-2];
    }
    path_x.push_back(prev_x);  path_y.push_back(prev_y);
    path_x.push_back(start_x); path_y.push_back(start_y);

    // Heading between the previous point and the start point is our current angle.
    angle = atan2(start_y - prev_y, start_x - prev_x);


    // Add waypoints for the current car position (s) at PATH_STEP increments,
    // staying in the same lane. This will ensure that our spline actually
    // follows the road.
    double  ego_d = lane_to_d(ego.lane);
    for (int i = 1; i <= WAYPOINTS_TO_ADD; ++i) {
        vector<double> next_wp = getXY(ego.s + PATH_STEP * i , ego_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        path_x.push_back(next_wp[0]); path_y.push_back(next_wp[1]);
    }


    // Our current list of points is:
    //  prev        point before the starting position
    //  start       starting position
    //  wp*         current position + PATH_STEP * i where i=1..WAYPOINTS_TO_ADD
    // The 'angle' is the heading between 'prev' and 'start'.

    // x = x * cos(angle) - y * sin(angle)
    // y = x * sin(angle) + y * cos(angle)

    // Before we create a spline for our path, adjust the (x, y)
    // coordinates so that the angle is 0 degrees in vehicle space.
    for (int i = 0; i < path_x.size(); i++) {
        double delta_x = path_x[i] - start_x;
        double delta_y = path_y[i] - start_y;

        path_x[i] = delta_x * cos(-angle) - delta_y * sin(-angle);
        path_y[i] = delta_x * sin(-angle) + delta_y * cos(-angle);
    }

    // Create a spline from our path points.
    tk::spline s;
    s.set_points(path_x, path_y);

    // Set the target x position to the current ego position plus one PATH_STEP.
    // The target y position is set using the spline we just created.
    // Recall that the ego vehicle object has already been updated with new trajectory info.
    vector<double> target = getXY(ego.s + path_s, ego_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    double  target_x = target[0];
    double  target_y = s(target_x);
    double  target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

    // Calculate the number of steps.
    // We want to cover the target distance at a given speed.
    // The car moves every 0.02 seconds. So the distance traveled
    // for each car movement is just (0.02 * speed). Divide the
    // target distance by this value to get the number of steps.
    double  N = target_dist / (0.02 * ego.v);

    // Change in x required to get to our target value in N steps.
    double  delta_x = target_x / N;

    // Add points until we reach the path length.
    prev_x = prev_y = 0.0;
    for (int i = 0; i < PATH_LENGTH - prev_path_size; i++) {

        double x = prev_x + delta_x;
        double y = s(x);    // Spline

        prev_x = x;

        // Undo the translation from global space to vehicle space
        // that we performed before creating the spline so that the
        // (x, y) values we send to the controller are in global space.
        double  new_x = start_x + (x * cos(angle) - y * sin(angle));
        double  new_y = start_y + (x * sin(angle) + y * cos(angle));

        next_x_vals.push_back(new_x);
        next_y_vals.push_back(new_y);
    }
}