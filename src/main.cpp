
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4251 4267 4275)
#endif // _WIN32
#include <uWS/uWS.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#ifdef _WIN32
#pragma warning(pop)
#endif // _WIN32
#include "helpers.h"
#include "json.hpp"
#include "JMT.h"
#include "spline.h"
#include "constants.h"
#include "vehicle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

void StraightLineTrajectory(double car_x, double car_y, double car_yaw,
                            vector<double>& next_x_vals, vector<double>& next_y_vals);
void CircleTrajectory(double car_x, double car_y, double car_yaw,
                      vector<double>& next_x_vals, vector<double>& next_y_vals);

void PathPlannerTrajectory(
    Vehicle& ego, vector<Vehicle>& traffic,
    vector<double>& map_waypoints_x, vector<double>& map_waypoints_y, vector<double>& map_waypoints_s,
    vector<double> previous_path_x, vector<double> previous_path_y,
    vector<double>& next_x_vals, vector<double>& next_y_vals);

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x, y, s and d normalized normal vectors.
    vector<double>  map_waypoints_x;
    vector<double>  map_waypoints_y;
    vector<double>  map_waypoints_s;
    vector<double>  map_waypoints_dx;
    vector<double>  map_waypoints_dy;

    // Waypoint map to read from
#ifdef _WIN32
#ifdef _DEBUG
    string          map_file_ = "../../data/highway_map.csv";
#else
    string          map_file_ = "../../../data/highway_map.csv";
#endif
#else
    string          map_file_ = "../data/highway_map.csv";
#endif
    std::ifstream   in_map_(map_file_.c_str(), std::ifstream::in);
    string          line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
        double  x;
        double  y;
        float   s;
        float   d_x;
        float   d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }
    std::wcout << "Read " << map_waypoints_x.size() << " waypoints from '" << map_file_.c_str() << "'." << std::endl;

    // Create the ego Vehicle object here. Initialization occurs in the
    // 'onConnection' message handler. This allows restarting the simulation
    // without exiting either the simulator or this app. Why is that desirable?
    // Because the other vehicle traffic is randomly generated.
    Vehicle ego;

#ifdef _WIN32
    h.onMessage([&ego,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy]
                (uWS::WebSocket<uWS::SERVER>* ws, char *data, size_t length, uWS::OpCode opCode) {
#else
    h.onMessage([&ego,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy]
                (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
#endif // _WIN32
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(data);
            if (s != "") {
                auto    j = json::parse(s);
                string  event = j[0].get<string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
          
                    // Main car's localization Data
                                                        // Initial values:
                    double  car_x     = j[1]["x"];      // 909.48
                    double  car_y     = j[1]["y"];      // 1128.67
                    double  car_s     = j[1]["s"];      // 124.8336
                    double  car_d     = j[1]["d"];      // 6.164833
                    double  car_yaw   = j[1]["yaw"];    // 0.0
                    double  car_speed = j[1]["speed"];  // 0.0

                    // Previous path data given to the Planner
                    auto    previous_path_x = j[1]["previous_path_x"];
                    auto    previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values 
                    double  end_path_s      = j[1]["end_path_s"];
                    double  end_path_d      = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side 
                    //   of the road.
                    auto    sensor_fusion = j[1]["sensor_fusion"];

                    /**
                    * TODO: define a path made up of (x,y) points that the car will visit
                    *   sequentially every .02 seconds
                    */
                    vector<double>  next_x_vals;
                    vector<double>  next_y_vals;

                    /*
                        Acceleration: rate of change of average speed over 0.2 second intervals
                        Jerk: average acceleration over 1 second intervals

                        Acceleration and Jerk should not exceed 10 m/s/s

                        Part of total acceleration is the normal component, AccN
                        This is the centripetal acceleration caused by turning.
                        (tighter, faster turn == higher AccN value)

                        Minimize total acceleration by gradually increasing/decreasing point path
                        spacing based on the 'car_speed' variable.

                        
                        Position (0, 0) is the center of the middle lane on the correct side of the road.

                        (waypoint_x, waypoint_y) = global map position of the waypoint
                            waypoints are along the center of the road, between the left lanes and right lanes

                        (waypoint_dx, waypoint_dy) = Frenet d unit normal vector pointing perpendicular to the road towards the right lane

                        left_lane_x = waypoint_x + (waypoint_dx * 2.0)
                        left_lane_y = waypoint_y + (waypoint_dx * 2.0)
                            multiply by 2.0 to get to the middle of the left lane since the lanes are 4.0 m wide

                        middle_lane_x = waypoint_x + (waypoint_dx * 6.0)
                        middle_lane_y = waypoint_y + (waypoint_dx * 6.0)
                            multiply by 6.0 to get to the middle of the middle lane
                            2.0 m to get to the middle of the left lane + 4.0 m to get to the next lane

                        right_lane_x = waypoint_x + (waypoint_dx * 10.0)
                        right_lane_y = waypoint_y + (waypoint_dx * 10.0)
                            multiply by 10.0 to get to the middle of the right lane
                            2.0 m to get to the middle of the left lane + 8.0 m to move over two lanes

                        getXY() helper function - converts Frenet (s, d) to (x, y)


                        The sensor_fusion variable contains all the information about the cars on the
                        right-hand side of the road.

                        The data format for each car is: [id, x, y, vx, vy, s, d]. The id is a unique
                        identifier for that car. The x, y values are in global map coordinates, and
                        the vx, vy values are the velocity components, also in reference to the global
                        map. Finally s and d are the Frenet coordinates for that car.

                        The vx, vy values can be useful for predicting where the cars will be in the
                        future. For instance, if you were to assume that the tracked car kept moving
                        along the road, then its future predicted Frenet s value will be its current
                        s value plus its (transformed) total velocity (m/s) multiplied by the time
                        elapsed into the future (s).


                        Bicycle model:

                        x' = x + d * cos(theta)
                        y' = y + d * sin(theta)
                        theta' = (theta + beta) mod (2 * pi)

                        Since yaw angle is with respect to fixed coordinates, longitudinal and lateral positions with
                        respect to the inertial fixed coordinates are found as follows:

                        r = psi_dot

                        v_X = v_x * cos(psi) - v_y * sin(psi)
                        v_Y = v_x * sin(psi) + v_y * cos(psi)

                        where v_X and v_Y denote the velocity components with respect to the fixed inertial coordinates.
                    */

                    // Update the ego vehicle.
                    if (previous_path_x.size() == 0) {
                        ego.lane = d_to_lane(car_d);
                        ego.s    = car_s;
                    } else {
                        // We can't just use 'end_path_s' because we may not keep all of the previous
                        // path points (see the PREVIOUS_POINTS_TO_KEEP constant). So find the last
                        // point that we are keeping and translate that (x, y) location to (s, d).
                        int prev_path_size = std::min(PREVIOUS_POINTS_TO_KEEP, (int) previous_path_x.size());
                        vector<double> sd =
                            getFrenet(previous_path_x[prev_path_size-1], previous_path_y[prev_path_size-1], car_yaw,
                                      map_waypoints_x, map_waypoints_y);
                        ego.lane = d_to_lane(sd[1]);
                        ego.s    = sd[0];
                    }

                    // Generate predictions for each vehicle in our sensor fusion data.
                    vector<Vehicle>             traffic;
                    map<int, vector<Vehicle>>   predictions;
                    for (auto car : sensor_fusion) {
                        int     id   = car[0];

                        double  s    = car[5];
                        double  d    = car[6];
                        double  vx   = car[3];
                        double  vy   = car[4];

                        int     lane = d_to_lane(d);
                        if (lane < 0)
                            continue;
                        double  v    = sqrt(vx * vx + vy * vy);

                        Vehicle car  = Vehicle(lane, s, v, 0.0, "CS");

                        traffic.push_back(car);
                        predictions[id] = car.generate_predictions();
                    }

                    // Add predictions for the ego vehicle.
                    predictions[-1] = ego.generate_predictions();

                    // Calculate a trajectory for the ego vehicle based on predictions.
                    vector<Vehicle> trajectory = ego.choose_next_state(predictions);

                    //// grab the current ego position to use as starting point
                    //ego.realize_next_state(trajectory);
                    //// grab the current ego position (now updated) to use as target

                    //StraightLineTrajectory(car_x, car_y, car_yaw, next_x_vals, next_y_vals);
                    //CircleTrajectory(car_x, car_y, car_yaw, next_x_vals, next_y_vals);

                    // Update the 'ego' vehicle (lane, velocity) and generate a path (next_x_vals, next_y_vals).
                    PathPlannerTrajectory(
                        ego, traffic,
                        map_waypoints_x, map_waypoints_y, map_waypoints_s,
                        previous_path_x, previous_path_y,
                        next_x_vals, next_y_vals);

                    // Send path data to the controller.
                    json    msgJson;
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto    msg = "42[\"control\","+ msgJson.dump()+"]";
#ifdef _WIN32
                    ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif // _WIN32
                }  // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
#ifdef _WIN32
                ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif // _WIN32
            } // end hasdata
        }  // end websocket if
    }); // end h.onMessage

#ifdef _WIN32
    h.onConnection([&h,&ego](uWS::WebSocket<uWS::SERVER>* ws, uWS::HttpRequest req) {
#else
    h.onConnection([&h,&ego](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
#endif // _WIN32
        std::cout << "Connected!!!" << std::endl;

        // Initialize the ego vehicle object.
        ego = Vehicle(-1, 0.0, 0.0, 0.0, "KL"); // (lane, s) are initialized in the first iteration.

        double  goal_s    = 0.0;
        double  goal_lane = 1.0;    // Stay in the middle lane (0 == left lane, 1 == middle lane, 2 == right lane)
        vector<double>  ego_config = {SPEED_LIMIT, (double) LANE_COUNT, goal_s, goal_lane, MAX_ACCELERATION};
        ego.configure(ego_config);
    });

#ifdef _WIN32
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER>* ws, int code, char *message, size_t length) {
        ws->close();
#else
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
#endif // _WIN32
        std::cout << "Disconnected" << std::endl;
    });

    int     port = 4567;
#ifdef _WIN32
    auto    host = "127.0.0.1";
    if (h.listen(host, port)) {
#else
    if (h.listen(port)) {
#endif // _WIN32
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
  
    h.run();
}

void StraightLineTrajectory(double car_x, double car_y, double car_yaw,
                            vector<double>& next_x_vals, vector<double>& next_y_vals) {

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
                      vector<double>& next_x_vals, vector<double>& next_y_vals) {

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

void PathPlannerTrajectory(
    Vehicle& ego, vector<Vehicle>& traffic,
    vector<double>& map_waypoints_x, vector<double>& map_waypoints_y, vector<double>& map_waypoints_s,
    vector<double>  previous_path_x, vector<double>  previous_path_y,
    vector<double>& next_x_vals,     vector<double>& next_y_vals) {

    int     prev_path_size = std::min(PREVIOUS_POINTS_TO_KEEP, (int) previous_path_x.size());

    // Before adding new path points, add a few path points from the previous
    // cycle to maintain continuity.
    for (int i = 0; i < prev_path_size; ++i) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }


    // *****************************************************************
    // ****************** ADJUST LANE / SPEED **************************
    // *****************************************************************

    // Lane identifiers for other cars
    bool car_ahead = false;
    bool car_left  = false;
    bool car_right = false;

    // Find ref_v to use, see if car is in lane
    for (auto car : traffic) {
        // Adjust the other vehicle 's' position by adding 'prev_path_size' steps.
        // We know that each step is 20ms (0.02s) and that the other vehicle
        // is moving at velocity 'car.v'.
        double check_car_s = car.s + (prev_path_size * 0.02 * car.v);

        // Identify whether the car is ahead, to the left, or to the right
        if (car.lane == ego.lane) {
            // Another car is ahead
            car_ahead |= (check_car_s > ego.s) && ((check_car_s - ego.s) < PREFERRED_BUFFER_LANE_CHANGE);
        } else if (car.lane - ego.lane == 1) {
            // Another car is to the right
            car_right |= ((ego.s - PREFERRED_BUFFER_LANE_CHANGE) < check_car_s) && ((ego.s + PREFERRED_BUFFER_LANE_CHANGE) > check_car_s);
        } else if (ego.lane - car.lane == 1) {
            // Another car is to the left
            car_left |= ((ego.s - PREFERRED_BUFFER_LANE_CHANGE) < check_car_s) && ((ego.s + PREFERRED_BUFFER_LANE_CHANGE) > check_car_s);
        }
    }

    // Modulate the speed to avoid collisions. Change lanes if it is safe to do so (nobody to the side)
    if (car_ahead) {
        // A car is ahead - change lanes if we can, else slow down.
        if (!car_right && ego.lane < 2)
            // No car to the right and we're not already in the right lane.
            ego.lane++;
        else if (!car_left && ego.lane > 0)
            // No car to the left and we're not already in the left lane.
            ego.lane--;
        else
            // Nowhere to shift -> slow down
            ego.v -= MAX_ACCELERATION;
    } else {
        // Stay in the center lane if possible.
        if (ego.lane != 1 && ((ego.lane == 2 && !car_left) || (ego.lane == 0 && !car_right)))
            ego.lane = 1;

        // If there is no car ahead and we're still below the speed limit, speed up!
        if (ego.v < SPEED_LIMIT) {
            ego.v += MAX_ACCELERATION;
            if (ego.v < SPEED_LIMIT / 2)
                ego.v += MAX_ACCELERATION;
        }
    }

    // *****************************************************************
    // *****************************************************************
    // *****************************************************************


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

        // Rather than using the bicycle model to calculate a hypothetical
        // previous car position, just backup by one PATH_STEP in the same
        // lane - this removes the need to know 'car_yaw'.
        xy = getXY(ego.s - PATH_STEP, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
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
    vector<double> next_wp0 = getXY(ego.s + PATH_STEP    , ego_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(ego.s + PATH_STEP * 2, ego_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(ego.s + PATH_STEP * 3, ego_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    path_x.push_back(next_wp0[0]); path_y.push_back(next_wp0[1]);
    path_x.push_back(next_wp1[0]); path_y.push_back(next_wp1[1]);
    path_x.push_back(next_wp2[0]); path_y.push_back(next_wp2[1]);


    // Our current list of points is:
    //  prev        point before the starting position
    //  start       starting position
    //  wp0         current position + PATH_STEP
    //  wp1         current position + PATH_STEP * 2
    //  wp2         current position + PATH_STEP * 3
    // The 'angle' is the heading between 'prev_pos' and 'pos'.

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
    vector<double> target = getXY(ego.s + PATH_STEP, ego_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
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
