
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
    double car_x, double car_y, double car_s, double car_yaw,
    Vehicle& ego,
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

    vector<double>  ego_config = {SPEED_LIMIT, (double) LANE_COUNT, 0.0, 1.0, MAX_ACCELERATION};
    Vehicle ego = Vehicle(1, 0.0, 0.0, 0.0, "KL");
    ego.configure(ego_config);

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
                    double  car_x     = j[1]["x"];
                    double  car_y     = j[1]["y"];
                    double  car_s     = j[1]["s"];
                    double  car_d     = j[1]["d"];
                    double  car_yaw   = j[1]["yaw"];
                    double  car_speed = j[1]["speed"];

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
                    */

                    // Translate sensor fusion data into a traffic map.
                    map<int, Vehicle>   traffic;
                    for (auto car : sensor_fusion) {
                        int id = car[0];
                        int lane = d_to_lane(car[6]);
                        double s  = car[5];
                        double vx = car[3];
                        double vy = car[4];
                        double v = sqrt(vx * vx + vy * vy);
                        traffic[id] = Vehicle(lane, s, v, 0.0, "CS");
                    }

                    // Generate predictions for each vehicle in the traffic map.
                    map<int, vector<Vehicle>>   predictions;
                    for (auto car : traffic) {
                        predictions[car.first] = car.second.generate_predictions();
                    }

                    // Add predictions for the ego vehicle.
                    predictions[-1] = ego.generate_predictions();

                    // Calculate a trajectory for the ego vehicle.
                    vector<Vehicle> trajectory = ego.choose_next_state(predictions);
                    ego.realize_next_state(trajectory);

                    //StraightLineTrajectory(car_x, car_y, car_yaw, next_x_vals, next_y_vals);
                    //CircleTrajectory(car_x, car_y, car_yaw, next_x_vals, next_y_vals);
#ifdef notdef

                    int     prev_path_size = std::min(PREVIOUS_POINTS_TO_KEEP, (int) previous_path_x.size());

                    // Add a few path points from the previous cycle to maintain continuity.
                    for (int i = 0; i < prev_path_size; ++i) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    double  pos_x;
                    double  pos_y;
                    double  angle;

                    // If there were no previous path points, start at the car's current position.
                    if (prev_path_size == 0) {
                        pos_x = car_x;
                        pos_y = car_y;
                        angle = deg2rad(car_yaw);
                    // Otherwise start at the end of the previous path.
                    } else {
                        pos_x = previous_path_x[prev_path_size - 1];
                        pos_y = previous_path_y[prev_path_size - 1];

                        // If we have enough path points, set the angle to the direction between
                        // the two most recent path points.
                        if (prev_path_size > 1) {
                            double pos_x2 = previous_path_x[prev_path_size - 2];
                            double pos_y2 = previous_path_y[prev_path_size - 2];
                            angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
                        // Otherwise just use the current car heading.
                        } else {
                            angle = deg2rad(car_yaw);
                        }
                    }

                    double  dist_inc = 0.5; // ~56 mph
                    int nextWaypoint = NextWaypoint(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
                    std::cout << "nextWaypoint: " << nextWaypoint << std::endl;
                    // how far between waypoints? how many should we try for?
                    for (int i = 0; i < 50 - prev_path_size; ++i, ++nextWaypoint) {
                        // Location of the next waypoint (middle of the middle lane).
                        double  next_x = map_waypoints_x[nextWaypoint] + map_waypoints_dx[nextWaypoint] * 6.0;
                        double  next_y = map_waypoints_y[nextWaypoint] + map_waypoints_dy[nextWaypoint] * 6.0;

                        // Angle from (pos_x, pos_y) to (next_x, next_y).
                        angle = atan2(next_y - pos_y, next_x - pos_x);

                        // Distance to the next waypoint.
                        double  dist = distance(pos_x, pos_y, next_x, next_y);
                        std::cout << "distance: " << dist << std::endl;
                        if (dist <= 0.000001)   // We're essentially at the next waypoint, so move on.
                            continue;   // Should probably decrement i here so we get the full 50 points
                                        // but 49 works just as well.

                        // How long are we going to take to get to the next waypoint?
                        double  T = dist / dist_inc;

                        // Translate (pos_x, pos_y) and (next_x, next_y) to Frenet s coordinates.
                        // Keep the car speed constant with 0.0 acceleration.
                        vector<double>  sd = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
                        double  s1, s1_dot, s1_dot_dot; // pos_x, pos_y
                        s1 = sd[0]; s1_dot = car_speed; s1_dot_dot = 0.0;
                        sd = getFrenet(next_x, next_y, angle, map_waypoints_x, map_waypoints_y);
                        double  s2, s2_dot, s2_dot_dot; // next_x, next_y
                        s2 = sd[0], s2_dot = car_speed; s2_dot_dot = 0.0;

                        // Calculate coefficients for JMT between (pos_x, pos_y) and (next_x, next_y).
                        vector<double>  start = {s1, s1_dot, s1_dot_dot};
                        vector<double>  end   = {s2, s2_dot, s2_dot_dot};
                        vector<double>  a = JMT(start, end, T);

                        // Use the JMT to add points between (pos_x, pos_y) and (next_x, next_y).
                        for (int t = 0; t < T && i < 50 - prev_path_size; ++t, ++i) {
                            // s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
                            double  next_s = 0.0;
                            for (int k = 0; k < 6; ++k)
                                next_s += a[k] * pow(t, k);

                            // Convert (s, d) to (x, y).
                            // For d we just use the value tranlated from (next_x, next_y), assuming we'll stay in the same lane.
                            vector<double>  xy = getXY(next_s, sd[1], map_waypoints_s, map_waypoints_x, map_waypoints_y);
                            pos_x = xy[0] + map_waypoints_dx[nextWaypoint] * 6.0;
                            pos_y = xy[1] + map_waypoints_dy[nextWaypoint] * 6.0;
                            next_x_vals.push_back(pos_x);
                            next_y_vals.push_back(pos_y);
                        }
                    }
#endif
                    PathPlannerTrajectory(
                        car_x, car_y, car_s, car_yaw,
                        ego,
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
    h.onConnection([&h](uWS::WebSocket<uWS::SERVER>* ws, uWS::HttpRequest req) {
#else
    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
#endif // _WIN32
        std::cout << "Connected!!!" << std::endl;
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
    double car_x, double car_y, double car_s, double car_yaw,
    Vehicle& ego,
    vector<double>& map_waypoints_x, vector<double>& map_waypoints_y, vector<double>& map_waypoints_s,
    vector<double> previous_path_x, vector<double> previous_path_y,
    vector<double>& next_x_vals, vector<double>& next_y_vals) {

    int     prev_path_size = std::min(PREVIOUS_POINTS_TO_KEEP, (int) previous_path_x.size());

    // Add a few path points from the previous cycle to maintain continuity.
    for (int i = 0; i < prev_path_size; ++i) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // Collect points for the path we want to follow.
    vector<double>  path_x;
    vector<double>  path_y;

    // Starting point & angle.
    double pos_x;
    double pos_y;
    double angle;

    // If we don't have at least two previous path points to use,
    // just use the car's current location as a starting point.
    if (prev_path_size < 2) {
        // Start at the current car position.
        pos_x = car_x;
        pos_y = car_y;
        angle = deg2rad(car_yaw);

        // Bicycle model:
        //  x' = x + d cos(theta)
        //  y' = y + d sin(theta)
        // We're backing up by one meter so d == 1 and we subtract instead of add.
        double prev_x = car_x - cos(car_yaw);
        double prev_y = car_y - sin(car_yaw);

        path_x.push_back(prev_x); path_y.push_back(prev_y);
        path_x.push_back(pos_x);  path_y.push_back(pos_y);

    } else {
        // Start at the current end-of-path.
        pos_x = previous_path_x[prev_path_size-1];
        pos_y = previous_path_y[prev_path_size-1];

        // Penultimate path point.
        double prev_x = previous_path_x[prev_path_size-2];
        double prev_y = previous_path_y[prev_path_size-2];

        // Heading between last two path points is our current angle.
        angle = atan2(pos_y - prev_y, pos_x - prev_x);

        path_x.push_back(prev_x); path_y.push_back(prev_y);
        path_x.push_back(pos_x);  path_y.push_back(pos_y);
    }


    // Add waypoints for current car position plus 30, 60, 90 meters.
    int lane_d = (int) round(ego.d);
    vector<double> next_wp0 = getXY(car_s+30, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s+60, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s+90, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    path_x.push_back(next_wp0[0]); path_y.push_back(next_wp0[1]);
    path_x.push_back(next_wp1[0]); path_y.push_back(next_wp1[1]);
    path_x.push_back(next_wp2[0]); path_y.push_back(next_wp2[1]);

    for (int i = 0; i < path_x.size(); i++) {
        // shift car reference angle to 0 degrees
        double shift_x = path_x[i] - pos_x;
        double shift_y = path_y[i] - pos_y;

        path_x[i] = (shift_x * cos(-angle) - shift_y * sin(-angle));
        path_y[i] = (shift_x * sin(-angle) + shift_y * cos(-angle));
    }

    // create a spline
    tk::spline s;
    s.set_points(path_x, path_y);

    // Set the target for 30m down the road.
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

    // Slowly increase speed to the max.
    if (ego.v < SPEED_LIMIT)
        ego.v += 0.1;

    // Calculate the number of steps.
    // We want to cover the target distance at a given speed.
    // The car moves every 0.02 seconds. So the distance traveled
    // for each car movement is just (0.02 * speed). Divide the
    // target distance by this value to get the number of steps.
    double N = target_dist / (0.02 * ego.v);

    // Change in x required to get to our target value in N steps.
    double delta_x = target_x / N;

    double prev_x = 0.0;
    for (int i = 1; i < 50 - prev_path_size; i++) {

        double x = prev_x + delta_x;
        double y = s(x);    // Spline

        prev_x = x;

        // rotate back to normal after rotating to vehicle space
        double new_x = pos_x + (x * cos(angle) - y * sin(angle));
        double new_y = pos_y + (x * sin(angle) + y * cos(angle));

        next_x_vals.push_back(new_x);
        next_y_vals.push_back(new_y);
    }
}
