
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
    double car_yaw,

    Vehicle& ego,
    vector<Vehicle>& traffic,
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
                    int closestWaypoint = ClosestWaypoint(car_x, car_y, map_waypoints_x, map_waypoints_y);
                    double  dx = map_waypoints_dx[closestWaypoint];
                    double  dy = map_waypoints_dy[closestWaypoint];

                    ego.lane = d_to_lane(car_d);
                    ego.s    = previous_path_x.size() > 0 ? end_path_s : car_s;
                    ego.d    = car_d;
                    //ego.v    = car_speed;

                    //vector<double> xy = getXY(car_s, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    //std::cout << "(" << car_x << ", " << car_y << ") : "
                    //          << "(" << xy[0] << ", " << xy[1] << ")" << std::endl;

                    // Generate predictions for each vehicle in our sensor fusion data.
                    vector<Vehicle>             traffic;
                    map<int, vector<Vehicle>>   predictions;
                    for (auto car : sensor_fusion) {
                        int     id   = car[0];

                        double  s    = car[5];
                        double  d    = car[6];
                        double  vx   = car[3];
                        double  vy   = car[4];
                        double  v    = sqrt(vx * vx + vy * vy);

                        int     lane = d_to_lane(d);
                        if (lane < 0)
                            continue;

                        Vehicle car  = Vehicle(lane, s, d, v, 0.0, "CS");

                        traffic.push_back(car);
                        predictions[id] = car.generate_predictions();
                    }

                    // Add predictions for the ego vehicle.
                    predictions[-1] = ego.generate_predictions();

                    //// Calculate a trajectory for the ego vehicle based on predictions.
                    //vector<Vehicle> trajectory = ego.choose_next_state(predictions);

                    //// grab the current ego position to use as starting point
                    //ego.realize_next_state(trajectory);
                    //// grab the current ego position (now updated) to use as target

                    //StraightLineTrajectory(car_x, car_y, car_yaw, next_x_vals, next_y_vals);
                    //CircleTrajectory(car_x, car_y, car_yaw, next_x_vals, next_y_vals);

                    PathPlannerTrajectory(
                        car_yaw,

                        ego,
                        traffic,
                        map_waypoints_x, map_waypoints_y, map_waypoints_s,
                        previous_path_x, previous_path_y,
                        next_x_vals, next_y_vals);
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
        double  goal_s    = 0.0;
        double  goal_lane = 2.0;
        vector<double>  ego_config = {SPEED_LIMIT, (double) LANE_COUNT, goal_s, goal_lane, MAX_ACCELERATION};
        int     lane = 1;       // Initial lane (1 == middle lane).
        double  s, d, v, a;     // Initial (s, d) position and velocity, acceleration.
        s = 0.0; d = 0.0; v = 0.0; a = 0.0;
        string  state = "KL";   // Keep Lane
        ego = Vehicle(lane, s, d, v, a, "KL");
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

#ifdef notdef
void PathPlannerTrajectory(
    double car_x, double car_y, double car_yaw,
    Vehicle& ego,
    vector<Vehicle>& traffic,
    vector<double>& map_waypoints_x, vector<double>& map_waypoints_y, vector<double>& map_waypoints_s,
    vector<double> previous_path_x, vector<double> previous_path_y,
    vector<double>& next_x_vals, vector<double>& next_y_vals) {


    // Reference velocity to target
    static double ref_vel = 0.0; // mph


    int     prev_path_size = std::min(PREVIOUS_POINTS_TO_KEEP, (int) previous_path_x.size());

    // Add a few path points from the previous cycle to maintain continuity.
    for (int i = 0; i < prev_path_size; ++i) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }


    int prev_size = (int)previous_path_x.size();

    //if (prev_size > 0) {
    //	car_s = end_path_s;
    //}

    // Lane identifiers for other cars
    bool too_close = false;
    bool car_left = false;
    bool car_right = false;

    // Find ref_v to use, see if car is in lane
    for (auto car : traffic) {
        // Check width of lane, in case cars are merging into our lane
        double check_speed = car.v;
        double check_car_s = car.s;

        // If using previous points can project an s value outwards in time
        // (What position we will be in in the future)
        // check s values greater than ours and s gap
        check_car_s += ((double)prev_size*0.02*car.v);

        int gap = 30; // m

        // Identify whether the car is ahead, to the left, or to the right
        if (car.lane == ego.lane) {
            // Another car is ahead
            too_close |= (check_car_s > ego.s) && ((check_car_s - ego.s) < gap);
        } else if (car.lane - ego.lane == 1) {
            // Another car is to the right
            car_right |= ((ego.s - gap) < check_car_s) && ((ego.s + gap) > check_car_s);
        } else if (ego.lane - car.lane == 1) {
            // Another car is to the left
            car_left |= ((ego.s - gap) < check_car_s) && ((ego.s + gap) > check_car_s);
        }
    }

    // Modulate the speed to avoid collisions. Change lanes if it is safe to do so (nobody to the side)
    //double acc = 0.224;
    //double max_speed = 49.5;
    if (too_close) {
        // A car is ahead
        // Decide to shift lanes or slow down
        if (!car_right && ego.lane < 2) {
            // No car to the right AND there is a right lane -> shift right
            ego.lane++;
        } else if (!car_left && ego.lane > 0) {
            // No car to the left AND there is a left lane -> shift left
            ego.lane--;
        } else {
            // Nowhere to shift -> slow down
            ref_vel -= MAX_ACCELERATION;
        }
    } else {
        if (ego.lane != 1) {
            // Not in the center lane. Check if it is safe to move back
            if ((ego.lane == 2 && !car_left) || (ego.lane == 0 && !car_right)) {
                // Move back to the center lane
                ego.lane = 1;
            }
        }
                
        if (ref_vel < SPEED_LIMIT) {
            // No car ahead AND we are below the speed limit -> speed limit
            ref_vel += MAX_ACCELERATION;
        }
    }

    ego.v = ref_vel;


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

    // Add waypoints for current car position (s) plus 30, 60, 90 meters,
    // staying in the same lane. This will ensure that our spline actually
    // follows the road.
    double  lane_d = lane_to_d(ego.lane);
    lane_d = 2 + 4 * ego.lane;
    std::cout << "ego: d=" << ego.d << " lane_d=" << lane_d
              << (ego.d == lane_d ? "OK" : "*****")
              << std::endl;
    vector<double> next_wp0 = getXY(ego.s+30, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(ego.s+60, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(ego.s+90, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    path_x.push_back(next_wp0[0]); path_y.push_back(next_wp0[1]);
    path_x.push_back(next_wp1[0]); path_y.push_back(next_wp1[1]);
    path_x.push_back(next_wp2[0]); path_y.push_back(next_wp2[1]);

    // Our current list of points is:
    //  prev_pos    point before the starting position
    //  pos         starting position
    //  wp0         current position + 20m
    //  wp1         current position + 40m
    //  wp2         current position + 60m
    // The 'angle' is the heading between 'prev_pos' and 'pos'.

    // x = x * cos(angle) - y * sin(angle)
    // y = x * sin(angle) + y * cos(angle)

    // Before we create a spline for our path, adjust the (x, y)
    // coordinates so that the angle is 0 degrees in vehicle space.
    for (int i = 0; i < path_x.size(); i++) {
        double delta_x = path_x[i] - pos_x;
        double delta_y = path_y[i] - pos_y;

        path_x[i] = delta_x * cos(-angle) - delta_y * sin(-angle);
        path_y[i] = delta_x * sin(-angle) + delta_y * cos(-angle);
    }

    // Create a spline from our path points.
    tk::spline s;
    s.set_points(path_x, path_y);

    // Set the target position to the current ego position.
    // Recall that the ego vehicle object has already been updated with new trajectory info.
    vector<double> target = getXY(ego.s+30, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    double  target_x = target[0];
    double  target_y = target[1];
    double  target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

    // Slowly increase speed to the max speed.
    //if (ego.v < SPEED_LIMIT)
    //    ego.v += 0.1;

    // Calculate the number of steps.
    // We want to cover the target distance at a given speed.
    // The car moves every 0.02 seconds. So the distance traveled
    // for each car movement is just (0.02 * speed). Divide the
    // target distance by this value to get the number of steps.
    double  N = target_dist / (0.02 * ego.v);

    // Change in x required to get to our target value in N steps.
    double  delta_x = target_x / N;

    // Add points until we reach the magic number of 50 (which at 20ms
    // per step is a horizon of 1s).
    double  prev_x = 0.0;
    for (int i = 1; i < 50 - prev_path_size; i++) {

        double x = prev_x + delta_x;
        double y = s(x);    // Spline

        prev_x = x;

        // Undo the translation from global space to vehicle space
        // that we performed before creating the spline so that the
        // (x, y) values we send to the controller are in global space.
        double  new_x = pos_x + (x * cos(angle) - y * sin(angle));
        double  new_y = pos_y + (x * sin(angle) + y * cos(angle));

        next_x_vals.push_back(new_x);
        next_y_vals.push_back(new_y);
    }
}
#endif

void PathPlannerTrajectory(
    double car_yaw,

    Vehicle& ego,
    vector<Vehicle>& traffic,
    vector<double>& map_waypoints_x, vector<double>& map_waypoints_y, vector<double>& map_waypoints_s,
    vector<double> previous_path_x, vector<double> previous_path_y,
    vector<double>& next_x_vals, vector<double>& next_y_vals) {

    int     prev_path_size = std::min(PREVIOUS_POINTS_TO_KEEP, (int) previous_path_x.size());
    prev_path_size = (int) previous_path_x.size();

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
        // Start at the current ego car position.
        vector<double> xy = getXY(ego.s, ego.d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        start_x = xy[0];
        start_y = xy[1];

        // Find the assumed previous ego car position.
        // Bicycle model:
        //  x' = x + d cos(theta)
        //  y' = y + d sin(theta)
        // We're backing up by one meter so d == 1 and we subtract instead of add.
        prev_x = start_x - cos(car_yaw);
        prev_y = start_y - sin(car_yaw);

        // Just use the current ego car heading as our current angle.
        angle = deg2rad(car_yaw);
    } else {
        // Start at the current end-of-path.
        start_x = previous_path_x[prev_path_size-1];
        start_y = previous_path_y[prev_path_size-1];

        // Penultimate path point.
        prev_x = previous_path_x[prev_path_size-2];
        prev_y = previous_path_y[prev_path_size-2];

        // Heading between last two path points is our current angle.
        angle = atan2(start_y - prev_y, start_x - prev_x);
    }
    path_x.push_back(prev_x);  path_y.push_back(prev_y);
    path_x.push_back(start_x); path_y.push_back(start_y);


    // Add waypoints for current car position (s) plus 30, 60, 90 meters,
    // staying in the same lane. This will ensure that our spline actually
    // follows the road.
    double  lane_d = lane_to_d(ego.lane);
    vector<double> next_wp0 = getXY(ego.s+30, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(ego.s+60, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(ego.s+90, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    path_x.push_back(next_wp0[0]); path_y.push_back(next_wp0[1]);
    path_x.push_back(next_wp1[0]); path_y.push_back(next_wp1[1]);
    path_x.push_back(next_wp2[0]); path_y.push_back(next_wp2[1]);


    // Our current list of points is:
    //  prev_pos    point before the starting position
    //  pos         starting position
    //  wp0         current position + 20m
    //  wp1         current position + 40m
    //  wp2         current position + 60m
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

    // Set the target position to the current ego position.
    // Recall that the ego vehicle object has already been updated with new trajectory info.
//    vector<double> target = getXY(ego.s+30, lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
//    double  target_x = target[0];
//    double  target_y = target[1];

    // Set the target position to 30m down the road along the spline we just created.
    double  target_x = 30.0;
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

    // Add points until we reach the magic number of 50 (which at 20ms
    // per step is a horizon of 1s).
    prev_x = prev_y = 0.0;
    for (int i = 0; i < 50 - prev_path_size; i++) {

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
