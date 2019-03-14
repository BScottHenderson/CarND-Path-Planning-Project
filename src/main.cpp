
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
#include "json.hpp"
#include "constants.h"
#include "helpers.h"
#include "spline.h"
#include "JMT.h"
#include "trajectory.h"
#include "vehicle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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
                        left_lane_y = waypoint_y + (waypoint_dy * 2.0)
                            multiply by 2.0 to get to the middle of the left lane since the lanes are 4.0 m wide

                        middle_lane_x = waypoint_x + (waypoint_dx * 6.0)
                        middle_lane_y = waypoint_y + (waypoint_dy * 6.0)
                            multiply by 6.0 to get to the middle of the middle lane
                            (2.0 m to get to the middle of the left lane, 4.0 m to get to the next lane)

                        right_lane_x = waypoint_x + (waypoint_dx * 10.0)
                        right_lane_y = waypoint_y + (waypoint_dy * 10.0)
                            multiply by 10.0 to get to the middle of the right lane
                            (2.0 m to get to the middle of the left lane, 8.0 m to move over two lanes)
                    */

                    // Update the ego vehicle based on new information from the controller.
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
                        //ego.lane = d_to_lane(sd[1]);
                        // Do *not* update the ego car lane for all non-initial iterations.
                        ego.s    = sd[0];
                    }

                    // Generate predictions for each vehicle in our sensor fusion data.
                    vector<Vehicle>             traffic;
                    map<int, vector<Vehicle>>   predictions;
                    for (auto car : sensor_fusion) {
                        // The data format for each car is: [id, x, y, vx, vy, s, d].
                        int     id   = car[0];

                        double  s    = car[5];
                        double  d    = car[6];
                        double  vx   = car[3];
                        double  vy   = car[4];

                        int     lane = d_to_lane(d);
                        if (lane < 0 || MAX_LANE < lane)
                            continue;
                        double  v    = sqrt(vx * vx + vy * vy);

                        Vehicle car  = Vehicle(lane, s, v, 0.0, "CS");

                        traffic.push_back(car);
                        predictions[id] = car.generate_predictions(PREDICTION_HORIZON);
                    }

                    // Add predictions for the ego vehicle.
                    predictions[-1] = ego.generate_predictions(PREDICTION_HORIZON);

                    // Calculate a trajectory for the ego vehicle based on predictions.
                    vector<Vehicle> trajectory = ego.choose_next_state(predictions);

                    // Update the ego vehicle state using the calculated trajectory.
                    //ego.realize_next_state(trajectory);

                    // Update the ego vehicle state.
                    UpdateEgo(ego, traffic, previous_path_x, previous_path_y);

                    // Generate a trajectory using the update ego information.
                    PathPlannerTrajectory(
                        ego,
                        map_waypoints_x, map_waypoints_y, map_waypoints_s,
                        previous_path_x, previous_path_y,
                        next_x_vals,     next_y_vals);

                    //StraightLineTrajectory(car_x, car_y, car_yaw, next_x_vals, next_y_vals);
                    //CircleTrajectory(car_x, car_y, car_yaw, next_x_vals, next_y_vals);

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
                                                // KL == Keep Lane

        double  goal_s    = 0.0;
        double  goal_lane = 1.0;    // Stay in the middle lane (0 == left lane, 1 == middle lane, 2 == right lane)
        vector<double>  ego_config =
            {SPEED_LIMIT, (double) LANE_COUNT, goal_s, goal_lane, MAX_ACCELERATION, UPDATE_INTERVAL};
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
