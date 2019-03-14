#ifndef VEHICLE_H
#define VEHICLE_H

#include <string>
#include <vector>
#include <map>

using std::string;
using std::vector;
using std::map;

class Vehicle {
public:
    // Constructors
    Vehicle();
    Vehicle(int lane, double s, double v, double a, string state="CS");

    // Destructor
    virtual ~Vehicle();

    // Vehicle functions
    vector<Vehicle>     choose_next_state(map<int, vector<Vehicle>> &predictions);
    vector<string>      successor_states();
    vector<Vehicle>     generate_trajectory(string state,
                                            map<int, vector<Vehicle>> &predictions);
    vector<double>      get_kinematics(map<int, vector<Vehicle>> &predictions,
                                       int lane);
    vector<Vehicle>     constant_speed_trajectory();
    vector<Vehicle>     keep_lane_trajectory(map<int, vector<Vehicle>> &predictions);
    vector<Vehicle>     lane_change_trajectory(string state,
                                               map<int, vector<Vehicle>> &predictions);
    vector<Vehicle>     prep_lane_change_trajectory(string state,
                                                    map<int, vector<Vehicle>> &predictions);

    void                increment(double dt);
    double              position_at(double t);

    bool                get_vehicle_behind(map<int, vector<Vehicle>> &predictions,
                                           int lane, Vehicle &rVehicle);
    bool                get_vehicle_ahead(map<int, vector<Vehicle>> &predictions,
                                          int lane,  Vehicle &rVehicle);

    vector<Vehicle>     generate_predictions(int horizon = 2);

    void                realize_next_state(vector<Vehicle> &trajectory);

    void                configure(vector<double> &road_data);

    // public Vehicle variables
    struct collider {
        bool    collision;  // is there a collision?
        int     time;       // time collision happens
    };

    map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};

    int         lane, goal_lane, lanes_available;
    double      s, goal_s;
    double      v, target_speed;
    double      a, max_acceleration;
    double      time_step;
    string      state;

    vector<Vehicle> traffic;
};

#endif VEHICLE_H
