#include "vehicle.h"
#include <string>
#include <vector>
#include <map>
#include <assert.h>
#include <algorithm>
#include <iterator>
#include "constants.h"
#include "helpers.h"
#include "cost.h"

// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, double s, double d, double v, double a, string state) {
    this->lane  = lane;
    this->s     = s;
    this->d     = d;
    this->v     = v;
    this->a     = a;
    this->state = state;
    max_acceleration = -1;
}

Vehicle::~Vehicle() {}

vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> &predictions) {
    /**
     * Here you can implement the transition_function code from the Behavior 
     *   Planning Pseudocode classroom concept.
     *
     * @param A predictions map. This is a map of vehicle id keys with predicted
     *   vehicle trajectories as values. Trajectories are a vector of Vehicle 
     *   objects representing the vehicle at the current timestep and one timestep
     *   in the future.
     * @output The best (lowest cost) trajectory corresponding to the next ego 
     *   vehicle state.
     *
     * Functions that will be useful:
     * 1. successor_states - Uses the current state to return a vector of possible
     *    successor states for the finite state machine.
     * 2. generate_trajectory - Returns a vector of Vehicle objects representing 
     *    a vehicle trajectory, given a state and predictions. Note that 
     *    trajectory vectors might have size 0 if no possible trajectory exists 
     *    for the state. 
     * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.
     */

    /*
    def transition_function(predictions, current_fsm_state, current_pose, cost_functions, weights):
        # only consider states which can be reached from current FSM state.
        possible_successor_states = successor_states(current_fsm_state)

        # keep track of the total cost of each state.
        costs = []
        for state in possible_successor_states:
            # generate a rough idea of what trajectory we would
            # follow IF we chose this state.
            trajectory_for_state = generate_trajectory(state, current_pose, predictions)

            # calculate the "cost" associated with that trajectory.
            cost_for_state = 0
            for i in range(len(cost_functions)) :
                # apply each cost function to the generated trajectory
                cost_function = cost_functions[i]
                cost_for_cost_function = cost_function(trajectory_for_state, predictions)

                # multiply the cost by the associated weight
                weight = weights[i]
                cost_for_state += weight * cost_for_cost_function

            costs.append({'state' : state, 'cost' : cost_for_state})

        # Find the minimum cost state.
        best_next_state = None
        min_cost = 9999999
        for i in range(len(possible_successor_states)):
            state = possible_successor_states[i]
            cost  = costs[i]
            if cost < min_cost:
                min_cost = cost
                best_next_state = state

        return best_next_state
    */

    vector<std::pair<double, vector<Vehicle>>>  costs;

    // For each state that is a possible successor to the current state ...
    vector<string>  states = this->successor_states();
    for (auto state : states) {
        // Generate a rough idea of the trajectory we would follow if we chose this state.
        vector<Vehicle> trajectory_for_state = generate_trajectory(state, predictions);

        // Calculate the cost associated with the trajectory.
        double  cost_for_state = calculate_cost(*this, predictions, trajectory_for_state);

        // Save the cost so we can determine the min cost trajectory.
        costs.push_back({cost_for_state, trajectory_for_state});
    }

    vector<Vehicle> min_cost_trajectory;
    double          min_cost = std::numeric_limits<double>::max();
    for (auto cost : costs) {
        if (cost.first < min_cost) {
            min_cost = cost.first;
            min_cost_trajectory = cost.second;
        }
    }

    return min_cost_trajectory;
}

vector<string> Vehicle::successor_states() {
    // Provides the possible next states given the current state for the FSM 
    //   discussed in the course, with the exception that lane changes happen 
    //   instantaneously, so LCL and LCR can only transition back to KL.
    vector<string>  states;

    // Keep Lanes is always an option.
    states.push_back("KL");

    // If we're in Keep Lanes then we can Prepare for a Lane Change [Left|Right].
    if (this->state.compare("KL") == 0) {
        states.push_back("PLCL");
        states.push_back("PLCR");

    // If we're in Prepare Lane Change Left then we can stay there or Lane Change Left.
    } else if (this->state.compare("PLCL") == 0) {
        if (this->lane != lanes_available - 1) {
            states.push_back("PLCL");
            states.push_back("LCL");
        }

    // If we're in Prepare Lane Change right then we can stay there or Lane Change Right.
    } else if (this->state.compare("PLCR") == 0) {
        if (this->lane != 0) {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }
    
    // If state is "LCL" or "LCR", then just return "KL"
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state,
                                             map<int, vector<Vehicle>> &predictions) {
    // Given a possible next state, generate the appropriate trajectory to realize
    //   the next state.
    vector<Vehicle> trajectory;
    if (state.compare("CS") == 0) {
        trajectory = constant_speed_trajectory();
    } else if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, predictions);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, predictions);
    }

    return trajectory;
}

vector<double> Vehicle::get_kinematics(map<int, vector<Vehicle>> &predictions,
                                       int lane) {
    // Gets next timestep kinematics (position, velocity, acceleration) 
    //   for a given lane. Tries to choose the maximum velocity and acceleration, 
    //   given other vehicle positions and accel/velocity constraints.
    double  max_velocity_accel_limit = this->max_acceleration + this->v;
    double  new_position;
    double  new_velocity;
    double  new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
        if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
            // must travel at the speed of traffic, regardless of preferred buffer
            new_velocity = vehicle_ahead.v;
        } else {
            double  max_velocity_in_front =
                (vehicle_ahead.s - this->s - PREFERRED_BUFFER_FRONT)
                + vehicle_ahead.v - 0.5 * this->a;
            new_velocity = std::min(std::min(max_velocity_in_front, max_velocity_accel_limit), 
                                    this->target_speed);
        }
    } else {
        new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
    }
    
    new_accel    = new_velocity - this->v;  // Equation: (v_1 - v_0) / t = acceleration
    new_position = (double) (this->s + new_velocity + new_accel / 2.0);
    
    return {new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
    // Generate a constant speed trajectory.
    double  next_pos = position_at(1);
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s,  this->d, this->v, this->a, this->state), 
                                  Vehicle(this->lane, next_pos, this->d, this->v,       0, this->state)};
    return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> &predictions) {
    // Generate a keep lane trajectory.
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->d, this->v, this->a, state)};
    vector<double>  kinematics = get_kinematics(predictions, this->lane);
    double          new_s = kinematics[0];
    double          new_v = kinematics[1];
    double          new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s, this->d, new_v, new_a, "KL"));
        // Keep Lane trajectory  ^^^^^^^^^^         ^^^^^^^
    return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state,
                                                     map<int, vector<Vehicle>> &predictions) {
    // Generate a trajectory preparing for a lane change.
    double  new_s;
    double  new_v;
    double  new_a;
    int     new_lane = this->lane + lane_direction[state];

    Vehicle vehicle_behind;
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->d, this->v, this->a, this->state)};
    vector<double>  curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

    if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
        // Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];    
    } else {
        // Choose kinematics with lowest velocity.
        vector<double>  best_kinematics;
        vector<double>  next_lane_new_kinematics = get_kinematics(predictions, new_lane);
        best_kinematics = (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1])
                          ? next_lane_new_kinematics : curr_lane_new_kinematics;
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }

    trajectory.push_back(Vehicle(this->lane, new_s, this->d, new_v, new_a, state));
        //                       ^^^^^^^^^^         ^^^^^^^
        // Note that we are *not* actually changing lanes in this state!
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state,
                                                map<int, vector<Vehicle>> &predictions) {
    // Generate a lane change trajectory.
    vector<Vehicle> trajectory;
    int             new_lane = this->lane + lane_direction[state];

    // Check if a lane change is possible (check if another vehicle occupies 
    //   that spot).
    for (auto pred : predictions) {
        auto next_lane_vehicle = pred.second[0];
        if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane)
// check for a buffer here rather than current 's' equal to other vehicle 's' ???
            // If lane change is not possible, return empty trajectory.
            return trajectory;
    }

    trajectory.push_back(Vehicle(this->lane, this->s, this->d, this->v, this->a, this->state));

    vector<double>  kinematics = get_kinematics(predictions, new_lane);
    double  new_d = lane_to_d(new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], new_d, kinematics[1], kinematics[2], state));
        // Lane Change trajectory^^^^^^^^                 ^^^^^

    return trajectory;
}

void Vehicle::increment(int dt = 1) {
    this->s = position_at(dt);
}

double Vehicle::position_at(int t) {
    return (double) (this->s + this->v*t + this->a*t*t/2.0);
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> &predictions, 
                                 int lane, Vehicle &rVehicle) {
    // Returns a true if a vehicle is found behind the current vehicle, false 
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    double  max_s = -1;
    bool    found_vehicle = false;
    for (auto pred : predictions) {
        auto temp_vehicle = pred.second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
  
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, 
                                int lane, Vehicle &rVehicle) {
    // Returns a true if a vehicle is found ahead of the current vehicle, false 
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    double  min_s = this->goal_s;
    bool    found_vehicle = false;
    for (auto pred : predictions) {
        auto temp_vehicle = pred.second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
  
    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
    // Generates predictions for non-ego vehicles to be used in trajectory 
    //   generation for the ego vehicle.
    vector<Vehicle> predictions;
    for (int i = 0; i < horizon; ++i) {
        double  next_s = position_at(i);
        double  next_v = 0;
        if (i < horizon - 1)
            next_v = position_at(i + 1) - this->s;
        predictions.push_back(Vehicle(this->lane, next_s, this->d, next_v, 0));
            //                        ^^^^^^^^^^          ^^^^^^^
            // Assume we're staying in the same lane.
    }
  
    return predictions;
}

void Vehicle::realize_next_state(vector<Vehicle> &trajectory) {
    // Sets state and kinematics for ego vehicle using the last state of the trajectory.
    Vehicle next_state = trajectory[trajectory.size()-1];
    this->state = next_state.state;
    this->lane  = next_state.lane;
    this->s     = next_state.s;
    this->d     = next_state.d;
    this->v     = next_state.v;
    this->a     = next_state.a;
}

void Vehicle::configure(vector<double> &road_data) {
    // Called by simulator before simulation begins. Sets various parameters which
    //   will impact the ego vehicle.
    assert(road_data.size() == 5);
    target_speed     =       road_data[0];
    lanes_available  = (int) road_data[1];
    goal_s           =       road_data[2];
    goal_lane        = (int) road_data[3];
    max_acceleration =       road_data[4];
}
