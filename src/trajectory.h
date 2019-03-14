#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include "vehicle.h"

void StraightLineTrajectory(double car_x, double car_y, double car_yaw,
                            std::vector<double>& next_x_vals, std::vector<double>& next_y_vals);
void CircleTrajectory(double car_x, double car_y, double car_yaw,
                      std::vector<double>& next_x_vals, std::vector<double>& next_y_vals);

void UpdateEgo(
    Vehicle& ego,
    std::vector<Vehicle>& traffic,
    std::vector<double> previous_path_x, std::vector<double> previous_path_y);

void PathPlannerTrajectory(
    Vehicle& ego,
    std::vector<double>& map_waypoints_x, std::vector<double>& map_waypoints_y, std::vector<double>& map_waypoints_s,
    std::vector<double>  previous_path_x, std::vector<double>  previous_path_y,
    std::vector<double>& next_x_vals,     std::vector<double>& next_y_vals);

#endif TRAJECTORY_H
