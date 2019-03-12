#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>

#include "constants.h"

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
std::string hasData(std::string s) {
    auto    found_null = s.find("null");
    auto    b1 = s.find_first_of("[");
    auto    b2 = s.find_first_of("}");
    if (found_null != std::string::npos) {
        return "";
    } else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
    double  dx = x2 - x1;
    double  dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

// Convert between lane number and Frenet d coordinate.
double  lane_to_d(int lane) {
    return lane * LANE_WIDTH + HALF_LANE_WIDTH;
}
int     d_to_lane(double d) {
    int     lane = (int) round((d - HALF_LANE_WIDTH) / LANE_WIDTH);
    if (lane >= LANE_COUNT)
        lane = -1;
    return lane;
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y,
                    const std::vector<double> &maps_x, 
                    const std::vector<double> &maps_y) {
    double  closestLen = std::numeric_limits<double>::max();
    int     closestWaypoint = 0;

    for (int i = 0; i < (int) maps_x.size(); ++i) {
        double  map_x = maps_x[i];
        double  map_y = maps_y[i];
        double  dist = distance(x, y, map_x, map_y);
        if (dist < closestLen) {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta,
                 const std::vector<double> &maps_x, 
                 const std::vector<double> &maps_y) {
    int     closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);
    double  map_x = maps_x[closestWaypoint];
    double  map_y = maps_y[closestWaypoint];

    double  heading = atan2(map_y - y, map_x - x);

    double  angle = fabs(theta - heading);
    angle = std::min(2*pi() - angle, angle);
    if (angle > pi()/2) {
        ++closestWaypoint;
        if (closestWaypoint == maps_x.size())
            closestWaypoint = 0;
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta, 
                              const std::vector<double> &maps_x, 
                              const std::vector<double> &maps_y) {
    int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);
    int prev_wp = next_wp - 1;
    if (next_wp == 0)
        prev_wp  = (int) maps_x.size() - 1;

    double  n_x = maps_x[next_wp] - maps_x[prev_wp];
    double  n_y = maps_y[next_wp] - maps_y[prev_wp];
    double  x_x = x - maps_x[prev_wp];
    double  x_y = y - maps_y[prev_wp];

    // Find the projection of 'x' onto 'n'.
    double  proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double  proj_x = proj_norm * n_x;
    double  proj_y = proj_norm * n_y;

    double  frenet_d = distance(x_x, x_y, proj_x, proj_y);

    // Determine if 'd' value is positive or negative by comparing it to a center point.
    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);
    if (centerToPos <= centerToRef)
        frenet_d = -frenet_d;

    // Calculate 's' value.
    double  frenet_s = 0;
        // Travel along the road until we get to 'prev_wp'.
    for (int i = 0; i < prev_wp; ++i)
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);
        // Add the perpendicular distance from the center of the road to
        // the actual (x, y) location.
    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d,
                          const std::vector<double> &maps_s, 
                          const std::vector<double> &maps_x, 
                          const std::vector<double> &maps_y) {
    // Find the waypoint that is before 's'.
    int     prev_wp = -1;
    while (s > maps_s[prev_wp + 1] && prev_wp < (int) maps_s.size() - 1)
        ++prev_wp;

    // We know 'prev_sp' is before 's' so get the next waypoint after that.
    // So that 's' is between 'prev_wp' and 'next_wp'. This will allow us
    // to determine the heading.
    int     next_wp = (prev_wp + 1) % maps_x.size();

    // Calcuate the heading between 'prev_sp' and 'next_wp'.
    double  heading = atan2(maps_y[next_wp] - maps_y[prev_wp], maps_x[next_wp] - maps_x[prev_wp]);

    // Distance from 'prev_wp' to 's'.
    double  seg_s = (s - maps_s[prev_wp]);

    // Calculate (x, y) along the segment that contains 's'.
    double  seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double  seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    // Adjust the heading from 'prev_wp' to 'next_wp' so that it is
    // perpendicular to that line.
    double  perp_heading = heading - pi()/2;

    // Translate (x, y) along the segment to actual (x, y) by
    // taking traveling along the perpendicular heading by a
    // distance of 'd'.
    double  x = seg_x + d * cos(perp_heading);
    double  y = seg_y + d * sin(perp_heading);

    return {x, y};
}
