#ifndef CONSTANTS_H
#define CONSTANTS_H

// Constants related to the track.
constexpr auto MAX_S                        = 6945.554; // The max s value before wrapping around the track back to 0.
constexpr auto LANE_COUNT                   = 3;        // Three lanes in each direction.
constexpr auto LANE_WIDTH                   = 4.0;      // Lane width in meters.
                                                        // Half of the lane width in meters.
constexpr auto HALF_LANE_WIDTH              = LANE_WIDTH / 2.0;

// Speed limit in meters/second (50 mph ~= 22.353 m/s).
constexpr auto SPEED_LIMIT                  = 22.25;

// Max acceleration in meters/second/second (0.1 m/s/s ~ 0.326 ft/s/s). 
constexpr auto MAX_ACCELERATION             = 0.1;

// Preferred buffer distance (meters) between the ego vehicle and other vehicles.
constexpr auto PREFERRED_BUFFER_FRONT       = 6.0;      // Distance to a vehicle in front.
constexpr auto PREFERRED_BUFFER_LANE_CHANGE = 30.0;     // Distance to any vehicle in the target lane for lane changes.

// Distance (meters) along the path for the next point.
constexpr auto PATH_STEP                    = 30.0;

// Number of points in a path for each iteration. (50 points at 20ms per point == 1s)
constexpr auto PATH_LENGTH                  = 50;

// Number of points to keep from the previous path for each iteration.
// Use a larger number to maintain a smoother trajectory.
// Use a smaller value to allow the ego vehicle to repond more quickly
// to changing conditions.
constexpr auto PREVIOUS_POINTS_TO_KEEP      = 35;

#endif CONSTANTS_H
