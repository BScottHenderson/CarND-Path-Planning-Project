#ifndef CONSTANTS_H
#define CONSTANTS_H

// Constants related to the track.
constexpr auto MAX_S                        = 6945.554; // The max s value before wrapping around the track back to 0.
constexpr auto LANE_COUNT                   = 3;        // Three lanes in each direction.
                                                        // Lane numbers start at zero.
constexpr auto MAX_LANE                     = LANE_COUNT - 1;
constexpr auto LANE_WIDTH                   = 4.0;      // Lane width in meters.
                                                        // Half of the lane width in meters.
constexpr auto HALF_LANE_WIDTH              = LANE_WIDTH / 2.0;

// Speed limit in meters/second (50 mph ~= 22.353 m/s).
constexpr auto SPEED_LIMIT                  = 22.25;

// Max acceleration in meters/second/second.
constexpr auto MAX_ACCELERATION             = 0.075;

// Number of time steps for predictions.
constexpr auto PREDICTION_HORIZON           = 5;

// Preferred buffer distance (meters) between the ego vehicle and other vehicles.
constexpr auto PREFERRED_BUFFER_FRONT       = 6.0;      // Distance to a vehicle in front.
constexpr auto PREFERRED_BUFFER_LANE_CHANGE = 30.0;     // Distance to any vehicle in the target lane for lane changes.

// Distance (meters) along the path for the next point.
constexpr auto PATH_STEP                    = 30.0;

// Number of waypoints to add to the list of path points before using
// spline to smooth the trajectory.
constexpr auto WAYPOINTS_TO_ADD             = 3;

// Interval at which vehicle updates occur.
constexpr auto UPDATE_INTERVAL              = 0.02;     // Update interval in seconds (20ms == 0.02s).

// Number of points in a path for 1 second.
// With a 0.02s update interval this value should be 50.
constexpr auto PATH_LENGTH                  = 1.0 / UPDATE_INTERVAL;

// Number of points to keep from the previous path for each iteration.
// Use a larger number to maintain a smoother trajectory.
// Use a smaller value to allow the ego vehicle to repond more quickly
// to changing conditions.
constexpr auto PREVIOUS_POINTS_TO_KEEP      = 20;

#endif CONSTANTS_H
