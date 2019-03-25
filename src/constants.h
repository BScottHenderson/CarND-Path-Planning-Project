#ifndef CONSTANTS_H
#define CONSTANTS_H

// Constants related to the track.
constexpr auto MAX_S                    = 6945.554; // The max s value before wrapping around the track back to 0.
constexpr auto LANE_COUNT               = 3;        // Three lanes in each direction.
                                                        // Lane numbers start at zero.
constexpr auto MAX_LANE                 = LANE_COUNT - 1;
constexpr auto LANE_WIDTH               = 4.0;      // Lane width in meters.
                                                    // Half of the lane width in meters.
constexpr auto HALF_LANE_WIDTH          = LANE_WIDTH / 2.0;

// Interval at which vehicle updates occur.
constexpr auto UPDATE_INTERVAL          = 0.02;     // Update interval in seconds (20ms == 0.02s).

// Speed limit in meters/second (50 mph ~= 22.353 m/s).
constexpr auto SPEED_LIMIT              = 22.25;

// Max acceleration in meters/second/second.
constexpr auto MAX_ACCELERATION         = 0.1;

// Number of time steps for the prediction horizon.  A lane change takes roughly 3s to
// complete at lower speeds. Use this value to determine the max prediction horizon.
constexpr auto MIN_PREDICTION_DURATION  = 0.1;
constexpr auto MAX_PREDICTION_DURATION  = 3.0;
constexpr auto MIN_PREDICTION_HORIZON   = (int) (MIN_PREDICTION_DURATION / UPDATE_INTERVAL);
constexpr auto MAX_PREDICTION_HORIZON   = (int) (MAX_PREDICTION_DURATION / UPDATE_INTERVAL);

// Slope of a line representing a linear relationship between prediction horizon and speed.
constexpr auto PREDICTION_HORIZON_SLOPE = (MAX_PREDICTION_HORIZON - MIN_PREDICTION_HORIZON) / SPEED_LIMIT;

// Preferred buffer distance (meters) between the ego vehicle and other vehicles.
constexpr auto PREFERRED_BUFFER         = 30.0;     // Distance to any vehicle in the target lane for lane changes.

// Distance (meters) along the path for the next point.
constexpr auto PATH_STEP                = 30.0;

// Number of waypoints to add to the list of path points before using
// spline to smooth the trajectory.
constexpr auto WAYPOINTS_TO_ADD         = 3;

// Number of points in a path for 1 second.
// With a 0.02s update interval this value should be 50.
constexpr auto PATH_LENGTH              = (int) (1 / UPDATE_INTERVAL + 1);

// Number of points to keep from the previous path for each iteration.
// Use a larger number to maintain a smoother trajectory.
// Use a smaller value to allow the ego vehicle to respond more quickly
// to changing conditions.
constexpr auto PREVIOUS_POINTS_TO_KEEP  = 20;

#endif CONSTANTS_H
