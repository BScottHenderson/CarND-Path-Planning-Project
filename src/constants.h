#ifndef CONSTANTS_H
#define CONSTANTS_H

constexpr auto MAX_S                   = 6945.554;  // The max s value before wrapping around the track back to 0.
constexpr auto LANE_COUNT              = 3;         // Three lanes in each direction.
constexpr auto LANE_WIDTH              = 4.0;       // Lane width in meters.
                                                    // Half of the lane width in meters - make center lane calculation easier.
constexpr auto HALF_LANE_WIDTH         = LANE_WIDTH / 2.0;
constexpr auto PREVIOUS_POINTS_TO_KEEP = 10;        // Number of points to keep from previous path.
constexpr auto SPEED_LIMIT             = 22.25;     // Max velocity in meters/second.
                                                    // Speed limit == 50 mph ~= 22.353 m/s
constexpr auto MAX_ACCELERATION        = 2.0;       // Max acceleration at each time step - can be negative for braking.
constexpr auto PREFERRED_BUFFER        = 6.0;       // Preferred buffer between the ego vehicle and the vehicle in front.

#endif CONSTANTS_H
