
#include "JMT.h"

std::vector<double> JMT(std::vector<double> &start, std::vector<double> &end, double T) {
    /**
     * Calculate the Jerk Minimizing Trajectory that connects the initial state
     * to the final state in time T.
     *
     * @param start - the vehicles start location given as a length three array
     *   corresponding to initial values of [s, s_dot, s_double_dot]
     * @param end - the desired end state for vehicle. Like "start" this is a
     *   length three array.
     * @param T - The duration, in seconds, over which this maneuver should occur.
     *
     * @output an array of length 6, each value corresponding to a coefficent in 
     *   the polynomial:
     *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
     *
     * EXAMPLE
     *   > JMT([0, 10, 0], [10, 10, 0], 1)
     *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
     */

    // The first three coefficients are just (s_i, s_dot_i, 0.5 * s_double_dot_i)
    double  a0 = start[0];
    double  a1 = start[1];
    double  a2 = start[2] * 0.5;

    double      T2 = T * T;
    double      T3 = T * T2;
    double      T4 = T * T3;
    double      T5 = T * T4;

    MatrixXd    t_coeffs(3, 3);
    t_coeffs <<
            T3,      T4,      T5,
        3 * T2,  4 * T3,  5 * T4,
        6 * T , 12 * T2, 20 * T3;

    MatrixXd    delta(3, 1);
    delta <<
        end[0] - (start[0] + start[1] * T + 0.5 * start[2] * T2),
        end[1] - (start[1] + start[2] * T),
        end[2] - start[2];

    // Division is just multiplication by the inverse.
    MatrixXd    alpha = t_coeffs.inverse() * delta;

    double  a3 = alpha(0);
    double  a4 = alpha(1);
    double  a5 = alpha(2);

    return {a0, a1, a2, a3, a4, a5};
}
