#ifndef JMT_H
#define JMT_H

#include <vector>

#include "Eigen-3.3/Eigen/Dense"
using Eigen::MatrixXd;

std::vector<double> JMT(std::vector<double> &start, std::vector<double> &end, double T);

#endif JMT_H
