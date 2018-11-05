
#ifndef TWX_ALGO_UTILS_H_
#define TWX_ALGO_UTILS_H_

#include "general_concepts.h"
#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace twx_algo
{
namespace utils
{
double Distance(double x1, double y1, double x2, double y2);

std::vector<Point> SimpleSmooth(const std::vector<Point> &data);

std::vector<double> Derivative(const std::vector<double> &coefs);

double CoefsApply(const std::vector<double> &coefs, double x);

double ComputeTheta(const std::vector<double> &coefs_x_d,
                    const std::vector<double> &coefs_y_d,
                    double t);
/**
 * '*_d' means 1-order derivative
 * '*_dd' means 2-order derivative
 * */
double ComputeKappa(const std::vector<double> &coefs_x_d,
                    const std::vector<double> &coefs_x_dd,
                    const std::vector<double> &coefs_y_d,
                    const std::vector<double> &coefs_y_dd,
                    double t);

PolynomialCoefs PolynomialFit(const std::vector<Point> &data, const int order);

} // namespace utils
} // namespace twx_algo
#endif // TWX_ALGO_UTILS_H_