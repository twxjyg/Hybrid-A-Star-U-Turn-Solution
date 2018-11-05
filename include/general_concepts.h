#ifndef TWX_ALGO_GENERAL_CONCEPTS_H_
#define TWX_ALGO_GENERAL_CONCEPTS_H_

#include <vector>

namespace twx_algo
{
struct PolynomialCoefs
{
    std::vector<double> coefs;
    double start;
    double end;
};
struct Point
{
    double x;
    double y;
    double theta;
    double kappa;
};

struct MotionConfig
{
    // constant speed of ego car
    double desired_speed; // m/s
    // length from ego rear_axis to front_axis
    double wheelbase;
    // max steering angle
    double max_steering_angle;
    // min steering angle
    double min_steering_angle;
};

struct Lane
{
    std::vector<Point> center_line;
};

} // namespace twx_algo

#endif //TWX_ALGO_GENERAL_CONCEPTS_H_