#include "utils.h"

namespace twx_algo
{
namespace utils
{
double Distance(double x1, double y1, double x2, double y2)
{
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}
std::vector<Point> SimpleSmooth(const std::vector<Point> &data)
{
    std::vector<Point> smoothed_data;
    smoothed_data.resize(data.size());
    if (data.size() < 3)
    {
        return data;
    }
    for (unsigned int i = 2; i < data.size(); i++)
    {
        const auto &before = data[i - 2];
        const auto &mid = data[i - 1];
        const auto &after = data[i];
        if (i == 2)
        {
            smoothed_data[i - 2] = before;
        }
        smoothed_data[i - 1].x = (before.x + mid.x + after.x) / 3;
        smoothed_data[i - 1].y = (before.y + mid.y + after.y) / 3;
        smoothed_data[i - 1].theta = (before.theta + mid.theta + after.theta) / 3;
        if (i == data.size() - 1)
        {
            smoothed_data[i] = after;
        }
    }
    return smoothed_data;
}

std::vector<double> Derivative(const std::vector<double> &coefs)
{
    std::vector<double> d_coef;
    for (unsigned int i = 0; i < coefs.size(); i++)
    {
        double co = i * coefs[i];
        if (co != 0.0)
        {
            d_coef.push_back(co);
        }
    }
    return d_coef;
}

double CoefsApply(const std::vector<double> &coefs, double x)
{
    unsigned int num = coefs.size();
    double value = 0.0;
    for (unsigned int i = 0; i < coefs.size(); i++)
    {
        value += coefs[i] * std::pow(x, i);
    }
    return value;
}

double ComputeTheta(const std::vector<double> &coefs_x_d,
                    const std::vector<double> &coefs_y_d,
                    double t)
{
    double x_d_t = CoefsApply(coefs_x_d, t);
    double y_d_t = CoefsApply(coefs_y_d, t);
    return std::atan2(y_d_t, x_d_t);
}
double ComputeKappa(const std::vector<double> &coefs_x_d,
                    const std::vector<double> &coefs_x_dd,
                    const std::vector<double> &coefs_y_d,
                    const std::vector<double> &coefs_y_dd,
                    double t)
{
    double x_d_t = CoefsApply(coefs_x_d, t);
    double y_d_t = CoefsApply(coefs_y_d, t);
    double x_dd_t = CoefsApply(coefs_x_dd, t);
    double y_dd_t = CoefsApply(coefs_y_dd, t);
    double upper_part = std::fabs(x_d_t * y_dd_t - y_d_t * x_dd_t);
    double lower_part = std::pow(std::pow(x_d_t, 2) + std::pow(y_d_t, 2), 1.5);
    return upper_part / lower_part;
}
// A * x = b
// A is filled with x_values
// b is filled with y_values
// x is our target [a_0, a_1, a_2, a_3]'
// householderQr this can help us sovle the matrix
// solve means to compute x at min{|Ax - b|}
PolynomialCoefs PolynomialFit(const std::vector<Point> &data, const int order)
{

    // Pre-assumption check:
    Eigen::VectorXd x_values(data.size());
    Eigen::VectorXd y_values(data.size());
    for (unsigned int i = 0; i < data.size(); i++)
    {
        x_values(i) = data[i].x;
        y_values(i) = data[i].y;
    }
    assert(x_values.size() == y_values.size());
    assert(order >= 1 && order <= x_values.size() - 1);

    // Initialize polynomial matrix A:
    Eigen::MatrixXd A(x_values.size(), order + 1);

    // Build polynomial matrix A:
    for (int i = 0; i < x_values.size(); i++)
    {
        A(i, 0) = 1.0;
    }
    for (int j = 0; j < x_values.size(); j++)
    {
        for (int i = 0; i < order; i++)
        {
            A(j, i + 1) = A(j, i) * x_values(j);
        }
    }

    // Solve coefficients:
    auto Q = A.householderQr();
    auto result = Q.solve(y_values);
    std::vector<double> coefs;
    for (unsigned int i = 0; i < result.size(); i++)
    {
        coefs.push_back(result(i));
    }
    // std::cout << "test coefs size:" << coefs.size() << std::endl;
    return PolynomialCoefs{coefs, data.front().x, data.back().x};
}
} // namespace utils
} // namespace twx_algo