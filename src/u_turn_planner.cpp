#include "u_turn_planner.h"
#include "utils.h"
#include <algorithm>
#include <fstream>

namespace twx_algo
{
UTurnPlanner::UTurnPlanner(const MotionConfig &motion_config) : motion_config_(motion_config),
                                                                a_star_solver_({motion_config.desired_speed, 40}, motion_config) {}
void UTurnPlanner::SetEnvironment(const Lane &start_lane, const Lane &end_lane)
{
    start_lane_ = start_lane;
    start_point_ = start_lane.center_line.back();
    end_lane_ = end_lane;
    end_point_ = end_lane.center_line.front();
    a_star_grid_.xy_resolution = motion_config_.desired_speed;
}
void UTurnPlanner::SetHybridAStarConfig(const hybrid_a_star::SearchConfig &search_config)
{
    a_star_solver_.SetSearchConfig(search_config);
}
std::vector<Point> UTurnPlanner::Plan(std::vector<twx_algo::Point> *a_star_output, bool* goal_reached)
{
    std::vector<hybrid_a_star::State> a_star_state_traj = a_star_solver_.Solve(start_point_, end_point_, goal_reached);
    std::reverse(a_star_state_traj.begin(), a_star_state_traj.end());
    std::vector<Point> a_star_traj;
    for (const auto &state : a_star_state_traj)
    {
        a_star_traj.push_back({state.x, state.y, state.theta});
    }
    // insert start_point_ as first point of traj
    a_star_traj.insert(a_star_traj.begin(), {start_point_.x, start_point_.y, start_point_.theta, start_point_.kappa});

    // re-present traj by time, X=f1(t), Y=f2(t)
    std::vector<Point> a_star_traj_x_t;
    std::vector<Point> a_star_traj_y_t;
    a_star_traj_x_t.resize(a_star_traj.size());
    a_star_traj_y_t.resize(a_star_traj.size());
    const auto search_config = a_star_solver_.GetSearchConfig();
    for (unsigned int i = 0; i < a_star_traj.size(); i++)
    {
        a_star_traj_x_t[i].x = i * search_config.time_step;
        a_star_traj_x_t[i].y = a_star_traj[i].x;
        a_star_traj_y_t[i].x = i * search_config.time_step;
        a_star_traj_y_t[i].y = a_star_traj[i].y;
    }
    *a_star_output = a_star_traj;
    // fit X and Y
    PolynomialCoefs coefs_x = utils::PolynomialFit(a_star_traj_x_t, 4);
    PolynomialCoefs coefs_y = utils::PolynomialFit(a_star_traj_y_t, 4);

    // pre-compute derivative for theta and kappa compute
    std::vector<double> coefs_x_d = utils::Derivative(coefs_x.coefs);
    std::vector<double> coefs_x_dd = utils::Derivative(coefs_x_d);
    std::vector<double> coefs_y_d = utils::Derivative(coefs_y.coefs);
    std::vector<double> coefs_y_dd = utils::Derivative(coefs_y_d);

    // re-sample fitted traj
    std::vector<Point> smoothed_trajectory;
    smoothed_trajectory.resize(a_star_traj.size());
    for (unsigned int i = 0; i < a_star_traj.size(); i++)
    {
        smoothed_trajectory[i].x = utils::CoefsApply(coefs_x.coefs, i * search_config.time_step);
        smoothed_trajectory[i].y = utils::CoefsApply(coefs_y.coefs, i * search_config.time_step);
        // TODO: re-compute theta
        smoothed_trajectory[i].theta = utils::ComputeTheta(coefs_x_d,
                                                           coefs_y_d,
                                                           i * search_config.time_step);
        smoothed_trajectory[i].kappa = utils::ComputeKappa(coefs_x_d,
                                                           coefs_x_dd,
                                                           coefs_y_d,
                                                           coefs_y_dd,
                                                           i * search_config.time_step);
    }
    return smoothed_trajectory;
}
} // namespace twx_algo