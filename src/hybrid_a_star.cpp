#include "hybrid_a_star.h"
#include <cmath>
#include <iostream>

namespace twx_algo
{
namespace hybrid_a_star
{
HybridAStarSolver::HybridAStarSolver(const GridInfo &map_info, const MotionConfig &motion_config) : map_info_(map_info),
                                                                                                    motion_config_(motion_config)
{
}

void HybridAStarSolver::SetSearchConfig(const SearchConfig &search_config)
{
    search_config_ = search_config;
}
SearchConfig HybridAStarSolver::GetSearchConfig()
{
    return search_config_;
}
int HybridAStarSolver::DiscretizePosition(double continuous_position)
{
    /*
  Returns the index into the grid for continuous position. So if x is 3.621, then this
  would return 3 to indicate that 3.621 corresponds to array index 3.
  */

    return (int)std::floor(continuous_position / map_info_.xy_resolution);
}
int HybridAStarSolver::DiscretizeTheta(double continuous_theta)
{
    /*
  Takes an angle (in radians) and returns which "stack" in the 3D configuration space
  this angle corresponds to.
  Angles near 0 go in the lower stacks while angles near 2 * pi go in the higher stacks.
  */

    double normalized_theta = std::fmod((continuous_theta + 2 * M_PI), (2 * M_PI));
    int discrete_theta = (int)(std::round(map_info_.theta_resolution * normalized_theta / (2 * M_PI))) % map_info_.theta_resolution;

    return discrete_theta;
}
double HybridAStarSolver::NormalizeTheta(double theta)
{
    return std::fmod((theta + 2 * M_PI), (2 * M_PI));
}

double HybridAStarSolver::Distance(double x1, double y1, double x2, double y2)
{
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}
double HybridAStarSolver::ManhattanDistance(double x1, double y1, double x2, double y2)
{
    return std::fabs(x1 - x2) + std::fabs(y1 - y2);
}
double HybridAStarSolver::GetHeuristicCost(const State &state, const GridPoint &discrete_goal)
{
    // double distance_cost = ManhattanDistance(state.x, state.y, discrete_goal.x, discrete_goal.y);
    double distance_cost = ManhattanDistance(DiscretizePosition(state.x),
                                    DiscretizePosition(state.y),
                                    discrete_goal.x, discrete_goal.y);
    double theta_cost = 0.0;
    double backward_distance = Distance(state.x, state.y, start_.x, start_.y);
    double frontward_distance = Distance(state.x, state.y, goal_.x, goal_.y);
    if (frontward_distance - backward_distance <=0)
    {
        theta_cost = (backward_distance/ frontward_distance) * std::fabs(NormalizeTheta(goal_.theta) - NormalizeTheta(state.theta));
        distance_cost *= frontward_distance/ backward_distance;
    }
    else
    {
        theta_cost =  (backward_distance/ frontward_distance) * std::fabs(NormalizeTheta(state.theta) - NormalizeTheta(theta_start_to_goal_));
    }
    double height_cost = frontward_distance + backward_distance - distance_start_to_goal_;

    return distance_cost + theta_cost + 0.2 * height_cost;
    // return 0.6 * distance_cost;
}

std::vector<State> HybridAStarSolver::Expand(
    const ExploreMap &explored,
    const State &current,
    const GridPoint &discrete_goal)
{

    double x = current.x;
    double y = current.y;
    double theta = current.theta;

    std::vector<State> next_states;
    for (double steering = motion_config_.min_steering_angle; steering <= motion_config_.max_steering_angle; steering += search_config_.steering_angle_step)
    {
        double delta = M_PI * steering / 180.0;
        double omega = motion_config_.desired_speed / motion_config_.wheelbase * std::tan(delta);

        double next_theta = theta + omega * search_config_.time_step;
        double next_x = x + motion_config_.desired_speed * std::cos(theta) * search_config_.time_step;
        double next_y = y + motion_config_.desired_speed * std::sin(theta) * search_config_.time_step;
        if (next_theta < 0.0 || next_theta > 2.0 * M_PI)
        {
            next_theta = std::fmod(next_theta + 2.0 * M_PI, 2.0 * M_PI);
        }

        GridPoint next_discrete_point;
        next_discrete_point.x = DiscretizePosition(next_x);
        next_discrete_point.y = DiscretizePosition(next_y);
        next_discrete_point.theta = DiscretizeTheta(next_theta);

        // already explored:

        if (explored[next_discrete_point.theta].count(next_discrete_point) &&
            explored[next_discrete_point.theta].at(next_discrete_point) == ExploreState::EXPLORED)
        {
            continue;
        }

        State next_state;
        next_state.x = next_x;
        next_state.y = next_y;
        next_state.theta = next_theta;
        next_state.actual_cost = current.actual_cost + search_config_.time_step;
        next_state.estimated_cost = next_state.actual_cost + GetHeuristicCost(next_state, discrete_goal);

        next_states.push_back(next_state);
    }

    return next_states;
}
std::vector<State> HybridAStarSolver::GetSteps(
    const StateMap &parent,
    const Point &start,
    State final)
{
    std::vector<State> path;

    State current = final;

    int x_discrete;
    int y_discrete;
    int theta_discrete;

    while (current.x != start.x || current.y != start.y)
    {
        path.push_back(current);

        GridPoint discrete_point;
        discrete_point.x = DiscretizePosition(current.x);
        discrete_point.y = DiscretizePosition(current.y);
        discrete_point.theta = DiscretizeTheta(current.theta);
        if (parent[discrete_point.theta].count(discrete_point))
        {
            current = parent[discrete_point.theta].at(discrete_point);
        }
    }

    return path;
}

std::vector<State> HybridAStarSolver::Solve(const Point &start, const Point &goal, bool *goal_reached)
{
    start_ = start;
    goal_ = goal;
    theta_start_to_goal_ = std::atan2(goal.y - start.y, goal.x - start.x);
    distance_start_to_goal_ = Distance(start.x, start.y, goal.x, goal.y);
    // explored configuration space
    ExploreMap explored;
    explored.resize(map_info_.theta_resolution);
    // parent state for reconstruction
    StateMap parent;
    parent.resize(map_info_.theta_resolution);

    // frontier
    StateHeap frontier;
    State init;
    init.x = start.x;
    init.y = start.y;
    init.theta = start.theta;
    init.actual_cost = 0.0;
    init.estimated_cost = 0.0;

    GridPoint goal_discrete_point;
    goal_discrete_point.x = DiscretizePosition(goal.x);
    goal_discrete_point.y = DiscretizePosition(goal.y);
    goal_discrete_point.theta = DiscretizeTheta(goal.theta);

    GridPoint init_discrete_point;
    init_discrete_point.x = DiscretizePosition(init.x);
    init_discrete_point.y = DiscretizePosition(init.y);
    init_discrete_point.theta = DiscretizeTheta(init.theta);

    frontier.AddState(init);

    explored[init_discrete_point.theta][init_discrete_point] = ExploreState::ON_FRONTIER;
    parent[init_discrete_point.theta][init_discrete_point] = init;

    State current = init;
    bool finished = false;

    while (!frontier.Empty())
    {
        current = frontier.GetState();

        GridPoint current_discrete_point;
        current_discrete_point.x = DiscretizePosition(current.x);
        current_discrete_point.y = DiscretizePosition(current.y);
        current_discrete_point.theta = DiscretizeTheta(current.theta);

        // update skipping stats:
        if (ExploreState::EXPLORED == explored[current_discrete_point.theta][current_discrete_point])
        {
            counter_.num_skipped += 1;
            continue;
        }
        explored[current_discrete_point.theta][current_discrete_point] = ExploreState::EXPLORED;
        counter_.num_explored += 1;

        // termination test:
        if (current_discrete_point.x == goal_discrete_point.x && current_discrete_point.y == goal_discrete_point.y)
        {
            finished = true;
            break;
        }

        // expand frontier
        for (const State &next : Expand(explored, current, goal_discrete_point))
        {
            frontier.AddState(next);
            GridPoint next_discrete_point;
            next_discrete_point.x = DiscretizePosition(next.x);
            next_discrete_point.y = DiscretizePosition(next.y);
            next_discrete_point.theta = DiscretizeTheta(next.theta);

            explored[next_discrete_point.theta][next_discrete_point] = ExploreState::ON_FRONTIER;
            parent[next_discrete_point.theta][next_discrete_point] = current;
        }

        if (frontier.Size() > counter_.num_frontier)
        {
            counter_.num_frontier = frontier.Size();
        }
    }
    if (finished)
    {
        std::cout << "found path to goal in: " << counter_.num_explored << "/" << counter_.num_skipped << "/" << counter_.num_frontier << " (Explored/Skipped/OnFrontier)"
                  << " expansions" << std::endl;
    }
    else
    {
        std::cerr << "no valid path." << std::endl;
    }
    *goal_reached = finished;
    return GetSteps(parent, start, current);
}
} // namespace hybrid_a_star
} // namespace twx_algo