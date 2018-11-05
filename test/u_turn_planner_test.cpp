#include "u_turn_planner.h"
#include "general_concepts.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <string>
#include <chrono>
#include <assert.h>
#include "hybrid_a_star.h"
#include "utils.h"

const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");

std::vector<std::string> Split(const std::string &s, char delim)
{
    std::stringstream ss(s);
    std::string item;
    std::vector<std::string> tokens;
    while (std::getline(ss, item, delim))
    {
        tokens.push_back(item);
    }
    return tokens;
}
int main(int argc, char const *argv[])
{

    // load planner config from command line
    twx_algo::MotionConfig motion_config = {
        argc > 1 && argv[1] != NULL ? std::stod(argv[1]) : 1.0,  // desired_speed or a-star simulation speed
        argc > 2 && argv[2] != NULL ? std::stod(argv[2]) : 4.5,  // ego car wheelbase
        argc > 3 && argv[3] != NULL ? std::stod(argv[3]) : 40.0, // max steering angle
        argc > 4 && argv[4] != NULL ? std::stod(argv[4]) : -40}; // min steering angle

    twx_algo::hybrid_a_star::SearchConfig search_config = {
        argc > 5 && argv[5] != NULL ? std::stod(argv[5]) : 20.0, // explore steering angle step
        argc > 6 && argv[6] != NULL ? std::stod(argv[6]) : 1.0}; // time duration each exploration

    // load lane env from file
    twx_algo::Lane start_lane;
    twx_algo::Lane end_lane;
    std::string lane_data_file_name;

    if (argc > 7 && argv[7] != NULL)
    {
        lane_data_file_name = argv[7];
        std::ifstream lane_stream(argv[7], std::ifstream::in);
        std::string lane_string;
        std::getline(lane_stream, lane_string);
        std::vector<std::string>
            lane_data = Split(lane_string, ',');
        if (lane_data.size() != 3)
        {
            exit(1);
        }
        start_lane.center_line = {
            {std::stod(lane_data[0]), std::stod(lane_data[1]), M_PI * std::stod(lane_data[2])}};
        std::getline(lane_stream, lane_string);
        lane_data = Split(lane_string, ',');
        if (lane_data.size() != 3)
        {
            exit(1);
        }
        end_lane.center_line = {
            {std::stod(lane_data[0]), std::stod(lane_data[1]), M_PI * std::stod(lane_data[2])}};
        lane_stream.close();
    }
    else
    {
        start_lane.center_line = {{0.0, 0.0, M_PI / 2}};
        end_lane.center_line = {{8, 0.0, -M_PI / 2}};
    }

    // init planner & do planning
    twx_algo::UTurnPlanner u_turn_planner(motion_config);

    u_turn_planner.SetEnvironment(start_lane, end_lane);

    u_turn_planner.SetHybridAStarConfig(search_config);

    std::vector<twx_algo::Point> a_star_trajectory;

    std::chrono::system_clock::system_clock::time_point start_time =
        std::chrono::system_clock::system_clock::now();
    bool goal_reached = false;
    std::vector<twx_algo::Point> trajectory = u_turn_planner.Plan(&a_star_trajectory, &goal_reached);
    std::chrono::system_clock::system_clock::time_point end_time =
        std::chrono::system_clock::system_clock::now();
    auto time_spent = end_time - start_time;
    std::uint64_t milliseconds_time = std::chrono::duration_cast<std::chrono::milliseconds>(time_spent).count();
    std::cout << red << "time spent:"
              << milliseconds_time << std::endl;
    std::cout << reset << std::endl;
    if (!goal_reached)
    {
        std::cout << red << "can not reach target goal, may be you need move a little bit" << std::endl;
    }

    // dump output
    std::ofstream a_star_out_stream(lane_data_file_name.empty() ? "trajectory.txt.astar" : lane_data_file_name + ".traj.astar", std::fstream::trunc);
    for (unsigned int i = 0; i < a_star_trajectory.size(); i++)
    {
        a_star_out_stream << a_star_trajectory[i].x << "," << a_star_trajectory[i].y << "," << a_star_trajectory[i].theta << "," << a_star_trajectory[i].kappa << std::endl;
    }
    a_star_out_stream.close();

    std::ofstream out_stream(lane_data_file_name.empty() ? "trajectory.txt" : lane_data_file_name + ".traj",
                             std::ofstream::trunc);
    double trajectory_length = 0.0;
    auto first_point = trajectory.front();
    for (const auto &point : trajectory)
    {
        out_stream << point.x << "," << point.y << "," << point.theta << "," << point.kappa << std::endl;
        trajectory_length += twx_algo::utils::Distance(point.x, point.y, first_point.x, first_point.y);
        first_point = point;
    }
    out_stream.close();

    std::cout << green << "-------------------Result Test------------------------" << reset << std::endl;
    std::cout << yellow << "Values:" << std::endl
              << "R == Distance(start, goal) / 2" << reset << std::endl;

    // 1. goal reaching test
    std::cout << "TEST:"
              << "goal must reached" << std::endl;
    auto const goal = end_lane.center_line.front();
    auto const traject_goal = trajectory.back();
    assert(goal.x - traject_goal.x < motion_config.desired_speed);
    assert(goal.y - traject_goal.y < motion_config.desired_speed);
    std::cout << green << "PASSED:"
              << "x_error:" << goal.x - traject_goal.x << " y_error:" << goal.y - traject_goal.y << reset << std::endl;

    double R = twx_algo::utils::Distance(start_lane.center_line.back().x, start_lane.center_line.back().y, goal.x, goal.y) / 2;
    std::cout << "TEST:"
              << "traject path must within 2*PI * R*R" << std::endl;
    assert(trajectory_length < 2 * M_PI * R);
    std::cout << green << "PASSED:"
              << "length_error:" << trajectory_length - 2 * M_PI * R << reset << std::endl;

    std::cout << "TEST:"
              << "trajectory length must bigger than 2*R" << std::endl;
    assert(trajectory_length > 2 * R);
    std::cout << green << "PASSED:"
              << "length_error:" << trajectory_length - 2 * R << reset << std::endl;

    std::cout << "TEST:"
              << "a-star traject size must == smoothed trajectory size" << std::endl;
    assert(trajectory.size() == a_star_trajectory.size());
    std::cout << green << "PASSED:"
              << "trajectory size :" << trajectory.size() << reset << std::endl;

    std::cout << "TEST:"
              << "time spent must within 1 seconds" << std::endl;
    assert(milliseconds_time < 1000);
    std::cout << green << "PASSED:"
              << "time spent:" << milliseconds_time << " milliseconds" << reset << std::endl;

    std::cout << "TEST:"
              << "start theta_error must within PI/2" << std::endl;
    assert(std::fabs(trajectory.front().theta - start_lane.center_line.back().theta) < M_PI / 2);
    std::cout << green << "PASSED:"
              << "start theta error:" << trajectory.front().theta - start_lane.center_line.back().theta << reset << std::endl;

    std::cout << "TEST:"
              << "end theta_error must within PI/2" << std::endl;
    assert(std::fabs(trajectory.back().theta - goal.theta) < M_PI / 2);
    std::cout << green << "PASSED:"
              << "end theta error:" << trajectory.back().theta - goal.theta << reset << std::endl;


    std::cout << green << "----------------------All Test Passed------------------------" << reset << std::endl;

    return 0;
}
