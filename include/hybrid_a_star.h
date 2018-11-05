#ifndef TWX_ALGO_HYBRID_A_STAR_HYBRID_A_STAR_H_
#define TWX_ALGO_HYBRID_A_STAR_HYBRID_A_STAR_H_

#include "general_concepts.h"
#include <queue>
#include <unordered_map>
#include <functional>

namespace twx_algo
{
namespace hybrid_a_star
{
struct GridInfo
{
    double xy_resolution;
    int theta_resolution;
};

struct GridPoint
{
    int x;
    int y;
    int theta;
    bool operator==(const GridPoint &other) const
    {
        return (x == other.x && y == other.y);
    }
};

struct GridPointHasher
{
    // return ((hash<float>()(k.getM()) ^ (hash<float>()(k.getC()) << 1)) >> 1);
    std::size_t operator()(const GridPoint &k) const
    {
        return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1);
    }
};

struct State
{
    double x;
    double y;
    double theta;
    double actual_cost;
    double estimated_cost;
};
struct SearchConfig
{
    double steering_angle_step;
    double time_step;
};
class StateHeap
{
  public:
    bool Empty()
    {
        return pq.empty();
    }

    size_t Size()
    {
        return pq.size();
    }

    void AddState(const State &state)
    {
        pq.push(state);
    }

    State GetState()
    {
        State state = pq.top();
        pq.pop();

        return state;
    }

  private:
    class StateComparator
    {
      public:
        bool operator()(const State &x, const State &y)
        {
            return x.estimated_cost > y.estimated_cost;
        }
    };

    std::priority_queue<State, std::vector<State>, StateComparator> pq;
};

class HybridAStarSolver
{
  private:
    enum class ExploreState
    {
        UNEXPLORED = 0,
        ON_FRONTIER = 1,
        EXPLORED = 2
    };

    struct Counter
    {
        size_t num_explored = 0;
        size_t num_skipped = 0;
        size_t num_frontier = 0;
    };

  public:
    using ExploreMap = std::vector<std::unordered_map<GridPoint, ExploreState, GridPointHasher>>;

    using StateMap = std::vector<std::unordered_map<GridPoint, State, GridPointHasher>>;

    HybridAStarSolver(const GridInfo &map_info, const MotionConfig &motion_config);

    void SetSearchConfig(const SearchConfig &search_config);

    std::vector<State> Solve(const Point &start, const Point &goal, bool *goal_reached);

    SearchConfig GetSearchConfig();

  private:
    double ManhattanDistance(double x1, double y1, double x2, double y2);

    double Distance(double x1, double y1, double x2, double y2);

    double NormalizeTheta(double theta);

    int DiscretizePosition(double continuous_position);

    int DiscretizeTheta(double continuous_theta);

    double GetHeuristicCost(const State &state, const GridPoint &discrete_goal);

    std::vector<State> Expand(
        const ExploreMap &explored,
        const State &current,
        const GridPoint &discrete_goal);

    std::vector<State> GetSteps(
        const StateMap &parent,
        const Point &start,
        State final);

  private:
    GridInfo map_info_;
    MotionConfig motion_config_;
    SearchConfig search_config_;
    Counter counter_;
    Point start_;
    Point goal_;
    double distance_start_to_goal_;
    double theta_start_to_goal_;
};
} // namespace hybrid_a_star
} // namespace twx_algo

#endif //TWX_ALGO_HYBRID_A_STAR_HYBRID_A_STAR_H_