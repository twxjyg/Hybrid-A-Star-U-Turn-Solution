## Problem
At urban scenario, we often need do U-Turn, this repo aims to design a trajectory generator, it can generate a trajectory with each point have position and smooth curve, and this trajectory can be used by car to do a U-Turn

Input：Two lane in a U-Turn scenario, each lane contains center line, and each center line contains sequence of pose{x,y,theta,kappa}


Output：A trajectory can connect two lane


## Solution：

Use A-Star to find drivable trajectory. Need notice that when our space is descreted, the path comes out from A-Star is not drivable, because it's not smooth and doesn't concern car motion model, for example car's maximum turn radius. For solve that issue, the Hybrid-A-Star solution comes out. In Hybrid-A-Star solution, we count car's motion model into search algorithm, make sure each step of search(exploration) is under constraint of motion model.

Rough compute process is:

1. Use motion model as a constraint to compute the next reachable position(Grid) in A-Star
    1. when we use motion model，HeuristicCost also need optimized according to U-turn scenario
2. Delete not drivable position(Grid)（Obstacle, Out of map,...）
3. Each reachable position we get corresponding descreted grid position and continuity position, and we stitch them together
4. After we finished searching, we get all continuity position（x，y，theta）
5. Check whether the (x,y,theta) meet the smooth condition(mainly curve smoothing)
6. Some visualization
    1. python module:matplotlib
7. About the test, I didn't bring in heavy test framework such as gtest，boost test and so on，I use shell scripts and c++ assert as a light weight solution， because when we were developing, we need execute the program again and again to confirm out code is work, and at the same time we need visualization, so I try to combine visualization and debugging together to help myself to speed up my developing. And use C++ to write test is harder to do visualization, and each changes in visualization all need re-compile.

### How to use my code:

1. Build：

    ```shell
    cd /path/to/U-Turn-Solution
    mkdir build
    cd build
    cmake ..
    make
    cd ..
    ```

2. Run test and display result：

    ```shell
    # 测试单个数据并可视化
    ./test_and_viz_one.sh ./test_data/lane_data_1.txt
    # 批量测试所有数据集
    ./test_all_and_quiet.sh
    ```
    **test data lane_data.3.txt is a very wired test data, its not a uturn scenario, so the result seems not very good**

3. About inner parameters：

    I use command-line args(1~7 parameters you need care about)：

    ```c++
        // load planner config from command line
        twx_algo::MotionConfig motion_config = {
            argc > 1 && argv[1] != NULL ? std::stod(argv[1]) : 1.0,  // desired_speed or A-Star simulation speed
            argc > 2 && argv[2] != NULL ? std::stod(argv[2]) : 4.5,  // ego car wheelbase
            argc > 3 && argv[3] != NULL ? std::stod(argv[3]) : 40.0, // max steering angle
            argc > 4 && argv[4] != NULL ? std::stod(argv[4]) : -40}; // min steering angle

        twx_algo::hybrid_a_star::SearchConfig search_config = {
            argc > 5 && argv[5] != NULL ? std::stod(argv[5]) : 20.0, // explore steering angle step
            argc > 6 && argv[6] != NULL ? std::stod(argv[6]) : 1.0}; // time duration each exploration
    ```

    the 7th parameter is the environment data, the two lanes of a U-Turn：

    ```c++
        if (argc > 7 && argv[7] != NULL)
        {
            lane_data_file_name = argv[7];
            std::ifstream lane_stream(argv[7], std::ifstream::in);
            ...
        }
    ```

    this is a example when I try to manually launch the binary：

    ```shell
        ./build/u_turn_planner_test 1.0 3 40 -40 40 1.0 ./test_data/lane_data.txt
    ```

4. Project folder:

```text
./test //entry file
./test_data //test data and result images

```

### Some key point：

1. Why called Hybrid-A-Star，‘Hybrid‘ means the path searching process is under constraint of motion model. That futher means in grid map, each exploration step, we can only reach the position where the car can reach.

2. I didn't use traditional 3-d array in A-Star to descrete position, I designed a more elegant way:

    ```c++
    std::vector<std::map<Point, State>>
    ```

    this way is like a sparse matrix， it's not space consuming as 3-d array and we don't need do position to index conversion with this structure. Only one thing we need implement is the Point comparator and hasher.
3. Car's motion model：
    ![Motion model](motion.jpeg)
    Code：
    ```c++
        double delta = M_PI * steering / 180.0;
        double omega = motion_config_.desired_speed / motion_config_.wheelbase * std::tan(delta);

        double next_theta = theta + omega * search_config_.time_step;
        double next_x = x + motion_config_.desired_speed * std::cos(theta) * search_config_.time_step;
        double next_y = y + motion_config_.desired_speed * std::sin(theta) * search_config_.time_step;
    ```
4. Final trajectory need to be re-sample，for more smooth and reach the condition of compute kappa
    I compute kappa by this fomular：

    ![kappa computation](kappa_calculation.png)


## Other possible solution 1:

Generate half circle curve as reference line, then use some control method to let the car follow the reference line.
Weakness is the half circle may not possible for a cat to drive.

## Other possible solution 2：

Use target lane entrance as the end-configuration space，then use Jerk minimal optimization method to compute the trajectory，then check the validation of the trajectory. This method also didn't concern motion model, so car may not follow the trajectory perfectly, this is the basic code for the Jerk minimal optimization：

```c++
/**
 * jerk-minimized trajectory solver
 * @param start Init configuration
 * @param end Final configuration
 * @return JMT params
 */ 
TrajectoryParams generate_jerk_minimized_trajectory(
    const TrajectoryReference &start, 
    const TrajectoryReference &end
) {
    TrajectoryParams trajectory_params;

    trajectory_params.T = end.timestamp - start.timestamp;

    // rectify round trip behavior:
    double rectified_start_s = start.s;
    double rectified_end_s = end.s;
    if (rectified_start_s > rectified_end_s) {
        rectified_start_s -= 6945.554;
    }

    trajectory_params.s = solve_jerk_minimized_trajectory(
        rectified_start_s, start.vs, start.as,
          rectified_end_s,   end.vs,   end.as,
        trajectory_params.T
    );
    trajectory_params.d = solve_jerk_minimized_trajectory(
        start.d, start.vd, start.ad,
          end.d,   end.vd,   end.ad,
        trajectory_params.T
    );
    
    return trajectory_params;
}
std::vector<double> solve_jerk_minimized_trajectory(
    double x0, double vx0, double ax0,
    double x1, double vx1, double ax1,
    double T
) {
    // pre-computed exponentials of T:
    std::vector<double> T_exp{1.0, T, pow(T, 2.0), pow(T, 3.0),pow(T, 4.0), pow(T, 5.0)};
    
    // system matrix:
    Eigen::MatrixXd A(3, 3); 
    A <<   T_exp[3],    T_exp[4],    T_exp[5], 
         3*T_exp[2],  4*T_exp[3],  5*T_exp[4], 
         6*T_exp[1], 12*T_exp[2], 20*T_exp[3];
    
    // desired output:
    Eigen::VectorXd b(3);

    b <<  x1 - (x0 + vx0*T_exp[1] + 0.5*ax0*T_exp[2]),
         vx1 - (vx0 + ax0*T_exp[1]),
         ax1 - ax0;
    
    // polynomial coefficients:
    Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);
   
    return {x0, vx0, 0.5*ax0, coeffs(0), coeffs(1), coeffs(2)};
}
```



