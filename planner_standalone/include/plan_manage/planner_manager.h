/**
 * @file planner_manager.h
 * @brief 规划器管理器类
 */

#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>
#include <memory>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>

namespace ego_planner
{

  // 规划器管理器参数
  struct PlannerManagerParams
  {
    // manager params
    double max_vel = 3.0;
    double max_acc = 3.0;
    double max_jerk = 4.0;
    double feasibility_tolerance = 0.05;
    double ctrl_pt_dist = 0.4;
    double planning_horizon = 7.5;

    // grid map params
    MappingParameters map_params;

    // optimizer params
    BsplineOptimizerParams opt_params;
  };

  // EGO Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void initPlanModules(const PlannerManagerParams &params);

    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;

  private:
    BsplineOptimizer::Ptr bspline_optimizer_rebound_;

    int continous_failures_count_{0};

    void updateTrajInfo(const UniformBspline &position_traj, const TimePoint time_now);

    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;
  };
} // namespace ego_planner

#endif
