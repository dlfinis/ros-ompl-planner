// Copyright 2016 Lorenzo Nardi

#ifndef ATTRACTOR_GUIDED_NAVIGATION_OMPL_GLOBAL_PLANNER_BASE_H_
#define ATTRACTOR_GUIDED_NAVIGATION_OMPL_GLOBAL_PLANNER_BASE_H_

#include <ros/ros.h>

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
// OMPL
// #include <attractor_guided_planner/AGPglobal.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/tools/config/MagicConstants.h>

#include <ompl_global_planner_base/PlannerConfig.h>

#include <string>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_global_planner_base {

class OmplGlobalPlannerBase : public nav_core::BaseGlobalPlanner {
 public:
  OmplGlobalPlannerBase();
  ~OmplGlobalPlannerBase();

  // plugin
  virtual void initialize(std::string name,
                          costmap_2d::Costmap2DROS *costmap_ros);
  virtual bool makePlan(const geometry_msgs::PoseStamped &start,
                        const geometry_msgs::PoseStamped &goal,
                        std::vector<geometry_msgs::PoseStamped> &plan);
  void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path);

 private:
  bool initialized_;

  costmap_2d::Costmap2DROS *costmap_ros_;
  base_local_planner::CostmapModel *costmap_model_;

  std::string tf_prefix_;
  boost::mutex mutex_;

  ob::StateSpacePtr ompl_state_space_;
  og::SimpleSetup *ompl_simple_setup_;
  ob::RealVectorBounds *bounds_;

  std::string frame_id_;
  std::string ompl_planner_;
  // int ompl_planner_;
  double collision_check_res_;
  double max_planning_time_, max_simplification_time_;
  std::string path_simplifier_;
  double map_resolution_;

  double range_;
  double border_fraction_;
  double goal_bias_;
  int max_nearest_neighbors_;
  double thr_similarity_;

  ros::Publisher global_plan_pub_, plan_waypoints_pub_;

  dynamic_reconfigure::Server<PlannerConfig> *dsrv_;

  void setupOmpl();

  void reconfigureCB(PlannerConfig &config, uint32_t level);

  void getXYThFromState(const ob::State *s, double *x, double *y, double *th);

  geometry_msgs::PoseStamped getPoseStampedFromState(const ob::State *s);

  bool isStateValid(const ob::SpaceInformation *simple_setup,
                    const ob::State *state);
};
}  // namespace ompl_global_planner_base

#endif  // ATTRACTOR_GUIDED_NAVIGATION_OMPL_GLOBAL_PLANNER_H_
