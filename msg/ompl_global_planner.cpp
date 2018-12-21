// Copyright 2016 Lorenzo Nardi

#include <ompl_global_planner_base.h>
#include <pluginlib/class_list_macros.h>

// register as a base global planner plugin
PLUGINLIB_EXPORT_CLASS(ompl_global_planner_base::OmplGlobalPlannerBase,
                       nav_core::BaseGlobalPlanner)

namespace ompl_global_planner_base {

OmplGlobalPlannerBase::OmplGlobalPlannerBase()
    : costmap_ros_(NULL),
      initialized_(false),
      ompl_state_space_(new ob::SE2StateSpace()),
      costmap_model_(NULL),
      dsrv_(NULL) {}

OmplGlobalPlannerBase::~OmplGlobalPlannerBase() {
  if (dsrv_) delete dsrv_;
}

void OmplGlobalPlannerBase::initialize(std::string name,
                                   costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    ROS_WARN("OmplGlobalPlannerBase has already been initialized");
    return;
  }
  ros::NodeHandle private_nh("~/" + name);
  costmap_ros_ = costmap_ros;
  costmap_model_ =
      new base_local_planner::CostmapModel(*costmap_ros_->getCostmap());
  // publishers
  global_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
  plan_waypoints_pub_ =
      private_nh.advertise<geometry_msgs::PoseArray>("waypoints", 1);
  // params
  private_nh.param("/move_base/global_costmap/resolution", map_resolution_,
                   0.05);
  // set bounds
  bounds_ = new ob::RealVectorBounds(2);
  bounds_->setLow(0, costmap_ros_->getCostmap()->getOriginX());
  bounds_->setHigh(0, costmap_ros_->getCostmap()->getSizeInMetersX() +
                          costmap_ros_->getCostmap()->getOriginX());
  bounds_->setLow(1, costmap_ros_->getCostmap()->getOriginY());
  bounds_->setHigh(1, costmap_ros_->getCostmap()->getSizeInMetersY() +
                          costmap_ros_->getCostmap()->getOriginY());
  ROS_INFO("bounds: %f %f %f %f", bounds_->low[0], bounds_->low[1],
           bounds_->high[0], bounds_->high[1]);
  ros::NodeHandle prefix_nh;
  tf_prefix_ = tf::getPrefixParam(prefix_nh);
  // dynamic reconfigure
  dsrv_ = new dynamic_reconfigure::Server<OmplGlobalPlannerBaseConfig>(private_nh);
  dynamic_reconfigure::Server<OmplGlobalPlannerBaseConfig>::CallbackType cb_;
  cb_ = boost::bind(&OmplGlobalPlannerBase::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb_);
  // setup ompl
  setupOmpl();
  initialized_ = true;
  ROS_INFO("OmplGlobalPlannerBase initialized!");
}

void OmplGlobalPlannerBase::reconfigureCB(OmplGlobalPlannerBaseConfig& config,
                                      uint32_t level) {
  // plugin params
  frame_id_ = config.frame_id;
  ompl_planner_ = config.ompl_planner;
  collision_check_res_ = config.collision_check_res;
  max_planning_time_ = config.max_planning_time;
  max_simplification_time_ = config.max_simplification_time;
  path_simplifier_ = config.path_simplifier;
  // planners params
  border_fraction_ = config.border_fraction;
  goal_bias_ = config.goal_bias;
  range_ = config.range;
  max_nearest_neighbors_ = config.max_nearest_neighbors;
  thr_similarity_ = config.thr_similarity;
  // setup ompl
  setupOmpl();
}

void OmplGlobalPlannerBase::setupOmpl() {
  // set bounds
  ompl_state_space_->as<ob::SE2StateSpace>()->setBounds(*bounds_);
  // init simple setup
  ompl_simple_setup_ = new og::SimpleSetup(ompl_state_space_);
  // set validity checker
  ompl_simple_setup_->setStateValidityChecker(
      boost::bind(&OmplGlobalPlannerBase::isStateValid, this,
                  ompl_simple_setup_->getSpaceInformation().get(), _1));
  ompl_simple_setup_->getSpaceInformation()->setStateValidityCheckingResolution(
      collision_check_res_);
}

void OmplGlobalPlannerBase::getXYThFromState(const ob::State* s, double* x,
                                         double* y, double* th) {
  ob::ScopedState<> ss(ompl_state_space_);
  ss = s;
  *x = ss[0];
  *y = ss[1];
  *th = ss[2];
}

geometry_msgs::PoseStamped OmplGlobalPlannerBase::getPoseStampedFromState(
    const ob::State* state) {
  double x, y, th;
  getXYThFromState(state, &x, &y, &th);
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = frame_id_;
  ps.header.stamp = ros::Time::now();
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.orientation.z = std::sin(0.5 * th);
  ps.pose.orientation.w = std::cos(0.5 * th);
  return ps;
}

bool OmplGlobalPlannerBase::isStateValid(
    const ob::SpaceInformation* ompl_simple_setup_, const ob::State* state) {
  if (!ompl_simple_setup_->satisfiesBounds(state)) {
    return false;
  }
  double x, y, theta;
  getXYThFromState(state, &x, &y, &theta);
  double footprint_cost = costmap_model_->footprintCost(
      x, y, theta, costmap_ros_->getRobotFootprint());
  // check validity of the robot footprint -- very conservative
  if (footprint_cost < 0 || footprint_cost == costmap_2d::LETHAL_OBSTACLE ||
      footprint_cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
      footprint_cost == costmap_2d::NO_INFORMATION) {
    return false;
  }
  return true;
}

bool OmplGlobalPlannerBase::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& global_path) {
  boost::mutex::scoped_lock lock(mutex_);
  if (!initialized_) {
    ROS_ERROR("OmplGlobalPlannerBase has not been initialized yet");
    return false;
  }
  // reset planning data
  // ompl_simple_setup_->clear();
  global_path.clear();
  // check that start and goal are in map frame
  if (tf::resolve(tf_prefix_, goal.header.frame_id) !=
      tf::resolve(tf_prefix_, frame_id_)) {
    ROS_ERROR("The goal pose must be in the %s frame. Now, in the %s frame.",
              tf::resolve(tf_prefix_, frame_id_).c_str(),
              tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
    return false;
  }
  if (tf::resolve(tf_prefix_, start.header.frame_id) !=
      tf::resolve(tf_prefix_, frame_id_)) {
    ROS_ERROR("The start pose must be in the %s frame. Now, in the %s frame.",
              tf::resolve(tf_prefix_, frame_id_).c_str(),
              tf::resolve(tf_prefix_, start.header.frame_id).c_str());
    return false;
  }
  // def start and goal state
  ob::ScopedState<> ss_start(ompl_state_space_), ss_goal(ompl_state_space_);
  ss_start = std::vector<double>{start.pose.position.x, start.pose.position.y,
                                 tf::getYaw(start.pose.orientation)};
  ss_goal = std::vector<double>{goal.pose.position.x, goal.pose.position.y,
                                tf::getYaw(goal.pose.orientation)};
  ompl_simple_setup_->setStartAndGoalStates(ss_start, ss_goal);

  // set planner
  // "KPIECE"
  if (ompl_planner_ = 0) {
    og::KPIECE1* planner =
        new og::KPIECE1(ompl_simple_setup_->getSpaceInformation());
    ompl_simple_setup_->setPlanner(ob::PlannerPtr(planner));
    planner->setGoalBias(goal_bias_);
    planner->setRange(range_);
    planner->setBorderFraction(border_fraction_);

  } else if (ompl_planner_ = 1) {  // "PRM"
    og::PRM* planner = new og::PRM(ompl_simple_setup_->getSpaceInformation());
    ompl_simple_setup_->setPlanner(ob::PlannerPtr(planner));
    planner->setMaxNearestNeighbors(max_nearest_neighbors_);

  } else if (ompl_planner_ = 2) {  // "LazyPRM"
    og::LazyPRM* planner = new og::LazyPRM(ompl_simple_setup_->getSpaceInformation());
    ompl_simple_setup_->setPlanner(ob::PlannerPtr(planner));
    planner->setMaxNearestNeighbors(max_nearest_neighbors_);

  }
  	else if (ompl_planner_ = 3) { // "RRT" 
    og::RRT* planner = new og::RRT(ompl_simple_setup_->getSpaceInformation());
    ompl_simple_setup_->setPlanner(ob::PlannerPtr(planner));
    planner->setGoalBias(goal_bias_);
    planner->setRange(range_);

  } else if (ompl_planner_ = 4) { //"RRTconnect"
    og::RRTConnect* planner =
        new og::RRTConnect(ompl_simple_setup_->getSpaceInformation());
    ompl_simple_setup_->setPlanner(ob::PlannerPtr(planner));
    planner->setRange(range_);

  } else if (ompl_planner_ = 5) { //"RRTstar"
    og::RRTstar* planner =
        new og::RRTstar(ompl_simple_setup_->getSpaceInformation());
    ompl_simple_setup_->setPlanner(ob::PlannerPtr(planner));
    planner->setRange(range_);

  } else if (ompl_planner_ == 6) { // "EGP"
    og::AGPglobal* planner =
        new og::AGPglobal(ompl_simple_setup_->getSpaceInformation());
    ompl_simple_setup_->setPlanner(ob::PlannerPtr(planner));
    planner->setRange(range_);
    planner->setSimilarityThr(thr_similarity_);

  } else {
    ROS_ERROR("OmplGlobalPlannerBase: the planner specified does not exist.");
  }
  ompl_simple_setup_->getPlanner()->printSettings(std::cout);
  // load settings
  ompl_simple_setup_->setup();
  // planning
  if (ompl_simple_setup_->solve(max_planning_time_)) {
    ROS_INFO("OmplGlobalPlannerBase: path successfully found using %s planner!",
             ompl_simple_setup_->getPlanner()->getName().c_str());
    og::PathGeometric& solution_path = ompl_simple_setup_->getSolutionPath();
    // path simplification
    if (path_simplifier_ == "simple") {
      ompl_simple_setup_->getPathSimplifier()->smoothBSpline(solution_path, 5,
                                                             map_resolution_);
    } else if (path_simplifier_ == "full") {
      ompl_simple_setup_->simplifySolution(max_simplification_time_);
      solution_path = ompl_simple_setup_->getSolutionPath();
    }
    ROS_INFO("OmplGlobalPlannerBase: %s path simplification.",
             path_simplifier_.c_str());

    // check for validity, and repair if possible
    const std::pair<bool, bool>& path_validity =
        solution_path.checkAndRepair(ompl::magic::MAX_VALID_SAMPLE_ATTEMPTS);
    if (!path_validity.second) {
      OMPL_WARN(
          "Path may slightly touch an invalid region of the space."
          "Replanning.");
      return false;
    } else if (!path_validity.first) {
      OMPL_DEBUG("Path was slightly touching an invalid region. Now fixed.");
    }

    // increase number of path points
    if (map_resolution_ > 0.0) {
      // number interpolated points proportional to map resolution
      int min_num_states = round(solution_path.length() / map_resolution_);
      solution_path.interpolate(min_num_states);
      ROS_DEBUG("States after interpolation(%d): %lu", min_num_states,
                solution_path.getStateCount());
    }
    // solution_path.printAsMatrix(std::cout);

    // ompl states to ros pose stamped
    std::vector<ob::State*>& result_states = solution_path.getStates();
    for (std::vector<ob::State*>::iterator it = result_states.begin();
         it != result_states.end(); ++it) {
      geometry_msgs::PoseStamped ps = getPoseStampedFromState(*it);
      global_path.push_back(ps);
    }
  } else {
    ROS_ERROR("OmplGlobalPlannerBase: failed to find a global_path.");
  }
  publishPlan(global_path);
  return !global_path.empty();
}

void OmplGlobalPlannerBase::publishPlan(
    const std::vector<geometry_msgs::PoseStamped>& path) {
  if (!initialized_) {
    ROS_ERROR("OmplGlobalPlannerBase has not been initialized yet.");
    return;
  }
  // create messages for the global_path
  nav_msgs::Path gui_path;
  gui_path.poses.resize(path.size());

  geometry_msgs::PoseArray gui_waypoints;
  gui_waypoints.poses.resize(path.size());

  if (!path.empty()) {
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;
    gui_waypoints.header.frame_id = path[0].header.frame_id;
    gui_waypoints.header.stamp = path[0].header.stamp;
  }
  // global_path in map
  for (unsigned int i = 0; i < path.size(); i++) {
    gui_path.poses[i] = path[i];
    gui_waypoints.poses[i] = path[i].pose;
  }
  global_plan_pub_.publish(gui_path);
  plan_waypoints_pub_.publish(gui_waypoints);
}

};  // namespace ompl_global_planner_base
