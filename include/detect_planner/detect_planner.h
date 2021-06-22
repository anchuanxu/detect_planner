/*********************************************************************
*
*  Copyright (c) 2020, Chuanxu An, Inc.
*  All rights reserved.
*
* Author: Chuanxu An
*********************************************************************/
#ifndef DETECT_PLANNER_H_
#define DETECT_PLANNER_H_
#include <ros/ros.h>
#include <angles/angles.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalID.h>
#include <tf/transform_datatypes.h>
#include <boost/thread/mutex.hpp>
#include <boost/bind/bind.hpp>
#include <robot_msg/FeedBack.h>
#include <robot_msg/SlamStatus.h>
#include <robot_msg/ElevatorState.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <tf2_ros/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_msg/auto_elevatorAction.h>
#include <geometry_msgs/Polygon.h>

#include <nav_msgs/GetPlan.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <fstream>
#include <ctime>
#define DETECT_PLANNER_LOG(x)       \
  {                                 \
    if (this->record_log_)          \
    {                               \
      this->log_ << x << std::endl; \
    }                               \
  }

namespace detect_planner
{

  enum DetectPlannerState
  {
    // NAV_WAIT,
    IDLE,
    NAV_DOOR,
    NAV_TAKE,
    RETURN_WAIT_AREA,
    FACE_DOOR,
    TAKE,
    NAV_OUT,
    RETURN_TAKE_AREA,
    FACE_DOOR_OUTSIDE,
    SOMETHING_ERROR
  };

#define DETECT_PLANNER_RECORD 1
  /**
   * @class DetectPlanner
   * @brief A small area navigation method.
   */
  class DetectPlanner
  {
  public:
    DetectPlanner(std::string name, tf2_ros::Buffer &tf);
    /**
       * @brief  Constructor for the DetectPlanner
       */
    void initialize();

    /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
    ~DetectPlanner();

  private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

    void naviStateCallback(const robot_msg::FeedBackConstPtr &msg);

    void elevatorStateCallback(const robot_msg::ElevatorStateConstPtr &msg);

    void publishZeroVelocity();

    void cartoCallback(const robot_msg::SlamStatus::ConstPtr &msg);

    void movebaseCancelCallback(const actionlib_msgs::GoalID::ConstPtr &msg);

    void executeCB(const robot_msg::auto_elevatorGoalConstPtr &goal);

    bool runPlan(geometry_msgs::Pose takePoint, geometry_msgs::Pose waitPoint, bool mode);

    void preemptCB();

    double Distance(geometry_msgs::Pose PointA, geometry_msgs::Pose PointB);

    void searchNearby(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped target_base_pose, double tolerance, geometry_msgs::PoseStamped &success_pose, bool &success_or_not);

    double inline normalizeAngle(double val, double min, double max)
    {
      double norm = 0.0;
      if (val >= min)
        norm = min + fmod((val - min), (max - min));
      else
        norm = max - fmod((min - val), (max - min));
      return norm;
    }

    //global variable
    bool move_base_cancel_;
    std::string base_frame_, laser_frame_, map_frame_;
    double elevatorLong_, elevatorWide_;
    double robotRadius_;
    bool initialized_;
    double recivedNewGoalTime, recivedNewGoalTimeEnd;
    bool initOdomStartPose;
    bool isPublishGoal_;
    bool isJudged_;
    bool isUpdateNaviState_;
    int naviStateFeedback_;
    double angle_W_to_T_; // angle frome wait point to take point
    double angle_T_to_W_; // angle frome take point to wait point
    bool recovery_can_clear_costmap_;
    double d_inside_face_door_front_L_, d_inside_face_door_back_L_, d_outside_elevator_, toleranceDistance_;
    double d_inside_elevator_W_;

    //action
    ros::NodeHandle ah_, ph_;
    std::string action_name_;
    actionlib::SimpleActionServer<robot_msg::auto_elevatorAction> as_;
    robot_msg::auto_elevatorFeedback feedback_;
    robot_msg::auto_elevatorResult result_;
    DetectPlannerState state_;
    DetectPlannerState state_last_;

    //sub
    ros::Subscriber carto_sub_;
    ros::Subscriber elevator_state_sub_;
    // ros::Subscriber laser_sub_;
    ros::Subscriber navigation_state_sub_;

    //pub
    ros::Publisher vel_pub_;
    ros::Publisher goal_pub_;
    ros::Publisher goal_cancel_pub_;

    //data
    // ros::Time receive_laser_time_;
    geometry_msgs::Polygon point_vec_;
    // sensor_msgs::LaserScan laser_data_;
    robot_msg::SlamStatus carto_pose_;
    // sensor_msgs::Imu imu_data_;
    // nav_msgs::Odometry odom_data_;
    robot_msg::FeedBack navi_state_;
    robot_msg::ElevatorState elevator_state_;

    //mutex
    boost::mutex laser_mutex_;
    boost::mutex carto_mutex_;
    boost::mutex cancle_mutex_;
    boost::mutex state_mutex_;
    boost::mutex imu_mutex_;
    boost::mutex odom_mutex_;
    boost::mutex ele_state_mutex_;
    boost::mutex navi_state_mutex_;

    //tf
    tf::StampedTransform transform;

    //log
    std::ofstream log_;
    bool record_log_;

    //make plan
    // nav_msgs::GetPlanRequest srv_getplan_req_;
    // nav_msgs::GetPlanResponse srv_getplan_resp_;
    nav_msgs::GetPlan srv_getplan_;
    ros::ServiceClient client_getplan_;
    // float offset_takePoint[9][2];
    int enter_try_cnt;

    // reconfigure
    dynamic_reconfigure::Reconfigure srv_enalbe_recovery_;
    ros::ServiceClient client_enalbe_recovery_;
    dynamic_reconfigure::BoolParameter bool_param_;

    dynamic_reconfigure::ReconfigureRequest srv_req_;
    dynamic_reconfigure::ReconfigureResponse srv_resp_;
    dynamic_reconfigure::IntParameter int_param_;
    dynamic_reconfigure::DoubleParameter double_param_;
    // dynamic_reconfigure::Config conf_;
  };
}; // namespace detect_planner
#endif
