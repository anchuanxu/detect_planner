/*********************************************************************
*
* Software License Agreement (BSD License)
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
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalID.h>
#include <tf/transform_datatypes.h>
#include <boost/thread/mutex.hpp>
#include <boost/bind/bind.hpp>
#include <robot_msg/SlamStatus.h>
#include <robot_msg/ElevatorState.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <tf2_ros/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_msg/auto_elevatorAction.h>

#include <fstream>
#include <ctime>
#define DETECT_PLANNER_LOG(x){if(this->record_log_){this->log_ << x << std::endl;}}

namespace detect_planner{

    enum DetectPlannerState
    {
        PART1,
        PART2,
        PART3,
        PART4,
        PART5,
        PART6
    };

  #define DETECT_PLANNER_RECORD 1
  /**
   * @class DetectPlanner
   * @brief A small area navigation method.
   */
  class DetectPlanner{
    public:

      DetectPlanner(std::string name, tf2_ros::Buffer& tf);
      /**
       * @brief  Constructor for the DetectPlanner
       */
      void initialize();

      //void Init(int current_floor, geometry_msgs::Pose takepose,
               // int target_floor,  geometry_msgs::Pose waitpose);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      ~DetectPlanner();

    private:
      void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

      void publishZeroVelocity();

      void getLaserData(sensor_msgs::LaserScan& data);

      void getCartoPose(robot_msg::SlamStatus& data);

      void getElevatorState(robot_msg::ElevatorState& data);

      void getLaserPoint(std::vector< std::pair<double,double> >& data);

      void getOdomData(nav_msgs::Odometry& data);

      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

      void cartoCallback(const robot_msg::SlamStatus::ConstPtr& msg);

      void elevatorCallback(const robot_msg::ElevatorState::ConstPtr& msg);

      void movebaseCancelCallback(const actionlib_msgs::GoalID::ConstPtr& msg);

      void subShutDown();

      void getLaserTobaselinkTF(std::string sensor_frame, std::string base_frame);

      double updateAngleDiff(robot_msg::SlamStatus carto, geometry_msgs::Pose  goal);

      bool HaveObstacles(std::vector<std::pair<double,double>> sensor_point,double x,double y);

      void goback(double distance);

      void turnAngle(double angle);

      void executeCB(const robot_msg::auto_elevatorGoalConstPtr& goal);

      bool runPlan(geometry_msgs::Pose takePoint, geometry_msgs::Pose waitPoint, int target_floor);

      void preemptCB();

      double inline normalizeAngle(double val, double min, double max)
      {
        double norm = 0.0;
        if (val >= min)
          norm = min + fmod((val - min), (max-min));
        else
          norm = max - fmod((min - val), (max-min));
        return norm;
      }

      //global variable
      ros::NodeHandle *nh_;
      bool move_base_cancel_;
      double pi;
      std::string base_frame_, laser_frame_;
      bool    initialized_;
      bool    doorOpen_;
      uint8_t takEleCount;

      //action
      ros::NodeHandle ah_,ph_;
      std::string     action_name_;
      actionlib::SimpleActionServer<robot_msg::auto_elevatorAction> as_;
      robot_msg::auto_elevatorFeedback feedback_;
      robot_msg::auto_elevatorResult   result_;
      DetectPlannerState               state_;

      //sub
      ros::Subscriber laser_sub_,odom_sub_,mbc_sub_,carto_sub_,elevator_sub_;

      //pub
      ros::Publisher  vel_pub_;

      //data
      ros::Time receive_laser_time_;
      std::vector<std::pair<double,double>> point_vec_;
      sensor_msgs::LaserScan laser_data_;
      nav_msgs::Odometry odom_data_;
      robot_msg::SlamStatus carto_data_;
      robot_msg::ElevatorState ele_data_;

      //mutex
      boost::mutex laser_mutex_;
      boost::mutex odom_mutex_;
      boost::mutex carto_mutex_;
      boost::mutex ele_mutex_;
      boost::mutex get_laser_mutex_;
      boost::mutex get_odom_mutex_;
      boost::mutex cancle_mutex_;

      //tf
      tf::StampedTransform transform;

      //log
      std::ofstream log_;
      bool record_log_;
  };
};  
#endif
