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
#include <sensor_msgs/Imu.h>
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
#include <geometry_msgs/Polygon.h>

#include <fstream>
#include <ctime>
#define DETECT_PLANNER_LOG(x){if(this->record_log_){this->log_ << x << std::endl;}}

namespace detect_planner{

    enum DetectPlannerState
    {
        OUTDOOR_ANGLE_ADJ,
        GO_STRAIGHT_OUTDOOR,
        GO_STRAIGHT_ACROSSDOOR,
        GO_STRAIGHT_INDOOR,
        INELE_ANGLE_ADJ,
        TAKE_ELEVATOR,
        IMU_ANGLE_ADJ,
        OUTOF_ELEVATOR
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

      void cartoCallback(const robot_msg::SlamStatus::ConstPtr& msg);

      void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

      void movebaseCancelCallback(const actionlib_msgs::GoalID::ConstPtr& msg);

      void getLaserTobaselinkTF(std::string sensor_frame, std::string base_frame);

      double updateAngleDiff(robot_msg::SlamStatus carto, geometry_msgs::Pose  goal);

      bool HaveObstacles(geometry_msgs::Polygon sensor_point,double x,double y);

      void goback(double distance);

      void turnAngle(double angle);

      void executeCB(const robot_msg::auto_elevatorGoalConstPtr& goal);

      bool runPlan(geometry_msgs::Pose takePoint, geometry_msgs::Pose waitPoint, bool mode);

      void preemptCB();

      double Distance(geometry_msgs::Pose PointA, geometry_msgs::Pose PointB);

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
      bool        move_base_cancel_;
      bool        cartoJump;
      double      pi;
      double      MaxSpeed;
      std::string base_frame_,   laser_frame_;
      double      elevatorLong_, elevatorWide_;
      double      robotRadius_;
      bool        initialized_;
      bool        closeCango, midCango, farCango;
      bool        closeHavePoint, midHavePoint, farHavePoint;
      bool        robotStop;
      bool        robotImuAngleJudge;
      double      t1, t2, t;
      double      robotImuAngle0, robotImuAngle1, robotImuAngle2, robotImuAngle3, robotImuAngle4, robotImuAngleDiff;
      double      recivedNewGoalTime, recivedNewGoalTimeEnd;
      bool        initOdomStartPose;
      double      delt_p, current_p, previous_p, previous_delt_p;
      double      delt_o, current_o, previous_o, previous_delt_o;
      geometry_msgs::Pose  odomPoseStart;

      //action
      ros::NodeHandle ah_,ph_;
      std::string     action_name_;
      actionlib::SimpleActionServer<robot_msg::auto_elevatorAction> as_;
      robot_msg::auto_elevatorFeedback feedback_;
      robot_msg::auto_elevatorResult   result_;
      DetectPlannerState               state_;

      //sub
      ros::Subscriber laser_sub_, mbc_sub_, carto_sub_, imu_sub_, odom_sub_;

      //pub
      ros::Publisher  vel_pub_;

      //data
      ros::Time receive_laser_time_;
      geometry_msgs::Polygon   point_vec_;
      sensor_msgs::LaserScan   laser_data_;
      robot_msg::SlamStatus    carto_data_;
      sensor_msgs::Imu         imu_data_;
      nav_msgs::Odometry       odom_data_;

      //mutex
      boost::mutex laser_mutex_;
      boost::mutex carto_mutex_;
      boost::mutex cancle_mutex_;
      boost::mutex state_mutex_;
      boost::mutex imu_mutex_;
      boost::mutex odom_mutex_;

      //tf
      tf::StampedTransform transform;

      //log
      std::ofstream log_;
      bool record_log_;
  };
};  
#endif
