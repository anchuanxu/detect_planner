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
#include <move_base_msgs/MoveBaseActionGoal.h>



namespace detect_planner{
  /**
   * @class DetectPlanner
   * @brief A small area navigation method.
   */
  class DetectPlanner{
    public:

      DetectPlanner();
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
      bool runPlan();

      ~DetectPlanner();

    private:
      void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

      void publishZeroVelocity();

      void getLaserData(sensor_msgs::LaserScan& data);

      void getCartoPose(robot_msg::SlamStatus& data);

      void getLaserPoint(std::vector< std::pair<double,double> >& data);

      void getOdomData(nav_msgs::Odometry& data);

      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

      void cartoCallback(const robot_msg::SlamStatus::ConstPtr& msg);

      void goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg);

      void movebaseCancelCallback(const actionlib_msgs::GoalID::ConstPtr& msg);

      void getLaserTobaselinkTF(std::string sensor_frame, std::string base_frame);

      double updateAngleDiff(robot_msg::SlamStatus carto, geometry_msgs::Pose  goal);

      bool HaveObstacles(std::vector<std::pair<double,double>> sensor_point,double x,double y);

      void goback(double distance);

      void turnAngle(double angle);

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
      double waitPoint_x_, waitPoint_y_, takePoint_x_, takePoint_y_;
      bool initialized_;
      bool doorOpen_;

      //sub
      ros::Subscriber laser_sub_,odom_sub_,mbc_sub_,carto_sub_,goal_sub_;

      //pub
      ros::Publisher  vel_pub_;

      //data
      std::vector<std::pair<double,double>> point_vec_;
      sensor_msgs::LaserScan laser_data_;
      nav_msgs::Odometry odom_data_;
      robot_msg::SlamStatus carto_data_;
      move_base_msgs::MoveBaseActionGoal goal_data_;

      //mutex
      boost::mutex laser_mutex_;
      boost::mutex odom_mutex_;
      boost::mutex carto_mutex_;
      boost::mutex get_laser_mutex_;
      boost::mutex get_odom_mutex_;
      boost::mutex cancle_mutex_;

      //tf
      tf::StampedTransform transform;
  };
};  
#endif
