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
      ros::NodeHandle *nh_;
      ros::Subscriber laser_sub_;
      //ros::Subscriber camera_sub_;
      ros::Subscriber odom_sub_;
      ros::Subscriber mbc_sub_;
      ros::Publisher  vel_pub_;
      std::vector<std::pair<double,double>> point_vec_;
      std::string base_frame_, laser_frame_;
      sensor_msgs::LaserScan laser_data_;
      nav_msgs::Odometry odom_data_;
      boost::mutex laser_mutex_;
      boost::mutex odom_mutex_;
      tf::StampedTransform transform;
      bool move_base_cancel_;
      double pi;


      void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
      void publishZeroVelocity();
      void getLaserData(sensor_msgs::LaserScan& data);
      void getLaserPoint(std::vector< std::pair<double,double> >& data);
      void getOdomData(nav_msgs::Odometry& data);
      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
      void movebaseCancelCallback(const actionlib_msgs::GoalID::ConstPtr& msg);
      void getLaserTobaselinkTF(std::string sensor_frame, std::string base_frame);
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
      /**
       * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
       * @param x_i The x position of the robot 
       * @param y_i The y position of the robot 
       * @param theta_i The orientation of the robot
       * @return 
       */

      bool initialized_;
  };
};  
#endif
