/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef DETECT_PLANNER_H_
#define DETECT_PLANNER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalID.h>
#include <tf/transform_datatypes.h>


namespace detect_planner{
  /**
   * @class DetectPlanner
   * @brief A small area navigation method.
   */
  class DetectPlanner : public nav_core::BaseGlobalPlanner {
    public:

      DetectPlanner();
      /**
       * @brief  Constructor for the DetectPlanner
       */
      DetectPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      /**
       * @brief  Constructor for the DetectPlanner Initialization function for the DetectPlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      ~DetectPlanner();

    private:
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
