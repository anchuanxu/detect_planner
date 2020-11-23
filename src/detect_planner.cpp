/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2020, Chuanxu An, Inc.
*  All rights reserved.
*
*********************************************************************/
#include <detect_planner/detect_planner.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <robot_msg/SlamStatus.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

namespace detect_planner {

  DetectPlanner::DetectPlanner()
  : initialized_(false){
    initialize();
  }

  void DetectPlanner::initialize(){
    if(!initialized_){
      nh_ = new ros::NodeHandle("~/");
      laser_sub_ = nh_->subscribe<sensor_msgs::LaserScan>("/scan",1,boost::bind(&DetectPlanner::scanCallback,this,_1));
      odom_sub_ = nh_->subscribe<nav_msgs::Odometry>("/odom",1,boost::bind(&DetectPlanner::odomCallback,this,_1));
      carto_sub_ = nh_->subscribe<robot_msg::SlamStatus>("/slam_status",1,boost::bind(&DetectPlanner::cartoCallback,this,_1));
      goal_sub_ = nh_->subscribe<move_base_msgs::MoveBaseActionGoal>("/move_base/goal",1,boost::bind(&DetectPlanner::goalCallback,this,_1));//话题要改
      vel_pub_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel",1);

      nh_->param("detect_planner/base_frame", base_frame_, std::string("base_link"));
      nh_->param("detect_planner/base_frame", laser_frame_, std::string("laser"));

      laser_sub_.shutdown();
      odom_sub_.shutdown();
      this->odom_data_.header.stamp = ros::Time::now();
      this->laser_data_.header.stamp = ros::Time::now();
      getLaserTobaselinkTF(laser_frame_, base_frame_);
      move_base_cancel_ = false;
      pi = 3.55;

      initialized_ = true;
    }
    else
      ROS_ERROR("This planner has already been initialized... doing nothing");
  }

  DetectPlanner::~DetectPlanner()
  {
    if(nh_)
    {
      delete nh_;
      nh_ = nullptr;
    }
  }

  bool DetectPlanner::runPlan(){

    ROS_INFO("---detect planner started!---");

    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    //订阅传感器话题
    //ros::NodeHandle private_nh("~/");
    laser_sub_ = nh_->subscribe<sensor_msgs::LaserScan>("/scan",1,boost::bind(&DetectPlanner::scanCallback,this,_1));
    odom_sub_ = nh_->subscribe<nav_msgs::Odometry>("/odom",1,boost::bind(&DetectPlanner::odomCallback,this,_1));
    carto_sub_ = nh_->subscribe<robot_msg::SlamStatus>("/slam_status",1,boost::bind(&DetectPlanner::cartoCallback,this,_1));
    //ros::Duration(2).sleep();

    //订阅话题进行持续判断是否有中断请求
    mbc_sub_ = nh_->subscribe<actionlib_msgs::GoalID>("/move_base/cancel",1,
                                        boost::bind(&DetectPlanner::movebaseCancelCallback,this,_1));
    if(move_base_cancel_ == true)
    {
      publishZeroVelocity();
      ROS_INFO("---move_base canceled---");
      move_base_cancel_ = false;

      laser_sub_.shutdown();
      odom_sub_.shutdown();
      return false;
    }

    //从里程计获取当前位姿
    nav_msgs::Odometry odom_data;
    //ros::Duration(0.5).sleep();
    this->getOdomData(odom_data);
    double robot_start_x = odom_data.pose.pose.position.x;
    double robot_start_y = odom_data.pose.pose.position.y;
    double robot_start_t = tf::getYaw(odom_data.pose.pose.orientation);
    uint32_t count_step = 0;
    while(isnan(robot_start_t))
    {
       ROS_INFO("can't get right robot post");
       if(count_step++ > 10)
       {
         laser_sub_.shutdown();
         odom_sub_.shutdown();
         ROS_ERROR("odom no data then return!");
         return false;
       }
       ros::Duration(0.05).sleep();
       this->getOdomData(odom_data);
       robot_start_x = odom_data.pose.pose.position.x;
       robot_start_y = odom_data.pose.pose.position.y;
       robot_start_t = tf::getYaw(odom_data.pose.pose.orientation);
       ros::spinOnce();
    }
    double robot_current_x = robot_start_x;
    double robot_current_y = robot_start_y;
   // double robot_current_t = robot_start_t; //用odom计算角度时用到。
    double distance = 0.0;


    //获取激光数据
    std::vector< std::pair<double,double> > laser_point;
    this->getLaserPoint(laser_point);//使用回调函数赋值
    count_step=0;
    while(laser_point.size() == 0) //如果激光没有数据，等待10s，如果还没有，退出恢复
    {
      ROS_INFO("can't get scan data");
      if(count_step++ > 10)
      {
        laser_sub_.shutdown();
        odom_sub_.shutdown();
        return false;
      }
      ros::Duration(1).sleep();
      this->getLaserPoint(laser_point);//使用回调函数赋值
      ros::spinOnce();
    }

    //部分变量声明和初始化
    bool go_forward = false;
    bool intoDone = false;//是否已经完成进入
    geometry_msgs::Twist cmd_vel;
    double start_time,end_time,interval_time;
    start_time = ros::Time::now().sec;

    //TODO 这里要加电梯口设别和位置矫正逻辑，保证其朝向是垂直于电梯门的，误差不要太大，矫正完毕后
    //进入前进逻辑
    double x_diff, y_diff, alpha,global_angle, angle_diff;
    geometry_msgs::Pose  global_pose, goal_pose;
    robot_msg::SlamStatus carto_data;
    this->getCartoPose(carto_data);
    global_pose = carto_data.pose;
    //goal_pose = goal_data_.goal.target_pose.pose;
    //先固定点进行写死，后续动态获取和写死都可以切换
    goal_pose.position.x = 1.8;
    goal_pose.position.y = 0.0;

    x_diff = goal_pose.position.x - global_pose.position.x;
    y_diff = goal_pose.position.y - global_pose.position.y;
    std::cout << "x,y = " << global_pose.position.x <<" , " << global_pose.position.y << std::endl;
    alpha = std::atan2(y_diff, x_diff);//路径直线的夹角
    global_angle = tf::getYaw(carto_data.pose.orientation);//机器人本身的朝向角
    angle_diff = alpha - global_angle;
    std::cout << "angle_diff = " << angle_diff << std::endl;
    double temp;
    ROS_INFO("angle correct start");
    while (ros::ok() && fabs(angle_diff) > 0.05) {
      cmd_vel.angular.z =  (angle_diff / 5.0);
      this->vel_pub_.publish(cmd_vel);

      //更新angle_diff
      temp = global_angle;
      ros::Duration(1).sleep();
      this->getCartoPose(carto_data);
      global_pose = carto_data.pose;
      x_diff = goal_pose.position.x - global_pose.position.x;
      y_diff = goal_pose.position.y - global_pose.position.y;
      //std::cout << "x,y = " << global_pose.position.x <<" , " << global_pose.position.y << std::endl;
      alpha = std::atan2(y_diff, x_diff);//路径直线的夹角
      global_angle = tf::getYaw(carto_data.pose.orientation);//机器人本身的朝向角
      angle_diff = alpha - global_angle;
      std::cout << "part2 angle_diff = " << angle_diff << std::endl;
      ros::spinOnce();
    }
    ROS_INFO("angle correct end");
    cmd_vel.angular.z =  0;
    this->vel_pub_.publish(cmd_vel);

    //进入循环，直行逻辑开始
    while(ros::ok())
    {
      this->getLaserPoint(laser_point);//获取下最新的激光数据
      go_forward = HaveObstacles(laser_point,0.4,0.35);
      if(go_forward == false)
      {
        ROS_INFO("please move your body");
        end_time = ros::Time::now().sec;
        interval_time = end_time - start_time;
        if(interval_time > 15)
        {
          ROS_INFO("robot can't move");
          return false;//等待15s无法上电梯就等待下一部
        }
        else
          ros::Duration(1).sleep();
      }

      //第一部分 行走至电梯里

      while(ros::ok() && go_forward == true && distance < 0.85)
      {
        cmd_vel.linear.x = 0.1;
        cmd_vel.angular.z = 0;
        this->vel_pub_.publish(cmd_vel);
        this->getLaserPoint(laser_point);
        go_forward = HaveObstacles(laser_point,0.4,0.35);

        if(go_forward == false)
        {
          publishZeroVelocity();
          //TODO:播放语音，请让一让，让可爱的机器人进去吧；
          ros::Rate r(20);
          uint8_t count = 0;
          while(ros::ok() && count++ < 100)
          {
            r.sleep();
            ros::spinOnce();
          }
          ROS_INFO("1 have sleep 5s !");
          this->getLaserPoint(laser_point);
          go_forward = HaveObstacles(laser_point,0.4,0.35);
          if(go_forward == false)
          {
            publishZeroVelocity();
            ROS_INFO("Unable to enter elevator, return to origin!");
            this->getOdomData(odom_data);
            robot_current_x = odom_data.pose.pose.position.x;
            robot_current_y = odom_data.pose.pose.position.y;

            double diff_x = robot_current_x - robot_start_x;
            double diff_y = robot_current_y - robot_start_y;

            distance = sqrt(diff_x*diff_x+diff_y*diff_y);
            std::cout << "distance = " << distance << std::endl;
            ROS_INFO("return origin over");
            goback((distance + 0.01));
            return false;//退出程序
          }
        }
        //更新一下已走的距离
        this->getOdomData(odom_data);
        robot_current_x = odom_data.pose.pose.position.x;
        robot_current_y = odom_data.pose.pose.position.y;

        double diff_x = robot_current_x - robot_start_x;
        double diff_y = robot_current_y - robot_start_y;

        distance = sqrt(diff_x*diff_x+diff_y*diff_y);
        //std::cout << "distance = " << distance << std::endl;
        if(distance > 0.83)
        {
          intoDone = true;
        }
        ros::spinOnce();
      }
      //第二部分电梯内小范围前进
      while(ros::ok() && go_forward == true && distance < 1.0) //跟电梯的长宽及障碍物有关
      {
        this->getLaserPoint(laser_point);
        go_forward = HaveObstacles(laser_point,0.3,0.35);
        if(go_forward == true)
        {
          cmd_vel.linear.x = 0.1;
          cmd_vel.angular.z = 0;
          this->vel_pub_.publish(cmd_vel);

          this->getLaserPoint(laser_point);
          go_forward = HaveObstacles(laser_point,0.3,0.35);
          if(go_forward == false)
          {
            ROS_INFO("1 have obs !");
            publishZeroVelocity();
            intoDone = true;
            ROS_INFO("i am stop here!");
            break;//实在无法前进了，就地停止
          }
          //更新一下已走的距离
          this->getOdomData(odom_data);
          robot_current_x = odom_data.pose.pose.position.x;
          robot_current_y = odom_data.pose.pose.position.y;

          double diff_x = robot_current_x - robot_start_x;
          double diff_y = robot_current_y - robot_start_y;

          distance = sqrt(diff_x*diff_x+diff_y*diff_y);
          std::cout << "part 2 distance = " << distance << std::endl;
          if(distance >= 1.0)
          {
            publishZeroVelocity();
            intoDone = true;
            ROS_INFO("test 1");
            go_forward = false;
            break;
          }
        }
        ros::spinOnce();
      }
      std::cout << "intoDone = " << intoDone << std::endl;
      std::cout << "go_forward = " << go_forward << std::endl;

      //第三部分　旋转180度
      while(ros::ok() && intoDone == true && go_forward == false)
      {
        ROS_INFO("turn my body");
        turnAngle(180);
        publishZeroVelocity();
        ROS_INFO("let's go");
        return true;
        //TODO:发送关闭电梯门指令
      }
    }
    ros::spinOnce();
}
  bool DetectPlanner::HaveObstacles(std::vector<std::pair<double, double> > sensor_point, double x, double y)
  {
    size_t index = 0;
    for(index = 0; index < sensor_point.size(); index++){
      if(sensor_point[index].first < x && fabs(sensor_point[index].second) < y)
      {
        return false;
      }
    }
    return true;
  }
  void DetectPlanner::publishZeroVelocity()
  {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    this->vel_pub_.publish(cmd_vel);
  }
  void DetectPlanner::turnAngle(double angle)
  {
    ros::Rate r(10);
    double angle_speed = 0.2;
    double angle_duration = (angle / 360 * 2 * pi) / angle_speed;
    int ticks = int(angle_duration * 10);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    while(ticks > 0)
    {
        ticks --;
        cmd_vel.angular.z = angle_speed;
        vel_pub_.publish(cmd_vel);
        r.sleep();
    }
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
    return ;
  }
  void DetectPlanner::goback(double distance)
  {
    ros::Rate r(10);
    double linear_speed = -0.2;
    double linear_duration = distance / fabs(linear_speed);

    int ticks = int(linear_duration * 10);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    while(ticks > 0)
    {
        ticks --;
        cmd_vel.linear.x = linear_speed;
        vel_pub_.publish(cmd_vel);
        r.sleep();
    }
    cmd_vel.linear.x = 0.0;
    vel_pub_.publish(cmd_vel);
  }
  void DetectPlanner::movebaseCancelCallback(const actionlib_msgs::GoalID::ConstPtr &msg)
  {
    move_base_cancel_ = true;
    return;
  }
  void DetectPlanner::getOdomData(nav_msgs::Odometry &data)
  {
    ros::Time now = ros::Time::now();
    if(now.toSec() - this->odom_data_.header.stamp.toSec() > 0.5){
      return;
    }
    data = this->odom_data_;
  }
  void DetectPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    ROS_INFO_ONCE("odom data recevied");
    boost::mutex::scoped_lock lock(this->odom_mutex_);
    this->odom_data_ = *msg;
  }
  void DetectPlanner::cartoCallback(const robot_msg::SlamStatus::ConstPtr &msg)
  {
    ROS_INFO("carto data recevied");
    boost::mutex::scoped_lock lock(this->carto_mutex_);
    this->carto_data_ = *msg;
  }
  void DetectPlanner::goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr &msg)
  {
    ROS_INFO_ONCE("goal recevied");
    boost::mutex::scoped_lock lock(this->carto_mutex_);
    this->goal_data_ = *msg;
  }
  void DetectPlanner::getLaserData(sensor_msgs::LaserScan &data)
  {
    ros::Time now = ros::Time::now();
    if(now.toSec() - this->laser_data_.header.stamp.toSec() > 0.5){
        return;
    }
    data = this->laser_data_;
  }
  void DetectPlanner::getCartoPose(robot_msg::SlamStatus &data)
  {
//    ros::Time now = ros::Time::now();
//    if(now.toSec() - this->carto_data_.header.stamp.toSec() > 5){
//        ROS_INFO_ONCE("time delay 5s ");
//        return;
//    }
//    ROS_INFO("get new carto info");
    data = this->carto_data_;
  }
  void DetectPlanner::getLaserPoint(std::vector<std::pair<double, double> > &data)
  {
    data = this->point_vec_;
  }
  void DetectPlanner::getLaserTobaselinkTF(std::string sensor_frame, std::string base_frame)
  {
    tf::TransformListener* tf_ = new tf::TransformListener(ros::Duration(10));
    while(ros::ok())
    {
      if(tf_->waitForTransform(base_frame, ros::Time::now(), sensor_frame, ros::Time::now(), sensor_frame, ros::Duration(1)))
      {
        tf_->lookupTransform(base_frame,sensor_frame,ros::Time::now(),transform);
        break;
      }
      ROS_WARN("frame %s to %s unavailable",base_frame.c_str(),sensor_frame.c_str());
      ros::Duration(0.5).sleep();
    }
    //debug
    std::cout << "laserTbase:" << std::endl;
    std::cout << "Tx:" << transform.getOrigin().getX() << std::endl;
    std::cout << "Ty:" << transform.getOrigin().getY() << std::endl;
    std::cout << "Tz:" << transform.getOrigin().getZ() << std::endl;
    std::cout << "Rx:" << transform.getRotation().getX() << std::endl;
    std::cout << "Ry:" << transform.getRotation().getY() << std::endl;
    std::cout << "Rz:" << transform.getRotation().getZ() << std::endl;
    std::cout << "Rw:" << transform.getRotation().getW() << std::endl;

    delete tf_;
    tf_ = nullptr;
  }
  void DetectPlanner::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    ROS_INFO_ONCE("scan data recevied");
    boost::mutex::scoped_lock lock(this->laser_mutex_);
    laser_data_ = *msg;
    this->point_vec_.clear();
    double min_angle = laser_data_.angle_min;
    double max_angle = laser_data_.angle_max;
    double laser_angle_increment = laser_data_.angle_increment; //雷达数据的角度间隔
    size_t laser_point_size = laser_data_.ranges.size();//雷达数据一帧的点数
    double theta = 0; //转角置为０；

    std::pair<double,double> tem_pair;
    for (size_t i = 0; i < laser_point_size; i++) {
      //做一些必要的判断，筛除无用的点
      if(isfinite(laser_data_.ranges[i]) == false)
        continue;
      if(isinf(laser_data_.ranges[i]) == true)
        continue;
      if(isnan(laser_data_.ranges[i]) == true)
        continue;
      tf::Quaternion q = transform.getRotation();
      theta = min_angle + i * laser_angle_increment + tf::getYaw(q);
      theta = normalizeAngle(theta,-M_PI,M_PI);
      if(theta >= -M_PI/2 && theta <= M_PI/2)
      {
       if(laser_data_.ranges[i] > 0)
       {
         tem_pair.first = laser_data_.ranges.at(i) * cos(theta) + transform.getOrigin().x();
         tem_pair.second = laser_data_.ranges.at(i) * sin(theta) + transform.getOrigin().y();
         this->point_vec_.push_back(tem_pair);
       }
      }
    }
  }
};
