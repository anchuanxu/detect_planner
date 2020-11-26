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
#include <math.h>

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

      ros::NodeHandle param_nh("~");
      param_nh.param<std::string>("detect_planner/base_frame", base_frame_, std::string("base_link"));
      param_nh.param<std::string>("detect_planner/base_frame", laser_frame_, std::string("laser"));
      param_nh.param<double>("detect_planner/waitPoint_x", waitPoint_x_, double(-1.0));
      param_nh.param<double>("detect_planner/waitPoint_y", waitPoint_y_, double(0.0));
      param_nh.param<double>("detect_planner/takePoint_x", takePoint_x_, double(0.6));
      param_nh.param<double>("detect_planner/takePoint_y", takePoint_y_, double(0.0));

      laser_sub_.shutdown();
      odom_sub_.shutdown();
      //carto_sub_.shutdown();
      this->odom_data_.header.stamp = ros::Time::now();
      this->laser_data_.header.stamp = ros::Time::now();
      getLaserTobaselinkTF(laser_frame_, base_frame_);
      move_base_cancel_ = false;
      pi = 3.55;
      doorOpen_ = false;

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
    //TODO :接收到电梯信号
    doorOpen_ = true;

    if(!doorOpen_)
    {
      ROS_ERROR("The elevator doors are not open");
      return false;
    }

    //订阅传感器话题
    laser_sub_ = nh_->subscribe<sensor_msgs::LaserScan>("/scan",1,boost::bind(&DetectPlanner::scanCallback,this,_1));
    odom_sub_ = nh_->subscribe<nav_msgs::Odometry>("/odom",1,boost::bind(&DetectPlanner::odomCallback,this,_1));
    carto_sub_ = nh_->subscribe<robot_msg::SlamStatus>("/slam_status",1,boost::bind(&DetectPlanner::cartoCallback,this,_1));

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
      carto_sub_.shutdown();
      return false;
    }

    //从里程计获取当前位姿
//    nav_msgs::Odometry odom_data;
//    this->getOdomData(odom_data);
//    double robot_start_x = odom_data.pose.pose.position.x;
    //double robot_start_y = odom_data.pose.pose.position.y;
    //double robot_start_t = tf::getYaw(odom_data.pose.pose.orientation);
//    uint32_t count_step = 0;
//    while(isnan(robot_start_t))
//    {
//       ROS_INFO("can't get right robot post");
//       if(count_step++ > 10)
//       {
//         laser_sub_.shutdown();
//         odom_sub_.shutdown();
//         ROS_ERROR("odom no data then return!");
//         return false;
//       }
//       ros::Duration(0.05).sleep();
//       this->getOdomData(odom_data);
//       robot_start_x = odom_data.pose.pose.position.x;
//       robot_start_y = odom_data.pose.pose.position.y;
//       robot_start_t = tf::getYaw(odom_data.pose.pose.orientation);
//       ros::spinOnce();
//    }
//    double robot_current_x = robot_start_x;
//    double robot_current_y = robot_start_y;
   // double robot_current_t = robot_start_t; //用odom计算角度时用到。

    //从carto_data获取到当前位姿
    robot_msg::SlamStatus carto_data;
    this->getCartoPose(carto_data);
    double robot_start_x = carto_data.pose.position.x;
    double robot_start_y = carto_data.pose.position.y;
    double robot_start_t = tf::getYaw(carto_data.pose.orientation);
    uint32_t count_step = 0;
    while(isnan(robot_start_t))
    {
       ROS_INFO("can't get right robot post");
       if(count_step++ > 10)
       {
         laser_sub_.shutdown();
         carto_sub_.shutdown();
         ROS_ERROR("carto no data then return!");
         return false;
       }
       ros::Duration(0.05).sleep();
       this->getCartoPose(carto_data);
       robot_start_x = carto_data.pose.position.x;
       robot_start_y = carto_data.pose.position.y;
       robot_start_t = tf::getYaw(carto_data.pose.orientation);
       ros::spinOnce();
    }
    double robot_current_x = robot_start_x;
    double robot_current_y = robot_start_y;
    double distance, distance2;


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
      this->getLaserPoint(laser_point);
      ros::spinOnce();
    }

    //部分变量声明和初始化
    bool go_forward = false;
    bool intoDone = false;//是否已经完成进入
    geometry_msgs::Twist cmd_vel;
    double start_time,end_time,interval_time;

    //进入前进逻辑
    double x_diff, y_diff, x_diff2, y_diff2, angle_diff;
    geometry_msgs::Pose  global_pose, goal_pose, goal2_pose;
    this->getCartoPose(carto_data);
    global_pose = carto_data.pose;

    //目标点1 也就是乘梯点
    goal_pose.position.x = takePoint_x_;
    goal_pose.position.y = takePoint_y_;

    //目标点2 也就是候梯点
    goal2_pose.position.x = waitPoint_x_;
    goal2_pose.position.y = waitPoint_y_;

    x_diff2 = goal2_pose.position.x - global_pose.position.x;
    y_diff2 = goal2_pose.position.y - global_pose.position.y;

    distance2 = sqrt((x_diff2 * x_diff2) +(y_diff2 * y_diff2)); //距离候梯点的距离
    if(distance2 > 1)
    {
      ROS_ERROR("robot not in waiting point");
      return false;
    }

    x_diff = goal_pose.position.x - global_pose.position.x;
    y_diff = goal_pose.position.y - global_pose.position.y;

    distance = sqrt((x_diff * x_diff) +(y_diff * y_diff));//机器人距离目标点的距离长度

    angle_diff = updateAngleDiff(carto_data,goal_pose);
    angle_diff = normalizeAngle(angle_diff,-M_PI,M_PI);
    std::cout << "angle_diff = " << angle_diff << std::endl;
    ROS_INFO("Robot initial posture correction start");
    while (ros::ok() && fabs(angle_diff) > 0.06) {
      if(angle_diff > 0.5)
      {
        cmd_vel.angular.z = 0.5;
      }
      else if(angle_diff < -0.5)
      {
        cmd_vel.angular.z = -0.5;
      }
      else {
        cmd_vel.angular.z = angle_diff;
      }
      cmd_vel.linear.x = 0;
      this->vel_pub_.publish(cmd_vel);

      //更新angle_diff
      this->getCartoPose(carto_data);
      angle_diff = updateAngleDiff(carto_data,goal_pose);
      angle_diff = normalizeAngle(angle_diff,-M_PI,M_PI);
      ros::spinOnce();
    }
    publishZeroVelocity();
    ROS_INFO("Robot initial posture correction end");
    start_time = ros::Time::now().sec;
    //进入循环，直行逻辑开始
    while(ros::ok())
    {
      this->getLaserPoint(laser_point);//获取下最新的激光数据
      go_forward = HaveObstacles(laser_point,0.4,0.35);
      while(ros::ok() && go_forward == false)
      {
        ROS_INFO("please move your body");
        end_time = ros::Time::now().sec;
        interval_time = end_time - start_time;
        std::cout << "waiting you move time = " << interval_time << std::endl;
        if(interval_time > 15)
        {
          ROS_INFO("robot can't move");
          return false;//等待15s无法上电梯就等待下一部
        }
        else
        {
          ros::Rate r(20);
          uint8_t count = 0;
          while(ros::ok() && count++ < 20)
          {
            r.sleep();
            ros::spinOnce();
          }
          this->getLaserPoint(laser_point);//获取下最新的激光数据
          go_forward = HaveObstacles(laser_point,0.4,0.35);
        }
      }

      //第1部分 行走至电梯前
      while(ros::ok() && go_forward == true && distance >= 0.82) //2cm容错误差
      {
        if(fabs(angle_diff) > 0.01)
        {
          cmd_vel.angular.z = (angle_diff / 2);
        }
        else {
          cmd_vel.angular.z = 0;
        }
        cmd_vel.linear.x = 0.1;
        this->vel_pub_.publish(cmd_vel);

        //更新angle_diff和distance
        this->getCartoPose(carto_data);
        global_pose = carto_data.pose;
        x_diff = goal_pose.position.x - global_pose.position.x;
        y_diff = goal_pose.position.y - global_pose.position.y;

        distance = sqrt((x_diff * x_diff) +(y_diff * y_diff));

        angle_diff = updateAngleDiff(carto_data,goal_pose);
        angle_diff = normalizeAngle(angle_diff,-M_PI,M_PI);

        this->getLaserPoint(laser_point);
        go_forward = HaveObstacles(laser_point,0.45,0.30); //有5cm的容错
        if(go_forward == false)
        {
          publishZeroVelocity();
          //TODO:播放语音，请让一让，让可爱的机器人进去吧；
          ros::Rate r(20);
          uint8_t count = 0;
          while(ros::ok() && count++ < 100 && go_forward == false)
          {
            ROS_INFO("please move your body");
            this->getLaserPoint(laser_point);
            go_forward = HaveObstacles(laser_point,0.45,0.30);
            r.sleep();
            ros::spinOnce();
          }
          if(go_forward == false)
          {
            publishZeroVelocity();
            ROS_INFO("Unable to enter elevator, return to origin!");
            this->getCartoPose(carto_data);
            robot_current_x = carto_data.pose.position.x;
            robot_current_y = carto_data.pose.position.y;

            double diff_x = robot_current_x - goal2_pose.position.x;
            double diff_y = robot_current_y - goal2_pose.position.y;
            double overDistance;

            overDistance = sqrt(diff_x*diff_x+diff_y*diff_y);//已经行走的距离
            std::cout << "toWaitPointDistance = " << overDistance  << std::endl;
            goback(overDistance + 0.01);
            ROS_INFO("return waiting point");
            return false;//退出程序
          }
        }
        //更新一下距离目标点的距离
        this->getCartoPose(carto_data);
        global_pose = carto_data.pose;
        x_diff = goal_pose.position.x - global_pose.position.x;
        y_diff = goal_pose.position.y - global_pose.position.y;
        distance = sqrt((x_diff * x_diff) + (y_diff * y_diff));
        std::cout << "goal distance = " << distance << std::endl;
        ros::spinOnce();
      }
      // 第二部分，开始进入电梯

      while(ros::ok() && go_forward == true && distance >= 0.4 && distance < 0.82)
      {
        if(fabs(angle_diff) > 0.01)
        {
          cmd_vel.angular.z = (angle_diff / 2);
        }
        else {
          cmd_vel.angular.z = 0;
        }
        cmd_vel.linear.x = 0.1;
        this->vel_pub_.publish(cmd_vel);

        //更新angle_diff和distance
        this->getCartoPose(carto_data);
        global_pose = carto_data.pose;
        x_diff = goal_pose.position.x - global_pose.position.x;
        y_diff = goal_pose.position.y - global_pose.position.y;

        distance = sqrt((x_diff * x_diff) +(y_diff * y_diff));

        angle_diff = updateAngleDiff(carto_data,goal_pose);
        angle_diff = normalizeAngle(angle_diff,-M_PI,M_PI);

        this->getLaserPoint(laser_point);
        go_forward = HaveObstacles(laser_point,0.2,0.30); //这里只能探测极限距离可不可以停靠 距离稍微短一些

        if(go_forward == false)
        {
          publishZeroVelocity();
          //TODO:播放语音，请让一让，让可爱的机器人进去吧；
          ros::Rate r(20);
          uint8_t count = 0;
          while(ros::ok() && count++ < 160 && go_forward == false) //等待时间久一点
          {
            ROS_INFO("please move your body");
            this->getLaserPoint(laser_point);
            go_forward = HaveObstacles(laser_point,0.2,0.30);
            r.sleep();
            ros::spinOnce();
          }
          if(go_forward == false)
          {
            publishZeroVelocity();
            ROS_INFO("Unable to enter elevator, return to origin!");
            this->getCartoPose(carto_data);
            robot_current_x = carto_data.pose.position.x;
            robot_current_y = carto_data.pose.position.y;

            double diff_x = robot_current_x - goal2_pose.position.x;
            double diff_y = robot_current_y - goal2_pose.position.y;
            double overDistance;

            overDistance = sqrt(diff_x*diff_x+diff_y*diff_y);//已经行走的距离
            std::cout << "toWaitPointDistance = " << overDistance  << std::endl;
            goback(overDistance + 0.01);
            ROS_INFO("return waiting point");
            return false;//退出程序
          }
        }
        //更新一下距离目标点的距离
        this->getCartoPose(carto_data);
        global_pose = carto_data.pose;
        x_diff = goal_pose.position.x - global_pose.position.x;
        y_diff = goal_pose.position.y - global_pose.position.y;
        distance = sqrt((x_diff * x_diff) + (y_diff * y_diff));
        std::cout << "came in, goal distance = " << distance << std::endl;
        if(distance <= 0.4)
        {
          ROS_INFO("I came in!");
          intoDone = true;
        }
        ros::spinOnce();
      }
      //第三部分电梯内小范围前进，到达乘梯点，到不了就停下。
      while(ros::ok() && go_forward == true && distance < 0.4) //跟电梯的长宽及障碍物有关
      {
        if(fabs(angle_diff) > 0.01)
        {
          cmd_vel.angular.z = (angle_diff / 2);
        }
        else {
          cmd_vel.angular.z = 0;
        }
        this->getLaserPoint(laser_point);
        go_forward = HaveObstacles(laser_point,0.3,0.35);
        if(go_forward == true)
        {
          cmd_vel.linear.x = 0.1;
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
          this->getCartoPose(carto_data);
          global_pose = carto_data.pose;
          x_diff = goal_pose.position.x - global_pose.position.x;
          y_diff = goal_pose.position.y - global_pose.position.y;
          distance = sqrt((x_diff * x_diff) + (y_diff * y_diff));
          std::cout << "into goal distance = " << distance << std::endl;

          if(distance <= 0.02)
          {
            publishZeroVelocity();
            intoDone = true;
            go_forward = false;
            break;
          }
        }

        //更新angle_diff
        this->getCartoPose(carto_data);
        angle_diff = updateAngleDiff(carto_data,goal_pose);
        angle_diff = normalizeAngle(angle_diff,-M_PI,M_PI);

        ros::spinOnce();
      }
      publishZeroVelocity();
      std::cout << "intoDone = " << intoDone << std::endl;
      std::cout << "go_forward = " << go_forward << std::endl;

      //第四部分　旋转180度 乘电梯
      while(ros::ok() && intoDone == true && go_forward == false)
      {
        ROS_INFO("turn my body");
        //turnAngle(180);
        this->getCartoPose(carto_data); //获取下carto的信息，更新启始点
        global_pose = carto_data.pose;

        angle_diff = updateAngleDiff(carto_data,goal2_pose);
        angle_diff = normalizeAngle(angle_diff,-M_PI,M_PI);
        std::cout << "goal2 angle_diff = " << angle_diff << std::endl;
        ROS_INFO("turn start");
        while (ros::ok() && fabs(angle_diff) > 0.06) {
          if(angle_diff > 0.5)
          {
            cmd_vel.angular.z = 0.5;
          }
          else if(angle_diff < -0.5)
          {
            cmd_vel.angular.z = -0.5;
          }
          else {
            cmd_vel.angular.z = angle_diff;
          }
          cmd_vel.linear.x = 0;
          this->vel_pub_.publish(cmd_vel);

          //更新angle_diff
          this->getCartoPose(carto_data);
          angle_diff = updateAngleDiff(carto_data,goal2_pose);
          angle_diff = normalizeAngle(angle_diff,-M_PI,M_PI);
          ros::spinOnce();
        }
        publishZeroVelocity();
        ROS_INFO("turn end");

        //此处乘电梯用时间控制，后续交给梯控程序控制，判断条件就是true or false
//        double startTakeEle, endTakeEle;
//        startTakeEle = ros::Time::now().sec;
//        for (endTakeEle = ros::Time::now().sec; endTakeEle - startTakeEle < 10.0;) {
//          publishZeroVelocity();
//          doorOpen_ = false;
//          ROS_INFO_ONCE("---Ride in an elevator---");
//        }

        //乘坐电梯时间 show time
        ros::Rate r(20);
        uint8_t count = 0;
        while(ros::ok() && count++ < 80)
        {
          ROS_INFO("---Ride in an elevator---");
          r.sleep();
          ros::spinOnce();
        }
        ROS_INFO("Arrived at the designated floor");
        doorOpen_ = true;
        //TODO:发送开电梯门指令
        //TODO:等待电梯们完全开启后判断 break,执行出电梯程序

        break;
      }

      //第五部分 出电梯
      //调整位姿态
      this->getCartoPose(carto_data); //获取下carto的信息，更新启始点
      global_pose = carto_data.pose;

      x_diff = goal2_pose.position.x - global_pose.position.x;
      y_diff = goal2_pose.position.y - global_pose.position.y;
      distance = sqrt((x_diff * x_diff) + (y_diff * y_diff));//距离目标点2的距离

      angle_diff = updateAngleDiff(carto_data,goal2_pose);
      angle_diff = normalizeAngle(angle_diff,-M_PI,M_PI);
      std::cout << "goal2 angle_diff = " << angle_diff << std::endl;
      ROS_INFO("sencond robot posture correction start");
      while (ros::ok() && fabs(angle_diff) > 0.1) {
        cmd_vel.angular.z =  (angle_diff / 2.0);
        cmd_vel.linear.x = 0;
        this->vel_pub_.publish(cmd_vel);

        //更新angle_diff
        this->getCartoPose(carto_data);
        angle_diff = updateAngleDiff(carto_data,goal2_pose);
        angle_diff = normalizeAngle(angle_diff,-M_PI,M_PI);
        ros::spinOnce();
      }
      publishZeroVelocity();
      ROS_INFO("second robot  posture correction end");

      //出电梯直行
      while(ros::ok() && doorOpen_ && distance > 0.02)
      {
        this->getLaserPoint(laser_point);//获取下最新的激光数据
        go_forward = HaveObstacles(laser_point,0.4,0.35);
        if(go_forward == false)
        {
          publishZeroVelocity();
          ROS_INFO("please move your body");//持续等待 直到可以出门
          ros::Rate r(20);
          uint8_t count = 0;
          while(ros::ok() && count++ < 20)
          {
            r.sleep();
            ros::spinOnce();
          }
          ROS_INFO("1 have sleep 1s !");
        }
        else {
          if(fabs(angle_diff) > 0.01)
          {
            cmd_vel.angular.z = (angle_diff / 2);
          }
          else {
            cmd_vel.angular.z = 0;
          }
          cmd_vel.linear.x = 0.1;
          this->vel_pub_.publish(cmd_vel);

          //更新angle_diff
          this->getCartoPose(carto_data);
          angle_diff = updateAngleDiff(carto_data,goal2_pose);
          angle_diff = normalizeAngle(angle_diff,-M_PI,M_PI);
        }
        //更新一下距离目标点2的距离
        this->getCartoPose(carto_data);
        global_pose = carto_data.pose;
        x_diff = goal2_pose.position.x - global_pose.position.x;
        y_diff = goal2_pose.position.y - global_pose.position.y;
        distance = sqrt((x_diff * x_diff) + (y_diff * y_diff));//距离目标点2的距离
        std::cout << "out distance = " << distance << std::endl;
        if(distance < 1.2)
        {
          intoDone = false;
        }
        if(distance < 0.02)
        {
          publishZeroVelocity();
          ROS_INFO("detect planner end");
          return true;
        }
        ros::spinOnce();
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
  double DetectPlanner::updateAngleDiff(robot_msg::SlamStatus carto, geometry_msgs::Pose goal)
  {
    double x_diff,y_diff,alpha,global_angle,angle_diff;

    x_diff = goal.position.x - carto.pose.position.x;
    y_diff = goal.position.y - carto.pose.position.y;

    alpha = std::atan2(y_diff, x_diff);//路径直线的夹角
    global_angle = tf::getYaw(carto.pose.orientation);//机器人本身的朝向角
    angle_diff = alpha - global_angle;
    return  angle_diff;
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
    ROS_INFO_ONCE("carto data recevied");
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
    ros::Time now = ros::Time::now();
    if(now.toSec() - this->carto_data_.header.stamp.toSec() > 5){
        ROS_INFO("time delay 5s ");
        return;
    }
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
