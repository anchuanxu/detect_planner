/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2020, Chuanxu An, Inc.
*  All rights reserved.
*
*********************************************************************/
#include <detect_planner/detect_planner.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <math.h>

namespace detect_planner {

  DetectPlanner::DetectPlanner(std::string name, tf2_ros::Buffer& tf)
  : initialized_(false),
    ph_("~"),
    action_name_(name),
    as_(ah_, name,boost::bind(&DetectPlanner::executeCB,this, _1),false)
  {
    //nh_ = nullptr;
    move_base_cancel_ = false;
    initialize();
    state_ = OUTDOOR_ANGLE_ADJ;
    as_.start();
    ROS_INFO_STREAM("Action server " << action_name_ << " start!");
  }

  void DetectPlanner::initialize(){
    if(!initialized_){
      laser_sub_ = ah_.subscribe<sensor_msgs::LaserScan>("/scan",10,boost::bind(&DetectPlanner::scanCallback,this,_1));
      carto_sub_ = ah_.subscribe<robot_msg::SlamStatus>("/slam_status",20,boost::bind(&DetectPlanner::cartoCallback,this,_1));
      mbc_sub_ = ah_.subscribe<actionlib_msgs::GoalID>("/move_base/cancel", 10, boost::bind(&DetectPlanner::movebaseCancelCallback, this, _1));
      imu_sub_ = ah_.subscribe<sensor_msgs::Imu>("/imu", 10, boost::bind(&DetectPlanner::imuCallback,this, _1));
      odom_sub_ = ah_.subscribe<nav_msgs::Odometry>("/odom",10,boost::bind(&DetectPlanner::odomCallback,this, _1));

      vel_pub_ = ah_.advertise<geometry_msgs::Twist>("/cmd_vel",10);
      //vel_pub_ = ah_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",10);

      ph_.param<std::string>("base_frame", base_frame_, std::string("base_link"));
      ph_.param<std::string>("laser_frame", laser_frame_, std::string("laser"));
      ph_.param<double>("elevatorLong", elevatorLong_, double(1.6));
      ph_.param<double>("elevatorWide", elevatorWide_, double(2.0));
      ph_.param<double>("robotRadius", robotRadius_, double(0.225));

      while(ros::ok() && !ph_.hasParam("elevatorLong") && !move_base_cancel_)
      {
        ROS_INFO("can't get param that elevatorLong !");
        ros::Duration(1).sleep();
      }

      receive_laser_time_ = ros::Time::now();
      getLaserTobaselinkTF(laser_frame_, base_frame_);
      move_base_cancel_ = false;
      pi = 3.55;
      MaxSpeed = 0.5;
      record_log_ = false;
      closeCango = false;
      midCango = false;
      farCango = false;
      robotStop = true;
      cartoJump = false;
      robotImuAngleJudge = false;
      initOdomStartPose = false;
      t = t1 = t2 = 0.0;
      recivedNewGoalTime = recivedNewGoalTimeEnd = 0.0;
      robotImuAngle0 = robotImuAngle1 = robotImuAngle2 = robotImuAngle3 = robotImuAngle4 = robotImuAngleDiff = 0.0;
      delt_p = current_p = previous_p = previous_delt_p = 0.0;
      delt_o = current_o = previous_o = previous_delt_o = 0.0;

      if(DETECT_PLANNER_RECORD)
      {
        log_.open("/tmp/detect_planner.log",std::ios::out);//std::ios::app //底部追加日志写法
      }
      if(!log_)
      {
        ROS_ERROR("open /tmp/detect_planner.log fail.");
      }
      else {
        record_log_ = true;
        DETECT_PLANNER_LOG("Welcome to use the smartest robot in the world.");
        std::time_t nowtime;
        struct tm* p;
        nowtime = std::time(nullptr);
        p = std::localtime(&nowtime);
        log_ << p->tm_year+1900 << "-" << p->tm_mon+1 << "-" << p->tm_mday << "," <<
        p->tm_hour << ":" << p->tm_min << ":" << p->tm_sec << std::endl;
      }

      move_base_cancel_ = false;
      initialized_ = true;
    }
    else
    {
        ROS_INFO("This planner has already been initialized... doing nothing");
    }

  }

  DetectPlanner::~DetectPlanner()
  {
    ROS_WARN_STREAM("Aaction server " << action_name_ << " shutdown! ");
  }

  void DetectPlanner::executeCB(const robot_msg::auto_elevatorGoalConstPtr &goal)
  {
    if (initialized_ == true)
    {
        feedback_.feedback = robot_msg::auto_elevatorFeedback::NONE;
        feedback_.feedback_text = "action server init";
        as_.publishFeedback(feedback_);
    }
    else{
       initialize();
       initialized_ = true;
       feedback_.feedback = robot_msg::auto_elevatorFeedback::NONE;
       feedback_.feedback_text = "action server init";
       as_.publishFeedback(feedback_);
    }

    geometry_msgs::Twist vel;
    geometry_msgs::Pose takePoint, waitPoint;
    bool mode;

    takePoint = goal->takepose;
    waitPoint = goal->waitpose;
    mode = goal->mode;
    log_ << "received w.x = " << waitPoint.position.x << std::endl;
    log_ << "received w.y = " << waitPoint.position.y << std::endl;
    log_ << "received t.x = " << takePoint.position.x << std::endl;
    log_ << "received t.y = " << takePoint.position.y << std::endl;
    log_ << "into elevator ? " << mode << std::endl;

    feedback_.feedback = robot_msg::auto_elevatorFeedback::SET_NEW_GOAL;
    feedback_.feedback_text = "Receive new instructions";
    as_.publishFeedback(feedback_);

    ros::Rate r(10);
    while (ros::ok())
    {
      if(as_.isPreemptRequested())
      {
        if(as_.isNewGoalAvailable())
        {
          robot_msg::auto_elevatorGoal new_goal = *as_.acceptNewGoal();

          takePoint = new_goal.takepose;
          waitPoint = new_goal.waitpose;
          mode = new_goal.mode;
          recivedNewGoalTime = ros::Time::now().sec;
          log_ << "robot should arrvied target floor !" << std::endl;
          log_ << "received target w.x = " << waitPoint.position.x << std::endl;
          log_ << "received target w.y = " << waitPoint.position.y << std::endl;
          log_ << "received target t.x = " << takePoint.position.x << std::endl;
          log_ << "received target t.y = " << takePoint.position.y << std::endl;
          log_ << "out elevator ? " << !mode << std::endl;
          feedback_.feedback = robot_msg::auto_elevatorFeedback::SET_NEW_GOAL;
          feedback_.feedback_text = "Receive new instructions";
          as_.publishFeedback(feedback_);
          ROS_INFO_THROTTLE(1, "detect planner, recive a new goal");
        }
        else {
          vel.linear.x = 0.0;
          vel.angular.z = 0.0;
          vel_pub_.publish(vel);
          feedback_.feedback = robot_msg::auto_elevatorFeedback::CANCLED;
          feedback_.feedback_text = "detect planner cancled!";
          as_.publishFeedback(feedback_);
          ROS_INFO("%s: Preempted", action_name_.c_str());
          result_.result = true;
          as_.setPreempted(result_,"DetectPlanner Preempted");
          return;
        }
      }
      //执行主要逻辑
      runPlan(takePoint,waitPoint,mode);
      as_.publishFeedback(feedback_);

      if(feedback_.feedback == robot_msg::auto_elevatorFeedback::SUCCESS)
      {
        result_.result = true;
        as_.setSucceeded(result_);
        ROS_INFO("---take elevator success---");
        return ;
      }
      if(feedback_.feedback == robot_msg::auto_elevatorFeedback::FAILURE)
      {
        result_.result = false;
        as_.setAborted(result_, "failed, defalut state i don't know why");
        ROS_ERROR("failed, defalut state i don't know why");
        ROS_INFO("---take elevator failure---");
        return ;
      }
        if (feedback_.feedback == robot_msg::auto_elevatorFeedback::FAILURE_FULL)
        {
            result_.result = false;
            as_.setAborted(result_, "full-failed ,waiting next elevator");
            ROS_ERROR("full-failed ,waiting next elevator");
            ROS_INFO("---take elevator failure---");
            return ;
        }
      r.sleep();
    }
  }

  bool DetectPlanner::runPlan(geometry_msgs::Pose takePoint, geometry_msgs::Pose waitPoint, bool mode) {

      if (!initialized_) {
          ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
          feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE;
          feedback_.feedback_text = "not init";
          as_.publishFeedback(feedback_);
          return false;
      }

      move_base_cancel_ = false;

      ROS_INFO_ONCE("---detect planner started!---");


      //部分变量声明和初始化
      bool go_forward = false;
      //bool cartoJump  = false;
      geometry_msgs::Twist cmd_vel;
      double start_time, end_time, interval_time;
      double angle_diff;
      geometry_msgs::Pose global_pose;
      double distance, distance2, totalDistance, toleranceDistance;
      double odomDistance = 0.0;
      geometry_msgs::Pose  odomPoseNow;
      double dp2, dp3;
      toleranceDistance = 0.1;

      dp2 = (2 * elevatorLong_ / 3) + robotRadius_ + toleranceDistance; //乘梯点位默认设置在电梯往里2/3处
      dp3 = (2 * elevatorLong_ / 3) - robotRadius_ - toleranceDistance; //乘梯点位默认设置在电梯往里2/3处

      global_pose = carto_data_.pose;
      odomPoseNow = odom_data_.pose.pose;


      totalDistance = Distance(waitPoint,takePoint); // 两点之间全长
      distance = Distance(global_pose,takePoint); // 机器人距离乘梯点的距离
      distance2 = Distance(global_pose,waitPoint); // 机器人距离候梯点的距离

      //进入状态机
      switch (state_) {
          case OUTDOOR_ANGLE_ADJ: {
              //调整机器人转角方向
              feedback_.feedback = robot_msg::auto_elevatorFeedback::POSE_ADJ;
              feedback_.feedback_text = "enter elevator";
              as_.publishFeedback(feedback_);

              angle_diff = normalizeAngle(updateAngleDiff(carto_data_, takePoint), -M_PI, M_PI);
              ROS_INFO_ONCE("Part1 angle diff =  %.2frad", angle_diff);
              ROS_INFO_ONCE("Robot initial posture correction start");

              if (fabs(angle_diff) > 0.1) {
                  cmd_vel.angular.z = (fabs(angle_diff) <= 0.5) ? angle_diff : (angle_diff > 0 ? 1 : -1) * 0.5;
                  cmd_vel.linear.x = 0;
                  this->vel_pub_.publish(cmd_vel);
              } else {
                  publishZeroVelocity();
                  robotImuAngle0 = tf::getYaw(imu_data_.orientation); //record now imu angle
                  ROS_INFO("Robot initial posture correction end");
                  DETECT_PLANNER_LOG("Part1 end");
                  state_ = GO_STRAIGHT_OUTDOOR;
              }
              ros::spinOnce();
              break;
          }
          case GO_STRAIGHT_OUTDOOR: {
              //直行到电梯门口
              feedback_.feedback = robot_msg::auto_elevatorFeedback::ENTER_ELEVATOR;
              feedback_.feedback_text = "enter elevator";
              as_.publishFeedback(feedback_);

              go_forward = farCango;
              if (!go_forward) {
                  publishZeroVelocity();
                  robotStop = true;
                  ros::Rate r2(1);
                  start_time = ros::Time::now().sec;
                  interval_time = 0;
                  while (ros::ok() && interval_time < 10 && !go_forward) {
                      ROS_INFO("please move your body");
                      publishZeroVelocity();
                      end_time = ros::Time::now().sec;
                      interval_time = end_time - start_time;
                      std::cout << "waiting you move time = " << interval_time  << "s"<< std::endl;
                      r2.sleep();
                      go_forward = farCango;
                  }
                  if (interval_time >= 10) {
                      publishZeroVelocity();
                      ROS_INFO("Unable to enter elevator, return to origin!");
                      global_pose = carto_data_.pose;
                      distance2 = Distance(global_pose,waitPoint); //已经行走的距离
                      ROS_INFO_ONCE("Part2 Robot to waitPoint distance =  %.2fm", distance2);
                      goback(distance2);
                      publishZeroVelocity();
                      ROS_INFO("returned waiting point");
                      DETECT_PLANNER_LOG("returned waiting point");
                      feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE_FULL;
                      feedback_.feedback_text = "failure full, return waiting point";
                      as_.publishFeedback(feedback_);

                      return false;//退出程序
                  }
              }
              else {
                  //更新已走距离
                  if (robotStop == true)
                  {
                      t1 = ros::Time::now().sec ;
                      ros::Rate r2(1);
                      r2.sleep();
                  }
                  global_pose = carto_data_.pose;
                  distance = Distance(global_pose,takePoint); // 机器人距离乘梯点的距离
                  distance2 = Distance(global_pose,waitPoint); // 机器人距离候梯点的距离
                  ROS_INFO_ONCE("Part2 Robot to takePoint distance =  %.2fm", distance);
                  angle_diff = normalizeAngle(updateAngleDiff(carto_data_, takePoint), -M_PI, M_PI);
                  double k, b;
                  k = 2; //速度平滑斜率
                  b = 0.1;
                  if (distance >= dp2) //没到电梯口
                  {
                      if (fabs(angle_diff) > 1) {
                          cmd_vel.angular.z = (angle_diff > 0 ? 1 : -1) * 0.2;
                      }else{
                          cmd_vel.angular.z = (fabs(angle_diff) > 0.05 && fabs(angle_diff) < 1) ? (angle_diff / 2) : angle_diff;
                      }

                      if(distance2 <= 0.6)
                      {
                              t2 = ros::Time::now().sec;
                              t = t2 - t1;
                              cmd_vel.linear.x = (k * t / 10 + b) >= MaxSpeed ? MaxSpeed : k * t / 10 + 0.05;
                              robotStop = false;
                      }
                      else if(distance2 > 0.6 && distance2 <= totalDistance - dp2)
                      {
                              t2 = ros::Time::now().sec;
                              t = t2 - t1;
                              cmd_vel.linear.x = (k * t / 10 + 0.05) >= MaxSpeed ? MaxSpeed : k * t / 10 + 0.05;
                              robotStop = false;
                      }
                      this->vel_pub_.publish(cmd_vel);
                  } else {
                      t = 0.0;
                      DETECT_PLANNER_LOG("Part2 end");
                      state_ = GO_STRAIGHT_ACROSSDOOR;
                  }
              }
              ros::spinOnce();
              break;
          }
          case GO_STRAIGHT_ACROSSDOOR: {
              //直行进入电梯
              feedback_.feedback = robot_msg::auto_elevatorFeedback::ENTER_ELEVATOR;
              feedback_.feedback_text = "enter elevator";
              as_.publishFeedback(feedback_);

              go_forward = farCango;
              if (!go_forward) {
                  robotStop = true;
                  publishZeroVelocity();
                  ros::Rate r2(1);
                  start_time = ros::Time::now().sec;
                  interval_time = 0;
                  while (ros::ok() && interval_time < 2 && !go_forward) {
                      ROS_INFO("please move your body");
                      publishZeroVelocity();
                      end_time = ros::Time::now().sec;
                      interval_time = end_time - start_time;
                      std::cout << "waiting you move time = " << interval_time << "s" << std::endl;
                      r2.sleep();
                      go_forward = farCango;
                  }
                  if (interval_time >= 2) {
                      publishZeroVelocity();
                      ROS_INFO("Unable to enter elevator, return to origin!");
                      global_pose = carto_data_.pose;
                      distance2 = Distance(global_pose,waitPoint); // 机器人距离候梯点的距离
                      ROS_INFO_ONCE("Part 3 Robot to waitPoint distance  =  %.2fm", distance2);
                      goback(distance2);
                      publishZeroVelocity();
                      ROS_INFO("returned waiting point");
                      DETECT_PLANNER_LOG("returned waiting point");
                      feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE_FULL;
                      feedback_.feedback_text = "failure full, return waiting point";
                      as_.publishFeedback(feedback_);

                      return false;//退出程序
                  }
              }
              else {
                  //更新已走距离
                  global_pose = carto_data_.pose;
                  distance = Distance(global_pose,takePoint); // 机器人距离乘梯点的距离
                  distance2 = Distance(global_pose,waitPoint); // 机器人距离候梯点的距离
                  ROS_INFO_ONCE("Part3 Robot to takePoint distance = %.2fm", distance);
                  angle_diff = normalizeAngle(updateAngleDiff(carto_data_, takePoint), -M_PI, M_PI);

                  double k, b;
                  k = 2;
                  b = 0.15;
                  if (distance >= dp3 || distance2 <= (toleranceDistance - dp3)) //没到电梯里
                  {
                      if (fabs(angle_diff) > 1) {
                          cmd_vel.angular.z = (angle_diff > 0 ? 1 : -1) * 0.2;
                      }else{
                          cmd_vel.angular.z = (fabs(angle_diff) > 0.05 && fabs(angle_diff) < 1) ? (angle_diff / 2) : angle_diff;
                      }
                      cmd_vel.linear.x  = (robotStop == true) ? 0.15 : ((distance * k + b) >= MaxSpeed ? MaxSpeed : distance * k + b);
                      this->vel_pub_.publish(cmd_vel);
                  } else if (distance < dp3 && distance2 > (toleranceDistance - dp3)) {
                      ROS_INFO("Robot came in!");
                      DETECT_PLANNER_LOG("Part3 end");
                      state_ = GO_STRAIGHT_INDOOR;
                  }
              }
              ros::spinOnce();
              break;
          }
          case GO_STRAIGHT_INDOOR: {
              //电梯内小范围前进，到达乘梯点，到不了就停下。
              feedback_.feedback = robot_msg::auto_elevatorFeedback::ENTER_ELEVATOR;
              feedback_.feedback_text = "enter elevator";
              as_.publishFeedback(feedback_);
              go_forward = farCango;
              if (!go_forward) {
                  ROS_INFO("The robot stops early when it encounters an obstacle!");
                  publishZeroVelocity();
                  state_ = INELE_ANGLE_ADJ;
              } else {
                  //更新已走距离和角度
                  global_pose = carto_data_.pose;
                  distance = Distance(global_pose,takePoint); // 机器人距离乘梯点的距离
                  distance2 = Distance(global_pose,waitPoint); // 机器人距离候梯点的距离
                  ROS_INFO_ONCE("Part4 in ele, Robot to takePoint distance =  %.2fm", distance);
                  angle_diff = normalizeAngle(updateAngleDiff(carto_data_, takePoint), -M_PI, M_PI);
                  double k = 2;
                  double b = 0.15;
                  if (distance > toleranceDistance && distance2 < (totalDistance - toleranceDistance)) {
                      if (fabs(angle_diff) > 1) {
                          cmd_vel.angular.z = (angle_diff > 0 ? 1 : -1) * 0.2;
                      }else{
                          cmd_vel.angular.z = (fabs(angle_diff) > 0.05 && fabs(angle_diff) < 1) ? (angle_diff / 2) : angle_diff;
                      }
                      if(distance >= 0.1)
                      {
                          cmd_vel.linear.x = robotStop == true ? 0.15 : ((distance * k + b) >= MaxSpeed ? MaxSpeed : distance * k + b);
                      }
                      else if (distance < 0.1  && distance2 <= totalDistance)
                      {
                          cmd_vel.linear.x = 0.1;
                      }
                      else
                      {
                          cmd_vel.linear.x = 0;
                      }
                      this->vel_pub_.publish(cmd_vel);
                  }
                  else {
                      publishZeroVelocity();
                      robotStop = false;
                      DETECT_PLANNER_LOG("Part4 end");
                      state_ = INELE_ANGLE_ADJ;
                  }
              }
              ros::spinOnce();
              break;
          }
          case INELE_ANGLE_ADJ: {
              //调整机器人转角方向
              global_pose = carto_data_.pose;
              angle_diff = normalizeAngle(updateAngleDiff(carto_data_, waitPoint), -M_PI, M_PI);
              ROS_INFO_ONCE("Part5 angle diff  =  %.2frad", angle_diff);
              ROS_INFO_ONCE("Robot second posture correction start");
              if (fabs(angle_diff) > 0.1) {
                  cmd_vel.angular.z = (fabs(angle_diff) <= 0.5) ? angle_diff : (angle_diff > 0 ? 1 : -1) * 0.5;
                  cmd_vel.linear.x = 0;
                  this->vel_pub_.publish(cmd_vel);
              }
              else{
                  publishZeroVelocity();
                  ROS_INFO_ONCE("Robot second posture correction end");
                  feedback_.feedback = robot_msg::auto_elevatorFeedback::ENTER_SUCCESS;
                  feedback_.feedback_text = "enter success elevator";
                  as_.publishFeedback(feedback_);

                  robotImuAngle4 = tf::getYaw(imu_data_.orientation) + angle_diff;

                  //use imu record robot angle
                  robotImuAngle1 = tf::getYaw(imu_data_.orientation);
                  robotImuAngleDiff = robotImuAngle1 - robotImuAngle0;
                  ROS_INFO("robotImuAngleDiff = %.2frad",robotImuAngleDiff);
                  if (fabs(fabs(robotImuAngleDiff) - M_PI) > 0.05)
                  {
                      double needTurnAngle;
                      needTurnAngle = (robotImuAngleDiff > 0 ? robotImuAngleDiff - M_PI : robotImuAngleDiff + M_PI);
                      ROS_INFO("need turn angle : %.2frad", needTurnAngle);
                      turnAngle(-needTurnAngle);
                      publishZeroVelocity();
                      robotImuAngle1 = tf::getYaw(imu_data_.orientation);
                      robotImuAngleDiff = robotImuAngle1 - robotImuAngle0;
                      ROS_INFO("now robotImuAngleDiff = %.2frad",robotImuAngleDiff);
                  }
                  robotImuAngle1 = tf::getYaw(imu_data_.orientation);
                  ROS_INFO("Part5 IMU angle first judge end!");
                  DETECT_PLANNER_LOG("Part5 end");
                  state_ = TAKE_ELEVATOR;
              }
              ros::spinOnce();
              break;
          }
          case TAKE_ELEVATOR: {
              if (!mode) {
                  ROS_INFO("Part6 end! Arrived at the target floor.");
                  feedback_.feedback = robot_msg::auto_elevatorFeedback::GET_OUT_ELEVATOR;
                  feedback_.feedback_text = "get out elevator";
                  as_.publishFeedback(feedback_);

                  DETECT_PLANNER_LOG("Part6 end");
                  state_ = IMU_ANGLE_ADJ;
                  robotStop = true; // part7 use time as variable to make velocity
              }
              else{
                  ROS_INFO_ONCE("---Ride in an elevator---");
                  feedback_.feedback = robot_msg::auto_elevatorFeedback::TAKE_THE_ELEVATOR;
                  feedback_.feedback_text = "enter success ,take the elevator";
                  as_.publishFeedback(feedback_);
                  state_ = TAKE_ELEVATOR;
              }
              ros::spinOnce();
              break;
          }
          case IMU_ANGLE_ADJ:  {
              //angle judge
              if (!robotImuAngleJudge)
              {
                  robotImuAngle2 = tf::getYaw(imu_data_.orientation);
                  robotImuAngleDiff = normalizeAngle((robotImuAngle2 - robotImuAngle1), -M_PI, M_PI);
                  ROS_INFO("robotImuAngleDiff = %.2frad",robotImuAngleDiff);
                  if (fabs(robotImuAngleDiff) > 0.05)
                  {
                      turnAngle(-robotImuAngleDiff);
                      robotImuAngle2 = tf::getYaw(imu_data_.orientation);
                      robotImuAngleDiff = normalizeAngle((robotImuAngle2 - robotImuAngle1), -M_PI, M_PI);
                      ROS_INFO("now robotImuAngleDiff = %.2frad",robotImuAngleDiff);
                  }
                      ROS_INFO("Part7 IMU angle second judge end!");
                      robotImuAngleJudge = true;
                      state_ = OUTOF_ELEVATOR;
                      DETECT_PLANNER_LOG("Part7 end");
              }
              ros::spinOnce();
              break;
          }
          case OUTOF_ELEVATOR: {
              //出电梯
              feedback_.feedback = robot_msg::auto_elevatorFeedback::GET_OUT_ELEVATOR;
              feedback_.feedback_text = "get out elevator";
              as_.publishFeedback(feedback_);

              go_forward = farCango;
              if (!go_forward) {
                  robotStop = true;
                  global_pose = carto_data_.pose;
                  distance = Distance(global_pose,takePoint); // 机器人距离乘梯点的距离

                  if (distance >= dp2) {
                      publishZeroVelocity();
                      recivedNewGoalTimeEnd = ros::Time::now().sec;
                      ROS_INFO("out elevator use time = %.2fs", (recivedNewGoalTimeEnd - recivedNewGoalTime));
                      t = 0.0; //reset time
                      recivedNewGoalTimeEnd = 0.0;
                      recivedNewGoalTime = 0.0;
                      robotStop = true;
                      robotImuAngleJudge = false;
                      initOdomStartPose = false;
                      cartoJump = false;
                      delt_p = current_p = previous_p = previous_delt_p = 0.0;
                      delt_o = current_o = previous_o = previous_delt_o = 0.0;

                      ROS_INFO("Part8 out elevator, Robot to waitPoint distance =  %.2fm, to takePoint distance= %.2fm", distance2,distance);
                      ROS_INFO("detect planner end in advance");
                      DETECT_PLANNER_LOG("Part8 end, detect planner end in advance");
                      feedback_.feedback = robot_msg::auto_elevatorFeedback::SUCCESS;
                      feedback_.feedback_text = "detect planner end in advance!";
                      as_.publishFeedback(feedback_);
                      state_ = OUTDOOR_ANGLE_ADJ;

                      return true;
                  } else {
                      publishZeroVelocity();
                      ROS_INFO("please move your body");
                      //angle judge
                      robotImuAngle3 = tf::getYaw(imu_data_.orientation);
                      robotImuAngleDiff = normalizeAngle((robotImuAngle3 - robotImuAngle2), -M_PI, M_PI);
                      ROS_INFO("robotImuAngleDiff = %.2frad",robotImuAngleDiff);

                      angle_diff = -robotImuAngleDiff;

                      ROS_INFO_ONCE("Part8 angle diff  =  %.2frad", angle_diff);
                      if (fabs(angle_diff) > 0.1) {
                          if (fabs(angle_diff) > 1) {
                              cmd_vel.angular.z = (angle_diff > 0 ? 1 : -1) * 0.2;
                          }else{
                              cmd_vel.angular.z = (fabs(angle_diff) > 0.05 && fabs(angle_diff) < 1) ? (angle_diff / 2) : angle_diff;
                          }
                          cmd_vel.linear.x = 0;
                          this->vel_pub_.publish(cmd_vel);
                      }
                  }
              }
              else {
                  if (robotStop == true)
                  {
                      t1 = ros::Time::now().sec;
                      ros::Rate r2(1);
                      r2.sleep();
                  }

                  if (cartoJump == false)
                  {
                      global_pose = carto_data_.pose;
                      distance = Distance(global_pose,takePoint); // 机器人距离乘梯点的距离
                      distance2 = Distance(global_pose,waitPoint); // 机器人距离候梯点的距离
                  }
                  else{
                      distance = previous_p + delt_o;
                      ROS_INFO("odom + carto distance = %.2f", distance);
                  }
                  current_p = distance;
                  if (previous_p == 0.0)
                  {
                      previous_p = current_p;
                  }
                  delt_p = current_p - previous_p;

                  if (delt_p > 3 * fabs(previous_delt_p) && !cartoJump && fabs(delt_p - previous_delt_p) > 0.19 && fabs(previous_delt_p) > 0.03) //carto jump!!!
                  {
                      distance = previous_p + delt_o;
                      current_p = distance;
                      cartoJump = true;
                      ROS_ERROR("carto data jump!!!");
                      ROS_INFO("delt_p = %.2f, previous_delt_p = %.2f ", delt_p, previous_delt_p);
                      ROS_WARN("carto distance = %.2f", distance);
                      // TODO topic pub
                  }
                  previous_p = current_p;
                  previous_delt_p = delt_p;

                  odomPoseNow = odom_data_.pose.pose;
                  if (!initOdomStartPose )
                  {
                      odomPoseStart = odomPoseNow;
                      initOdomStartPose = true;
                  }

                  odomPoseNow = odom_data_.pose.pose;
                  odomDistance = Distance(odomPoseNow,odomPoseStart);
                  current_o = odomDistance;
                  delt_o = current_o - previous_o;
                  if (delt_o > 3 * fabs(previous_delt_o) && fabs(delt_o - previous_delt_o) > 0.1 && fabs(previous_delt_o) > 0.04)
                  {
                      ROS_ERROR("odom data jump !!!");
                  }
                  previous_o = current_o;
                  previous_delt_o = delt_o;

                  //ROS_INFO("distanceP = %.2f, distanceO = %.2f",distance, odomDistance);
                  //ROS_WARN("delt_p = %.2f, delt_o = %.2f", delt_p, delt_o);

                  robotImuAngle3 = tf::getYaw(imu_data_.orientation);
                  robotImuAngleDiff = normalizeAngle((robotImuAngle3 - robotImuAngle4), -M_PI, M_PI);
                  //ROS_INFO("robotImuAngleDiff = %.2frad",robotImuAngleDiff);
                  angle_diff = -robotImuAngleDiff;


                  double k1, k2, b1, b2;
                  k1 = 2;
                  k2 = 1.5;
                  b1 = 0.1;
                  b2 = 0.05;

                  if (distance < totalDistance) {
                      if (fabs(angle_diff) > 1) {
                          cmd_vel.angular.z = (angle_diff > 0 ? 1 : -1) * 0.2;
                      }else{
                          cmd_vel.angular.z = (fabs(angle_diff) > 0.05 && fabs(angle_diff) < 1) ? (angle_diff / 2) : angle_diff;
                      }
                      if (distance <= totalDistance - 0.2)
                      {
                              t2 = ros::Time::now().sec;
                              t = t2 - t1;
                              cmd_vel.linear.x = (k1 * t / 10 + 0.05) >= MaxSpeed ? MaxSpeed : k1 * t / 10 + 0.05;
                              robotStop = false;
                      }else{
                          cmd_vel.linear.x = (k2 * distance2 + b2) >= MaxSpeed ? MaxSpeed : k2 * distance2 + b2;
                      }
                      this->vel_pub_.publish(cmd_vel);
                  }
                  else{
                  recivedNewGoalTimeEnd = ros::Time::now().sec;
                  ROS_INFO("out elevator use time = %.2fs", (recivedNewGoalTimeEnd - recivedNewGoalTime));
                  if (recivedNewGoalTimeEnd - recivedNewGoalTime <= 5)
                  {
                      publishZeroVelocity();
                      ROS_INFO("I recived a wrong goal! waitting new goal ...");
                  }
                  else
                  {
                      publishZeroVelocity();
                      t = 0.0; //reset time
                      recivedNewGoalTimeEnd = 0.0;
                      recivedNewGoalTime = 0.0;
                      robotStop = true;
                      robotImuAngleJudge = false;
                      initOdomStartPose = false;
                      cartoJump = false;
                      delt_p = current_p = previous_p = previous_delt_p = 0.0;
                      delt_o = current_o = previous_o = previous_delt_o = 0.0;

                      ROS_INFO("carto distance to takePoint = %.2fm",distance);
                      ROS_INFO("odom distance to takePoint = %.2fm",odomDistance);
                      ROS_INFO("detect planner end");
                      DETECT_PLANNER_LOG("Part8 end, detect planner end")
                      feedback_.feedback = robot_msg::auto_elevatorFeedback::SUCCESS;
                      feedback_.feedback_text = "detect planner end!";
                      as_.publishFeedback(feedback_);
                      state_ = OUTDOOR_ANGLE_ADJ;

                      return true;
                  }
              }
              }
              ros::spinOnce();
              break;
          }
          default: {
              as_.setAborted(result_ ,"failure, ento default state");
              return false;//退出程序
          }
      }
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
      sensor_msgs::Imu imu_start;
    {
        boost::mutex::scoped_lock lock(this->imu_mutex_);
        imu_start =  imu_data_;
    }

      double angle_start = tf::getYaw(imu_start.orientation);
      double angle_diff = 0;
      sensor_msgs::Imu imu_now;
      double angle_speed = angle > 0 ? 0.1 : -0.1;
      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = angle_speed;

      ros::Rate r(10);
      while(fabs(angle_diff) < std::fabs(angle))
      {
          {
              boost::mutex::scoped_lock lock(this->imu_mutex_);
              imu_now = imu_data_;
          }
          double angle_now = tf::getYaw(imu_data_.orientation);
          angle_diff = std::fabs(angle_now - angle_start);
          angle_diff = normalizeAngle(angle_diff, -M_PI, M_PI);
          vel_pub_.publish(cmd_vel);
          ros::spinOnce();
          r.sleep();

      }
      ROS_INFO("turn angle end");
      return;
/***********************
    ros::Rate r(10);
    double angle_speed = 0.1;
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
    *****************/
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
  double DetectPlanner::Distance(geometry_msgs::Pose PointA, geometry_msgs::Pose PointB) {
      double x_diff ,y_diff;
      x_diff = PointA.position.x - PointB.position.x;
      y_diff = PointA.position.y - PointB.position.y;
      return sqrt((x_diff * x_diff) + (y_diff * y_diff));
  }
  void DetectPlanner::goback(double distance)
  {
    ros::Rate r(10);
    double linear_speed = -0.1;
    double linear_duration = distance / fabs(linear_speed);

    int ticks = int(linear_duration * 10);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    ROS_WARN("back start");
    while(ticks > 0)
    {
        ticks --;
        cmd_vel.linear.x = linear_speed;
        vel_pub_.publish(cmd_vel);
        r.sleep();
    }
    ROS_WARN("back end");
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
    ROS_WARN("pub 0 speed");
    return;
  }
  void DetectPlanner::movebaseCancelCallback(const actionlib_msgs::GoalID::ConstPtr &msg)
  {
    boost::mutex::scoped_lock lock(this->cancle_mutex_);
    move_base_cancel_ = true;
    publishZeroVelocity();
    DETECT_PLANNER_LOG("detect planner cancle");
    return;
  }
  void DetectPlanner::cartoCallback(const robot_msg::SlamStatus::ConstPtr &msg)
  {
    ROS_INFO_ONCE("carto data recevied");
    boost::mutex::scoped_lock lock(this->carto_mutex_);
    this->carto_data_ = *msg;
  }
  void DetectPlanner::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
  {
      boost::mutex::scoped_lock lock(this->imu_mutex_);
      this->imu_data_ = *msg;
  }

  void DetectPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
      boost::mutex::scoped_lock lock(this->odom_mutex_);
      this->odom_data_ = *msg;
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
  bool DetectPlanner::HaveObstacles(geometry_msgs::Polygon sensor_point, double x, double y)
  {
      boost::mutex::scoped_lock  lock(this->state_mutex_);
      std::vector<geometry_msgs::Point32>::iterator iter;
      for(iter = sensor_point.points.begin(); iter != sensor_point.points.end(); iter++)
      {
          if (iter->x < x && fabs(iter -> y) < y )
          {
              return false;
          }
      }
      return true;
  }
  void DetectPlanner::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    ROS_INFO_ONCE("DetectPlanner scan data recevied");
    boost::mutex::scoped_lock lock(this->laser_mutex_);

    receive_laser_time_ = ros::Time::now();//重置接收到laser的时间
    laser_data_ = *msg;

    //point_vec_.points.clear();
    closeHavePoint = false;
    midHavePoint = false;
    farHavePoint = false;
    double min_angle = laser_data_.angle_min;
    double max_angle = laser_data_.angle_max;
    double laser_angle_increment = laser_data_.angle_increment; //雷达数据的角度间隔
    size_t laser_point_size = laser_data_.ranges.size();//雷达数据一帧的点数
    double theta = 0; //转角置为０；
    geometry_msgs::Point32 tem_point;

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
         tem_point.x = laser_data_.ranges[i] * cos(theta);
         tem_point.y = laser_data_.ranges[i] * sin(theta);
         tem_point.z = 0;
         if(tem_point.x > -0.1 && tem_point.x < 1.0 && fabs(tem_point.y) < 0.5)
         {
             //this->point_vec_.points.push_back(tem_point);
             if(fabs(tem_point.x) <= 0.2 && fabs(tem_point.y) < 0.25)
             {
                closeHavePoint = true;
                midHavePoint = true;
                farHavePoint = true;
             }
             else if (fabs(tem_point.x) <= 0.3 && fabs(tem_point.y) < 0.25)
             {
                 midHavePoint = true;
                 farHavePoint = true;
             }
             else if (fabs(tem_point.x) <= 0.45 && fabs(tem_point.y) < 0.25)
             {
                 farHavePoint = true;
             }
         }
       }
      }
    }
    closeCango = (closeHavePoint == true) ? false : true;
    midCango = (midHavePoint == true) ? false : true;
    farCango = (farHavePoint == true) ? false : true;
  }
};
