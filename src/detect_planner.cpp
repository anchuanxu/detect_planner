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

      vel_pub_ = ah_.advertise<geometry_msgs::Twist>("/cmd_vel",10);

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
      record_log_ = false;
      closeCango = false;
      midCango = false;
      farCango = false;

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
      ROS_ERROR("This planner has already been initialized... doing nothing");
  }

  DetectPlanner::~DetectPlanner()
  {
    ROS_WARN_STREAM("Aaction server " << action_name_ << " shutdown! ");
  }

  void DetectPlanner::executeCB(const robot_msg::auto_elevatorGoalConstPtr &goal)
  {
    geometry_msgs::Twist vel;
    geometry_msgs::Pose takePoint, waitPoint;
    int current_floor_, target_floor_;

    takePoint = goal->takepose;
    waitPoint = goal->waitpose;
    current_floor_ = goal->current_floor;
    target_floor_  = goal->target_floor;

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
          current_floor_ = new_goal.current_floor;
          target_floor_  = new_goal.target_floor;
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
      runPlan(takePoint,waitPoint,current_floor_,target_floor_);
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
        as_.setSucceeded(result_);
        ROS_INFO("---take elevator failure---");
        return ;
      }
      r.sleep();
    }
  }

  bool DetectPlanner::runPlan(geometry_msgs::Pose takePoint, geometry_msgs::Pose waitPoint, int current_floor, int target_floor) {

      if (!initialized_) {
          ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
          feedback_.feedback = robot_msg::auto_elevatorFeedback::NONE;
          feedback_.feedback_text = "not init";
          as_.publishFeedback(feedback_);

          return false;
      }

      DETECT_PLANNER_LOG("detect planner start");

      move_base_cancel_ = false;

//      feedback_.feedback = robot_msg::auto_elevatorFeedback::WAITING_ELEVATOR;
//      feedback_.feedback_text = "waiting elevator";
//      as_.publishFeedback(feedback_);


      ROS_INFO_ONCE("---detect planner started!---");
      feedback_.feedback = robot_msg::auto_elevatorFeedback::SET_NEW_GOAL;
      feedback_.feedback_text = "set new goal";
      as_.publishFeedback(feedback_);

      //部分变量声明和初始化
      bool go_forward = false;
      bool intoDone = false;//是否已经完成进入
      geometry_msgs::Twist cmd_vel;
      double start_time, end_time, interval_time;
      double angle_diff;
      geometry_msgs::Pose global_pose;
      double distance, distance2, totalDistance, toleranceDistance;
      double dp2, dp3;

      dp2 = (elevatorLong_ / 2) + robotRadius_ + toleranceDistance;
      dp3 = (elevatorLong_ / 2) - robotRadius_ - toleranceDistance;

      global_pose = carto_data_.pose;

      toleranceDistance = 0.05;
      totalDistance = Distance(waitPoint,takePoint); // 两点之间全长
      distance = Distance(global_pose,takePoint); // 机器人距离乘梯点的距离
      distance2 = Distance(global_pose,waitPoint); // 机器人距离候梯点的距离

      //进入状态机
      switch (state_) {
          case OUTDOOR_ANGLE_ADJ: {
              //调整机器人转角方向

              DETECT_PLANNER_LOG("OUTDOOR_ANGLE_ADJ");

              angle_diff = normalizeAngle(updateAngleDiff(carto_data_, takePoint), -M_PI, M_PI);
              ROS_INFO_ONCE("part1 angle diff =  %.2f", angle_diff);
              ROS_INFO_ONCE("Robot initial posture correction start");

              if (fabs(angle_diff) > 0.1) {
                  if (angle_diff > 0.5) {
                      cmd_vel.angular.z = 0.5;
                  } else if (angle_diff < -0.5) {
                      cmd_vel.angular.z = -0.5;
                  } else if (angle_diff >= -0.5 && angle_diff <= 0.5){
                      cmd_vel.angular.z = angle_diff;
                  }

                  cmd_vel.linear.x = 0;
                  this->vel_pub_.publish(cmd_vel);
              } else {
                  publishZeroVelocity();
                  ROS_INFO("Robot initial posture correction end");
                  DETECT_PLANNER_LOG("Robot initial posture correction end");
                  state_ = GO_STRAIGHT_OUTDOOR;
              }
              ros::spinOnce();
              break;
          }
          case GO_STRAIGHT_OUTDOOR: {
              //直行到电梯门口
              DETECT_PLANNER_LOG("GO_STRAIGHT_OUTDOOR");
              feedback_.feedback = robot_msg::auto_elevatorFeedback::ENTER_ELEVATOR;
              feedback_.feedback_text = "enter elevator";
              as_.publishFeedback(feedback_);
              go_forward = farCango;
              if (!go_forward) {
                  publishZeroVelocity();
                  ros::Rate r2(1);
                  start_time = ros::Time::now().sec;
                  interval_time = 0;
                  while (ros::ok() && interval_time < 10 && !go_forward) {
                      ROS_INFO("please move your body");
                      end_time = ros::Time::now().sec;
                      interval_time = end_time - start_time;
                      std::cout << "waiting you move time = " << interval_time << std::endl;
                      r2.sleep();
                      go_forward = farCango;
                  }
                  if (interval_time >= 10) {
                      publishZeroVelocity();
                      DETECT_PLANNER_LOG("Unable to enter elevator, return to o rigin!");
                      ROS_INFO("Unable to enter elevator, return to origin!");
                      global_pose = carto_data_.pose;
                      distance2 = Distance(global_pose,waitPoint); //已经行走的距离
                      ROS_INFO_ONCE("part2 toWaitPointDistance =  %.2f", distance2);
                      goback(distance2);

                      ROS_INFO("return waiting point");
                      DETECT_PLANNER_LOG("return waiting point");
                      feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE;
                      feedback_.feedback_text = "failure, return waiting point";
                      as_.publishFeedback(feedback_);

                      return false;//退出程序
                  }
              } else {
                  //更新已走距离
                  global_pose = carto_data_.pose;
                  distance = Distance(global_pose,takePoint); // 机器人距离乘梯点的距离
                  distance2 = Distance(global_pose,waitPoint); // 机器人距离候梯点的距离
                  ROS_INFO_ONCE("part2 goal distance =  %.2f", distance);
                  angle_diff = normalizeAngle(updateAngleDiff(carto_data_, takePoint), -M_PI, M_PI);
                  double k, b;
                  k = 0.5; //速度平滑斜率
                  b = 0.05;
                  if (distance >= dp2) //没到电梯口
                  {
                      if (fabs(angle_diff) > 0.05 && fabs(angle_diff) < 1) {
                          cmd_vel.angular.z = (angle_diff / 2);
                      } else {
                          cmd_vel.angular.z = angle_diff;
                      }
                      if(distance > totalDistance && distance2 > 0)
                      {
                          cmd_vel.linear.x = 0.05;
                      }
                      else if(distance2 <= 0.6 && distance2 >= 0 && distance <= totalDistance)
                      {
                          cmd_vel.linear.x = k * distance2 + b;
                      }
                      else if(distance2 > 0.6 && distance2 <= totalDistance - dp2)
                      {
                          cmd_vel.linear.x = 0.35;
                      }
                      this->vel_pub_.publish(cmd_vel);
                  } else {
                      state_ = GO_STRAIGHT_ACROSSDOOR;
                  }
              }
              ros::spinOnce();
              break;
          }
          case GO_STRAIGHT_ACROSSDOOR: {
              //直行进入电梯
              DETECT_PLANNER_LOG("GO_STRAIGHT_ACROSSDOOR");
              feedback_.feedback = robot_msg::auto_elevatorFeedback::ENTER_ELEVATOR;
              feedback_.feedback_text = "enter elevator";
              as_.publishFeedback(feedback_);
              go_forward = midCango;
              if (!go_forward) {
                  publishZeroVelocity();
                  ros::Rate r2(1);
                  start_time = ros::Time::now().sec;
                  interval_time = 0;
                  while (ros::ok() && interval_time < 2 && !go_forward) {
                      ROS_INFO("please move your body");
                      end_time = ros::Time::now().sec;
                      interval_time = end_time - start_time;
                      std::cout << "waiting you move time = " << interval_time << std::endl;
                      r2.sleep();
                      go_forward = midCango;
                  }
                  if (interval_time >= 2) {
                      publishZeroVelocity();
                      DETECT_PLANNER_LOG("Unable to enter elevator, return to origin!");
                      ROS_INFO("Unable to enter elevator, return to origin!");
                      global_pose = carto_data_.pose;
                      distance2 = Distance(global_pose,waitPoint); // 机器人距离候梯点的距离
                      ROS_INFO_ONCE("part 3 toWaitPointDistance  =  %.2f", distance2);
                      goback(distance2);
                      ROS_INFO("return waiting point");
                      DETECT_PLANNER_LOG("return waiting point");
                      feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE;
                      feedback_.feedback_text = "failure, return waiting point";
                      as_.publishFeedback(feedback_);
                      return false;//退出程序
                  }
              } else {
                  //更新已走距离
                  global_pose = carto_data_.pose;
                  distance = Distance(global_pose,takePoint); // 机器人距离乘梯点的距离
                  distance2 = Distance(global_pose,waitPoint); // 机器人距离候梯点的距离
                  ROS_INFO_ONCE("part3 goal distance = %.2f", distance);
                  angle_diff = normalizeAngle(updateAngleDiff(carto_data_, takePoint), -M_PI, M_PI);

                  double k, b;
                  k = 0.3;
                  b = 0.0275;
                  if (distance >= dp3 || distance2 <= (toleranceDistance - dp3)) //没到电梯里
                  {
                      if (fabs(angle_diff) > 0.05 && fabs(angle_diff) < 1) {
                          cmd_vel.angular.z = (angle_diff / 2);
                      } else {
                          cmd_vel.angular.z = angle_diff;
                      }
                      cmd_vel.linear.x = distance * k + b;
                      this->vel_pub_.publish(cmd_vel);
                  } else if (distance < dp3 && distance2 > (toleranceDistance - dp3)) {
                      ROS_INFO("I came in!");
                      DETECT_PLANNER_LOG("I came in!");
                      state_ = GO_STRAIGHT_INDOOR;
                  }
              }
              ros::spinOnce();
              break;
          }
          case GO_STRAIGHT_INDOOR: {
              //电梯内小范围前进，到达乘梯点，到不了就停下。
              DETECT_PLANNER_LOG("GO_STRAIGHT_INDOOR");
              feedback_.feedback = robot_msg::auto_elevatorFeedback::ENTER_ELEVATOR;
              feedback_.feedback_text = "enter elevator";
              as_.publishFeedback(feedback_);
              go_forward = closeCango;
              if (!go_forward) {
                  ROS_INFO("i have obs !");
                  DETECT_PLANNER_LOG("i have obs!");
                  publishZeroVelocity();
                  intoDone = true;
                  ROS_INFO("i am stop here!");
                  DETECT_PLANNER_LOG("i am stop here!");
                  state_ = INELE_ANGLE_ADJ;
              } else {
                  //更新已走距离和角度
                  global_pose = carto_data_.pose;
                  distance = Distance(global_pose,takePoint); // 机器人距离乘梯点的距离
                  distance2 = Distance(global_pose,waitPoint); // 机器人距离候梯点的距离
                  ROS_INFO_ONCE("part4 in ele, goal distance =  %.2f", distance);
                  angle_diff = normalizeAngle(updateAngleDiff(carto_data_, takePoint), -M_PI, M_PI);
                  double k = 0.3;
                  double b = 0.0275;
                  if (distance > toleranceDistance && distance2 < (totalDistance - toleranceDistance)) {
                      if (fabs(angle_diff) > 0.05 && fabs(angle_diff) < 1) {
                          cmd_vel.angular.z = (angle_diff / 2);
                      } else {
                          cmd_vel.angular.z = angle_diff;
                      }
                      if(distance >= 0.075)
                      {
                          cmd_vel.linear.x = distance * k + b;
                      }
                      else if (distance < 0.075 && distance >= 0 && distance2 <= totalDistance)
                      {
                          cmd_vel.linear.x = 0.05;
                      }
                      else if (distance > 0 && distance2 > totalDistance)
                      {
                          cmd_vel.linear.x = 0;
                      }
                      this->vel_pub_.publish(cmd_vel);
                  }
                  if (distance <= toleranceDistance || distance2 >= (totalDistance - toleranceDistance)) {
                      publishZeroVelocity();
                      intoDone = true;
                      state_ = INELE_ANGLE_ADJ;
                  }
              }
              ros::spinOnce();
              break;
          }
          case INELE_ANGLE_ADJ: {
              //调整机器人转角方向
              DETECT_PLANNER_LOG("INELE_ANGLE_ADJ");
              global_pose = carto_data_.pose;
              angle_diff = normalizeAngle(updateAngleDiff(carto_data_, waitPoint), -M_PI, M_PI);
              ROS_INFO_ONCE("part5 angle diff  =  %.2f", angle_diff);
              ROS_INFO_ONCE("Robot second posture correction start");
              if (fabs(angle_diff) > 0.1) {
                  if (angle_diff > 0.5) {
                      cmd_vel.angular.z = 0.5;
                  } else if (angle_diff < -0.5) {
                      cmd_vel.angular.z = -0.5;
                  } else {
                      cmd_vel.angular.z = angle_diff;
                  }
                  cmd_vel.linear.x = 0;
                  this->vel_pub_.publish(cmd_vel);
              }
              else{
                  publishZeroVelocity();
                  ROS_INFO_ONCE("Robot second posture correction end");

                  if (current_floor == target_floor) {
                      ROS_INFO("Arrived at the %d floor", target_floor);
                      log_ << "Arrived at the " << target_floor << " floor!" << std::endl;
                      feedback_.feedback = robot_msg::auto_elevatorFeedback::TAKE_THE_ELEVATOR;
                      feedback_.feedback_text = "take the elevator";
                      as_.publishFeedback(feedback_);

                      state_ = OUTOF_ELEVATOR;
                  }
                  else{
                      ROS_INFO_ONCE("---Ride in an elevator---");
                      DETECT_PLANNER_LOG("---Ride in an elevator---");
                      ROS_INFO_ONCE("current floor is %d, target floor is %d", current_floor, target_floor);
                      state_ = INELE_ANGLE_ADJ;
                  }
              }
              ros::spinOnce();
              break;
          }
          case OUTOF_ELEVATOR: {
              //出电梯
              DETECT_PLANNER_LOG("OUTOF_ELEVATOR");
              feedback_.feedback = robot_msg::auto_elevatorFeedback::GET_OUT_ELEVATOR;
              feedback_.feedback_text = "get out elevator";
              as_.publishFeedback(feedback_);
              go_forward = farCango;
              if (!go_forward) {
                  if (distance > dp2) {
                      publishZeroVelocity();
                      ROS_INFO_ONCE("part6 out elevator,goal2 distance =  %.2f goal distance= %.2f", distance2,distance);
                      ROS_INFO("detect planner end in advance");
                      DETECT_PLANNER_LOG("detect planner end in advance");
                      feedback_.feedback = robot_msg::auto_elevatorFeedback::SUCCESS;
                      feedback_.feedback_text = "detect planner success!";
                      as_.publishFeedback(feedback_);
                  } else {
                      publishZeroVelocity();
                      ROS_INFO("please move your body");
                  }
              } else {
                  global_pose = carto_data_.pose;
                  distance = Distance(global_pose,takePoint); // 机器人距离乘梯点的距离
                  distance2 = Distance(global_pose,waitPoint); // 机器人距离候梯点的距离
                  ROS_INFO_ONCE("part6 out elevator,goal2 distance =  %.2f", distance2);
                  angle_diff = normalizeAngle(updateAngleDiff(carto_data_, waitPoint), -M_PI, M_PI);
                  double k1, k2, b1, b2;
                  k1 = 0.25;
                  k2 = 1.5;
                  b1 = 0.05;
                  b2 = 0.05;
                  if (distance2 < (totalDistance - dp2) && distance > dp2) {
                      intoDone = false;
                  }
                  if (distance2 > toleranceDistance && distance < (totalDistance - toleranceDistance)) {
                      if (fabs(angle_diff) > 0.05 && fabs(angle_diff) < 0.5) {
                          cmd_vel.angular.z = (angle_diff / 2);
                      } else {
                          cmd_vel.angular.z = angle_diff;
                      }
                      if (distance >= 0 && distance <= 1.2)
                      {
                          cmd_vel.linear.x = k1 * distance + b1;
                      }
                      else if (distance > 1.2 && distance <= (totalDistance - 0.2))
                      {
                          cmd_vel.linear.x = 0.35;
                      }
                      else if (distance > (totalDistance - 0.2) && distance2 < 0.2 && distance2 >= toleranceDistance)
                      {
                          cmd_vel.linear.x = k2 * distance2 + b2;
                      }
                      else if (distance2 < toleranceDistance || distance > (totalDistance - toleranceDistance))
                      {
                          cmd_vel.linear.x = 0;
                          publishZeroVelocity();
                      }
                      this->vel_pub_.publish(cmd_vel);
                  } else if (distance2 <= toleranceDistance || distance >= (totalDistance - toleranceDistance)) {
                      publishZeroVelocity();
                      ROS_INFO("detect planner end");
                      DETECT_PLANNER_LOG("detect planner end");
                      feedback_.feedback = robot_msg::auto_elevatorFeedback::SUCCESS;
                      feedback_.feedback_text = "detect planner success!";
                      as_.publishFeedback(feedback_);

                      return true;
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
             if(fabs(tem_point.x) <= 0.15 && fabs(tem_point.y) < 0.25)
             {
                closeHavePoint = true;
                midHavePoint = true;
                farHavePoint = true;
             }
             else if (fabs(tem_point.x) <= 0.2 && fabs(tem_point.y) < 0.25)
             {
                 midHavePoint = true;
                 farHavePoint = true;
             }
             else if (fabs(tem_point.x) <= 0.3 && fabs(tem_point.y) < 0.25)
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
