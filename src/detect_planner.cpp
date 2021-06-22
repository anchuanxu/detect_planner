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
    state_ = NAV_TAKE;
    as_.start();
    ROS_INFO_STREAM("Action server " << action_name_ << " start!");
  }

  void DetectPlanner::initialize(){
    if(!initialized_){
      laser_sub_ = ah_.subscribe<sensor_msgs::LaserScan>("/scan",10,boost::bind(&DetectPlanner::scanCallback,this,_1));
      carto_sub_ = ah_.subscribe<robot_msg::SlamStatus>("/slam_status",20,boost::bind(&DetectPlanner::cartoCallback,this,_1));
    //   mbc_sub_ = ah_.subscribe<actionlib_msgs::GoalID>("/move_base/cancel", 10, boost::bind(&DetectPlanner::movebaseCancelCallback, this, _1));
    //   imu_sub_ = ah_.subscribe<sensor_msgs::Imu>("/imu", 10, boost::bind(&DetectPlanner::imuCallback,this, _1));
    //   odom_sub_ = ah_.subscribe<nav_msgs::Odometry>("/odom",10,boost::bind(&DetectPlanner::odomCallback,this, _1));
      navigation_state_sub_ = ah_.subscribe<robot_msg::FeedBack>("/navi_state", 100, boost::bind(&DetectPlanner::naviStateCallback,this, _1));
      elevator_state_sub_ = ah_.subscribe<robot_msg::ElevatorState>("/elevatorState", 10, boost::bind(&DetectPlanner::elevatorStateCallback, this, _1));

      vel_pub_ = ah_.advertise<geometry_msgs::Twist>("/cmd_vel",10);
      //vel_pub_ = ah_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",10);
      goal_pub_  = ah_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
      goal_cancel_pub_ = ah_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

      ph_.param<std::string>("base_frame", base_frame_, std::string("base_link"));
      ph_.param<std::string>("laser_frame", laser_frame_, std::string("laser"));
      ph_.param<std::string>("map_frame", map_frame_, std::string("map"));
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
      takePointCanstand_ = true;
      isJudged_ = false;
      isUpdateNaviState_ = false;

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
    ROS_INFO("--- DetectPlanner::executeCB ---");
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

    // flag intialization
    takePointCanstand_ = true;
    isJudged_ = false;
    isUpdateNaviState_ = false;
    isPublishGoal_ = false;

    takePoint = goal->takepose;
    waitPoint = goal->waitpose;
    mode = goal->mode;
    log_ << "received w.x = " << waitPoint.position.x << std::endl;
    log_ << "received w.y = " << waitPoint.position.y << std::endl;
    log_ << "received t.x = " << takePoint.position.x << std::endl;
    log_ << "received t.y = " << takePoint.position.y << std::endl;
    log_ << "into elevator ? " << mode << std::endl;

    angle_T_to_W_ = std::atan2(waitPoint.position.y - takePoint.position.y, waitPoint.position.x - takePoint.position.x);
    angle_W_to_T_ = std::atan2(takePoint.position.y - waitPoint.position.y, takePoint.position.x - waitPoint.position.x);
    printf("angle_T_to_W_: %f, angle_W_to_T_: %f\n", angle_T_to_W_, angle_W_to_T_);

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
          angle_T_to_W_ = std::atan2(waitPoint.position.y - takePoint.position.y, waitPoint.position.x - takePoint.position.x);
          angle_W_to_T_ = std::atan2(takePoint.position.y - waitPoint.position.y, takePoint.position.x - waitPoint.position.x);
          printf("angle_T_to_W_: %f, angle_W_to_T_: %f\n", angle_T_to_W_, angle_W_to_T_);
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
        as_.setSucceeded(result_);
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

      geometry_msgs::PoseStamped target_pose;

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

      dp2 = (elevatorLong_ / 2) + robotRadius_ + toleranceDistance;
      dp3 = (elevatorLong_ / 2) - robotRadius_ - toleranceDistance;

      global_pose = carto_pose_.pose;
      odomPoseNow = odom_data_.pose.pose;


      totalDistance = Distance(waitPoint,takePoint); // 两点之间全长
      distance = Distance(global_pose,takePoint); // 机器人距离乘梯点的距离
      distance2 = Distance(global_pose,waitPoint); // 机器人距离候梯点的距离

      //进入状态机
      switch (state_) {
          case NAV_TAKE: {
              feedback_.feedback = robot_msg::auto_elevatorFeedback::ENTER_ELEVATOR;
              feedback_.feedback_text = "navi to take point";
              as_.publishFeedback(feedback_);

              if (!isPublishGoal_)
              {
                  target_pose.header.frame_id = map_frame_;
                  target_pose.pose = takePoint;
                  geometry_msgs::Quaternion temp_q = tf::createQuaternionMsgFromYaw(angle_T_to_W_);
                  target_pose.pose.orientation = temp_q;
                  goal_pub_.publish(target_pose);
                  ROS_INFO("publish take point");
                  isPublishGoal_ = true;
              }

              naviStateFeedback_ = navi_state_.feedback;
              if ((naviStateFeedback_ == 100) || (naviStateFeedback_ == 103) ) // in planning or controlling 
              {
                  isUpdateNaviState_ = true;
              }

              if (takePointCanstand_ || isJudged_)
              {
                  isJudged_ = true;
                  naviStateFeedback_ = navi_state_.feedback;
                  if (naviStateFeedback_ == 104 && isUpdateNaviState_)
                  {
                      ROS_INFO("into elevator take point success");
                      feedback_.feedback = robot_msg::auto_elevatorFeedback::ENTER_SUCCESS;
                      feedback_.feedback_text = "enter success elevator";
                      as_.publishFeedback(feedback_);
                      state_ = TAKE;
                      isPublishGoal_ = false;
                      isJudged_ = false;
                      isUpdateNaviState_ = false;
                  }
                  else if(naviStateFeedback_ == 101 || naviStateFeedback_ == 102 || naviStateFeedback_ == 106 || naviStateFeedback_ == 107){
                      double distanceRto = Distance(carto_pose_.pose, takePoint);
                      if (distanceRto < 0.5) //机器人身子已进入电梯，0.1为容错, 距离waitpoint小于1m，认为已经进入电梯
                      {
                          actionlib_msgs::GoalID  goalCancel;
                          goal_cancel_pub_.publish(goalCancel);
                          ROS_INFO("into elevator door stand success， nearby");
                          feedback_.feedback = robot_msg::auto_elevatorFeedback::ENTER_SUCCESS;
                          feedback_.feedback_text = "enter success elevator";
                          as_.publishFeedback(feedback_);
                          state_ = TAKE;
                          isPublishGoal_ = false;
                          isUpdateNaviState_ = false;
                          isJudged_ = false;
                      }
                      else {
                          std::cout << "distanceRtoT = " << distanceRto << "; 2 * robotRadius_ + 0.1 = " << 2 * robotRadius_ + 0.1 << std::endl;
                          state_ = RETURN_WAIT_AREA;
                          isPublishGoal_ = false;
                          isUpdateNaviState_ = false;
                          isJudged_ = false;
                      }
                  }
                  else{
                      state_ = NAV_TAKE;
                  }
              }
              else{
                      /*if (doorCanstand_)
                      {
                          state_ = NAV_DOOR;
                          isPublishGoal_ = false;
                          isUpdateNaviState_ = false;
                          isJudged_ = false;
                      } else */{
                          state_ = RETURN_WAIT_AREA;
                          isPublishGoal_ = false;
                          isUpdateNaviState_ = false;
                          isJudged_ = false;
                      }
              }
              break;
          }
          case RETURN_WAIT_AREA: {
              feedback_.feedback = robot_msg::auto_elevatorFeedback::ENTER_ELEVATOR;
              feedback_.feedback_text = "navi to wait point";
              as_.publishFeedback(feedback_);

              if (!isPublishGoal_)
              {
                  target_pose.header.frame_id = map_frame_;
                  target_pose.pose = waitPoint;
                  goal_pub_.publish(target_pose);
                  ROS_INFO("publish wait point");
                  isPublishGoal_ = true;
              }

              naviStateFeedback_ = navi_state_.feedback;
              if ((naviStateFeedback_ == 100) || (naviStateFeedback_ == 103) ) // in planning or controlling 
              {
                  isUpdateNaviState_ = true;
              }

              naviStateFeedback_ = navi_state_.feedback;
              if (navi_state_.feedback == 104 && isUpdateNaviState_)
              {
                  ROS_INFO("into elevator failed");
                  feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE; //TODO: 这里可以返回满员，但是不知道userlogic有没有处理
                  feedback_.feedback_text = "failure full, return waiting point";
                  as_.publishFeedback(feedback_);
                  state_ = NAV_TAKE;
                  isPublishGoal_ = false;
                  isUpdateNaviState_ = false;
                  return  false;
              } else if (navi_state_.feedback == 101 || navi_state_.feedback == 102){
                  double distanceRtoW;
                  distanceRtoW = Distance(carto_pose_.pose, waitPoint);
                  if (distanceRtoW > 2 * robotRadius_)
                  {
                      actionlib_msgs::GoalID  goalCancel;
                      goal_cancel_pub_.publish(goalCancel);
                      ROS_INFO("into elevator failed");
                      feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE;//TODO: 这里可以返回满员，但是不知道userlogic有没有处理
                      feedback_.feedback_text = "failure full, return waiting point";
                      as_.publishFeedback(feedback_);
                      state_ = NAV_TAKE;
                      isPublishGoal_ = false;
                      isUpdateNaviState_ = false;
                      return  false;
                  }
                  state_ = RETURN_WAIT_AREA;
              } else{
                  state_ = RETURN_WAIT_AREA;
              }
              break;
          }
          case TAKE: {
              ROS_INFO_ONCE("---Ride in an elevator---");
              feedback_.feedback = robot_msg::auto_elevatorFeedback::TAKE_THE_ELEVATOR;
              feedback_.feedback_text = "enter success ,take the elevator";
              as_.publishFeedback(feedback_);
              state_ = TAKE;

              if (!mode) {
                state_ = NAV_OUT;
                isPublishGoal_ = false;
                isUpdateNaviState_ = false;
              }
              ros::spinOnce();
              break;
          }
          case NAV_OUT: {
              //出电梯
              feedback_.feedback = robot_msg::auto_elevatorFeedback::GET_OUT_ELEVATOR;
              feedback_.feedback_text = "get out elevator";
              as_.publishFeedback(feedback_);

              if (!isPublishGoal_)
              {
                  target_pose.header.frame_id = map_frame_;
                  target_pose.pose = waitPoint;
                  geometry_msgs::Quaternion temp_q = tf::createQuaternionMsgFromYaw(angle_T_to_W_);
                  target_pose.pose.orientation = temp_q;
                  goal_pub_.publish(target_pose);
                  ROS_INFO("publish wait point");
                  isPublishGoal_ = true;
              }

              naviStateFeedback_ = navi_state_.feedback;
              if ((naviStateFeedback_ == 100) || (naviStateFeedback_ == 103) ) // in planning or controlling 
              {
                  isUpdateNaviState_ = true;
              }

              if (navi_state_.feedback == 104 && isUpdateNaviState_)
              {
                ROS_INFO("detect planner end in advance");
                isPublishGoal_ = false;
                state_ = NAV_TAKE;
                feedback_.feedback = robot_msg::auto_elevatorFeedback::SUCCESS;
                feedback_.feedback_text = "out elevator success";
                as_.publishFeedback(feedback_);
                return true;
              } 
              
              double distanceRtoW;
              distanceRtoW = Distance(carto_pose_.pose, waitPoint);
              if (distanceRtoW < 2 * robotRadius_)
              {
                  ROS_INFO("detect planner end in advance， nearby");
                  isPublishGoal_ = false;
                  state_ = NAV_TAKE;
                  feedback_.feedback = robot_msg::auto_elevatorFeedback::SUCCESS;
                  feedback_.feedback_text = "out elevator success";
                  as_.publishFeedback(feedback_);
                  return true;
              } 
              /*else if (navi_state_.feedback == 101 || navi_state_.feedback == 102){
                  
                  state_ = NAV_OUT;
              } else*/{
                  state_ = NAV_OUT;
              }

            ros::spinOnce();
            break;
          }
          default: {
              as_.setAborted(result_ ,"failure, enter default state");
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
//   void DetectPlanner::movebaseCancelCallback(const actionlib_msgs::GoalID::ConstPtr &msg)
//   {
//     boost::mutex::scoped_lock lock(this->cancle_mutex_);
//     move_base_cancel_ = true;
//     publishZeroVelocity();
//     DETECT_PLANNER_LOG("detect planner cancle");
//     return;
//   }

  void DetectPlanner::naviStateCallback(const robot_msg::FeedBackConstPtr &msg) {
      ROS_INFO_ONCE("navigation state recevied");
      boost::mutex::scoped_lock lock(this->navi_state_mutex_);
      this->navi_state_ = *msg;
  }

  void DetectPlanner::elevatorStateCallback(const robot_msg::ElevatorStateConstPtr &msg) {
      ROS_INFO_ONCE("elevator state recevied");
      boost::mutex::scoped_lock  lock(this->ele_state_mutex_);
      this->elevator_state_ = *msg;
  }

  void DetectPlanner::cartoCallback(const robot_msg::SlamStatus::ConstPtr &msg)
  {
    ROS_INFO_ONCE("carto data recevied");
    boost::mutex::scoped_lock lock(this->carto_mutex_);
    this->carto_pose_ = *msg;
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
