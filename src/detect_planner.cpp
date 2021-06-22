/*********************************************************************
*
*  Copyright (c) 2020, Chuanxu An, Inc.
*  All rights reserved.
*
*********************************************************************/
#include <detect_planner/detect_planner.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <math.h>

namespace detect_planner
{

  DetectPlanner::DetectPlanner(std::string name, tf2_ros::Buffer &tf)
      : initialized_(false),
        ph_("~"),
        action_name_(name),
        as_(ah_, name, boost::bind(&DetectPlanner::executeCB, this, _1), false)
  {
    //nh_ = nullptr;
    move_base_cancel_ = false;
    initialize();
    state_last_ = IDLE;
    state_ = IDLE;
    as_.start();
    ROS_INFO_STREAM("Action server " << action_name_ << " start!");
  }

  void DetectPlanner::initialize()
  {
    if (!initialized_)
    {
      carto_sub_ = ah_.subscribe<robot_msg::SlamStatus>("/slam_status", 20, boost::bind(&DetectPlanner::cartoCallback, this, _1));
      navigation_state_sub_ = ah_.subscribe<robot_msg::FeedBack>("/navi_state", 100, boost::bind(&DetectPlanner::naviStateCallback, this, _1));
      elevator_state_sub_ = ah_.subscribe<robot_msg::ElevatorState>("/elevatorState", 10, boost::bind(&DetectPlanner::elevatorStateCallback, this, _1));

      vel_pub_ = ah_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
      goal_pub_ = ah_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
      goal_cancel_pub_ = ah_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

      ph_.param<std::string>("base_frame", base_frame_, std::string("base_link"));
      ph_.param<std::string>("laser_frame", laser_frame_, std::string("laser"));
      ph_.param<std::string>("map_frame", map_frame_, std::string("map"));
      ph_.param<double>("elevatorLong", elevatorLong_, double(1.6));
      ph_.param<double>("elevatorWide", elevatorWide_, double(2.0));
      ph_.param<double>("robotRadius", robotRadius_, double(0.225));

      while (ros::ok() && !ph_.hasParam("elevatorLong") && !move_base_cancel_)
      {
        ROS_INFO("can't get param that elevatorLong !");
        ros::Duration(1).sleep();
      }

      move_base_cancel_ = false;
      record_log_ = false;
      isJudged_ = false;
      isUpdateNaviState_ = false;
      recovery_can_clear_costmap_ = true;
      enter_try_cnt = 0;

      if (DETECT_PLANNER_RECORD)
      {
        log_.open("/tmp/detect_planner.log", std::ios::out); //std::ios::app //底部追加日志写法
      }
      if (!log_)
      {
        ROS_ERROR("open /tmp/detect_planner.log fail.");
      }
      else
      {
        record_log_ = true;
        DETECT_PLANNER_LOG("Welcome to use the smartest robot in the world.");
        std::time_t nowtime;
        struct tm *p;
        nowtime = std::time(nullptr);
        p = std::localtime(&nowtime);
        log_ << p->tm_year + 1900 << "-" << p->tm_mon + 1 << "-" << p->tm_mday << "," << p->tm_hour << ":" << p->tm_min << ":" << p->tm_sec << std::endl;
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
    else
    {
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
    isJudged_ = false;
    isUpdateNaviState_ = false;
    isPublishGoal_ = false;
    recovery_can_clear_costmap_ = true;
    enter_try_cnt = 0;

    toleranceDistance_ = 0.2;  //TODO: 这个为了安全起见，先设置为0.2， default:0.1
    // 这里的距离为与take point的距离，暂时默认take point就处在电梯的中心
    d_inside_elevator_ = (elevatorLong_ / 2.0) - robotRadius_ - toleranceDistance_;      //用于判断是否在电梯里面的距离
    d_outside_elevator_ = (elevatorLong_ / 2.0) + robotRadius_ + toleranceDistance_ * 2; //用于判断是否在电梯外面的距离
    printf("d_inside_elevator_: %f, d_outside_elevator_: %f\n", d_inside_elevator_, d_outside_elevator_);

    takePoint = goal->takepose;
    waitPoint = goal->waitpose;
    mode = goal->mode;
    std::cout << "received w.x = " << waitPoint.position.x << std::endl;
    std::cout << "received w.y = " << waitPoint.position.y << std::endl;
    std::cout << "received t.x = " << takePoint.position.x << std::endl;
    std::cout << "received t.y = " << takePoint.position.y << std::endl;
    std::cout << "into elevator ? " << mode << std::endl;

    if (mode)
    {
      state_last_ = state_;
      state_ = NAV_TAKE; //mode=1,默认机器人在电梯外
    }
    else
    {
      state_last_ = state_;
      state_ = NAV_OUT; //mode=0， 默认机器人在电梯里面
    }

    angle_T_to_W_ = std::atan2(waitPoint.position.y - takePoint.position.y, waitPoint.position.x - takePoint.position.x);
    angle_W_to_T_ = std::atan2(takePoint.position.y - waitPoint.position.y, takePoint.position.x - waitPoint.position.x);
    printf("angle_T_to_W_: %f, angle_W_to_T_: %f\n", angle_T_to_W_, angle_W_to_T_);

    feedback_.feedback = robot_msg::auto_elevatorFeedback::SET_NEW_GOAL;
    feedback_.feedback_text = "Receive new instructions";
    as_.publishFeedback(feedback_);

    // make plan
    // client_getplan_ = ph_.serviceClient<nav_msgs::GetPlan>("/move_base/GlobalPlanner/make_plan"); //这个可以设置起点, 这个不受move base状态限制
    client_getplan_ = ph_.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan"); //这个以当前位置为起点,这个必须move base为inactive
    // nav_msgs::GetPlan srv_getplan_;
    client_enalbe_recovery_ = ph_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/set_parameters"); //清除costmap

    dynamic_reconfigure::Config temp_conf;
    bool_param_.name = "recovery_behavior_enabled";
    bool_param_.value = false;
    temp_conf.bools.push_back(bool_param_);
    srv_enalbe_recovery_.request.config = temp_conf;
    client_enalbe_recovery_.call(srv_enalbe_recovery_);
    temp_conf.bools.clear();

    int_param_.name = "combination_method";
    int_param_.value = 0; // 0 for overwrite; 1 for maximum
    // dynamic_reconfigure::Config temp_conf;
    temp_conf.ints.push_back(int_param_);
    srv_req_.config = temp_conf;
    ros::service::call("/move_base/global_costmap/obstacle_layer/set_parameters", srv_req_, srv_resp_);
    ros::service::call("/move_base/local_costmap/obstacle_layer/set_parameters", srv_req_, srv_resp_);
    temp_conf.ints.clear();

    // disable obstacle layer in global costmap
    bool_param_.name = "enabled";
    bool_param_.value = false;
    temp_conf.bools.push_back(bool_param_);
    srv_req_.config = temp_conf;
    ros::service::call("/move_base/global_costmap/obstacle_layer/set_parameters", srv_req_, srv_resp_);
    temp_conf.bools.clear();

    uint8_t feedback_last = robot_msg::auto_elevatorFeedback::NONE;

    ros::Rate r(10);
    while (ros::ok())
    {
      if (as_.isPreemptRequested())
      {
        if (as_.isNewGoalAvailable())
        {
          // robot_msg::auto_elevatorGoal new_goal;
          printf("state_: %d\n", state_);
          robot_msg::auto_elevatorGoal new_goal = *as_.acceptNewGoal();
          if ((state_ != IDLE) && (state_ != TAKE))
          {
            ROS_INFO("Acition is running, try later!!");
            // as_.setAborted();
            // new_goal = *as_.acceptNewGoal();
            continue;
          }
          // robot_msg::auto_elevatorGoal new_goal = *as_.acceptNewGoal();

          takePoint = new_goal.takepose;
          waitPoint = new_goal.waitpose;
          mode = new_goal.mode;
          recivedNewGoalTime = ros::Time::now().sec;
          std::cout << "robot should arrvied target floor !" << std::endl;
          std::cout << "received target w.x = " << waitPoint.position.x << std::endl;
          std::cout << "received target w.y = " << waitPoint.position.y << std::endl;
          std::cout << "received target t.x = " << takePoint.position.x << std::endl;
          std::cout << "received target t.y = " << takePoint.position.y << std::endl;
          std::cout << "out elevator ? " << !mode << std::endl;
          if (mode)
          {
            state_last_ = state_;
            state_ = NAV_TAKE; //mode=1,默认机器人在电梯外
          }
          else
          {
            state_last_ = state_;
            state_ = NAV_OUT; //mode=0， 默认机器人在电梯里面
          }
          angle_T_to_W_ = std::atan2(waitPoint.position.y - takePoint.position.y, waitPoint.position.x - takePoint.position.x);
          angle_W_to_T_ = std::atan2(takePoint.position.y - waitPoint.position.y, takePoint.position.x - waitPoint.position.x);
          printf("angle_T_to_W_: %f, angle_W_to_T_: %f\n", angle_T_to_W_, angle_W_to_T_);
          feedback_.feedback = robot_msg::auto_elevatorFeedback::SET_NEW_GOAL;
          feedback_.feedback_text = "Receive new instructions";
          as_.publishFeedback(feedback_);
          ROS_INFO_THROTTLE(1, "detect planner, recive a new goal");
        }
        else
        {
          vel.linear.x = 0.0;
          vel.angular.z = 0.0;
          vel_pub_.publish(vel);
          feedback_.feedback = robot_msg::auto_elevatorFeedback::CANCLED;
          feedback_.feedback_text = "detect planner cancled!";
          as_.publishFeedback(feedback_);
          ROS_INFO("%s: Preempted", action_name_.c_str());
          result_.result = true;
          as_.setPreempted(result_, "DetectPlanner Preempted");
          return;
        }
      }
      //执行主要逻辑
      runPlan(takePoint, waitPoint, mode);
      if(feedback_last != feedback_.feedback)
      {
        feedback_last = feedback_.feedback;
        as_.publishFeedback(feedback_);
      }

      if ((state_ == TAKE) || (state_ == IDLE))
      {
        result_.result = true;
        as_.setSucceeded(result_);
        ROS_INFO("---take elevator end---");

        bool_param_.name = "recovery_behavior_enabled";
        bool_param_.value = true;
        temp_conf.bools.push_back(bool_param_);
        srv_enalbe_recovery_.request.config = temp_conf;
        client_enalbe_recovery_.call(srv_enalbe_recovery_);
        temp_conf.bools.clear();

        int_param_.value = 1; // 0 for overwrite; 1 for maximum
        // dynamic_reconfigure::Config temp_conf;
        temp_conf.ints.push_back(int_param_);
        srv_req_.config = temp_conf;
        ros::service::call("/move_base/global_costmap/obstacle_layer/set_parameters", srv_req_, srv_resp_);
        ros::service::call("/move_base/local_costmap/obstacle_layer/set_parameters", srv_req_, srv_resp_);
        temp_conf.ints.clear();

        bool_param_.name = "enabled";
        bool_param_.value = true;
        temp_conf.bools.push_back(bool_param_);
        srv_req_.config = temp_conf;
        ros::service::call("/move_base/global_costmap/obstacle_layer/set_parameters", srv_req_, srv_resp_);
        temp_conf.bools.clear();

        return;
      }
      else if (state_ == SOMETHING_ERROR)
      {
        result_.result = false;
        as_.setAborted(result_, "failed, defalut state i don't know why");
        ROS_ERROR("failed, defalut state i don't know why");
        ROS_INFO("---take elevator failure---");

        bool_param_.name = "recovery_behavior_enabled";
        bool_param_.value = true;
        temp_conf.bools.push_back(bool_param_);
        srv_enalbe_recovery_.request.config = temp_conf;
        client_enalbe_recovery_.call(srv_enalbe_recovery_);
        temp_conf.bools.clear();

        int_param_.value = 1; // 0 for overwrite; 1 for maximum
        // dynamic_reconfigure::Config temp_conf;
        temp_conf.ints.push_back(int_param_);
        srv_req_.config = temp_conf;
        ros::service::call("/move_base/global_costmap/obstacle_layer/set_parameters", srv_req_, srv_resp_);
        ros::service::call("/move_base/local_costmap/obstacle_layer/set_parameters", srv_req_, srv_resp_);
        temp_conf.ints.clear();

        bool_param_.name = "enabled";
        bool_param_.value = true;
        temp_conf.bools.push_back(bool_param_);
        srv_req_.config = temp_conf;
        ros::service::call("/move_base/global_costmap/obstacle_layer/set_parameters", srv_req_, srv_resp_);
        temp_conf.bools.clear();

        return;
      }

      // if (feedback_.feedback == robot_msg::auto_elevatorFeedback::FAILURE_FULL)
      // {
      //   result_.result = false;
      //   as_.setAborted(result_, "full-failed ,waiting next elevator");
      //   ROS_ERROR("full-failed ,waiting next elevator");
      //   ROS_INFO("---take elevator failure---");

      //   bool_param_.name = "recovery_behavior_enabled";
      //   bool_param_.value = true;
      //   temp_conf.bools.push_back(bool_param_);
      //   srv_enalbe_recovery_.request.config = temp_conf;
      //   client_enalbe_recovery_.call(srv_enalbe_recovery_);
      //   temp_conf.bools.clear();

      //   int_param_.value = 1; // 0 for overwrite; 1 for maximum
      //   dynamic_reconfigure::Config temp_conf;
      //   temp_conf.ints.push_back(int_param_);
      //   srv_req_.config = temp_conf;
      //   ros::service::call("/move_base/global_costmap/obstacle_layer/set_parameters", srv_req_, srv_resp_);
      //   ros::service::call("/move_base/local_costmap/obstacle_layer/set_parameters", srv_req_, srv_resp_);
      //   temp_conf.ints.clear();

      //   bool_param_.name = "enabled";
      //   bool_param_.value = true;
      //   temp_conf.bools.push_back(bool_param_);
      //   srv_req_.config = temp_conf;
      //   ros::service::call("/move_base/global_costmap/obstacle_layer/set_parameters", srv_req_, srv_resp_);
      //   temp_conf.bools.clear();

      //   return;
      // }

      // if (feedback_.feedback == robot_msg::auto_elevatorFeedback::FAILURE_OUT)
      // {
      //   result_.result = false;
      //   as_.setAborted(result_, "out-failed ,waiting next elevator");
      //   ROS_ERROR("out-failed ,waiting next elevator");
      //   ROS_INFO("---get out of elevator failure---");

      //   bool_param_.name = "recovery_behavior_enabled";
      //   bool_param_.value = true;
      //   temp_conf.bools.push_back(bool_param_);
      //   srv_enalbe_recovery_.request.config = temp_conf;
      //   client_enalbe_recovery_.call(srv_enalbe_recovery_);
      //   temp_conf.bools.clear();

      //   int_param_.value = 1; // 0 for overwrite; 1 for maximum
      //   dynamic_reconfigure::Config temp_conf;
      //   temp_conf.ints.push_back(int_param_);
      //   srv_req_.config = temp_conf;
      //   ros::service::call("/move_base/global_costmap/obstacle_layer/set_parameters", srv_req_, srv_resp_);
      //   ros::service::call("/move_base/local_costmap/obstacle_layer/set_parameters", srv_req_, srv_resp_);
      //   temp_conf.ints.clear();

      //   bool_param_.name = "enabled";
      //   bool_param_.value = true;
      //   temp_conf.bools.push_back(bool_param_);
      //   srv_req_.config = temp_conf;
      //   ros::service::call("/move_base/global_costmap/obstacle_layer/set_parameters", srv_req_, srv_resp_);
      //   temp_conf.bools.clear();

      //   return;
      // }

      r.sleep();
    }
  }

  void DetectPlanner::searchNearby(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped target_base_pose, double yaw_angle, float offset_array[][2], int offset_array_size, geometry_msgs::PoseStamped &success_pose, bool &success_or_not)
  {
    success_or_not = false;
    geometry_msgs::PoseStamped search_pose;
    srv_getplan_.request.start = start_pose;
    for (int tempi = 0; tempi < offset_array_size; ++tempi)
    {
      search_pose = target_base_pose;
      search_pose.pose.position.x += (offset_array[tempi][0] * std::cos(yaw_angle) - offset_array[tempi][1] * std::sin(yaw_angle));
      search_pose.pose.position.y += (offset_array[tempi][0] * std::sin(yaw_angle) + offset_array[tempi][1] * std::cos(yaw_angle));
      srv_getplan_.request.goal = search_pose;
      srv_getplan_.request.tolerance = 0.2;
      if (client_getplan_.call(srv_getplan_))
      {
        // std::cout << srv_getplan_.response.plan;
        // printf("srv_getplan_.response.plan.poses.size(): %ld\n", srv_getplan_.response.plan.poses.size());
        if (srv_getplan_.response.plan.poses.size() == 0)
        {
          // target not arrivable
          ROS_INFO("make plan fail!!");
          continue;
        }
        else
        {
          // make available plan
          ROS_INFO("make plan successfully!");
          // std::cout << search_pose;
          success_pose = search_pose;
          success_or_not = true;
          break;
        }
      }
      else
      {
        ROS_INFO("call make plan fail!!");
        actionlib_msgs::GoalID goalCancel;
        goal_cancel_pub_.publish(goalCancel);
        break;
      }
    }
  }

  bool DetectPlanner::runPlan(geometry_msgs::Pose takePoint, geometry_msgs::Pose waitPoint, bool mode)
  {

    if (!initialized_)
    {
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE;
      feedback_.feedback_text = "not init";
      // as_.publishFeedback(feedback_);
      return false;
    }

    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::PoseStamped start_pose;

    move_base_cancel_ = false;

    ROS_INFO_ONCE("---detect planner started!---");

    //部分变量声明和初始化
    geometry_msgs::Pose global_pose;

    global_pose = carto_pose_.pose;

    float offset_takePoint[25][2] = {{0, 0}, {0, 0.2}, {0, -0.2}, {0.2, 0}, {0.2, 0.2}, {0.2, -0.2}, {-0.2, 0}, {-0.2, 0.2}, {-0.2, -0.2}, {0, 0.4}, {0, -0.4}, {0.2, 0.4}, {0.2, -0.4}, {-0.2, 0.4}, {-0.2, -0.4}, {-0.4, 0.4}, {-0.4, 0.2}, {-0.4, 0}, {-0.4, -0.2}, {-0.4, -0.4}, {0.4, 0.4}, {0.4, 0.2}, {0.4, 0}, {0.4, -0.2}, {0.4, -0.4}}; //TODO: 更加密集的搜索

    float offset_waitPoint[25][2] = {
        {0, 0},
        {0, 0.5},
        {0, -0.5},
        {-0.5, 0},
        {-0.5, 0.5},
        {-0.5, -0.5},
        {0, 1},
        {0, -1},
        {-0.5, 1},
        {-0.5, -1},
        {-1, 0},
        {-1, 0.5},
        {-1, -0.5},
        {-1, 1},
        {-1, -1},
        {-1.5, 0},
        {-1.5, -0.5},
        {-1.5, 0.5},
        {-1.5, -1},
        {-1.5, 1},
        {-2, 0},
        {-2, -0.5},
        {-2, 0.5},
        {-2, -1},
        {-2, 1},
    }; //TODO: 更加密集的搜索, //这里的坐标系以候梯点指向梯内点为x轴，向左为y轴

    //进入状态机
    switch (state_)
    {
    case NAV_TAKE:
    case RETURN_TAKE_AREA:
    {
      feedback_.feedback = robot_msg::auto_elevatorFeedback::ENTER_ELEVATOR;
      feedback_.feedback_text = "navi to take point";
      // as_.publishFeedback(feedback_);

      if (enter_try_cnt >= 10) //失败尝试的次数,目前所有搜索耗时大概0.2s。TODO: 可以改为超时，可能更加人性化
      {
        enter_try_cnt = 0;
        if (state_ == NAV_TAKE)
        {
          state_last_ = state_;
          state_ = RETURN_WAIT_AREA;
          ROS_INFO("Get into elevator error after many try, return to wait area!!");
          isPublishGoal_ = false;
          isUpdateNaviState_ = false;
          isJudged_ = false;
          break;
        }
        else
        {
          ROS_INFO("Return take area fail!!");
          feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE; //返回梯内区域失败，返回failure
          feedback_.feedback_text = "failure, return, ask for help!";
          // as_.publishFeedback(feedback_);
          state_last_ = state_;
          state_ = SOMETHING_ERROR;
          isPublishGoal_ = false;
          isUpdateNaviState_ = false;
          return false;
        }
      }      

      if (!isPublishGoal_)
      {

        double distanceRto = Distance(carto_pose_.pose, takePoint);
        printf("distanceRto: %f, d_inside_elevator_: %f\n", distanceRto, d_inside_elevator_);
        if (distanceRto < d_inside_elevator_) //机器人身子已进入电梯，0.1为容错, 距离waitpoint小于1m，认为已经进入电梯
        {
          actionlib_msgs::GoalID goalCancel;
          goal_cancel_pub_.publish(goalCancel);
          ROS_INFO("into elevator door stand success, nearby");
          if (state_ == RETURN_TAKE_AREA)
          {
            feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE_OUT; //这里返回出梯失败
            feedback_.feedback_text = "failure full, return taking point";
          }
          else
          {
            feedback_.feedback = robot_msg::auto_elevatorFeedback::ENTER_SUCCESS;
            feedback_.feedback_text = "enter success elevator";
          }
          // as_.publishFeedback(feedback_);
          state_last_ = state_;
          state_ = FACE_DOOR;
          ROS_INFO("---state_: [%d], Turning around to face door---", state_);
          isPublishGoal_ = false;
          isUpdateNaviState_ = false;
          isJudged_ = false;
          enter_try_cnt = 0;
          break;
        }

        start_pose.header.frame_id = map_frame_;
        start_pose.pose = this->carto_pose_.pose;
        target_pose.header.frame_id = map_frame_;
        target_pose.pose = takePoint;
        geometry_msgs::Quaternion temp_q = tf::createQuaternionMsgFromYaw(angle_T_to_W_);
        target_pose.pose.orientation = temp_q;
        bool get_result = false;
        geometry_msgs::PoseStamped success_pose;
        int offset_takePoint_length = sizeof(offset_takePoint) / sizeof(offset_takePoint[0]);
        searchNearby(start_pose, target_pose, angle_W_to_T_, offset_takePoint, offset_takePoint_length, success_pose, get_result);
        // std::cout << "success_pose:" << success_pose;
        if (get_result)
        {
          searchNearby(start_pose, target_pose, angle_W_to_T_, offset_takePoint, offset_takePoint_length, success_pose, get_result);
          if (get_result)
          {
            goal_pub_.publish(success_pose);
            ROS_INFO("Publish take point");
            isPublishGoal_ = true;
            isUpdateNaviState_ = false;
            // enter_try_cnt = 0;
          }
        }
        else
        {
          ++enter_try_cnt;
          printf("wait 0.15s, enter_try_cnt: %d\n", enter_try_cnt);
          ros::Duration(0.15).sleep();
        }
      }

      naviStateFeedback_ = navi_state_.feedback;

      if (!isUpdateNaviState_)
      {
        if ((naviStateFeedback_ == 100) || (naviStateFeedback_ == 103)) // in planning or controlling
        {
          isUpdateNaviState_ = true;
        }
      }
      else
      {

        if (naviStateFeedback_ == 104)
        {
          ROS_INFO("into elevator take point success");
          if (state_ == RETURN_TAKE_AREA)
          {
            feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE_OUT; //这里可以返回出电梯失败
            feedback_.feedback_text = "failure full, return taking point";
          }
          else
          {
            feedback_.feedback = robot_msg::auto_elevatorFeedback::ENTER_SUCCESS;
            feedback_.feedback_text = "enter success elevator";
          }
          // as_.publishFeedback(feedback_);
          state_last_ = state_;
          state_ = FACE_DOOR;
          ROS_INFO("---state_: [%d], Turning around to face door---", state_);
          isPublishGoal_ = false;
          isJudged_ = false;
          isUpdateNaviState_ = false;
          enter_try_cnt = 0;
        }
        else if (naviStateFeedback_ == 101 || naviStateFeedback_ == 102 || naviStateFeedback_ == 106 || naviStateFeedback_ == 107)
        {
          printf("naviStateFeedback_: %d\n", naviStateFeedback_);
          double distanceRto = Distance(carto_pose_.pose, takePoint);
          if (distanceRto < d_inside_elevator_) //机器人身子已进入电梯，0.1为容错, 距离waitpoint小于1m，认为已经进入电梯
          {
            actionlib_msgs::GoalID goalCancel;
            goal_cancel_pub_.publish(goalCancel);
            ROS_INFO("into elevator door stand success, nearby");
            if (state_ == RETURN_TAKE_AREA)
            {
              feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE_OUT; //这里返回出梯失败
              feedback_.feedback_text = "failure out, return taking point";
            }
            else
            {
              feedback_.feedback = robot_msg::auto_elevatorFeedback::ENTER_SUCCESS;
              feedback_.feedback_text = "enter success elevator";
            }
            // as_.publishFeedback(feedback_);
            state_last_ = state_;
            state_ = FACE_DOOR;
            ROS_INFO("---state_: [%d], Turning around to face door---", state_);
            isPublishGoal_ = false;
            isUpdateNaviState_ = false;
            isJudged_ = false;
            enter_try_cnt = 0;
            break;
          }
          else
          {
            actionlib_msgs::GoalID goalCancel; //cancel current goal for not using recovery behaviour
            goal_cancel_pub_.publish(goalCancel);

            if (state_ == RETURN_TAKE_AREA)
            {
              ROS_INFO("return take area error, try agian!!");
            }
            else
            {
              ROS_INFO("get into elevator error, try agian!!");
            }
            isPublishGoal_ = false;
            isUpdateNaviState_ = false;
            isJudged_ = false;

            ++enter_try_cnt;
            printf("navigation fail, enter_try_cnt: %d\n", enter_try_cnt);

          }
        }
        double distanceRto = Distance(carto_pose_.pose, takePoint);
        if (distanceRto < d_inside_elevator_) //机器人身子已进入电梯，0.1为容错, 距离waitpoint小于1m，认为已经进入电梯
        {
          ROS_INFO("into elevator door stand success, nearby");

          if (state_ == RETURN_TAKE_AREA)
          {
            feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE_OUT; //这里返回出梯失败
            feedback_.feedback_text = "failure out, return taking point";
          }
          else
          {
            feedback_.feedback = robot_msg::auto_elevatorFeedback::ENTER_SUCCESS;
            feedback_.feedback_text = "enter success elevator";
          }
          // as_.publishFeedback(feedback_);
        }
      }

      break;
    }
    case FACE_DOOR:
    case FACE_DOOR_OUTSIDE:
    {
      // printf("state_: %d\n", state_);
      // ROS_INFO("---state_: [%d], Turning around to face door---", state_);
      if(state_ == FACE_DOOR)
      {
        if(state_last_ == NAV_TAKE)
        {
          feedback_.feedback = robot_msg::auto_elevatorFeedback::ENTER_SUCCESS;
          feedback_.feedback_text = "enter success elevator";
        }
        else
        {
          // RETURN_TAKE_AREA
          feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE_OUT; //这里返回出梯失败
          feedback_.feedback_text = "failure out, return taking point";
        }
      }
      else
      {
        // FACE_DOOR_OUTSIDE
        feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE_FULL; //这里可以返回满员
        feedback_.feedback_text = "failure full, return waiting point nearby";
      }
      // as_.publishFeedback(feedback_);

      // ROS_INFO_ONCE("Robot initial posture correction start");
      double global_angle = tf::getYaw(carto_pose_.pose.orientation); //机器人本身的朝向角
      double angle_diff = 0;
      if(state_ == FACE_DOOR)
      {
        angle_diff = normalizeAngle(angle_T_to_W_ - global_angle, -M_PI, M_PI);
      }
      else
      {
        // FACE_DOOR_OUTSIDE
        // printf("angle_W_to_T_: %f , global_angle: %f\n", angle_W_to_T_, global_angle);
        angle_diff = normalizeAngle(angle_W_to_T_ - global_angle, -M_PI, M_PI);
      }
      
      // ROS_INFO("Part angle diff =  %.2frad", angle_diff);

      geometry_msgs::Twist cmd_vel;
      if (fabs(angle_diff) > 0.1)
      {
        cmd_vel.angular.z = (fabs(angle_diff) <= 0.5) ? angle_diff : (angle_diff > 0 ? 1 : -1) * 0.5;
        cmd_vel.linear.x = 0;
        this->vel_pub_.publish(cmd_vel);
      }
      else
      {
        cmd_vel.angular.z = 0;
        cmd_vel.linear.x = 0;
        this->vel_pub_.publish(cmd_vel);
        ROS_INFO("Robot initial posture correction end");
        if(state_ == FACE_DOOR)
        {
          ROS_INFO("---Ride in an elevator---");
          if(state_last_ == NAV_TAKE)
          {
            feedback_.feedback = robot_msg::auto_elevatorFeedback::TAKE_THE_ELEVATOR;
            feedback_.feedback_text = "enter success ,take the elevator";
          }
          else
          {
            // RETURN_TAKE_AREA
            feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE_OUT; //这里返回出梯失败
            feedback_.feedback_text = "failure out, return taking point";
          }
          state_last_ = state_;
          state_ = TAKE;
        }
        else
        {
          // FACE_DOOR_OUTSIDE
          state_last_ = state_;
          state_ = IDLE;
          return true;
        }
      }
      // this->vel_pub_.publish(cmd_vel);
      
      break;
    }
    // case TAKE:
    // {
    //   ROS_INFO_ONCE("---Ride in an elevator---");
    //   feedback_.feedback = robot_msg::auto_elevatorFeedback::TAKE_THE_ELEVATOR;
    //   feedback_.feedback_text = "enter success ,take the elevator";
    //   // as_.publishFeedback(feedback_);

    //   break;
    // }
    case RETURN_WAIT_AREA:
    case NAV_OUT:
    {
      //出电梯
      feedback_.feedback = robot_msg::auto_elevatorFeedback::GET_OUT_ELEVATOR;
      feedback_.feedback_text = "get out elevator";
      // as_.publishFeedback(feedback_);

      if (enter_try_cnt >= 10) //失败尝试的次数,目前所有搜索耗时大概0.2s。TODO: 可以改为超时，可能更加人性化
      {
        enter_try_cnt = 0;
        if (state_ == RETURN_WAIT_AREA)
        {
          ROS_INFO("Return wait area fail!!");
          feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE; //返回候梯区域失败，返回failure
          feedback_.feedback_text = "failure, return, ask for help!";
          // as_.publishFeedback(feedback_);
          state_last_ = state_;
          state_ = SOMETHING_ERROR;
          isPublishGoal_ = false;
          isUpdateNaviState_ = false;
          break;
          // return false;
        }
        else
        {
          state_last_ = state_;
          state_ = RETURN_TAKE_AREA;
          ROS_INFO("Get out elevator error after many try, return to take area!!");
          isPublishGoal_ = false;
          isUpdateNaviState_ = false;
          isJudged_ = false;
          break;
        }
      }

      if (!isPublishGoal_)
      {

        double distanceRtoW = Distance(carto_pose_.pose, waitPoint);
        double distanceRtoT = Distance(carto_pose_.pose, takePoint);
        printf("distanceRtoW: %f, distanceRtoT: %f\n", distanceRtoW, distanceRtoT);
        if ((distanceRtoW < 1.0) && (distanceRtoT > d_outside_elevator_))
        {
          if (state_ == RETURN_WAIT_AREA)
          {
            ROS_INFO("FAILURE_FULL, into elevator failed");
            feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE_FULL; //这里可以返回满员
            feedback_.feedback_text = "failure full, return waiting point nearby";
            // as_.publishFeedback(feedback_);
            // state_ = IDLE;
            state_last_ = state_;
            state_ = FACE_DOOR_OUTSIDE;
            ROS_INFO("---state_: [%d], Turning around to face door---", state_);
            isPublishGoal_ = false;
            isUpdateNaviState_ = false;
            break;
            // return false;
          }
          else
          {
            // NAV_OUT
            ROS_INFO("out: detect planner end in advance, nearby");
            isPublishGoal_ = false;
            state_last_ = state_;
            state_ = IDLE;
            feedback_.feedback = robot_msg::auto_elevatorFeedback::SUCCESS;
            feedback_.feedback_text = "out elevator success";
            // as_.publishFeedback(feedback_);
            return true;
          }
        }

        start_pose.header.frame_id = map_frame_;
        start_pose.pose = this->carto_pose_.pose;
        target_pose.header.frame_id = map_frame_;
        target_pose.pose = waitPoint;
        geometry_msgs::Quaternion temp_q;
        if (state_ == RETURN_WAIT_AREA)
        {
          temp_q = tf::createQuaternionMsgFromYaw(angle_W_to_T_);
        }
        else
        {
          // NAV_OUT
          temp_q = tf::createQuaternionMsgFromYaw(angle_T_to_W_);
        }
        target_pose.pose.orientation = temp_q;

        bool get_result = false;
        geometry_msgs::PoseStamped success_pose;
        int offset_waitPoint_length = sizeof(offset_waitPoint) / sizeof(offset_waitPoint[0]);
        searchNearby(start_pose, target_pose, angle_W_to_T_, offset_waitPoint, offset_waitPoint_length, success_pose, get_result);
        if (get_result)
        {
          searchNearby(start_pose, target_pose, angle_W_to_T_, offset_waitPoint, offset_waitPoint_length, success_pose, get_result);
          if (get_result)
          {
            goal_pub_.publish(success_pose);
            ROS_INFO("Publish wait point");
            isPublishGoal_ = true;
            isUpdateNaviState_ = false;
            // enter_try_cnt = 0;
          }
        }
        else
        {
          ++enter_try_cnt;
          printf("wait 0.15s, enter_try_cnt: %d\n", enter_try_cnt);
          ros::Duration(0.15).sleep();
        }
      }

      naviStateFeedback_ = navi_state_.feedback;

      if (!isUpdateNaviState_)
      {
        if ((naviStateFeedback_ == 100) || (naviStateFeedback_ == 103)) // in planning or controlling
        {
          isUpdateNaviState_ = true;
        }
      }
      else
      {

        if (naviStateFeedback_ == 104)
        {
          if (state_ == RETURN_WAIT_AREA)
          {
            ROS_INFO("into elevator failed, set state_ = FACE_DOOR_OUTSIDE");
            feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE_FULL; //这里可以返回满员
            feedback_.feedback_text = "failure full, return waiting point";
            // as_.publishFeedback(feedback_);
            // state_ = IDLE;
            state_last_ = state_;
            state_ = FACE_DOOR_OUTSIDE;
            ROS_INFO("---state_: [%d], Turning around to face door---", state_);
            isPublishGoal_ = false;
            isUpdateNaviState_ = false;
            enter_try_cnt = 0;
            // return false;
            break;
          }
          else
          {
            // NAV_OUT
            ROS_INFO("Get out elevator successful!");
            feedback_.feedback = robot_msg::auto_elevatorFeedback::SUCCESS;
            feedback_.feedback_text = "out elevator success";
            // as_.publishFeedback(feedback_);
            state_last_ = state_;
            state_ = IDLE;
            isPublishGoal_ = false;
            isUpdateNaviState_ = false;
            enter_try_cnt = 0;
            return true;
          }
        }
        else if (naviStateFeedback_ == 101 || naviStateFeedback_ == 102 || naviStateFeedback_ == 106 || naviStateFeedback_ == 107)
        {
          printf("naviStateFeedback_: %d\n", naviStateFeedback_);
          actionlib_msgs::GoalID goalCancel; //cancel current goal for not using recovery behaviour
          goal_cancel_pub_.publish(goalCancel);
          if (state_ == RETURN_WAIT_AREA)
          {
            ROS_INFO("return wait area error, try agian!!");
          }
          else
          {
            ROS_INFO("get out elevator error, try agian!!");
          }
          isPublishGoal_ = false;
          isUpdateNaviState_ = false;
          ++enter_try_cnt;
          printf("navigation fail, enter_try_cnt: %d\n", enter_try_cnt);
        }

        double distanceRto = Distance(carto_pose_.pose, takePoint);
        if (distanceRto > d_outside_elevator_)
        {
          if (state_ == RETURN_WAIT_AREA)
          {
            // ROS_INFO("into elevator failed, nearby!");
            feedback_.feedback = robot_msg::auto_elevatorFeedback::FAILURE_FULL; //这里可以返回满员
            feedback_.feedback_text = "failure full, return waiting point nearby";
            // as_.publishFeedback(feedback_);
            // state_ = IDLE;
            // state_ = FACE_DOOR_OUTSIDE;
            // isPublishGoal_ = false;
            // isUpdateNaviState_ = false;
            // return false;
            // break;
          }
          else
          {
            // NAV_OUT
            // ROS_INFO("out: detect planner end in advance, nearby");
            isPublishGoal_ = false;
            state_last_ = state_;
            state_ = IDLE;
            feedback_.feedback = robot_msg::auto_elevatorFeedback::SUCCESS;
            feedback_.feedback_text = "out elevator success";
            // as_.publishFeedback(feedback_);
            return true;
          }
        }
      }
      break;
    }
    default:
    {
      as_.setAborted(result_, "failure, enter default state");
      return false; //退出程序
    }
      ros::spinOnce();
    }
  }

  double DetectPlanner::Distance(geometry_msgs::Pose PointA, geometry_msgs::Pose PointB)
  {
    double x_diff, y_diff;
    x_diff = PointA.position.x - PointB.position.x;
    y_diff = PointA.position.y - PointB.position.y;
    return sqrt((x_diff * x_diff) + (y_diff * y_diff));
  }
  void DetectPlanner::naviStateCallback(const robot_msg::FeedBackConstPtr &msg)
  {
    ROS_INFO_ONCE("navigation state recevied");
    boost::mutex::scoped_lock lock(this->navi_state_mutex_);
    this->navi_state_ = *msg;
  }

  void DetectPlanner::elevatorStateCallback(const robot_msg::ElevatorStateConstPtr &msg)
  {
    ROS_INFO_ONCE("elevator state recevied");
    boost::mutex::scoped_lock lock(this->ele_state_mutex_);
    this->elevator_state_ = *msg;
  }

  void DetectPlanner::cartoCallback(const robot_msg::SlamStatus::ConstPtr &msg)
  {
    ROS_INFO_ONCE("carto data recevied");
    boost::mutex::scoped_lock lock(this->carto_mutex_);
    this->carto_pose_ = *msg;
  }
}; // namespace detect_planner
