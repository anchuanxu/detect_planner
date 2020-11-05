#include <detect_planner/detect_planner.h>


int main(int argc, char** argv){

  ros::init(argc, argv, "detect_planner_node");

  std::string name;

  costmap_2d::Costmap2DROS* costmap_ros;

  detect_planner::DetectPlanner detect_plan(name,costmap_ros);

//  detect_plan.initialize(name,  costmap_ros);
  detect_plan.runPlan();

  while(ros::ok())
  {
    ROS_INFO("runPlan done");
    ros::Duration(1).sleep();
  }
//  ros::spin();

  return(0);
}

