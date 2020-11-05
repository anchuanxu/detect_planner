#include <detect_planner/detect_planner.h>


int main(int argc, char** argv){

  ros::init(argc, argv, "detect_planner_node");
  ros::NodeHandle param_nh;
 // bool runPlan = true;
 // param_nh.setParam("runPlan",runPlan);
  detect_planner::DetectPlanner detect_plan;

  detect_plan.runPlan();

  ros::MultiThreadedSpinner s(4);


  while(ros::ok())
  {
    ROS_INFO("runPlan done");
    ros::Duration(1).sleep();
  }
  ros::spin(s);

  return(0);
}

