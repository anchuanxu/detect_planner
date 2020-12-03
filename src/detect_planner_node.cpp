#include <detect_planner/detect_planner.h>



int main(int argc, char** argv){

  ros::init(argc, argv, "detect_planner_node");
  tf2_ros::Buffer                 tfBuffer;
  tf2_ros::TransformListener  tfListener(tfBuffer);
  detect_planner::DetectPlanner  detectPlannerAction("detect_planner_server",tfBuffer);



//  detect_planner::DetectPlanner detect_plan;

//  detect_plan.runPlan();

//  ros::MultiThreadedSpinner s(4);


//  while(ros::ok())
//  {
//    ROS_INFO("runPlan done");
//    ros::Duration(1).sleep();
//  }
  ros::spin();
  return 0;
}

