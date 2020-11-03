#include <detect_planner/detect_planner.h>


int main(int argc, char** argv){

  ros::init(argc, argv, "detect_planner_node");

  bool done = false;
  detect_planner::DetectPlanner detect_plan;
  //detect_plan.makePlan();

  ros::spin();

  return(0);
}

