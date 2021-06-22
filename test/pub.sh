 #!/bin/sh
echo "$0"
echo "$1"

rostopic pub -1 /detect_planner_server/goal robot_msg/auto_elevatorActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  mode: $1
  takepose:
    position: {x: 1.5, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  waitpose:
    position: {x: -2.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
