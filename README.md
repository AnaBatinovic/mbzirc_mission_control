### Requirements:
. netwrokx (for task structure representation): `pip install networkx`

### Installation:
Clone this repository in the `src` directory of your ROS workspace, checkout branch `real-world` and run `catkin build`.

### Running the example:
1. `roslaunch mbzirc_mission_control ch2_uav.launch`
2. When requesting a new task, call `request_next_task` service.
3. When done with task, call `register_completed_task` service.
