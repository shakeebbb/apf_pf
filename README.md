# 1. Overview
The package provides a framework to generate 3D velocity commands directly from a depth image stream.

# 2. Nodes

## 2.1 filter_point_node

### 2.1.1 Subscribed Topics
~/pt_cloud_in: Ordered pointcloud from the depth camera in optical frame. \
~/cam_info_in: Camera info topic from the depth camera. 

### 2.1.2 Published Topics
~/viz_out: Vizualization topic. \
~/pt_out: Most probable location of the occupied region inside the depth image stream. \
~/twist_out: Repulsive velocity out. (in qmdp mode) \
~/force_out: Repulsive force out. (in apf mode)

## 2.2 goal_to_vel_node
~/pose_in: Robot position in. \
~/goal_pt_in: Goal point in. \
~/rep_vec_in: Repulsive velocity/force in from filter_point_node. \
~/twist_out: Final velocity action for the robot.

