# 1. Overview
The package provides a framework to generate 3D velocity commands directly from a depth image stream. The work is currently under review has two modes 'apf' and 'qmdp'. The discussion about the 'apf' mode is provided in the manuscript

@article{ahmad2020apfpf, \
  title={APF-PF: Probabilistic Depth Perception for 3D Reactive Obstacle Avoidance}, \
  author={Shakeeb Ahmad and Zachary N. Sunberg and J. Sean Humbert}, \
  journal={arXiv preprint arXiv:2010.08063}, \
  year={2020}
}

# 2. Nodes

## 2.1 filter_point_node

### 2.1.1 Subscribed Topics
~pt_cloud_in: Ordered pointcloud from the depth camera in optical frame. \
~cam_info_in: Camera info topic from the depth camera. 

### 2.1.2 Published Topics
~viz_out: Vizualization topic. \
~pt_out: Most probable location of the occupied region inside the depth image stream. \
~twist_out: Repulsive velocity out. (in qmdp mode) \
~force_out: Repulsive force out. (in apf mode)

### 2.1.3 Parameters
~read_from_file: Read transition, reward and alpha vector matrices from a file \
~matrices_file_path: Folder to save the transition, reward and alpha vector matrices \
~n_threads: Number of threads to run multi-thread \
~distance_interval: Discretization interval \
~pixel_interval: Discretization interval \
~min_distance: Mininimum depth horizon \
~max_distance: Maximum depth horizon \
~n_particles_vox: Number of particles \
~trans_noise_sdev: [\sigma_s, \sigma_z]  \
~obsv_noise_sdev: [\sigma_o, \sigma_n] 

~max_action_values: Maximum velocity in x,y and z directions (only in qmdp mode) \
~min_action_values: Minimum velocities in x,y and z directions (only in qmdp mode) \
~action_intervals: Discretization intervals for the velocities in x,y and z directions (only in qmdp mode)

~reward_Q: Reward Q matrix (diagonal) (only in qmdp mode) \
~repulsive_potential_max_distance: Maximum effective repulsive potential distance \
~repulsive_potential_gain: Repulsive potential gain 

~alpha_vector_iterations: Number of iterations for alpha vector update (only in qmdp mode)

~lookahead_time: Lookahead time to calculate the reward (only in qmdp mode) \
~sampling_time: Sampling time for the lookahead path to calculate the reward (only in qmdp mode) \
~base_frame_id: Robot body frame id

~output_apf: True enables apf mode \
~output_qmdp: True enables qmdp mode 

~publish_viz: True enables RViz visualization msg out \
~display_cout: True enables verbose

## 2.2 goal_to_vel_node

### 2.1.1 Subscribed Topics
~pose_in: Robot position in. \
~goal_pt_in: Goal point in. \
~rep_vec_in: Repulsive velocity/force in from filter_point_node. 

### 2.1.2 Published Topics
~twist_out: Final velocity action for the robot.

### 2.2.3 Parameters
~att_vel: Forward vertical and yaw rates for the attractor pull \
~rep_vec_timeout_secs: Timeout for repulsive vector \
~process_rate_secs: Final action publish rate \
~success_radius: Success radius \
~holonomic: False enables non-holonomic \
~yaw_err_bound_nonzero_fore_vel_in_rad: No forward velocity if the yaw error is greater than this value. Set to 1.57 to avoid negative forward velocities

~mode: 'apf' or 'qmdp' \
~parabolic_attractor_bound: Radius around the goal to enable parabolic attractor (only in apf mode) \
~attractor_gain: Attractor gain (only in apf mode)

~display_cout: True enables verbose

~pose_frame_id: Frame id of the incoming pose

~base_frame_id: Robot body frame id \
~pose_frame_output: True enables the output in pose/world frame


