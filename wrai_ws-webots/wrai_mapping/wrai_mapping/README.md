# WRAI Simultaneous Localisation and Mapping

A ROS package that turns relative cone measurements into absolute positions, and calculates the vehicle position fromn those measurements.

### ROS 2 Subscriptions

| Topic Name        | Type                                                                                                              | Purpose                                           |
| ----------        | ----                                                                                                              | -------                                           |
| `/camera_cones`   | [wrai_msgs/ConeMeasurementArray](https://github.com/Warwick-Racing/WRAI1-Main/blob/-/wrai_msgs/msg/ConeMeasurementArray.msg) | Cone measurements from the camera                 |
| `/odom`           | [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)                            | The current position and orientation of the car   |

### ROS 2 Publishers

| Topic Name | Type                                                                                                     | Purpose                             |
| ---------- | ----                                                                                                     | -------                             |
|`/cones`    | [wrai_msgs/ConeArray](https://github.com/Warwick-Racing/WRAI1-Main/blob/-/wrai_msgs/msg/ConeArray.msg)   | An array of the current known cones |

### ROS 2 Services 

| Service Name | Type | Purpose |
| ------------ | ---- | ------- |
| N/A | N/A | N/A |

### Parameters

NB: if a parameter has no default value, a value must be provided.

| Name                              | Type     | Default         | Purpose |
| -----                             | ----     | -------         | ------- |
| `start_state`                     | float[]  | [0.0, 0.0, 0.0] | Where the car starts  |
| `num_particles`                   | int      | 20              | The number of particles in the particle filter  |
| `slop`                            | float    | 0.01            | The slop of the time synchronizer  |
| `type_thresh`                     | float    | 0.75            | The minimum probability threshold for a cone type to be confirmed   |
| `association_threshold`           | float    | 0.75            | If an observation is within this distance to a landmark, they are associated. |
| `landmark_combination_threshold`  | float    | 0.50            | if two landmarks are within this distance of eachother, they are combined   |
| `obs_distance_thresh`             | float    | 15.0            | The furthest that we can observe cones from.   |
| `obs_angle_thresh`                | float    | pi/3            | Maximum FOV /2.    |
| `type_obs_prob`                   | float    | 0.75            | Probability of a cone type (colour) observation to be true.   |
| `motion_dist_noise`               | float    | 0.10            | Noise of the measurement of distance travelled.    |
| `motion_angle_noise`              | float    | 0.01            | Noise of the measurement of angle turned.   |
| `obs_dist_noise`                  | float    | 0.10            | Noise of landmark(cone) range observations.   |
| `obs_angle_noise`                 | float    | 0.0025          | Noise of landmark(cone) angle observations.   |