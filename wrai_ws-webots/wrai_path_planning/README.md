# WRAI Path Planning

A ROS package that generates the path through a set of cones.

### ROS 2 Subscriptions

| Topic Name | Type | Purpose |
| ---------- | ---- | ------- |
|`/cones` | [wrai_msgs/ConeArray](https://github.com/Warwick-Racing/WRAI1-Main/blob/-/wrai_msgs/msg/ConeArray.msg)       | An array of the current known cones |
<!-- | `/odom`    | [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) | The current position and orientation of the car  | -->

### ROS 2 Publishers

| Topic Name | Type | Purpose |
| ---------- | ---- | ------- |
| `/midpoints`     | [wrai_msgs/Midpoints](https://github.com/Warwick-Racing/WRAI1-Main/blob/-/wrai_msgs/msg/Midpoints.msg) | Publishes the midpoints of the cones |

### ROS 2 Services 

| Service Name | Type | Purpose |
| ------------ | ---- | ------- |
| N/A | N/A | N/A |

### Parameters

NB: if a parameter has no default value, a value must be provided.

| Name | Type | Default | Purpose |
| ----- | ---- |  ------ | ------- |
| `distance_between_cones`  | float    | 7.0  | The maximum distance between cones  |
