# WRAI Path Following

A ROS package that generates control commands for the car so that it follows a path. 

### ROS 2 Subscriptions

| Topic Name | Type | Purpose |
| ---------- | ---- | ------- |
| `/odom`    | [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) | The current position and orientation of the car  |
|`/midpoints` | [wrai_msgs/Midpoints](https://github.com/Warwick-Racing/WRAI1-Main/blob/-/wrai_msgs/msg/Midpoints.msg)       | An array of ordered waypoints to follow |


### ROS 2 Publishers

| Topic Name | Type | Purpose |
| ---------- | ---- | ------- |
| `/cmd`     | [ackermann_msgs/AckermannDriveStamped](https://docs.ros.org/en/noetic/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) | Publishes the desired velocity and steering angle |

### ROS 2 Services 

| Service Name | Type | Purpose |
| ------------ | ---- | ------- |
| `/reset` | [std_srvs/Trigger](http://docs.ros.org/en/noetic/api/std_srvs/html/srv/Trigger.html) | Resets follower state. |

### Parameters

NB: if a parameter has no default value, a value must be provided.

| Name | Type | Default | Purpose |
| ----- | ---- |  ------ | ------- |
| `path_smoothing`          | float    | 0.95 | how much to smooth the path, higher is smoother (0 - 1)  |
| `path_spacing`            | float    | 0.2  |  the space inbetween injected points (m) |
| `path_tolerance`          | float    | 0.1  | how well to smooth the path, lower number is a higher quality path  |
| `lookahead_distance`      | float    | 2.5  |  pure pursuit lookahead distance (m) |
| `max_deceleration`        | float    | 1.0  | maximum amount of braking (ms^-2)  |
| `wheelbase`               | float    | 1.58 | length of the vehicle wheelbase (m)  |
| `min_speed`               | float    | 0.5  | minimum vehicle speed (ms^-1)  |
| `max_speed`               | float    | 3.0  | maximum vehicle speed (ms^-1)  |
| `max_centripetal_accel`   | float    | 5.0  |  maximum centripetal acceleration, this affects the velocity the vehicle will take corners at (ms^-2) |
