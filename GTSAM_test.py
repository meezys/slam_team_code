from controller import Robot, Supervisor, DistanceSensor, Motor
from planarslam import GraphSLAM
import numpy as np
from cone_type import ConeType
import gtsam
import math
import numpy as np

# Create the Robot instance
robot = Supervisor()

# Get the time step of the current world
timestep = int(robot.getBasicTimeStep())

# Get the robot's sensors and actuators
range_finder = robot.getDevice("range-finder")
range_finder.enable(timestep)
left_motor = robot.getDevice("motor_1")
right_motor = robot.getDevice("motor_2")
left_ps = robot.getDevice('ps_1')
left_ps.enable(timestep)
right_ps = robot.getDevice('ps_2')
right_ps.enable(timestep)
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Set up GraphSLAM
start_state = np.array([0, 0, 0])
association_threshold = 0.5
landmarks = []
isamgiven = gtsam.ISAM2()
#landmark_tree = KDTree([landmark for landmark, _ in cone_environment])
graph_slam = GraphSLAM(start_state, association_threshold, landmarks)
poselist = []
graph = gtsam.NonlinearFactorGraph()
estimates = gtsam.Values()
slam = GraphSLAM(start_state, association_threshold, landmarks)

def detect_cones(range_data, current_pose):
    """Detect cones based on range data and convert to Cartesian coordinates."""
    cones = []
    angle_increment = 360 / len(range_data)  # Assuming 360-degree coverage
    for i, distance in enumerate(range_data):
        if distance < 1:  # Assuming a threshold to filter out distant objects
            angle_degrees = i * angle_increment
            angle_radians = math.radians(angle_degrees)
            # Convert polar coordinates to Cartesian coordinates
            x = distance * math.cos(angle_radians)
            y = distance * math.sin(angle_radians)
            # Transform coordinates based on the robot's current pose
            world_x = current_pose[0] + x * math.cos(current_pose[2]) - y * math.sin(current_pose[2])
            world_y = current_pose[1] + x * math.sin(current_pose[2]) + y * math.cos(current_pose[2])
            cone_type = ConeType.UNKNOWN  # Placeholder for actual type detection logic
            
            cones.append((world_x, world_y, cone_type))
    return cones

def calculate_pose(left_position, right_position):
    """
    Calculate the robot's pose (x, y, theta) based on the motor encoder positions.
    """
    wheel_radius = 0.025  # Wheel radius in meters
    wheel_base = 0.045  # Distance between wheels in meters
    
    left_distance = left_position * wheel_radius
    right_distance = right_position * wheel_radius
    
    delta_distance = right_distance - left_distance
    delta_theta = delta_distance / wheel_base
    
    x = (left_distance + right_distance) / 2 * np.cos(delta_theta / 2)
    y = (left_distance + right_distance) / 2 * np.sin(delta_theta / 2)
    theta = delta_theta
    
    return np.array([x, y, theta])

def control_motors(target_pose, optimized_pose):

    kp = 0.5  # Proportional gain for simple control
    angle_error = target_pose[2] - optimized_pose[2]
    forward_speed = 5.0  # Base forward speed
    left_speed = forward_speed - kp * angle_error
    right_speed = forward_speed + kp * angle_error
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

# Main loop
while robot.step(timestep) != -1:
    # Read range finder data
    range_data = range_finder.getRangeImage()
    
    left_position = left_ps.getValue()
    right_position = right_ps.getValue()
    current_pose = calculate_pose(left_position, right_position)
    observations = detect_cones(range_data, current_pose)

    slam.step(start_state, np.array([0, 0, 0]), current_pose, observations, estimates, graph, landmarks, poselist, isamgiven)
    
    # Get the optimized pose estimate
    optimized_pose = slam.optimise(estimates)
    
    optimized_gtsam_pose = optimized_pose.atPose2(poselist[-1])
    optimized_robot_pose = np.array([optimized_gtsam_pose.x(), optimized_gtsam_pose.y(), optimized_gtsam_pose.theta()])
    
    # Control the robot's motors based on the optimized pose
    
    control_motors(current_pose, optimized_robot_pose)
    
# Cleanup
robot.cleanup()
