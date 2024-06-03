from controller import Robot, Camera, Motor, PositionSensor
from planarslam import GraphSLAM
import numpy as np
import cv2
from cone_type import ConeType
import gtsam
import math

# Create the Robot instance
robot = Robot()

# Get the time step of the current world
timestep = int(robot.getBasicTimeStep())

# Get the robot's sensors and actuators
camera = robot.getDevice('zedcam left camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)

front_right_motor = robot.getDevice('front right wheel motor')
front_left_motor = robot.getDevice('front left wheel motor')
rear_right_motor = robot.getDevice('rear right wheel motor')
rear_left_motor = robot.getDevice('rear left wheel motor')

# front_right_ps = robot.getDevice('front right wheel sensor')
# front_left_ps = robot.getDevice('front left wheel sensor')
# rear_right_ps = robot.getDevice('rear right wheel sensor')
# rear_left_ps = robot.getDevice('rear left wheel sensor')

# front_right_ps.enable(timestep)
# front_left_ps.enable(timestep)
# rear_right_ps.enable(timestep)
# rear_left_ps.enable(timestep)

front_right_motor.setPosition(float('inf'))
front_left_motor.setPosition(float('inf'))
rear_right_motor.setPosition(float('inf'))
rear_left_motor.setPosition(float('inf'))

front_right_motor.setVelocity(0.0)
front_left_motor.setVelocity(0.0)
rear_right_motor.setVelocity(0.0)
rear_left_motor.setVelocity(0.0)

# Set up GraphSLAM
start_state = np.array([0, 0, 0])
association_threshold = 0.5
landmarks = []
isamgiven = gtsam.ISAM2()
graph_slam = GraphSLAM(start_state, association_threshold, landmarks)
poselist = []
graph = gtsam.NonlinearFactorGraph()
estimates = gtsam.Values()
slam = GraphSLAM(start_state, association_threshold, landmarks)

# Store the starting position
starting_pose = np.array([0, 0, 0])

def detect_cones(image):
    """Detect cones based on camera image."""
    cones = []
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Define color ranges for blue and yellow cones
    blue_lower = np.array([100, 150, 0])
    blue_upper = np.array([140, 255, 255])
    yellow_lower = np.array([20, 100, 100])
    yellow_upper = np.array([30, 255, 255])
    
    # Create masks for blue and yellow
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
    
    # Find contours
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in blue_contours:
        x, y, w, h = cv2.boundingRect(contour)
        cones.append((x + w / 2, y + h / 2, ConeType.BLUE))
    
    for contour in yellow_contours:
        x, y, w, h = cv2.boundingRect(contour)
        cones.append((x + w / 2, y + h / 2, ConeType.YELLOW))
    
    return cones

def calculate_pose(front_left_position, front_right_position, rear_left_position, rear_right_position):
    """
    Calculate the robot's pose (x, y, theta) based on the motor encoder positions.
    """
    wheel_radius = 0.254  # Wheel radius in meters (from FsaiWheel_2022)
    wheel_base = 1.545  # Distance between wheels in meters (from FsaiVehicle_2022)
    
    front_left_distance = front_left_position * wheel_radius
    front_right_distance = front_right_position * wheel_radius
    rear_left_distance = rear_left_position * wheel_radius
    rear_right_distance = rear_right_position * wheel_radius
    
    delta_distance = (front_right_distance + rear_right_distance) - (front_left_distance + rear_left_distance)
    delta_theta = delta_distance / wheel_base
    
    x = (front_left_distance + front_right_distance + rear_left_distance + rear_right_distance) / 4 * np.cos(delta_theta / 2)
    y = (front_left_distance + front_right_distance + rear_left_distance + rear_right_distance) / 4 * np.sin(delta_theta / 2)
    theta = delta_theta
    
    return np.array([x, y, theta])

def control_motors(target_pose, optimized_pose):
    kp = 0.5  # Proportional gain for simple control
    angle_error = target_pose[2] - optimized_pose[2]
    forward_speed = 5.0  # Base forward speed
    left_speed = forward_speed - kp * angle_error
    right_speed = forward_speed + kp * angle_error
    front_left_motor.setVelocity(left_speed)
    front_right_motor.setVelocity(right_speed)
    rear_left_motor.setVelocity(left_speed)
    rear_right_motor.setVelocity(right_speed)

def has_completed_lap(current_pose, starting_pose, threshold=0.5):
    """
    Check if the robot has completed one lap by comparing the current pose with the starting pose.
    """
    distance = np.linalg.norm(current_pose[:2] - starting_pose[:2])
    return distance < threshold

# Main loop
lap_completed = False
while robot.step(timestep) != -1:
    # Get camera image
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    
    # Detect cones
    cones = detect_cones(image)
    
    # Get wheel positions
    front_left_position = front_left_ps.getValue()
    front_right_position = front_right_ps.getValue()
    rear_left_position = rear_left_ps.getValue()
    rear_right_position = rear_right_ps.getValue()
    
    # Calculate current pose
    current_pose = calculate_pose(front_left_position, front_right_position, rear_left_position, rear_right_position)
    
    # SLAM update
    observations = [(cone[0], cone[1], cone[2]) for cone in cones]
    slam.step(start_state, np.array([0, 0, 0]), current_pose, observations, estimates, graph, landmarks, poselist, isamgiven)
    
    # Get the optimized pose estimate
    optimized_pose = slam.optimise(estimates)
    
    optimized_gtsam_pose = optimized_pose.atPose2(poselist[-1])
    optimized_robot_pose = np.array([optimized_gtsam_pose.x(), optimized_gtsam_pose.y(), optimized_gtsam_pose.theta()])
    
    # Control the robot's motors based on the optimized pose
    control_motors(current_pose, optimized_robot_pose)
    
    # Check if the robot has completed one lap
    if has_completed_lap(current_pose, starting_pose):
        lap_completed = True
        break

# Stop the robot
front_left_motor.setVelocity(0.0)
front_right_motor.setVelocity(0.0)
rear_left_motor.setVelocity(0.0)
rear_right_motor.setVelocity(0.0)
