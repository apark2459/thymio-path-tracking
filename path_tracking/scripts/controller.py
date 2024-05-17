#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
import tf.transformations
import numpy as np
import geometry_msgs.msg

# Get parameters
ks = rospy.get_param('/controller/ks',default = 1) # along track gain
kn = rospy.get_param('/controller/kn',default = 20) # cross track gain
kth = rospy.get_param('/controller/kth',default = 5) # heading gain
velocity = rospy.get_param('/reference/velocity') # velocity
last_two_theta = [] # message buffer to hold last two reference thetas (for omega calculation)

# Other global variables
cmd_vel = geometry_msgs.msg.Twist() # variable to store Twist message
last_reference_pose = geometry_msgs.msg.Pose2D() # variable to store reference pose from previous time step
current_reference_pose = geometry_msgs.msg.Pose2D() # variable to store reference pose at current time step
robot_pose = geometry_msgs.msg.PoseStamped() # variable to store pose of robot
first_pose_received = False # variable to indicate that first pose message has been received

def angle_diff(reference_pose,robot_pose):
    """
    Computes the angle sum of reference pose and robot pose using quaternions. We are summing the two angles because
    robot_pose gives us negative angle (negative in CCW direction) due to y axis pointing into the floor on the motion
    capture system. By summing, we actually get the angle difference between the reference heading and the robot heading.
    Must use quaternions to compute angle difference because using Euler angle (at least for yaw) leads to Gimbal lock.

    reference pose: current reference pose (Pose2D object)
    robot_pose: current robot pose (PoseStamped object)

    return: angle difference between reference pose and robot pose
    """
    # Convert reference_pose angle to quaternion
    ref_quat = geometry_msgs.msg.Quaternion()
    ref_quat.x = 0
    ref_quat.y = 0
    ref_quat.z = np.sin(reference_pose.theta/2)
    ref_quat.w = np.cos(reference_pose.theta/2)
    ref_quat_list = [ref_quat.x,ref_quat.y,ref_quat.z,ref_quat.w] # convert to list

    # Compute angle sum in quaternion
    robot_quat = robot_pose.pose.orientation
    robot_quat_list = [robot_quat.x,robot_quat.z,robot_quat.y,robot_quat.w] # convert to list
    quat_sum = tf.transformations.quaternion_multiply(robot_quat_list,ref_quat_list)

    # Convert to Euler angles
    angle_sum = tf.transformations.euler_from_quaternion(quat_sum)

    # Return angle about z axis
    return angle_sum[2]

def compute_omgega(current_theta,last_theta,dt):
    """
    Computes the angular velocity of reference trajectory
    current_theta: reference heading at current time step (float)
    last_theta: reference heading at last time step (float)
    dt: time step  (float)
    return: angular velocity of reference trajectory
    """
    omega = (current_theta-last_theta)/dt
    return omega

def compute_control(ks,kn,kth,robot_pose,velocity,omega):
    """
    Given gains ks, kn, kth, robot's current pose, reference velocity and omega, computes the closed-loop control input.
    ks: along-track gain (float)
    kn: cross-track gain (float)
    kth: heading gain (float)
    robot_pose: current reference pose (PoseStamped object)
    velocity: reference velocity (float)
    omega: reference angular velocity (float)
    return: closed-loop control input (2x1 np array)
    """
    open_loop_control = np.array([[velocity],[omega]]) # compute open loop control
    error = compute_error(robot_pose,"reference") # compute error
    gain = np.array([[ks,0,0],[0,kn,kth]])
    closed_loop_control = open_loop_control + np.dot(gain,error) # compute closed loop control
    return closed_loop_control

def compute_error(robot_pose,target_frame_id):
    """
    Computes the error corresponding to the robot's current pose when compared to the reference pose. Need to input
    frame_id of reference pose because error is defined as coordinates of robot pose in reference pose frame.

    robot_pose: current reference pose (PoseStamped object)
    target_frame_id: target frame id (string); should be the frame id of the reference pose
    return: error of robot pose with respect to reference pose
    """
    robot_position = [robot_pose.pose.position.x,robot_pose.pose.position.z,0] # convert robot position into list
    robot_position_in_refpose_frame = transform_point(robot_position,target_frame_id) # get robot position in reference pose frame
    error = np.array([[-robot_position_in_refpose_frame.point.x],[-robot_position_in_refpose_frame.point.y],[angle_diff(current_reference_pose,robot_pose)]]) # compute error
    return error

def transform_point(point,target_frame_id):
    """
    Transforms a point from world frame to a target frame
    point: list [x,y,z] of point you want to transform
    target_frame: target frame id (string)
    return: coordinates of point in target frame (list [x,y,z])
    """
    global tf_buffer, listener
    point_world = geometry_msgs.msg.PointStamped()
    point_world.header.frame_id = "world" # specify the reference frame of  point object
    point_world.header.stamp = rospy.Time.now() # assign timestamp
    point_world.point.x = point[0]
    point_world.point.y = point[1]
    point_world.point.z = point[2]

    # Transform the point to target frame
    try:
        # Ensure that the transformation is available
        if tf_buffer.can_transform(target_frame_id, 'world', rospy.Time(0), rospy.Duration(1.0)):
            point_reference = tf_buffer.transform(point_world, target_frame_id, rospy.Duration(1.0))
            return point_reference
        else:
            rospy.logwarn("Transformation not available.")
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr("TF error when transforming point: %s", str(e))

def reference_callback(data):
    global current_reference_pose, last_two_theta
    current_reference_pose = data
    last_two_theta.append(data.theta)
    if len(last_two_theta) > 2: # Keep only the last two thetas
        last_two_theta.pop(0)

def mocap_callback(data):
    """
    Callback function for mocap subscriber.
    data: PoseStamped message
    """
    global robot_pose, first_pose_received
    robot_pose = data # store mocap data to robot_pose
    first_pose_received = True

##### Main routine #####
if __name__ == '__main__':
    rospy.init_node('controller') # Init node

    ##### Publishers and Subscribers #####
    # Publisher for sending commands to Thymio
    cmd_pub = rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist, queue_size=10)
    # Subscriber for getting reference pose
    ref_sub = rospy.Subscriber("reference_trajectory",geometry_msgs.msg.Pose2D,reference_callback)
    # Subscriber for getting robot pose (subscribing to mocap node)
    mocap_sub = rospy.Subscriber("/mocap_node/Thymio2/pose", geometry_msgs.msg.PoseStamped, mocap_callback)
    # Buffer and listener for keeping track of reference pose frame
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Wait for first pose from mocap
    while not first_pose_received:
        rospy.loginfo("Waiting for the first mocap message...")
        rospy.sleep(1)

    # Wait for at least two reference poses (to get omega)
    while len(last_two_theta) < 2:
        rospy.loginfo("Waiting for at least two reference poses (to get omega)...")
        rospy.sleep(1)

    rate = rospy.Rate(10.0)
    dt = 1/10

    ##### loop #####
    while not rospy.is_shutdown():
        omega = compute_omgega(last_two_theta[-1],last_two_theta[-2],dt) # Compute omega
        control = compute_control(ks,kn,kth,robot_pose,velocity,omega) # Compute control

        cmd_vel.linear.x = control[0]
        cmd_vel.angular.z = control[1]
        cmd_pub.publish(cmd_vel) # Publish control

        rate.sleep()  # Wait for next loop
