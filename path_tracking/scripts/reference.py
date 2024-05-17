#!/usr/bin/env python3
import rospy
import tf2_ros
import math
import numpy as np
import geometry_msgs.msg
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import tf.transformations
import ast

##### Get parameters #####
waypoints = rospy.get_param("/reference/waypoints",default=[[0,0],[1,0],[1,1],[0,1]]) # trajectory to follow (default is a square)
waypoints = ast.literal_eval(waypoints) # convert string to list
velocity = rospy.get_param("/reference/velocity",default=0.1) # desired velocity

##### Other global variables (that aren't parameters) #####
segment_num = 0 # counter variable for keeping track of which segment robot is on
robot_pose = None # variable to store current robot pose
current_ref_pose = [0,0,0] # variable to store current reference pose
last_ref_pose = [0,0,0] # variable to store reference pose at the last time step
first_pose_received = False # variable to indicate that first pose message has been received

def mocap_callback(data):
    """
    Callback function for mocap subscriber.
    data: Pose object
    """
    global robot_pose, first_pose_received
    robot_pose = data # store mocap data to robot_pose
    first_pose_received = True

def get_duration(Point1,Point2,velocity):
    """
    Calculates the duration of a segment between two points.
    Point1: first point (list [x1,y1])
    Point2: second point (list [x2,y2])
    velocity: reference velocity (float)
    return: duration of the segment between Point1 and Point2
    """
    x1 = Point1[0]
    y1 = Point1[1]
    x2 = Point2[0]
    y2 = Point2[1]
    distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    duration = distance/velocity
    return duration

def get_current_ref_pose(waypoints,current_time,velocity,current_pose,last_ref_pose):
    """
    Calculates the reference pose of the trajectory defined by a list of waypoints corresponding to the current time.
    waypoints: list of waypoints (list containing points [[x1,y1],[x2,y2],...])
    velocity: velocity (float)
    current_time: current time (i.e. time elapsed since start of loop) (float)
    return: next pose of the trajectory (list [x,y,theta])
    """
    duration_list = [0]
    # Calculate duration for each segment
    for i in range(len(waypoints)):
        if i < len(waypoints)-1:
            Point1 = waypoints[i]
            Point2 = waypoints[i+1]
        elif i == len(waypoints)-1: # when we're at the last waypoint and going back to starting point
            Point1 = waypoints[i]
            Point2 = waypoints[0]
        duration_list.append(get_duration(Point1,Point2,velocity))
    total_duration = sum(duration_list) # total duration to traverse entire trajectory
    time_elapsed = current_time % total_duration # so that we can repeat trajectory indefinitely

    cum_duration_list = np.cumsum(duration_list).tolist() # get cumulative durations

    # Get current segment number
    for i in range(len(cum_duration_list)):
        if cum_duration_list[i] < time_elapsed < cum_duration_list[i+1]:
            segment_num = i

    # Call segment_pose() on appropriate segment
    segment_time = time_elapsed - cum_duration_list[segment_num] # time since current segment started
    if segment_num < (len(waypoints)-1): # if not the last segment
        Point1 = waypoints[segment_num]
        Point2 = waypoints[segment_num+1]
    if segment_num == (len(waypoints)-1): # handle wrapping
        Point1 = waypoints[segment_num]
        Point2 = waypoints[0]
    duration = duration_list[segment_num+1]
    current_ref_pose = pose_for_segment(Point1,Point2,segment_time,duration) # calculate x,y first
    theta = get_theta(current_ref_pose,last_ref_pose) # then calculate theta
    current_ref_pose.append(theta)
    return current_ref_pose

def pose_for_segment(Point1,Point2,segment_time,duration):
    """
    Given two points that make up a segment and the time since start of segment, it returns the current reference pose.
    Point1: first point (list [x1,y1])
    Point2: second point (list [x2,y2])
    segnum:current segment number (int)
    segment_time: time since segment started (float)
    duration: duration it takes to complete segment (float)
    return: next pose in the segment (list [x,y,theta])
    """
    x1 = Point1[0]
    y1 = Point1[1]
    x2 = Point2[0]
    y2 = Point2[1]
    x = x1 + (x2-x1)*(segment_time/duration)
    y = y1 + (y2-y1)*(segment_time/duration)
    return [x,y]

def get_theta(current_ref_pose,last_ref_pose):
    """
    Given (x,y) of current reference pose and (x,y) of last reference pose, calculates and returns the theta of current
    reference pose. Basically calculates theta by constructing a tangent line and getting the angle of that tangent line.
    current_ref_pose: current reference pose (list [x1,y1])
    last_ref_pose: last reference pose (list [x2,y2])
    return: heading of reference pose frame (float)
    """
    x1 = last_ref_pose[0]
    y1 = last_ref_pose[1]
    x2 = current_ref_pose[0]
    y2 = current_ref_pose[1]
    theta = math.atan2(y2-y1,x2-x1)
    return theta

def create_marker(x,y):
    """
    Creates Marker message for visualizing the reference trajectory in RViz.
    x: x coordinate of the marker (float)
    y: y coordinate of the marker (float)
    return: Marker message
    """
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "reference_points"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.g = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    return marker

def publish_transform(x,y,theta):
    """
    Broadcasts transform from world frame to reference pose frame to a TransformBroadcaster.
    x: x coordinate of the reference pose (float)
    y: y coordinate of the reference pose (float)
    theta: heading of the reference pose (float)
    """
    global br
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world" # parent frame
    t.child_frame_id = "reference" # child frame
    # Translation
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0
    # Rotation
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = math.sin(theta/2)
    t.transform.rotation.w = math.cos(theta/2)
    # Boardcast transformation
    br.sendTransform(t)

def publish_transform2(robot_pose):
    """
    Broadcasts transform from world frame to robot pose frame to a TransformBroadcaster
    x: x coordinate of the robot pose (float)
    y: y coordinate of the robot pose (float)
    theta: heading of the robot pose (float)
    """
    global br
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world" # parent frame
    t.child_frame_id = "robot" # child frame
    # Translation
    t.transform.translation.x = robot_pose.pose.position.x
    t.transform.translation.y = robot_pose.pose.position.z
    t.transform.translation.z = robot_pose.pose.position.y
    # Rotation
    t.transform.rotation.x = robot_pose.pose.orientation.x
    t.transform.rotation.y = robot_pose.pose.orientation.z
    t.transform.rotation.z = -robot_pose.pose.orientation.y
    t.transform.rotation.w = robot_pose.pose.orientation.w
    # Boardcast transformation
    br.sendTransform(t)

##### Main routine #####
if __name__ == '__main__':
    rospy.init_node("reference") # Init node

    ##### Publishers and Subscribers #####
    # Publisher for the publishing reference trajectory
    ref_pub = rospy.Publisher("/reference_trajectory",geometry_msgs.msg.Pose2D,queue_size=1)
    # Publisher for visualizing reference trajectory in RViz
    vis_pub = rospy.Publisher("visualization_marker",Marker,queue_size=1)
    # Subscriber for getting robot pose (subscribing to mocap node)
    mocap_sub = rospy.Subscriber("/mocap_node/Thymio2/pose",geometry_msgs.msg.PoseStamped,mocap_callback) # pose
    rate = rospy.Rate(10.0) # Define loop execution frequency
    # Transform broadcaster for broadcasting reference pose frame
    br = tf2_ros.TransformBroadcaster()
    # Buffer and listener for reference pose frame
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Wait for first pose from mocap
    while not first_pose_received:
        rospy.loginfo("Waiting for the first mocap message...")
        rospy.sleep(1)
    initial_pose = robot_pose
    rospy.loginfo(f"\nRobot initial pose:\n{initial_pose.pose}")

    # Redefine waypoints to be relative to initial pose
    for i in range(len(waypoints)):
        waypoints[i][0] = waypoints[i][0] + initial_pose.pose.position.x
        waypoints[i][1] = waypoints[i][1] + initial_pose.pose.position.z
    rospy.loginfo(f"\nWaypoints:\n{waypoints}")

    initial_time = rospy.Time.now().to_sec()  # initial time right before looping starts
    ##### loop #####
    while not rospy.is_shutdown():
        abs_time = rospy.Time.now().to_sec() # keep track of absolute time every loop
        loop_time = abs_time - initial_time # time since loop started

        # Get current reference pose
        current_ref_pose = get_current_ref_pose(waypoints,loop_time,velocity,current_ref_pose,last_ref_pose)

        # Create reference pose message (Pose2D message)
        reference_msg = geometry_msgs.msg.Pose2D()
        reference_msg.x = current_ref_pose[0]
        reference_msg.y = current_ref_pose[1]
        reference_msg.theta = current_ref_pose[2]

        marker_point = create_marker(current_ref_pose[0],current_ref_pose[1]) # Create Marker for visualizing reference trajectory

        ref_pub.publish(reference_msg) # Publish reference pose message
        vis_pub.publish(marker_point) # Publish Marker object for visualization

        publish_transform(current_ref_pose[0],current_ref_pose[1],current_ref_pose[2]) # Broadcast transformation from world frame to reference pose frame
        publish_transform2(robot_pose) # Broadcast transformaiton from world frame to robot pose frame

        last_ref_pose = current_ref_pose # current reference pose becomes last reference pose for next loop

        rate.sleep() # Wait for next loop
