#!/usr/bin/env python
import math
import rospy # New
from geometry_msgs.msg import *
from std_msgs.msg import *
import tf

def locations_doc(loc_name):
    loc_tf = PoseStamped()
    loc_tf.header.stamp = rospy.Time.now()
    loc_tf.header.frame_id = "map"
    loc_tf.pose.position.x, loc_tf.pose.position.y, loc_tf.pose.position.z = 0,0,0
    x,y = 0,0

    if loc_name == 'Living_Room':
        x = 10.0
        y = -2.5
    elif loc_name == 'Dinning_Room':
        x = 8.5
        y = -4.0
    elif loc_name == 'Corridor':
        x = 6.0
        y = -4.0
    elif loc_name == 'Kitchen':
        x = 7.0
        y = -2
    elif loc_name == 'Entrance':
        x = 1.6
        y = -4.5
    elif loc_name == 'Hallway':
        x = 1.0
        y = -3.0
    elif loc_name == 'Bedroom':
        x = 5.167
        y = -1.350
    elif loc_name == 'Docs_Office':
        x = 0.0
        y = -2.308
    elif loc_name == 'Mark_Office':
        x = 0.0
        y = -5.542
    else:
        print("No se donde es eso")
        loc_name = 'nowhere'
    #Origin,0.00,0.00

    loc_tf.pose.position.x, loc_tf.pose.position.y = x,y
    print "Going to ", loc_name
    return loc_tf

def move_to_goal(goal, pub_goal):
    msg_goal = goal
    #print(msg_goal)
    pub_goal.publish(msg_goal)

def move_distance_angle(goal_dist, goal_angle, pub_goal_dist_angle):
    msg_dist_angle = Float32MultiArray()
    msg_dist_angle.data = [goal_dist, goal_angle]
    pub_goal_dist_angle.publish(msg_dist_angle)

def move_distance(goal_dist, pub_goal_dist):
    msg_dist = Float32()
    #msg_dist = 0.1
    msg_dist.data = goal_dist
    pub_goal_dist.publish(msg_dist)


def detect_waveing():
    x,y,z = 3,4,5
    x,y,z = float(x), float(y), float(z)
    p = PointStamped()
    p.header.frame_id = 'camera_link'
    p.point.x, p.point.y, p.point.z = x,y,z
    return p.point

def main():
    print("Go_To_Place Node - - - Running \n")
    rospy.init_node("gotoplace_node", anonymous=True)
    loc_name = sys.argv[1]
    loc_tf = locations_doc(loc_name)
    print("Coordenadas:")
    print(loc_tf.pose.position)
    rate = rospy.Rate(1)
    pub_goal_dist_angle = rospy.Publisher("/simple_move/goal_dist_angle_mio", Float32MultiArray, queue_size=10)
    pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, latch=True,queue_size=10)
    #piece_point.x, piece_point.y = 0.0,0.0
    #distance = math.sqrt(piece_point.x**2 + piece_point.y**2) - 0.30
    #angle = math.atan2(piece_point.y, piece_point.x)
    #goal_dist = 0.01
    #while not rospy.is_shutdown():
    move_to_goal(loc_tf, pub_goal)
    rate.sleep()

if __name__ == "__main__":
    main()