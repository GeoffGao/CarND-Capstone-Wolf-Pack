#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import time
'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200# Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        # Initialize the node with the Master Process
        rospy.init_node('waypoint_updater')
        
        # Subscribers
        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        # Publishers
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1, latch = True)
       
        # Member variables
        self.car_pose = None
        self.car_position = None
        self.car_orientation = None
        self.waypoints = []
        self.final_waypoints = []
        self.do_work()

    def do_work(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
                if (self.car_position != None and self.waypoints != None):
                       self.generate_final_waypoints(self.car_position, self.waypoints)
                       self.publish()
                else:
                       if self.car_position == None:
                               rospy.logwarn("/current_pose not received")
                       if self.waypoints == None:
                               rospy.logwarn("/base_waypoints not received")
                rate.sleep()

    def pose_cb(self, msg):
        self.car_pose = msg.pose
        self.car_position = self.car_pose.position       

    def waypoints_cb(self, msg):
        for waypoint in msg.waypoints:
                self.waypoints.append(waypoint)
        self.base_waypoints_sub.unregister()
        rospy.loginfo("Unregistered from /base_waypoints topic")

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def generate_final_waypoints(self, position, waypoints):
        closestWaypoint = self.closest_waypoint(position, waypoints)
        #rospy.logwarn(closestWaypoint)
        velocity = rospy.get_param('~/waypoint_loader/velocity', 40.0) * 0.44704
        self.final_waypoints = []
        if ((closestWaypoint + LOOKAHEAD_WPS) < len(waypoints)):
                for idx in range(closestWaypoint, closestWaypoint + LOOKAHEAD_WPS):
                        self.set_waypoint_velocity(waypoints, idx, velocity)
                        self.final_waypoints.append(waypoints[idx])
        else:
                for idx in range(closestWaypoint, len(waypoints)):
                        self.set_waypoint_velocity(waypoints, idx, velocity)
                        self.final_waypoints.append(waypoints[idx])
    
    def publish(self):
        final_waypoints_msg = Lane()
        #final_waypoints_msg.header.frame_id = '/world'
        #final_waypoints_msg.header.stamp = rospy.time(0)
        final_waypoints_msg.waypoints = list(self.final_waypoints)
        #rospy.loginfo(final_waypoints_msg)
        self.final_waypoints_pub.publish(final_waypoints_msg)    

    def closest_waypoint(self, position, waypoints):
        closestLen = float("inf")
        closestWaypoint = 0
        dist = 0.0
        for idx in range(0, len(waypoints)):
                x = position.x
                y = position.y
                map_x = waypoints[idx].pose.pose.position.x
                map_y = waypoints[idx].pose.pose.position.y
                dist = self.distance_any(x, y, map_x, map_y)
                if (dist < closestLen):
                        closestLen = dist
                        closestWaypoint = idx
        return closestWaypoint

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance_any(self, x1, y1, x2, y2):
        return math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
        
