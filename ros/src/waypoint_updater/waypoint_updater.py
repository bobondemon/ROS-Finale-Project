#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from tf.transformations import quaternion_from_euler

import math

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.loginfo('='*10+' [CSChen]: Init WaypointUpdater Node')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # rospy.Subscriber('/traffic_waypoint',Int32, self.traffic_cb)


        self.final_lane_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.rate = rospy.Rate(10)
        start_time = 0
        self.cur_pose = None
        self.base_lane = None
        self.final_lane = None

        

        while not start_time:
            start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():

            q_rot1 = quaternion_from_euler(math.pi, 0, 0)
            rospy.loginfo('='*10+' [CSChen]: rpy (math.pi, 0, 0) = (x,y,z,w) {}'.format(q_rot1))

            q_rot2 = quaternion_from_euler(0, 0, 0)
            rospy.loginfo('='*10+' [CSChen]: rpy (0, 0, 0) = (x,y,z,w) {}'.format(q_rot2))

            q_rot3 = quaternion_from_euler(0, math.pi, 0)
            rospy.loginfo('='*10+' [CSChen]: rpy (0, math.pi, 0) = (x,y,z,w) {}'.format(q_rot3))

            q_rot4 = quaternion_from_euler(0, 0, math.pi)
            rospy.loginfo('='*10+' [CSChen]: rpy (0, 0, math.pi) = (x,y,z,w) {}'.format(q_rot4))

            q_rot5 = quaternion_from_euler(0, math.pi, math.pi)
            rospy.loginfo('='*10+' [CSChen]: rpy (0, math.pi, math.pi) = (x,y,z,w) {}'.format(q_rot5))

            # elapsed = rospy.Time.now().to_sec() - start_time
            self.final_lane = self.filter_past_wp(self.base_lane,self.cur_pose)
            if not self.final_lane == None:
                self.final_lane_pub.publish(self.final_lane)
            self.rate.sleep()

        rospy.spin()

    def filter_past_wp(self,base_lane,cur_pose):
        if base_lane == None:
            # rospy.loginfo('='*10+' [CSChen]: base_line is None')
            return None
        if cur_pose == None:
            # rospy.loginfo('='*10+' [CSChen]: cur_pose is None')
            return None
        rtn_waypoint = []
        counter = 1
        for bwp in base_lane.waypoints:
            if bwp.pose.pose.position.x > cur_pose.pose.position.x and counter <= LOOKAHEAD_WPS:
                # wp = Waypoint(bwp)
                bwp.twist.twist.linear.x = 50
                rtn_waypoint.append(bwp)
                counter += 1
            # rtn_waypoint.append(bwp)
            # counter += 1
        # rospy.loginfo('='*10+' [CSChen]: publishing {} of waypoints'.format(len(rtn_waypoint)))
        rtn_final_lane = Lane()
        rtn_final_lane.header = base_lane.header
        rtn_final_lane.waypoints = rtn_waypoint
        v = self.get_waypoint_velocity(rtn_waypoint[-1])
        # rospy.loginfo('='*10+' [CSChen]: rtn_waypoint velocity = {}'.format(v))
        return rtn_final_lane

    def pose_cb(self, msg):
        # msg type: geometry_msgs/PoseStamped
        # TODO: Implement
        # rospy.loginfo('='*10+' [CSChen]: cur_pose = {},{},{}; cur_pose.header.stamp = {}'.format(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,msg.header.stamp))
        # print('='*10+' [CSChen]: cur_pose')
        # rospy.loginfo('='*10+' [CSChen]: cur_pose = {}'.format(msg)
        self.cur_pose = msg

    def waypoints_cb(self, lane):
        # TODO: Implement
        if self.base_lane==None:
            rospy.loginfo('='*10+' [CSChen]: Assigning base_waypoint')
            self.base_lane = lane
        else:
            rospy.loginfo('='*10+' [CSChen]: Having more than two times of base_waypoint')
            # print('='*10+' [CSChen]: Having more than two times of base_waypoint')

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
