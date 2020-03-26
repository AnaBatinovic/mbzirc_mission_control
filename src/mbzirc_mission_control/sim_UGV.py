#!/usr/bin/env python
# -*- coding: utf-8 -*-
import random

import rospy
from geodesy import utm

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseActionResult

from mbzirc_mission_control.srv import GoPosition, GoGPSPosition, PickupUGV, PlaceUGV, TeleportBrick, TeleportBrickRequest
from mbzirc_mission_control.msg import HWStatusUGV

from utilities import CustomLogger
logger = CustomLogger()


class SimUGV(object):
    def __init__(self):
        # Initialize class variables.
        self.pose_estimate = None
        self.move_goal_status = 'None'
        self.utm_origin = utm.fromLatLong(*rospy.get_param('/MISSION_CTRL/arena_params/GPS_origin')[0:3])

        # TODO: make it parametric
        self.pick_success_rate = 1
        self.place_success_rate = 1

        # Create subscribers.
        rospy.Subscriber('odometry', Odometry, self.odometry_cb, queue_size=1)
        rospy.Subscriber('move_base/result', MoveBaseActionResult, self.move_base_cb, queue_size=1)

        # Create publishers.
        self.goal_pose_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        self.hardware_pub = rospy.Publisher('hardware_status', HWStatusUGV, queue_size=1)

        # Create services.
        rospy.Service('go_to_gps', GoGPSPosition, self.go_to_GPS_position_srv)
        rospy.Service('go_to_position', GoPosition, self.go_to_local_position_srv)
        rospy.Service('brick_pickup', PickupUGV, self.pick_srv)
        rospy.Service('brick_place', PlaceUGV, self.place_srv)

        self.hardware_pub.publish(HWStatusUGV.READY, '')

        rospy.spin()

    def odometry_cb(self, msg):
        self.pose_estimate = msg.pose.pose

    def move_base_cb(self, msg):
        rospy.loginfo('Move Base Status: %s', msg.status.text)
        if msg.status.status == GoalStatus.SUCCEEDED:  # Success, goal reached.
            self.move_goal_status = 'Reached'
        elif msg.status.status == GoalStatus.ABORTED:  # Failure.
            self.move_goal_status = 'Aborted'
            # TODO: implement recovery behaviour

    def go_to_local_position_srv(self, req):
        """
        Service for commanding the new desired position in local frame.

        Args:
            req (GoPositionRequest): Service request with desired position.

        Returns:
            True if desired position is reached, False otherwise.
        """
        rospy.loginfo('Going to position:\n%s', req.pose)
        success = self.go_to_position(req.pose)
        return success

    def go_to_GPS_position_srv(self, req):
        """
        Service for commanding the new desired position in global (GPS) frame.

        Args:
            req (GoGPSPositionRequest): Service request with desired position.

        Returns:
            True if desired position is reached, False otherwise.
        """
        rospy.loginfo('Going to GPS position:\n%s', req.pose)
        local_goal = self.gps_to_local(req.pose)
        success = self.go_to_position(local_goal)
        return success

    def pick_srv(self, req):
        if random.random() < self.pick_success_rate:
            rospy.wait_for_service('/teleport_brick')
            try:
                teleport = rospy.ServiceProxy('/teleport_brick', TeleportBrick)
                request = TeleportBrickRequest()
                request.brick_id = req.brick
                request.step = 'delete'
                resp = teleport(request)
                if resp is False:
                    return False
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr('Service call failed with error: %s', e)
                return False

            outcome = 'Success'
            logger.info('PICKING UP BRICK %s... %s', req.brick, outcome)
        else:
            outcome = 'Failed'
            logger.warn('PICKING UP BRICK %s... %s', req.brick, outcome)
        return outcome == 'Success'

    def place_srv(self, req):
        # TODO: ako ne uspije postavljanje cigle jer nedostaje u kosari ili se nesto drugo dogodi,
        #       treba pod istom oznakom postaviti drugu cigle iste boje, a ako ni to ne uspije, dojavit
        #       da nije uspjelo postavljanje cigle s tocno tim indeskom.
        if random.random() < self.place_success_rate:
            rospy.wait_for_service('/teleport_brick')
            try:
                teleport = rospy.ServiceProxy('/teleport_brick', TeleportBrick)
                request = TeleportBrickRequest()
                request.brick_id = req.brick
                request.step = 'spawn'
                request.pose = req.pose
                resp = teleport(request)
                if resp is False:
                    return False
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr('Service call failed with error: %s', e)
                return False

            outcome = 'Success'
            logger.info('PLACING BRICK %s... %s', req.brick, outcome)
        else:
            outcome = 'Failed'
            logger.warn('PLACING BRICK %s... %s', req.brick, outcome)
        return outcome

    def go_to_position(self, goal_pose):
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.pose = goal_pose
        self.goal_pose_pub.publish(goal)

        # Wait to get to the goal point.
        while self.move_goal_status == 'None':
            rospy.sleep(0.5)

        if self.move_goal_status == 'Reached':
            rospy.loginfo('Goal reached.')
            self.move_goal_status = 'None'
            return True
        else:
            rospy.logerr('Goal aborted!')
            self.move_goal_status = 'None'
            return False

    def gps_to_local(self, gps_goal):
        utm_goal = utm.fromMsg(gps_goal.position)
        local_goal = Pose()
        local_goal.position.x = utm_goal.easting - self.utm_origin.easting
        local_goal.position.y = utm_goal.northing - self.utm_origin.northing
        local_goal.position.z = gps_goal.position.altitude
        local_goal.orientation = gps_goal.orientation
        return local_goal

if __name__ == "__main__":
    rospy.init_node("simualted_UGV")

    try:
        logger.name = rospy.get_name()
        node = SimUGV()
    except rospy.ROSInterruptException:
        pass
