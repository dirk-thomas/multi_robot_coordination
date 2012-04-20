#!/usr/bin/env python

from threading import Timer

import roslib;
roslib.load_manifest('icra_coordinator')
roslib.load_manifest('std_srvs')
roslib.load_manifest('arm_service_interface')
roslib.load_manifest('pr2_service_interface')
roslib.load_manifest('vehicle_service_interface')
import rospy

from std_srvs.srv import *

class IcraCoordinator:

    def __init__(self):
        self._trigger_timers = {}
        self._notify_timers = {}
        self._timer = None

        self._vehicle_pick_up_completed = True

        rospy.Service('arm/notify_pick_up_completed', Empty, self.notify_arm_pick_up_completed)
        rospy.Service('arm/notify_place_down_completed', Empty, self.notify_arm_place_down_completed)

        rospy.Service('pr2/notify_pick_up_completed', Empty, self.notify_pr2_pick_up_completed)
        rospy.Service('pr2/notify_place_down_completed', Empty, self.notify_pr2_place_down_completed)

        rospy.Service('vehicle/notify_pick_up_completed', Empty, self.notify_vehicle_pick_up_completed)
        rospy.Service('vehicle/notify_place_down_completed', Empty, self.notify_vehicle_place_down_completed)

        rospy.loginfo('IcraCoordinator ready')

        self._timer = Timer(3.0, self._trigger_arm_pick_up)
        self._timer.start()


    def _trigger_arm_pick_up(self):
        self._trigger_pick_up('arm')

    def notify_arm_pick_up_completed(self, req):
        return self._notify_pick_up_completed('arm', self._trigger_vehicle_pick_up_and_arm_place_down)

    def _trigger_vehicle_pick_up_and_arm_place_down(self):
        self._vehicle_pick_up_completed = False
        self._trigger_pick_up('vehicle')
        self._trigger_place_down('arm')

    def notify_vehicle_pick_up_completed(self, req):
        self._vehicle_pick_up_completed = True
        return EmptyResponse()

    def notify_arm_place_down_completed(self, req):
        return self._notify_place_down_completed('arm', self._trigger_pr2_pick_up)

    def _trigger_pr2_pick_up(self):
        self._trigger_pick_up('pr2')

    def notify_pr2_pick_up_completed(self, req):
        return self._notify_pick_up_completed('pr2', self._wait_for_vehicle_pick_up)

    def _wait_for_vehicle_pick_up(self):
        if not self._vehicle_pick_up_completed:
            if not rospy.is_shutdown():
                self._timer = Timer(1.0, self._wait_for_vehicle_pick_up)
                self._timer.start()
            return
        self._trigger_place_down('pr2')

    def notify_pr2_place_down_completed(self, req):
        return self._notify_place_down_completed('pr2', self._trigger_vehicle_place_down)

    def _trigger_vehicle_place_down(self):
        self._trigger_place_down('vehicle')

    def notify_vehicle_place_down_completed(self, req):
        return self._notify_place_down_completed('vehicle', self._trigger_arm_pick_up)


    def _trigger_pick_up(self, robot_name):
        self._trigger(robot_name, 'pick_up')

    def _trigger_place_down(self, robot_name):
        self._trigger(robot_name, 'place_down')

    def _trigger(self, robot_name, operation):
        if robot_name in self._trigger_timers:
            self._trigger_timers[robot_name].cancel()
        proxy = rospy.ServiceProxy('%s/trigger_%s' % (robot_name, operation), Empty)
        req = EmptyRequest()
        rospy.loginfo('IcraCoordinator._trigger_%s(%s)' % (operation, robot_name))
        try:
            proxy(req)
        except rospy.service.ServiceException:
            rospy.logwarn('IcraCoordinator._trigger_%s(%s) service unavailable - trying again' % (operation, robot_name))
            try:
                rospy.wait_for_service('%s/trigger_%s' % (robot_name, operation), 1.0)
                rospy.loginfo('IcraCoordinator._trigger_%s(%s) service available again' % (operation, robot_name))
                self._trigger(robot_name, operation)
            except rospy.exceptions.ROSException:
                if not rospy.is_shutdown():
                    self._trigger_timers[robot_name] = Timer(1.0, lambda: self._trigger(robot_name, operation))
                    self._trigger_timers[robot_name].start()


    def _notify_pick_up_completed(self, robot_name, next_callback):
        return self._notify_completed(robot_name, next_callback, 'pick_up')

    def _notify_place_down_completed(self, robot_name, next_callback):
        return self._notify_completed(robot_name, next_callback, 'place_down')

    def _notify_completed(self, robot_name, next_callback, operation):
        rospy.loginfo('IcraCoordinator._notify_%s_completed(%s)\n' % (operation, robot_name))
        if robot_name in self._notify_timers:
            self._notify_timers[robot_name].cancel()
        if not rospy.is_shutdown():
            self._notify_timers[robot_name] = Timer(2.0, next_callback)
            self._notify_timers[robot_name].start()
        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('icra_coordinator')
    coordinator = IcraCoordinator()
    rospy.spin()
