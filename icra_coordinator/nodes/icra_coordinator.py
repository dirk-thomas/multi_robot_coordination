#!/usr/bin/env python

from threading import Timer

import roslib;
roslib.load_manifest('icra_coordinator')
roslib.load_manifest('pr2_service_interface')
import rospy

from icra_coordinator.srv import *

class IcraCoordinator:

    def __init__(self):
        self._timer = None
        rospy.Service('pr2/notify_pick_up_completed', NotifyPickUpCompleted, self.notify_pick_up_completed)
        rospy.loginfo('IcraCoordinator ready')

        self._timer = Timer(3.0, self._trigger_pick_up)
        self._timer.start()

    def _trigger_pick_up(self):
        self._timer = None
        proxy = rospy.ServiceProxy('pr2/trigger_pick_up', TriggerPickUp)
        req = TriggerPickUpRequest()
        rospy.loginfo('IcraCoordinator._trigger_pick_up()')
        try:
            proxy(req)
        except rospy.service.ServiceException:
            rospy.logwarn('IcraCoordinator._trigger_pick_up() service unavailable - trying again')
            try:
                rospy.wait_for_service('pr2/trigger_pick_up', 1.0)
                rospy.loginfo('IcraCoordinator._trigger_pick_up() service available again')
                self._trigger_pick_up()
            except rospy.exceptions.ROSException:
                self._timer = Timer(1.0, self._trigger_pick_up)
                self._timer.start()

    def notify_pick_up_completed(self, req):
        rospy.loginfo('IcraCoordinator.notify_pick_up_completed()\n')
        if self._timer is not None:
            self._timer.cancel()
        self._timer = Timer(3.0, self._trigger_pick_up)
        self._timer.start()
        return NotifyPickUpCompletedResponse()


if __name__ == '__main__':
    rospy.init_node('icra_coordinator')
    coordinator = IcraCoordinator()
    rospy.spin()
