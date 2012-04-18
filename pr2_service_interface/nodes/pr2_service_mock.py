#!/usr/bin/env python

from threading import Timer

import roslib;
roslib.load_manifest('icra_coordinator')
import rospy

from icra_coordinator import ServiceHandler
from icra_coordinator.srv import *

class Pr2ServiceMock:

    def __init__(self):
        self._callback_id = 0
        self._timer = None
        self._pick_up_service = ServiceHandler('icra/pr2/trigger_pick_up', TriggerPickUp, self._trigger_pick_up, 'icra/notify_pick_up_completed', NotifyPickUpCompleted)
        rospy.loginfo('Pr2ServiceMock ready')

    def _trigger_pick_up(self, id):
        rospy.loginfo('Pr2ServiceMock.trigger_pick_up()')
        self._callback_id = id
        self._timer = Timer(3.0, self._pick_up_completed)
        self._timer.start()
        return TriggerPickUpResponse()

    def _pick_up_completed(self):
        rospy.loginfo('Pr2ServiceMock._pick_up_completed()')
        self._pick_up_service.notify(self._callback_id)


if __name__ == '__main__':
    rospy.init_node('pr2_service_interface')
    mock = Pr2ServiceMock()
    rospy.spin()
