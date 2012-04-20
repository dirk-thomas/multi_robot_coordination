#!/usr/bin/env python

from threading import Timer

import roslib;
roslib.load_manifest('icra_coordinator')
roslib.load_manifest('std_srvs')
import rospy

from icra_coordinator import ServiceHandler
from std_srvs.srv import *

class Pr2ServiceMock:

    def __init__(self):
        self._callback_id = 0
        self._timer = None
        self._pick_up_service = ServiceHandler('pr2/trigger_pick_up', self._trigger_pick_up, 'pr2/notify_pick_up_completed')
        self._place_down_service = ServiceHandler('pr2/trigger_place_down', self._trigger_place_down, 'pr2/notify_place_down_completed')
        rospy.loginfo('Pr2ServiceMock ready')

    def _trigger_pick_up(self, id):
        rospy.loginfo('Pr2ServiceMock.trigger_pick_up()')
        self._callback_id = id
        self._timer = Timer(3.0, self._pick_up_completed)
        self._timer.start()
        return EmptyResponse()

    def _pick_up_completed(self):
        rospy.loginfo('Pr2ServiceMock._pick_up_completed()\n')
        self._pick_up_service.notify(self._callback_id)

    def _trigger_place_down(self, id):
        rospy.loginfo('Pr2ServiceMock.trigger_place_down()')
        self._callback_id = id
        self._timer = Timer(1.0, self._place_down_completed)
        self._timer.start()
        return EmptyResponse()

    def _place_down_completed(self):
        rospy.loginfo('Pr2ServiceMock._place_down_completed()\n')
        self._place_down_service.notify(self._callback_id)


if __name__ == '__main__':
    rospy.init_node('pr2_service_interface')
    mock = Pr2ServiceMock()
    rospy.spin()
