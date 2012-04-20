#!/usr/bin/env python

from random import randint
from threading import Timer

import roslib;
roslib.load_manifest('icra_coordinator')
import rospy

from icra_coordinator import ServiceHandler
from icra_coordinator.srv import *

class VehicleServiceMock:

    def __init__(self):
        self._callback_id = 0
        self._timer = None
        self._pick_up_service = ServiceHandler('vehicle/trigger_pick_up', self._trigger_pick_up, 'vehicle/notify_pick_up_completed')
        self._place_down_service = ServiceHandler('vehicle/trigger_place_down', self._trigger_place_down, 'vehicle/notify_place_down_completed')
        rospy.loginfo('VehicleServiceMock ready')

    def _trigger_pick_up(self, id):
        rospy.loginfo('VehicleServiceMock.trigger_pick_up()')
        self._callback_id = id
        self._timer = Timer(randint(4, 8), self._pick_up_completed)
        self._timer.start()
        return EmptyResponse()

    def _pick_up_completed(self):
        rospy.loginfo('VehicleServiceMock._pick_up_completed()\n')
        self._pick_up_service.notify(self._callback_id)

    def _trigger_place_down(self, id):
        rospy.loginfo('VehicleServiceMock.trigger_place_down()')
        self._callback_id = id
        self._timer = Timer(1.0, self._place_down_completed)
        self._timer.start()
        return EmptyResponse()

    def _place_down_completed(self):
        rospy.loginfo('VehicleServiceMock._place_down_completed()\n')
        self._place_down_service.notify(self._callback_id)


if __name__ == '__main__':
    rospy.init_node('vehicle_service_interface')
    mock = VehicleServiceMock()
    rospy.spin()
