#!/usr/bin/env python

from random import random
import threading

import roslib;
roslib.load_manifest('std_srvs')
import rospy

from std_srvs.srv import *

class ServiceHandler:

    def __init__(self, service_name, callback, notification_name):
        self._service_name = service_name
        self._callback = callback
        self._notification_name = notification_name

        self._last_callback_id = None
        self._timer = None

        self._service = rospy.Service(service_name, Empty, self._call_callback)

    def _call_callback(self, req):
        rospy.logdebug('ServiceHandler._call_callback() incoming service call "%s"' % self._service_name)
        if self._last_callback_id is not None:
            rospy.logwarn('ServiceHandler._call_callback() previous service call "%s" has not yet notified completion' % self._service_name)
        self._last_callback_id = random()
        return self._callback(self._last_callback_id)

    def notify(self, callback_id):
        self._pending_notification = callback_id
        self._notify()

    def _notify(self):
        rospy.logdebug('ServiceHandler._notify()')
        callback_id = self._pending_notification
        self._pending_notification = None

        if self._last_callback_id is None:
            rospy.logwarn('ServiceHandler._notify() no pending notification - called notify() multiple times for one callback?')
            return

        if callback_id != self._last_callback_id:
            rospy.logwarn('ServiceHandler._notify() callback is outdated - notification will be suppressed')
            return

        try:
            rospy.wait_for_service(self._notification_name, 1.0)
            self._call(callback_id)
        except rospy.exceptions.ROSException:
            if self._pending_notification is not None:
                # skip if overridden/outdated while waited for service
                rospy.logwarn('ServiceHandler._notify() callback is outdated - notification will be suppressed')
                return
            rospy.logwarn('ServiceHandler._notify() service "%s" unavailable - trying again' % self._notification_name)
            # retry soon
            self._pending_notification = callback_id
            self._timer = threading.Timer(1.0, self._notify)
            self._timer.start()

    def _call(self, callback_id):
        if callback_id != self._last_callback_id:
            rospy.logwarn('ServiceHandler._call() callback is outdated - notification will be suppressed')
            return

        proxy = rospy.ServiceProxy(self._notification_name, Empty)
        req = EmptyRequest()
        try:
            proxy(req)
            rospy.logdebug('ServiceHandler._call() notification for service "%s" sent' % self._service_name)
            self._last_callback_id = None
        except rospy.service.ServiceException:
            rospy.logwarn('ServiceHandler._call() service "%s" unavailable - trying again' % self._notification_name)
            # cancel pending timer
            if self._timer is not None:
                self._timer.cancel()
            # retry soon
            self._timer = threading.Timer(1.0, self._notify)
            self._timer.start()
