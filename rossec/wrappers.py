#!/usr/bin/env python
import rospy


class Publisher:
    """
    Security wrapper for the Publisher class
    """

    def __init__(self, *args, **kwargs):
        data_class = args[1]
        print("Data Class: ", data_class)
        self.pub = rospy.Publisher(*args, **kwargs)

    def publish(self, *args, **kwargs):
        self.pub.publish(*args, **kwargs)

class Subscriber:
    """
    Security wrapper for the Subscriber class
    """

    def __init__(self, *args, **kwargs):
        data_class = args[1]
        callback = args[2] if len(args) > 2 else kwargs.get('callback', None)
        callback_args = args[3] if len(args) > 3 else kwargs.get('callback_args', None)
        print("Data Class: ", data_class)
        print("Callback: ", callback)
        print("Callback Args: ", callback_args)
        self.sub = rospy.Subscriber(*args, **kwargs)

    def unregister(self):
        self.sub.unregister()

