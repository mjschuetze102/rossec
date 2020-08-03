#!/usr/bin/env python
import rospy
from rospy.msg import args_kwds_to_message
from rossec.msg import SecuredMessage
from rossec.security import SecurityWrapper

try:     # Python 2.x
    from cStringIO import StringIO
except:  # Python 3.x
    from io import StringIO


class Publisher(SecurityWrapper):
    """
    Security wrapper for the Publisher class
    """

    def __init__(self, *args, **kwargs):
        super(Publisher, self).__init__()
        self.name = args[0] if len(args) > 0 else kwargs.get('name', None)
        self.data_class = args[1] if len(args) > 1 else kwargs.get('data_class', None)
        
        # Change values in args list as needed
        args = args[2:]
        kwargs['name'] = self.name
        kwargs['data_class'] = SecuredMessage
        self.pub = rospy.Publisher(*args, **kwargs)

    def publish(self, *args, **kwargs):
        # Convert arguments into an instance of provided data_class
        orig = args_kwds_to_message(self.data_class, args, kwargs)

        # Get the attribute information
        buff = StringIO()
        orig.serialize(buff)
        contents = buff.getvalue()
        buff.close()
        
        # Create the new message to send
        msg = SecuredMessage()
        msg.MessageType = str(self.data_class)
        msg.MessageContent = self.encode(contents)
        self.pub.publish(msg)

class Subscriber(SecurityWrapper):
    """
    Security wrapper for the Subscriber class
    """

    def __init__(self, *args, **kwargs):
        super(Subscriber, self).__init__()
        self.name = args[0] if len(args) > 0 else kwargs.get('name', None)
        self.data_class = args[1] if len(args) > 1 else kwargs.get('data_class', None)
        self.callback = args[2] if len(args) > 2 else kwargs.get('callback', None)
        self.callback_args = args[3] if len(args) > 3 else kwargs.get('callback_args', None)
        
        # Change values in args list as needed
        args = args[4:]
        kwargs['name'] = self.name
        kwargs['data_class'] = SecuredMessage
        kwargs['callback'] = self.__callback__
        kwargs['callback_args'] = None
        self.sub = rospy.Subscriber(*args, **kwargs)

    def __callback__(self, data):
        # Create the original message from the encrypted data
        obj = self.data_class()
        obj.deserialize(self.decode(data.MessageContent))

        try:     # If callback takes args
            self.callback(obj, self.callback_args)
        except:  # If callback does not take args
            self.callback(obj)

    def unregister(self):
        self.sub.unregister()

