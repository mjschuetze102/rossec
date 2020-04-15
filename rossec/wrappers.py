#!/usr/bin/env python
import json
import rospy
from rossec.msg import SecuredMessage
from rossec.security import AESEncryption


class Encryptable(object):
    """
    Initializes the security aspect of the wrappers
    """

    def __init__(self):
        self.AES = AESEncryption(bytes("12345678901234567890123456789012"))

class Publisher(Encryptable):
    """
    Security wrapper for the Publisher class
    """

    def __init__(self, *args, **kwargs):
        Encryptable.__init__(self)
        self.data_class = args[1]
        args = args[:1] + (SecuredMessage,) + args[2:]
        self.pub = rospy.Publisher(*args, **kwargs)

    def publish(self, *args, **kwargs):
        msg = SecuredMessage()
        msg.MessageType = str(self.data_class)
        msg.MessageContent = self.AES.encode(json.dumps(args[0]))
        self.pub.publish(msg)

class Subscriber(Encryptable):
    """
    Security wrapper for the Subscriber class
    """

    def __init__(self, *args, **kwargs):
        Encryptable.__init__(self)
        self.data_class = args[1]
        self.callback = args[2] if len(args) > 2 else kwargs.get('callback', None)
        self.callback_args = args[3] if len(args) > 3 else kwargs.get('callback_args', None)
        args = args[:1] + (SecuredMessage,) + args[3:]
        kwargs['callback'] = self.__callback__
        self.sub = rospy.Subscriber(*args, **kwargs)

    def __callback__(self, data):
        parsed = json.loads(self.AES.decode(data.MessageContent))
        self.callback(self.data_class(parsed))

    def unregister(self):
        self.sub.unregister()

