#!/usr/bin/env python
from rossec.security import AES_CBC


class SecurityWrapper(object):
    """
    Initializes the security aspect of the wrappers
    """

    def __init__(self, name, encryption=AES_CBC()):
        self.encryption = encryption
        self.__load_key__(name)

    def __load_key__(self, name):
        self.encryption.load_key(bytes("12345678901234567890123456789012"))

    def encode(self, contents):
        return self.encryption.encode(contents)

    def decode(self, contents):
        return self.encryption.decode(contents)

