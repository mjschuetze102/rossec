#!/usr/bin/env python
from rossec.security import AESEncryption


class SecurityWrapper(object):
    """
    Initializes the security aspect of the wrappers
    """

    def __init__(self):
        self.AES = self.__load_key__(self.name)

    def __load_key__(self, name):
        return AESEncryption(bytes("12345678901234567890123456789012"))

    def encode(self, contents):
        return self.AES.encode(contents)

    def decode(self, contents):
        return self.AES.decode(contents)
    
