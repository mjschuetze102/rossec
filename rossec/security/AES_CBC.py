from base64 import b64encode, b64decode
from Crypto.Cipher import AES
from Crypto.Random import get_random_bytes
from Crypto.Util.Padding import pad, unpad


class AES_CBC:

    __BLOCK_LENGTH = 128
    __KEY_LENGTH = 256

    def __init__(self):
        # Only need to generate secret key once
        self.__sec_key = get_random_bytes(AESEncryption.__KEY_LENGTH // 8)  # Division converts bits to bytes

    def load_key(self, key):
        self.__sec_key = key

    def encode(self, plaintext):
        """
        Encrypts an plaintext message
        :param plaintext: message to be encrypted
        :return: 16 byte initialization vector concatenated to the front of the cipher text
        """
        # Need to generate initialization vector for every encoding
        iv = get_random_bytes(AESEncryption.__BLOCK_LENGTH // 8)  # Division converts bits to bytes

        # AES - Advanced Encryption Standard
        # CBC - Cipher Block Chaining
        # PKCS7 - Padding Standard to pad data up to a 255 byte (2040 bit) block
        #         1 byte missing: 0x01 is added
        #         2 byte missing: 0x0202 is added
        #         ...
        #         8 byte missing: 0x0808080808080808 is added
        cipher = AES.new(self.__sec_key, AES.MODE_CBC, iv)
        encoded = cipher.encrypt(pad(plaintext.encode('utf-8'), 16, 'pkcs7'))

        # Attach the initialization vector to the encoding so the decoding process has access to it
        return b64encode(iv + encoded)

    def decode(self, iv_cipher):
        """
        Decrypts an encoded message
        :param iv_cipher: 16 byte initialization vector concatenated to the front of the cipher text
        :return: String containing the original encrypted message
        """
        iv_cipher = b64decode(iv_cipher)

        # Remove initialization vector from the front of the cipher text
        iv = iv_cipher[:AESEncryption.__BLOCK_LENGTH // 8]  # Division converts bits to bytes
        ciphertext = iv_cipher[AESEncryption.__BLOCK_LENGTH // 8:]

        # AES - Advanced Encryption Standard
        # CBC - Cipher Block Chaining
        # PKCS7 - Padding Standard to pad data up to a 255 byte (2040 bit) block
        #         1 byte missing: 0x01 is added
        #         2 byte missing: 0x0202 is added
        #         ...
        #         8 byte missing: 0x0808080808080808 is added
        cipher = AES.new(self.__sec_key, AES.MODE_CBC, iv)
        return unpad(cipher.decrypt(ciphertext), 16, 'pkcs7').decode()

