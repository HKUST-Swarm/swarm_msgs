"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import Point3d_t

import Time_t

class IMUData_t(object):
    __slots__ = ["timestamp", "dt", "acc", "gyro"]

    __typenames__ = ["Time_t", "double", "Point3d_t", "Point3d_t"]

    __dimensions__ = [None, None, None, None]

    def __init__(self):
        self.timestamp = Time_t()
        self.dt = 0.0
        self.acc = Point3d_t()
        self.gyro = Point3d_t()

    def encode(self):
        buf = BytesIO()
        buf.write(IMUData_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        assert self.timestamp._get_packed_fingerprint() == Time_t._get_packed_fingerprint()
        self.timestamp._encode_one(buf)
        buf.write(struct.pack(">d", self.dt))
        assert self.acc._get_packed_fingerprint() == Point3d_t._get_packed_fingerprint()
        self.acc._encode_one(buf)
        assert self.gyro._get_packed_fingerprint() == Point3d_t._get_packed_fingerprint()
        self.gyro._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != IMUData_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return IMUData_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = IMUData_t()
        self.timestamp = Time_t._decode_one(buf)
        self.dt = struct.unpack(">d", buf.read(8))[0]
        self.acc = Point3d_t._decode_one(buf)
        self.gyro = Point3d_t._decode_one(buf)
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if IMUData_t in parents: return 0
        newparents = parents + [IMUData_t]
        tmphash = (0x22f066428d5d11f6+ Time_t._get_hash_recursive(newparents)+ Point3d_t._get_hash_recursive(newparents)+ Point3d_t._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if IMUData_t._packed_fingerprint is None:
            IMUData_t._packed_fingerprint = struct.pack(">Q", IMUData_t._get_hash_recursive([]))
        return IMUData_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

