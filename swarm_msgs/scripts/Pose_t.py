"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class Pose_t(object):
    __slots__ = ["position", "orientation"]

    __typenames__ = ["float", "float"]

    __dimensions__ = [[3], [4]]

    def __init__(self):
        self.position = [ 0.0 for dim0 in range(3) ]
        self.orientation = [ 0.0 for dim0 in range(4) ]

    def encode(self):
        buf = BytesIO()
        buf.write(Pose_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack('>3f', *self.position[:3]))
        buf.write(struct.pack('>4f', *self.orientation[:4]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != Pose_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return Pose_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = Pose_t()
        self.position = struct.unpack('>3f', buf.read(12))
        self.orientation = struct.unpack('>4f', buf.read(16))
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if Pose_t in parents: return 0
        tmphash = (0xaff8430a89bc6633) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if Pose_t._packed_fingerprint is None:
            Pose_t._packed_fingerprint = struct.pack(">Q", Pose_t._get_hash_recursive([]))
        return Pose_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", Pose_t._get_packed_fingerprint())[0]

