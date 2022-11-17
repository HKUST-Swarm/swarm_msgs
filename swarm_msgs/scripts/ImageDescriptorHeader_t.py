"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import Time_t

import Pose_t

import SlidingWindow_t

class ImageDescriptorHeader_t(object):
    __slots__ = ["timestamp", "drone_id", "reference_frame_id", "matched_frame", "matched_drone", "is_lazy_frame", "image_desc_size", "image_desc", "image_desc_size_int8", "image_desc_int8", "pose_drone", "camera_extrinsic", "prevent_adding_db", "msg_id", "frame_id", "feature_num", "camera_index", "camera_id", "cur_td", "sld_win_status"]

    __typenames__ = ["Time_t", "int32_t", "int32_t", "int64_t", "int32_t", "boolean", "int32_t", "float", "int32_t", "int8_t", "Pose_t", "Pose_t", "boolean", "int64_t", "int64_t", "int32_t", "int32_t", "int32_t", "float", "SlidingWindow_t"]

    __dimensions__ = [None, None, None, None, None, None, None, ["image_desc_size"], None, ["image_desc_size_int8"], None, None, None, None, None, None, None, None, None, None]

    def __init__(self):
        self.timestamp = Time_t()
        self.drone_id = 0
        self.reference_frame_id = 0
        self.matched_frame = 0
        self.matched_drone = 0
        self.is_lazy_frame = False
        self.image_desc_size = 0
        self.image_desc = []
        self.image_desc_size_int8 = 0
        self.image_desc_int8 = []
        self.pose_drone = Pose_t()
        self.camera_extrinsic = Pose_t()
        self.prevent_adding_db = False
        self.msg_id = 0
        self.frame_id = 0
        self.feature_num = 0
        self.camera_index = 0
        self.camera_id = 0
        self.cur_td = 0.0
        self.sld_win_status = SlidingWindow_t()

    def encode(self):
        buf = BytesIO()
        buf.write(ImageDescriptorHeader_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        assert self.timestamp._get_packed_fingerprint() == Time_t._get_packed_fingerprint()
        self.timestamp._encode_one(buf)
        buf.write(struct.pack(">iiqibi", self.drone_id, self.reference_frame_id, self.matched_frame, self.matched_drone, self.is_lazy_frame, self.image_desc_size))
        buf.write(struct.pack('>%df' % self.image_desc_size, *self.image_desc[:self.image_desc_size]))
        buf.write(struct.pack(">i", self.image_desc_size_int8))
        buf.write(struct.pack('>%db' % self.image_desc_size_int8, *self.image_desc_int8[:self.image_desc_size_int8]))
        assert self.pose_drone._get_packed_fingerprint() == Pose_t._get_packed_fingerprint()
        self.pose_drone._encode_one(buf)
        assert self.camera_extrinsic._get_packed_fingerprint() == Pose_t._get_packed_fingerprint()
        self.camera_extrinsic._encode_one(buf)
        buf.write(struct.pack(">bqqiiif", self.prevent_adding_db, self.msg_id, self.frame_id, self.feature_num, self.camera_index, self.camera_id, self.cur_td))
        assert self.sld_win_status._get_packed_fingerprint() == SlidingWindow_t._get_packed_fingerprint()
        self.sld_win_status._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != ImageDescriptorHeader_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return ImageDescriptorHeader_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = ImageDescriptorHeader_t()
        self.timestamp = Time_t._decode_one(buf)
        self.drone_id, self.reference_frame_id, self.matched_frame, self.matched_drone = struct.unpack(">iiqi", buf.read(20))
        self.is_lazy_frame = bool(struct.unpack('b', buf.read(1))[0])
        self.image_desc_size = struct.unpack(">i", buf.read(4))[0]
        self.image_desc = struct.unpack('>%df' % self.image_desc_size, buf.read(self.image_desc_size * 4))
        self.image_desc_size_int8 = struct.unpack(">i", buf.read(4))[0]
        self.image_desc_int8 = struct.unpack('>%db' % self.image_desc_size_int8, buf.read(self.image_desc_size_int8))
        self.pose_drone = Pose_t._decode_one(buf)
        self.camera_extrinsic = Pose_t._decode_one(buf)
        self.prevent_adding_db = bool(struct.unpack('b', buf.read(1))[0])
        self.msg_id, self.frame_id, self.feature_num, self.camera_index, self.camera_id, self.cur_td = struct.unpack(">qqiiif", buf.read(32))
        self.sld_win_status = SlidingWindow_t._decode_one(buf)
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if ImageDescriptorHeader_t in parents: return 0
        newparents = parents + [ImageDescriptorHeader_t]
        tmphash = (0xa18d8e8b8bc63894+ Time_t._get_hash_recursive(newparents)+ Pose_t._get_hash_recursive(newparents)+ Pose_t._get_hash_recursive(newparents)+ SlidingWindow_t._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if ImageDescriptorHeader_t._packed_fingerprint is None:
            ImageDescriptorHeader_t._packed_fingerprint = struct.pack(">Q", ImageDescriptorHeader_t._get_hash_recursive([]))
        return ImageDescriptorHeader_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", ImageDescriptorHeader_t._get_packed_fingerprint())[0]

