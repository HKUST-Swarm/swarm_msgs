/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#ifndef __ImageDescriptorHeader_t_hpp__
#define __ImageDescriptorHeader_t_hpp__

#include <lcm/lcm_coretypes.h>

#include <vector>
#include "Time_t.hpp"
#include "Pose_t.hpp"
#include "Pose_t.hpp"


class ImageDescriptorHeader_t
{
    public:
        Time_t     timestamp;

        int32_t    drone_id;

        int64_t    matched_frame;

        int32_t    matched_drone;

        int8_t     is_lazy_frame;

        int32_t    image_desc_size;

        std::vector< float > image_desc;

        int32_t    image_desc_size_int8;

        std::vector< int8_t > image_desc_int8;

        Pose_t     pose_drone;

        Pose_t     camera_extrinsic;

        int8_t     prevent_adding_db;

        int64_t    msg_id;

        int64_t    frame_id;

        int32_t    feature_num;

        int32_t    camera_index;

    public:
        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void *buf, int offset, int maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline int getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to read while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void *buf, int offset, int maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "ImageDescriptorHeader_t"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int ImageDescriptorHeader_t::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int ImageDescriptorHeader_t::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int ImageDescriptorHeader_t::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t ImageDescriptorHeader_t::getHash()
{
    static int64_t hash = static_cast<int64_t>(_computeHash(NULL));
    return hash;
}

const char* ImageDescriptorHeader_t::getTypeName()
{
    return "ImageDescriptorHeader_t";
}

int ImageDescriptorHeader_t::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = this->timestamp._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->drone_id, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->matched_frame, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->matched_drone, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_encode_array(buf, offset + pos, maxlen - pos, &this->is_lazy_frame, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->image_desc_size, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->image_desc_size > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->image_desc[0], this->image_desc_size);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->image_desc_size_int8, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->image_desc_size_int8 > 0) {
        tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->image_desc_int8[0], this->image_desc_size_int8);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = this->pose_drone._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->camera_extrinsic._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_encode_array(buf, offset + pos, maxlen - pos, &this->prevent_adding_db, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->msg_id, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->frame_id, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->feature_num, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->camera_index, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int ImageDescriptorHeader_t::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = this->timestamp._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->drone_id, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->matched_frame, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->matched_drone, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_decode_array(buf, offset + pos, maxlen - pos, &this->is_lazy_frame, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->image_desc_size, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->image_desc_size) {
        this->image_desc.resize(this->image_desc_size);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->image_desc[0], this->image_desc_size);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->image_desc_size_int8, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->image_desc_size_int8) {
        this->image_desc_int8.resize(this->image_desc_size_int8);
        tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->image_desc_int8[0], this->image_desc_size_int8);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = this->pose_drone._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->camera_extrinsic._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_decode_array(buf, offset + pos, maxlen - pos, &this->prevent_adding_db, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->msg_id, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->frame_id, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->feature_num, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->camera_index, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int ImageDescriptorHeader_t::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += this->timestamp._getEncodedSizeNoHash();
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __boolean_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __float_encoded_array_size(NULL, this->image_desc_size);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int8_t_encoded_array_size(NULL, this->image_desc_size_int8);
    enc_size += this->pose_drone._getEncodedSizeNoHash();
    enc_size += this->camera_extrinsic._getEncodedSizeNoHash();
    enc_size += __boolean_encoded_array_size(NULL, 1);
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    return enc_size;
}

uint64_t ImageDescriptorHeader_t::_computeHash(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == ImageDescriptorHeader_t::getHash)
            return 0;
    const __lcm_hash_ptr cp = { p, ImageDescriptorHeader_t::getHash };

    uint64_t hash = 0x4208209742869302LL +
         Time_t::_computeHash(&cp) +
         Pose_t::_computeHash(&cp) +
         Pose_t::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

#endif
