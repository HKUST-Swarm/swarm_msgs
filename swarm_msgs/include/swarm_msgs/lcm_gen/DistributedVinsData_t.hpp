/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#ifndef __DistributedVinsData_t_hpp__
#define __DistributedVinsData_t_hpp__

#include <lcm/lcm_coretypes.h>

#include <vector>
#include "Time_t.hpp"
#include "Pose_t.hpp"
#include "Pose_t.hpp"


class DistributedVinsData_t
{
    public:
        Time_t     timestamp;

        int32_t    drone_id;

        int32_t    sld_win_len;

        int32_t    reference_frame_id;

        std::vector< int64_t > frame_ids;

        std::vector< Pose_t > frame_poses;

        int32_t    camera_num;

        std::vector< int64_t > cam_ids;

        std::vector< Pose_t > extrinsic;

        int64_t    solver_token;

        int32_t    iteration_count;

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
         * Returns "DistributedVinsData_t"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int DistributedVinsData_t::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int DistributedVinsData_t::decode(const void *buf, int offset, int maxlen)
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

int DistributedVinsData_t::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t DistributedVinsData_t::getHash()
{
    static int64_t hash = static_cast<int64_t>(_computeHash(NULL));
    return hash;
}

const char* DistributedVinsData_t::getTypeName()
{
    return "DistributedVinsData_t";
}

int DistributedVinsData_t::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = this->timestamp._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->drone_id, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->sld_win_len, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->reference_frame_id, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->sld_win_len > 0) {
        tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->frame_ids[0], this->sld_win_len);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    for (int a0 = 0; a0 < this->sld_win_len; a0++) {
        tlen = this->frame_poses[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->camera_num, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->camera_num > 0) {
        tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->cam_ids[0], this->camera_num);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    for (int a0 = 0; a0 < this->camera_num; a0++) {
        tlen = this->extrinsic[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->solver_token, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->iteration_count, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int DistributedVinsData_t::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = this->timestamp._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->drone_id, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->sld_win_len, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->reference_frame_id, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->sld_win_len) {
        this->frame_ids.resize(this->sld_win_len);
        tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->frame_ids[0], this->sld_win_len);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    try {
        this->frame_poses.resize(this->sld_win_len);
    } catch (...) {
        return -1;
    }
    for (int a0 = 0; a0 < this->sld_win_len; a0++) {
        tlen = this->frame_poses[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->camera_num, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->camera_num) {
        this->cam_ids.resize(this->camera_num);
        tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->cam_ids[0], this->camera_num);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    try {
        this->extrinsic.resize(this->camera_num);
    } catch (...) {
        return -1;
    }
    for (int a0 = 0; a0 < this->camera_num; a0++) {
        tlen = this->extrinsic[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->solver_token, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->iteration_count, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int DistributedVinsData_t::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += this->timestamp._getEncodedSizeNoHash();
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int64_t_encoded_array_size(NULL, this->sld_win_len);
    for (int a0 = 0; a0 < this->sld_win_len; a0++) {
        enc_size += this->frame_poses[a0]._getEncodedSizeNoHash();
    }
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int64_t_encoded_array_size(NULL, this->camera_num);
    for (int a0 = 0; a0 < this->camera_num; a0++) {
        enc_size += this->extrinsic[a0]._getEncodedSizeNoHash();
    }
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    return enc_size;
}

uint64_t DistributedVinsData_t::_computeHash(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == DistributedVinsData_t::getHash)
            return 0;
    const __lcm_hash_ptr cp = { p, DistributedVinsData_t::getHash };

    uint64_t hash = 0xd1d5ab5fbb01f070LL +
         Time_t::_computeHash(&cp) +
         Pose_t::_computeHash(&cp) +
         Pose_t::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

#endif
