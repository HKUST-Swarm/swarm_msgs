/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#ifndef __HGrid_t_hpp__
#define __HGrid_t_hpp__

#include <lcm/lcm_coretypes.h>

#include <vector>


class HGrid_t
{
    public:
        double     stamp;

        int32_t    pt_num;

        std::vector< float > points1_x;

        std::vector< float > points1_y;

        std::vector< float > points1_z;

        std::vector< float > points2_x;

        std::vector< float > points2_y;

        std::vector< float > points2_z;

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
         * Returns "HGrid_t"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int HGrid_t::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int HGrid_t::decode(const void *buf, int offset, int maxlen)
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

int HGrid_t::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t HGrid_t::getHash()
{
    static int64_t hash = static_cast<int64_t>(_computeHash(NULL));
    return hash;
}

const char* HGrid_t::getTypeName()
{
    return "HGrid_t";
}

int HGrid_t::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->stamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->pt_num, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->pt_num > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->points1_x[0], this->pt_num);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->pt_num > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->points1_y[0], this->pt_num);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->pt_num > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->points1_z[0], this->pt_num);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->pt_num > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->points2_x[0], this->pt_num);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->pt_num > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->points2_y[0], this->pt_num);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->pt_num > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->points2_z[0], this->pt_num);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int HGrid_t::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->stamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->pt_num, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->pt_num) {
        this->points1_x.resize(this->pt_num);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->points1_x[0], this->pt_num);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->pt_num) {
        this->points1_y.resize(this->pt_num);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->points1_y[0], this->pt_num);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->pt_num) {
        this->points1_z.resize(this->pt_num);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->points1_z[0], this->pt_num);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->pt_num) {
        this->points2_x.resize(this->pt_num);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->points2_x[0], this->pt_num);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->pt_num) {
        this->points2_y.resize(this->pt_num);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->points2_y[0], this->pt_num);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->pt_num) {
        this->points2_z.resize(this->pt_num);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->points2_z[0], this->pt_num);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int HGrid_t::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __float_encoded_array_size(NULL, this->pt_num);
    enc_size += __float_encoded_array_size(NULL, this->pt_num);
    enc_size += __float_encoded_array_size(NULL, this->pt_num);
    enc_size += __float_encoded_array_size(NULL, this->pt_num);
    enc_size += __float_encoded_array_size(NULL, this->pt_num);
    enc_size += __float_encoded_array_size(NULL, this->pt_num);
    return enc_size;
}

uint64_t HGrid_t::_computeHash(const __lcm_hash_ptr *)
{
    uint64_t hash = 0x341b555e18d22101LL;
    return (hash<<1) + ((hash>>63)&1);
}

#endif
