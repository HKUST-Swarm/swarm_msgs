#pragma once
#include <stdint.h>
namespace Swarm {
typedef int64_t TsType;
typedef int64_t FrameIdType;

inline int TSShort(TsType ts) {
    return (ts/1000000)%10000000;
}

inline TsType TSLong(TsType ts) {
    return (ts/1000000)%10000000000;
}
}