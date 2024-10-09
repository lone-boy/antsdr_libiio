//
// Created by jcc on 23-4-15.
//

#ifndef SDRDREAM_COMMON_H
#define SDRDREAM_COMMON_H

#include <vector>
#include <cstring>
#include <complex>
#define NCARRIERS 63


typedef float    F32;
typedef double   F64;
typedef int8_t   I8;
typedef int16_t  I16;
typedef int32_t  I32;
typedef int64_t  I64;
typedef uint8_t  U8;
typedef uint16_t U16;
typedef uint32_t U32;
typedef uint64_t U64;
typedef bool     BOOL;

typedef std::complex<F32> CF32;
typedef std::complex<F64> CF64;
typedef std::complex<I8>  CI8;
typedef std::complex<I16> CI16;
typedef std::complex<I32> CI32;
typedef std::complex<I64> CI64;
typedef std::complex<U8>  CU8;
typedef std::complex<U16> CU16;
typedef std::complex<U32> CU32;
typedef std::complex<U64> CU64;

enum class Result : uint8_t {
    SUCCESS = 0,
    ERROR = 1,
    SKIP = 2,
    ERROR_TIMEOUT,
    ERROR_BEYOND_CAPACITY,
};

enum class Direction : int8_t {
    Forward = 1,
    Backward = -1,
};

typedef struct
{
    void *user;
    int16_t *data;
    int channels;
    int length;
} sdr_transfer;


#endif //SDRDREAM_COMMON_H
