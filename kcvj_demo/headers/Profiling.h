#ifndef KCVJ_PROFILE_H
#define KCVJ_PROFILE_H


#ifdef WIN32
#include <Windows.h>
#include <cstdint>
typedef uint64_t uint64;
#else
#include <inttypes.h>
#include <sys/time.h>
#include <ctime>
#endif

int64_t GetTimeMs64();

#endif
