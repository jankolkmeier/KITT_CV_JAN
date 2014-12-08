#ifndef KCVJ_PROFILE_H
#define KCVJ_PROFILE_H

#include <inttypes.h>

#ifdef WIN32
#include <Windows.h>
#else
#include <sys/time.h>
#include <ctime>
#endif

int64_t GetTimeMs64();

#endif
