/**
 * @file  mathx.h
 * @brief 常用浮点/角度小工具（header-only）
 */
#ifndef MATHX_H
#define MATHX_H

#include <math.h>
#include <stdint.h>

#ifndef PI_F
#define PI_F 3.14159265358979f
#endif

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/** 把角度折算到 (-180, 180] */
static inline float wrap180(float deg)
{
    while (deg > 180.0f)  deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

/** 最短角度差 a-b，结果 (-180,180] */
static inline float ang_diff(float a, float b)
{
    return wrap180(a - b);
}

static inline float signf(float x)
{
    return (x >= 0.0f) ? 1.0f : -1.0f;
}

static inline float sq(float x) { return x * x; }

/** 一阶低通：y += alpha*(x-y)，alpha=dt/(tau+dt) */
static inline float lpf(float y, float x, float alpha)
{
    return y + alpha * (x - y);
}

static inline float deg2rad(float d) { return d * PI_F / 180.0f; }
static inline float rad2deg(float r) { return r * 180.0f / PI_F; }

#endif /* MATHX_H */
