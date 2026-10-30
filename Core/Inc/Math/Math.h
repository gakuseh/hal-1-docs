//
// Created by dyrel on 3/3/2026.
//

/**
 * @file Math.h
 * @ingroup platform
 * @brief Small math helpers. Also pulls in Vector3D.
 */

#ifndef KINGFISHER_SW_MATH_H
#define KINGFISHER_SW_MATH_H

#include <cmath>
#include <stddef.h>
#include "Vector3D.h"

/// Holder for static math helpers.
struct MATHEMATICS {
    /**
     * @brief Linearly maps @p in from [inMin, inMax] to [outMin, outMax]. Does not clamp.
     * @return The mapped value, or @p outMin if inMin equals inMax. Unused.
     */
    static float Map(float in, float inMin, float inMax, float outMin, float outMax);
};

#endif //KINGFISHER_SW_MATH_H
