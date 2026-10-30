//
// Created by dyrel on 3/5/2026.
//

/**
 * @file Vector3TripleModRedundancy.h
 * @ingroup platform
 * @brief Two-out-of-three voting for 3-axis sensor readings.
 */

#ifndef KINGFISHER_SW_TRIPLEMODREDUNDANCY_H
#define KINGFISHER_SW_TRIPLEMODREDUNDANCY_H

#include "Vector3D.h"

/// Two-out-of-three voting for 3-axis readings. Used for the IMUs.
typedef struct {
    /**
     * @brief Returns the mean of the two vectors closest to each other (Euclidean distance).
     * @param v1,v2,v3 The three readings.
     */
    static Vector3D<float> Vote(const Vector3D<float>& v1,
                            const Vector3D<float>& v2,
                            const Vector3D<float>& v3);
private:
    /// @brief Euclidean distance between @p a and @p b.
    static float distance(const Vector3D<float>& a, const Vector3D<float>& b);
    /// @brief Component-wise mean of @p a and @p b.
    static Vector3D<float> average(const Vector3D<float>& a, const Vector3D<float>& b);
} TMR;

#endif //KINGFISHER_SW_TRIPLEMODREDUNDANCY_H
