//
// Created by dyrel on 4/11/2026.
//

/**
 * @file FloatTripleModRedundancy.h
 * @ingroup platform
 * @brief Two-out-of-three voting for scalar sensor readings.
 */

#ifndef KINGFISHER_SW_FLOATTRIPLEMODREDUNDANCY_H
#define KINGFISHER_SW_FLOATTRIPLEMODREDUNDANCY_H

#include <cmath>

/// Two-out-of-three voting for scalar readings. Used for the barometers.
typedef struct {
    /**
     * @brief Returns the mean of the two values that agree most closely.
     *
     * It also checks whether that pair differs by more than 5 % of the mean,
     * but nothing acts on the result yet (a TODO in the source).
     *
     * @param v1,v2,v3 The three readings.
     */
    static float Vote(float v1, float v2, float v3);
}  TMRFloat;

#endif //KINGFISHER_SW_FLOATTRIPLEMODREDUNDANCY_H
