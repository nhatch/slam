
#pragma once

#include "utils.h"

transform_t do_icp(const points_t &cloud1, const points_t &cloud2);

// Returns the transform that takes cloud1 -> cloud2 (ie tf * p1 = p2)
// Assumes points have already been associated (i.e. clouds are ordered)
transform_t associated_closed_form(const points_t &cloud1, const points_t &cloud2);
