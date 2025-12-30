#pragma once

#include "../../inc/MarlinConfig.h"

#if HAS_EXTRUDERS
  typedef struct LinearAdvance {
    float prev_advanced_traj_e = 0.0f;
    float prev_advanced_e_rate = 0.0f;
    float prev_volatility = 0.0f;
    float prev_smoothing_traj_es[FTM_SMOOTHING_ORDER];
    float max_alpha = ADVANCE_K;
    uint32_t lin_adv_lookahead_steps = FTM_LIN_ADV_LOOKAHEAD_STEPS;
    xyze_float_t traj_queue[FTM_LIN_ADV_LOOKAHEAD_STEPS];

    void reset();
    void offset_position(const float offset);
  } linear_advance_t;
#endif