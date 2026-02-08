#pragma once

#include "../../inc/MarlinConfig.h"

#if FTM_HAS_LIN_ADVANCE
  typedef struct LinearAdvance {
    float prev_raw_traj_e = 0.0f;
    float prev_advanced_traj_e = 0.0f;
    float prev_advanced_e_rate = 0.0f;
    float prev_volatility = 0.0f;
    float prev_smoothing_traj_es[FTM_LIN_ADV_SMOOTHING_ORDER] = { 0.0f };
    float max_alpha = 1.0f - expf(-(FTM_TS) * (FTM_LIN_ADV_SMOOTHING_ORDER) / FTM_LIN_ADV_SMOOTH_TIME);
    uint32_t delay_steps = FTM_LIN_ADV_DELAY_STEPS;

    void reset();
    void offset_position(const float offset);
    void fill_smoothing_buffer(const xyze_float_t pos);
    uint32_t get_delay_steps(const char axis) const;
  } linear_advance_t;
#endif