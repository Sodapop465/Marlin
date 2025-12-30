#include "linear_advance.h"

void LinearAdvance::reset() {
    prev_advanced_traj_e = 0.0f;
    prev_advanced_e_rate = 0.0f;
    prev_volatility = 0.0f;
    for (uint8_t i = 0; i < FTM_SMOOTHING_ORDER; i++) {
        prev_smoothing_traj_es[i] = 0.0f;
    }
    for (uint32_t i = 0; i < lin_adv_lookahead_steps; i++) {
        traj_queue[i] = { 0.0f };
    }
}

void LinearAdvance::offset_position(const float offset) {
    prev_advanced_traj_e += offset;
    for (uint8_t i = 0; i < FTM_SMOOTHING_ORDER; i++) {
        prev_smoothing_traj_es[i] += offset;
    }
    for (uint32_t i = 0; i < lin_adv_lookahead_steps; i++) {
        traj_queue[i].e += offset;
    }
}