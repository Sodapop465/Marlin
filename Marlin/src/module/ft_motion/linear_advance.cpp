#include "linear_advance.h"

void LinearAdvance::reset() {
  prev_raw_traj_e = 0.0f;
  prev_advanced_traj_e = 0.0f;
  prev_advanced_e_rate = 0.0f;
  prev_volatility = 0.0f;
  for (uint8_t i = 0; i < FTM_LIN_ADV_SMOOTHING_ORDER; i++) {
    prev_smoothing_traj_es[i] = 0.0f;
  }
}
void LinearAdvance::offset_position(const float offset) {
  prev_raw_traj_e += offset;
  prev_advanced_traj_e += offset;
  for (uint8_t i = 0; i < FTM_LIN_ADV_SMOOTHING_ORDER; i++) {
    prev_smoothing_traj_es[i] += offset;
  }
}

void LinearAdvance::fill_smoothing_buffer(const xyze_float_t pos) {
  for (uint8_t i = 0; i < FTM_LIN_ADV_SMOOTHING_ORDER; i++) {
    prev_smoothing_traj_es[i] = pos.e;
  }
}

uint32_t LinearAdvance::get_delay_steps(const char axis) const {
  if (axis == 'E' || axis == 'e') {
    return delay_steps;
  }
  return 0;
}