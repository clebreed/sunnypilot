"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Eases the final brake-pressure build as the car settles to a stop. In the stopping state the output ramps
toward the hold accel at a fixed rate; below SETTLE_V_BP[-1] this scales that per-step build down so the
last fraction of a m/s tapers in instead of clamping on (approach braking is untouched). This is the one
regime where the output is intentionally softer than stock. Gated by the StopSettleSoften param (read live).
"""

import numpy as np

from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState

SETTLE_V_BP = [0.3, 1.2, 2.5]      # m/s: the build step is eased below the top point, full rate at/above it
SETTLE_SCALE_V = [0.25, 0.6, 1.0]  # fraction of the per-step brake build applied across the band


class LongControlExt(LongControl):
  def __init__(self, CP, CP_SP, params=None):
    super().__init__(CP, CP_SP)
    self._params = params or Params()
    self._frame = 0
    self._settle_soft = self._params.get_bool("StopSettleSoften")

  def update(self, active, CS, a_target, should_stop, accel_limits):
    if self._frame % int(1.0 / DT_CTRL) == 0:
      self._settle_soft = self._params.get_bool("StopSettleSoften")
    self._frame += 1

    prev_accel = self.last_output_accel
    accel = super().update(active, CS, a_target, should_stop, accel_limits)
    # Soften only while the stop ramp is adding brake (accel going more negative) from an already-coasting
    # output, so we shrink the build step without ever turning it into throttle or reducing held brake.
    if self._settle_soft and self.long_control_state == LongCtrlState.stopping and prev_accel <= 0.0 and accel < prev_accel:
      scale = float(np.interp(CS.vEgo, SETTLE_V_BP, SETTLE_SCALE_V))
      accel = float(np.clip(prev_accel + (accel - prev_accel) * scale, accel_limits[0], accel_limits[1]))
      self.last_output_accel = accel
    return self.last_output_accel
