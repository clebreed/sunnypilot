"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.sunnypilot.selfdrive.controls.lib.longcontrol_ext import LongControlExt, SETTLE_V_BP


class FakeParams:
  def __init__(self, store=None):
    self.store = dict(store or {})

  def get_bool(self, key):
    return bool(self.store.get(key, False))


def _CP():
  tuning = SimpleNamespace(kpBP=[0.0], kpV=[1.0], kiBP=[0.0], kiV=[0.0])
  return SimpleNamespace(longitudinalTuning=tuning, stopAccel=-2.0, stoppingDecelRate=0.8,
                         startAccel=0.0, vEgoStarting=0.5, startingState=False)


def _CS(v_ego, brake=False, standstill=False):
  return SimpleNamespace(vEgo=v_ego, aEgo=0.0, brakePressed=brake,
                         cruiseState=SimpleNamespace(standstill=standstill))


CP_SP = SimpleNamespace(enableGasInterceptor=False)
LIMITS = (-3.0, 2.0)


def _stock():
  return LongControl(_CP(), CP_SP)


def _ext(enabled=True):
  return LongControlExt(_CP(), CP_SP, params=FakeParams({"StopSettleSoften": enabled}))


def _run(c, v_ego, frames):
  return [c.update(True, _CS(v_ego), 0.0, True, LIMITS) for _ in range(frames)]


def test_disabled_matches_stock():
  assert _run(_ext(enabled=False), 0.3, 30) == _run(_stock(), 0.3, 30)  # off => byte-stock


def test_low_speed_softens_brake_build():
  soft = _run(_ext(), 0.3, 30)
  stock = _run(_stock(), 0.3, 30)
  assert soft[-1] > stock[-1]                                # softer (less brake) at the final settle
  assert all(s >= b - 1e-9 for s, b in zip(soft, stock, strict=True))   # never harder than stock anywhere


def test_high_speed_unchanged():
  v = SETTLE_V_BP[-1] + 0.5                                  # above the band => full stock rate
  assert _run(_ext(), v, 20) == _run(_stock(), v, 20)


def test_never_adds_throttle_or_releases_brake():
  c = _ext()
  prev = c.last_output_accel
  for _ in range(40):
    a = c.update(True, _CS(0.3), 0.0, True, LIMITS)
    assert a <= 1e-9                                         # never turns the stop into throttle
    assert a <= prev + 1e-9                                  # only ever builds brake, never releases it
    prev = a


def test_only_acts_in_stopping_state():
  # moving, not stopping => pid state => identical to stock
  ext = _ext()
  stock = _stock()
  out_ext = [ext.update(True, _CS(15.0), -0.5, False, LIMITS) for _ in range(10)]
  out_stock = [stock.update(True, _CS(15.0), -0.5, False, LIMITS) for _ in range(10)]
  assert out_ext == out_stock
