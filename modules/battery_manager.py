"""
battery_manager.py -- Mission admission control and charging policy.

Implements the six operating rules as a single decision point, so mission
acceptance cannot drift out of step with charging behaviour.

  R1  battery >= ACCEPT_PCT              -> accept new missions
  R2  battery <  ACCEPT_PCT, idle        -> reject, return to dock
  R3  battery drops below mid-mission    -> finish CURRENT delivery only,
                                            then dock; accept nothing further
  R4  emergency missions override the threshold; dock immediately after
  R5  charge automatically once docked; expose %, status, time remaining
  R6  stop charging at CHARGE_STOP_PCT (95), never 100, then READY

Charging stops short of full deliberately: Li-ion cycled to 100 % ages faster,
and the 18650 pack has no balancing circuitry.
"""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from typing import Optional


class BatteryVerdict(Enum):
    ACCEPT = "accept"
    REJECT_LOW = "reject_low"
    FINISH_THEN_DOCK = "finish_then_dock"
    ACCEPT_EMERGENCY = "accept_emergency"


@dataclass
class ChargeStatus:
    charging: bool = False
    percent: float = 0.0
    target: float = 95.0
    eta_seconds: Optional[float] = None
    complete: bool = False


class BatteryManager:
    ACCEPT_PCT = 30.0
    CHARGE_STOP_PCT = 95.0
    CRITICAL_PCT = 12.0

    def __init__(self, charge_rate_pct_per_s: float = 0.35) -> None:
        self.charge_rate = charge_rate_pct_per_s
        self.cycles = 0
        self._was_charging = False
        self.lockout = False          # set by R3/R4: no further missions

    # ── admission control ──────────────────────────────────────
    def evaluate(self, battery_pct: float, *, mission_active: bool,
                 emergency: bool = False) -> BatteryVerdict:
        if emergency:
            self.lockout = True                       # R4: dock straight after
            return BatteryVerdict.ACCEPT_EMERGENCY
        if battery_pct >= self.ACCEPT_PCT and not self.lockout:
            return BatteryVerdict.ACCEPT               # R1
        if mission_active:
            self.lockout = True                        # R3
            return BatteryVerdict.FINISH_THEN_DOCK
        return BatteryVerdict.REJECT_LOW               # R2

    def may_accept(self, battery_pct: float) -> bool:
        return battery_pct >= self.ACCEPT_PCT and not self.lockout

    # ── charging ───────────────────────────────────────────────
    def charge_step(self, battery_pct: float, dt: float) -> tuple:
        """Advance charging. Returns (new_pct, ChargeStatus)."""
        if battery_pct >= self.CHARGE_STOP_PCT:        # R6
            if self._was_charging:
                self.cycles += 1
                self._was_charging = False
            self.lockout = False                       # ready again
            return battery_pct, ChargeStatus(
                charging=False, percent=battery_pct,
                target=self.CHARGE_STOP_PCT, eta_seconds=0.0, complete=True)

        self._was_charging = True
        new = min(self.CHARGE_STOP_PCT, battery_pct + self.charge_rate * dt)
        remaining = max(0.0, self.CHARGE_STOP_PCT - new)
        eta = remaining / self.charge_rate if self.charge_rate > 0 else None
        return new, ChargeStatus(charging=True, percent=new,
                                 target=self.CHARGE_STOP_PCT,
                                 eta_seconds=eta, complete=False)
