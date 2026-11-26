"""Live one-step analyzer for the ball-beam outer loop.

Serial line at 230400 baud:
  theta_rad, theta_ref_rad, voltage_V, ball_pos_m, ball_ref_m
  (or ball_pos_m, ball_ref_m, voltage_V)
A leading millis column is optional. Without it, each sample is stamped
with the host clock.
Skips the first reference change, waits until the ball position is steady
at a reference level (0.10 m or 0.25 m by default), then records the next
step (0.10 -> 0.25 m or 0.25 -> 0.10 m). Markers are the samples behind
e_ss, target-band t_s, overshoot, and the +/-6 V check.

    python step_response_analyzer.py
    python step_response_analyzer.py --file logs/full_YYYYMMDD_HHMMSS.csv
    python step_response_analyzer.py --self-test
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import queue
import sys
import textwrap
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime

# Lab 3(e) / Demo IRL outer-loop defaults. Override from the command line.
LAB_TS_MAX_S = 8.5
LAB_OS_MAX_PCT = 45.0
LAB_V_POS_V = 6 - 0.08  # fail when logged voltage is above this
LAB_V_NEG_V = 6 + 0.46  # fail when logged voltage is below the negative of this
LAB_ESS_TOL_A_M = 0.006 # at 0.10 m, ball must remain in [0.094, 0.106] m
LAB_ESS_TOL_B_M = 0.010 # at 0.25 m, ball must remain in [0.240, 0.260] m
LAB_REF_A_M = 0.10
LAB_REF_B_M = 0.25

# Ball-position tail & steady-state thresholds (outer loop: 20 s period, 10 s dwell).
TAIL_S = 0.50
SS_STD_MAX_M = 0.003         # m; max tail std to call steady state
SS_SLOPE_MAX_MPS = 0.015     # m/s; max tail slope to call steady state
SS_DRIFT_MAX_M = 0.003       # m; max window-to-window drift
AT_TARGET_M = 0.05           # tail mean this close to 0.10 m counts as arrived
SETTLE_MEDIAN_N = 5          # 10 ms at 2 ms loop; drops single-sample ADC spikes
PREVIEW_S = 25.0             # waiting preview window (covers > 1 full 20 s period)
PRE_STEP_S = 0.50            # pre-step baseline included before t=0 on step plots


@dataclass
class Spec:
    ts_max: float = LAB_TS_MAX_S
    os_max_pct: float = LAB_OS_MAX_PCT
    v_pos: float = LAB_V_POS_V
    v_neg: float = LAB_V_NEG_V
    ess_tol_a: float = LAB_ESS_TOL_A_M
    ess_tol_b: float = LAB_ESS_TOL_B_M
    tail_s: float = TAIL_S
    ss_std_max: float = SS_STD_MAX_M
    ss_slope_max: float = SS_SLOPE_MAX_MPS
    ss_drift_max: float = SS_DRIFT_MAX_M
    at_target_m: float = AT_TARGET_M
    settle_median_n: int = SETTLE_MEDIAN_N
    min_move_frac: float = 0.10
    min_move_m: float = 0.005
    still_time_s: float = 1.50
    step_delta: float = 0.02
    ref_a: float = LAB_REF_A_M
    ref_b: float = LAB_REF_B_M
    ref_tol: float = 0.02
    step_dir: str = "both"  # "both", "pos" (ref_a->ref_b), or "neg" (ref_b->ref_a)
    runaway_abs_m: float = 0.60
    runaway_err_m: float = 0.30
    growth_ratio: float = 1.15
    growth_floor_m: float = 0.015
    bucket_s: float = 0.50


@dataclass
class SteadyState:
    confident: bool
    moved: bool
    reason: str
    y_ss: float | None = None
    std: float | None = None
    slope: float | None = None
    t0: float | None = None
    t1: float | None = None


@dataclass
class Metrics:
    elapsed: float
    n_post: int
    dt_ms: float | None
    y0: float
    r0: float
    r1: float
    lab_step: bool
    ss: SteadyState
    e_ss: float | None
    band_half: float | None
    ts: float | None
    ts_y: float | None
    outside_now: bool
    ts_note: str
    overshoot_pct: float | None
    peak_t: float | None
    peak_y: float | None
    os_note: str
    v_min: float | None
    v_max: float | None
    v_min_t: float | None
    v_max_t: float | None
    viol_t: list = field(default_factory=list)
    viol_v: list = field(default_factory=list)
    stability: str = "INCONCLUSIVE"
    stability_reason: str = ""
    growth_marks: list = field(default_factory=list)
    pass_stable: bool | None = None
    pass_ess: bool | None = None
    pass_ts: bool | None = None
    pass_os: bool | None = None
    pass_v: bool | None = None

    @property
    def ss_confident(self) -> bool:
        return self.ss.confident

    @property
    def moved(self) -> bool:
        return self.ss.moved

    @property
    def y_ss(self) -> float | None:
        return self.ss.y_ss


# ---------------------------------------------------------------------------
# Numerics
# ---------------------------------------------------------------------------

def _mean(xs: list[float]) -> float:
    return sum(xs) / len(xs)


def _median(xs: list[float]) -> float:
    ys = sorted(xs)
    n = len(ys)
    mid = n // 2
    if n % 2:
        return ys[mid]
    return 0.5 * (ys[mid - 1] + ys[mid])


def _stdev(xs: list[float]) -> float:
    n = len(xs)
    if n < 2:
        return 0.0
    m = _mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (n - 1))


def _slope(t: list[float], y: list[float]) -> float:
    n = len(t)
    if n < 2:
        return 0.0
    mt = _mean(t)
    my = _mean(y)
    var_t = sum((ti - mt) ** 2 for ti in t)
    if var_t < 1e-18:
        return 0.0
    cov = sum((ti - mt) * (yi - my) for ti, yi in zip(t, y))
    return cov / var_t


def _window(t: list[float], y: list[float], t0: float, t1: float, right_open: bool):
    ts: list[float] = []
    ys: list[float] = []
    for ti, yi in zip(t, y):
        if ti < t0:
            continue
        if right_open:
            if ti >= t1:
                continue
        elif ti > t1:
            continue
        ts.append(ti)
        ys.append(yi)
    return ts, ys


def _median_dt_ms(t: list[float]) -> float | None:
    dts = [(t[i + 1] - t[i]) * 1000.0 for i in range(len(t) - 1) if t[i + 1] > t[i]]
    if not dts:
        return None
    return _median(dts)


def _is_lab_step(r0: float, r1: float, spec: Spec) -> bool:
    return (
        abs(r0 - spec.ref_a) <= spec.ref_tol and abs(r1 - spec.ref_b) <= spec.ref_tol
    ) or (
        abs(r0 - spec.ref_b) <= spec.ref_tol and abs(r1 - spec.ref_a) <= spec.ref_tol
    )


def _voltage(t, v, v_pos, v_neg):
    imax = 0
    imin = 0
    viol_t: list[float] = []
    viol_v: list[float] = []
    for i, vi in enumerate(v):
        if vi > v[imax]:
            imax = i
        if vi < v[imin]:
            imin = i
        if vi > v_pos or vi < -v_neg:
            viol_t.append(t[i])
            viol_v.append(vi)
    return v[imin], t[imin], v[imax], t[imax], viol_t, viol_v


def _stability(t, y, y0, r1, spec: Spec):
    if not t:
        return "INCONCLUSIVE", "no samples", []
    step_amp = abs(r1 - y0)
    for ti, yi in zip(t, y):
        if abs(yi) > spec.runaway_abs_m or abs(yi - r1) > step_amp + spec.runaway_err_m:
            return (
                "UNSTABLE",
                f"runaway: y={yi:.3f} m at t={ti:.3f} s",
                [],
            )
    span = t[-1] - t[0]
    if span < spec.bucket_s:
        return "INCONCLUSIVE", f"less than {spec.bucket_s:.2f} s of step data", []

    n_buckets = int(math.floor(span / spec.bucket_s + 1e-9))
    bucket_peak: list[float | None] = []
    bucket_mark: list[tuple[float, float] | None] = []
    for b in range(n_buckets):
        lo = t[0] + b * spec.bucket_s
        hi = lo + spec.bucket_s
        best_e = None
        best = None
        for ti, yi in zip(t, y):
            if lo <= ti < hi:
                err = abs(yi - r1)
                if best_e is None or err > best_e:
                    best_e = err
                    best = (ti, yi)
        bucket_peak.append(best_e)
        bucket_mark.append(best)

    if n_buckets >= 3:
        last = bucket_peak[-3:]
        marks = bucket_mark[-3:]
        if all(p is not None for p in last) and all(m is not None for m in marks):
            growing = (
                last[1] > last[0] * spec.growth_ratio
                and last[2] > last[1] * spec.growth_ratio
                and last[2] > spec.growth_floor_m
            )
            if growing:
                nums = ", ".join(f"{p:.3f}" for p in last)
                reason = f"tracking-error peaks growing over the last {3 * spec.bucket_s:.2f} s ({nums} m)"
                return "UNSTABLE", reason, [m for m in marks if m is not None]
    return "STABLE", "bounded; tracking-error peaks are not growing", []


def _steady_state(t, y, y0, r1, spec: Spec) -> SteadyState:
    elapsed = t[-1] - t[0] if t else 0.0
    need = 2.0 * spec.tail_s
    if not t or elapsed < need:
        return SteadyState(False, False, f"need {need:.2f} s before the tail can be called flat")

    t_end = t[-1]
    tail_t, tail_y = _window(t, y, t_end - spec.tail_s, t_end, right_open=False)
    prev_t, prev_y = _window(t, y, t_end - 2 * spec.tail_s, t_end - spec.tail_s, right_open=True)
    if len(tail_y) < 5 or len(prev_y) < 5:
        return SteadyState(False, False, "not enough samples in the tail")
    if tail_t[-1] - tail_t[0] < 0.6 * spec.tail_s:
        return SteadyState(False, False, "tail window is too short")

    y_ss = _mean(tail_y)
    std = _stdev(tail_y)
    slope = _slope(tail_t, tail_y)
    drift = abs(y_ss - _mean(prev_y))
    flat = std <= spec.ss_std_max and abs(slope) <= spec.ss_slope_max and drift <= spec.ss_drift_max
    move = abs(y_ss - y0)
    min_move = max(spec.min_move_m, spec.min_move_frac * abs(r1 - y0))
    moved = move >= min_move
    base = SteadyState(
        confident=False,
        moved=moved,
        reason="",
        y_ss=y_ss,
        std=std,
        slope=slope,
        t0=tail_t[0],
        t1=tail_t[-1],
    )
    if not flat:
        base.reason = (
            f"tail not flat (std {std:.4f} m, slope {slope:+.4f} m/s, "
            f"drift {drift:.4f} m)"
        )
        base.moved = False
        return base
    if not moved and elapsed < spec.still_time_s:
        base.reason = "still at the initial position"
        base.moved = False
        return base
    base.confident = True
    base.moved = moved
    if moved:
        base.reason = "flat tail"
    else:
        base.reason = "flat, but the ball never left the initial position"
    return base


def _trailing_median(y, n: int) -> list[float]:
    """Median of the last n samples at each index. n <= 1 returns the raw series."""
    if n <= 1:
        return list(y)
    out: list[float] = []
    for i in range(len(y)):
        out.append(_median(y[max(0, i - n + 1) : i + 1]))
    return out


def _settling(t, y, y_ss, half, median_n: int = 1):
    """(t_s, median_at_t_s, ends_outside). A trailing median ignores one-sample spikes."""
    judged = _trailing_median(y, median_n)
    last_out = None
    for i, yi in enumerate(judged):
        if abs(yi - y_ss) > half:
            last_out = i
    if last_out is None:
        return t[0], judged[0], False
    if last_out == len(judged) - 1:
        return None, None, True
    i = last_out + 1
    return t[i], judged[i], False


def _overshoot(t, y, y0, y_ss):
    move = y_ss - y0
    if move == 0:
        return None
    sigma = 1.0 if move > 0 else -1.0
    best_i = 0
    best = -1e300
    for i, yi in enumerate(y):
        exc = sigma * (yi - y_ss)
        if exc > best:
            best = exc
            best_i = i
    excursion = max(0.0, best)
    pct = excursion / abs(move) * 100.0
    return pct, t[best_i], y[best_i]


def _target_tol(r1: float, spec: Spec) -> float:
    if abs(r1 - spec.ref_a) <= spec.ref_tol:
        return spec.ess_tol_a
    if abs(r1 - spec.ref_b) <= spec.ref_tol:
        return spec.ess_tol_b
    return spec.ess_tol_b


def compute_metrics(t, y, voltage, y0, r0, r1, spec: Spec) -> Metrics | None:
    if not t:
        return None
    v_min, v_min_t, v_max, v_max_t, viol_t, viol_v = _voltage(t, voltage, spec.v_pos, spec.v_neg)
    stability, stability_reason, growth_marks = _stability(t, y, y0, r1, spec)
    ss = _steady_state(t, y, y0, r1, spec)

    e_ss = None
    band_half = _target_tol(r1, spec)
    ts = ts_y = None
    outside_now = False
    ts_note = "waiting for a flat steady state"
    os_pct = peak_t = peak_y = None
    os_note = "waiting for a flat steady state"
    pass_ess = pass_ts = pass_os = None

    if ss.confident and ss.y_ss is not None:
        e_ss = r1 - ss.y_ss
        if ss.moved:
            ts, ts_y, outside_now = _settling(
                t, y, r1, band_half, spec.settle_median_n
            )
            pass_ess = abs(e_ss) <= band_half and not outside_now
            elapsed = t[-1] - t[0]
            if outside_now:
                ts_note = f"still outside [{r1 - band_half:.3f}, {r1 + band_half:.3f}] m at t={elapsed:.3f} s"
                pass_ts = False if elapsed > spec.ts_max else None
            elif spec.settle_median_n > 1:
                ts_note = (
                    f"first sample whose trailing {spec.settle_median_n}-sample "
                    "median stays in target band"
                )
                pass_ts = ts is not None and ts <= spec.ts_max
            else:
                ts_note = "first sample inside target band that stays inside"
                pass_ts = ts is not None and ts <= spec.ts_max
            ov = _overshoot(t, y, y0, ss.y_ss)
            if ov is not None:
                os_pct, peak_t, peak_y = ov
                os_note = "peak past y_ss, in the direction of the step"
                pass_os = os_pct < spec.os_max_pct
        else:
            pass_ess = abs(e_ss) <= band_half
            ts_note = "ball never left the initial position"
            os_note = "ball never left the initial position"

    pass_stable = {"STABLE": True, "UNSTABLE": False}.get(stability)

    return Metrics(
        elapsed=t[-1] - t[0],
        n_post=len(t),
        dt_ms=_median_dt_ms(t),
        y0=y0,
        r0=r0,
        r1=r1,
        lab_step=_is_lab_step(r0, r1, spec),
        ss=ss,
        e_ss=e_ss,
        band_half=band_half,
        ts=ts,
        ts_y=ts_y,
        outside_now=outside_now,
        ts_note=ts_note,
        overshoot_pct=os_pct,
        peak_t=peak_t,
        peak_y=peak_y,
        os_note=os_note,
        v_min=v_min,
        v_max=v_max,
        v_min_t=v_min_t,
        v_max_t=v_max_t,
        viol_t=viol_t,
        viol_v=viol_v,
        stability=stability,
        stability_reason=stability_reason,
        growth_marks=growth_marks,
        pass_stable=pass_stable,
        pass_ess=pass_ess,
        pass_ts=pass_ts,
        pass_os=pass_os,
        pass_v=(len(viol_t) == 0),
    )


def _word(flag: bool | None) -> str:
    if flag is True:
        return "PASS"
    if flag is False:
        return "FAIL"
    return "...."


def overall(m: Metrics) -> str:
    flags = (m.pass_stable, m.pass_ess, m.pass_ts, m.pass_os, m.pass_v)
    if any(f is False for f in flags):
        return "SPEC FAIL"
    if any(f is None for f in flags):
        return "MEASURING"
    return "ALL PASS"


def _signature(m: Metrics):
    def r(x, n):
        return None if x is None else round(x, n)

    return (
        m.stability,
        m.pass_stable,
        m.pass_ess,
        m.pass_ts,
        m.pass_os,
        m.pass_v,
        r(m.e_ss, 4),
        r(m.ts, 3),
        r(m.overshoot_pct, 1),
        r(m.y_ss, 4),
        r(m.v_min, 2),
        r(m.v_max, 2),
        m.ss_confident,
        m.moved,
    )


def oneline(m: Metrics) -> str:
    stab = {"STABLE": "STBL", "UNSTABLE": "UNST", "INCONCLUSIVE": "WAIT"}.get(m.stability, "????")
    ess = f"{m.e_ss:+.4f}" if m.e_ss is not None else "   ...."
    if m.ts is not None:
        ts = f"{m.ts:.3f}s"
    elif m.outside_now:
        ts = "outside"
    else:
        ts = "   ...."
    osp = f"{m.overshoot_pct:5.1f}%" if m.overshoot_pct is not None else "  ...."
    vmax = f"{m.v_max:+5.2f}" if m.v_max is not None else "  ...."
    vmin = f"{m.v_min:+5.2f}" if m.v_min is not None else "  ...."
    return (
        f"t={m.elapsed:6.3f}s {stab} {_word(m.pass_stable)}  "
        f"ess={ess} {_word(m.pass_ess)}  "
        f"Ts={ts:>7} {_word(m.pass_ts)}  "
        f"OS={osp} {_word(m.pass_os)}  "
        f"V={vmax}/{vmin} {_word(m.pass_v)}"
    )


def format_report(m: Metrics, spec: Spec, end_reason: str | None, step_index: int) -> str:
    dt = "unknown" if m.dt_ms is None else f"{m.dt_ms:.2f} ms"
    lines = [
        f"Step {step_index}: y_ref {m.r0:+.4f} -> {m.r1:+.4f} m"
        + ("" if m.lab_step else f"  (not {spec.ref_a:.2f} -> {spec.ref_b:.2f})"),
        f"y(0-) = {m.y0:+.4f} m    {m.elapsed:.3f} s, {m.n_post} samples, dt {dt}",
    ]
    if m.dt_ms is not None and (m.dt_ms > 15.0 or m.dt_ms < 0.4):
        lines.append("WARNING: sample interval is far from the 2 ms loop. Check baud 230400.")
    if end_reason:
        lines.append(f"Capture: {end_reason}")
    lines.append(f"Stability: {m.stability} {_word(m.pass_stable)}  {m.stability_reason}")
    if m.e_ss is None:
        lines.append(f"e_ss: {_word(m.pass_ess)}  {m.ss.reason}")
    else:
        tol = m.band_half if m.band_half is not None else _target_tol(m.r1, spec)
        lines.append(
            f"e_ss = {m.e_ss:+.4f} m {_word(m.pass_ess)}  "
            f"y_ss={m.y_ss:+.4f} over [{m.ss.t0:.3f}, {m.ss.t1:.3f}] s  "
            f"std {m.ss.std:.4f} slope {m.ss.slope:+.4f}  band [{m.r1 - tol:.3f}, {m.r1 + tol:.3f}] m"
        )
    if m.ts is not None and m.band_half is not None:
        lines.append(
            f"t_s = {m.ts:.3f} s {_word(m.pass_ts)}  "
            f"band [{m.r1 - m.band_half:.3f}, {m.r1 + m.band_half:.3f}] m  <= {spec.ts_max:.3f} s  {m.ts_note}"
        )
    else:
        lines.append(f"t_s: {_word(m.pass_ts)}  {m.ts_note}  <= {spec.ts_max:.3f} s")
    if m.overshoot_pct is not None:
        lines.append(
            f"OS = {m.overshoot_pct:.2f}% {_word(m.pass_os)}  "
            f"peak {m.peak_y:+.4f} at {m.peak_t:.3f} s  < {spec.os_max_pct:.1f}%"
        )
    else:
        lines.append(f"OS: {_word(m.pass_os)}  {m.os_note}  < {spec.os_max_pct:.1f}%")
    voltage = (
        f"Voltage: {_word(m.pass_v)}  "
        f"outside +{spec.v_pos:.1f}/-{spec.v_neg:.1f} V: {len(m.viol_t)}"
    )
    if m.v_max is not None:
        voltage += f"  max {m.v_max:+.2f} @ {m.v_max_t:.3f}s  min {m.v_min:+.2f} @ {m.v_min_t:.3f}s"
    lines.append(voltage)
    return "\n".join(lines)


def emit_report(title: str, body: str) -> None:
    bar = "=" * 72
    print("\n" + bar)
    print(title)
    print(bar)
    print(body)
    print(bar, flush=True)


# ---------------------------------------------------------------------------
# One-step capture
# ---------------------------------------------------------------------------

class StepCapture:
    """Turns a stream of (t_ms, y, ref, voltage) into one step record."""

    def __init__(self, spec: Spec):
        self.spec = spec
        self.phase = "wait"  # wait, live, frozen
        self.pre: deque = deque()
        self.preview: deque = deque()
        self.step: dict | None = None
        self.last_ref: float | None = None
        self.last_t_ms: float | None = None
        self.next_index = 1
        self.end_reason: str | None = None
        self.saved = False
        self.capped = False
        # The first reference edge after connect is not recorded. After that,
        # a capture starts from any steady allowed start dwell into the opposite level.
        self.skipped_first = False
        self.ready = False
        self.dwell_target: float | None = None
        self.dwell: deque = deque()
        self.hold_note = "skipping the first reference step"

    def _allowed_starts(self) -> list[tuple[float, float]]:
        low = self.spec.ref_a
        high = self.spec.ref_b
        if self.spec.step_dir == "pos":
            return [(low, high)]
        if self.spec.step_dir == "neg":
            return [(high, low)]
        return [(low, high), (high, low)]

    def _wait_prompt(self) -> str:
        low = self.spec.ref_a
        high = self.spec.ref_b
        if self.spec.step_dir == "pos":
            return f"waiting for steady {low:+.3f} m -> {high:+.3f} m"
        if self.spec.step_dir == "neg":
            return f"waiting for steady {high:+.3f} m -> {low:+.3f} m"
        return f"waiting for steady {low:+.3f} or {high:+.3f} m"

    def arm(self) -> None:
        """Arm the next capture without skipping an extra step."""
        self.phase = "wait"
        self.step = None
        self.pre.clear()
        self.preview.clear()
        self.dwell.clear()
        self.ready = False
        self.dwell_target = None
        self.skipped_first = True
        self.end_reason = None
        self.saved = False
        self.capped = False
        self.hold_note = self._wait_prompt()

    def freeze(self, reason: str) -> bool:
        if self.phase != "live" or self.step is None:
            return False
        if len(self.step["t"]) <= self.step["n_pre"]:
            return False
        self.phase = "frozen"
        self.end_reason = reason
        return True

    def metrics(self) -> Metrics | None:
        if not self.step:
            return None
        n = self.step["n_pre"]
        t = self.step["t"][n:]
        if not t:
            return None
        return compute_metrics(
            t,
            self.step["y"][n:],
            self.step["v"][n:],
            self.step["y0"],
            self.step["r0"],
            self.step["r1"],
            self.spec,
        )

    def _trim_preview(self, t_ms: float) -> None:
        while self.preview and t_ms - self.preview[0][0] > PREVIEW_S * 1000.0:
            self.preview.popleft()

    def _trim_pre(self, t_ms: float) -> None:
        while self.pre and t_ms - self.pre[0][0] > (PRE_STEP_S + 0.1) * 1000.0:
            self.pre.popleft()

    def _begin(
        self,
        t_ms: float,
        y: float,
        ref: float,
        voltage: float,
        theta: float | None = None,
        theta_ref: float | None = None,
    ) -> None:
        cands = [yi for tm, yi, _rf, _vv, _th, _thr in self.pre if 0.0 < (t_ms - tm) <= 40.0]
        if not cands and self.pre:
            cands = [self.pre[-1][1]]
        y0 = _median(cands) if cands else y
        r0 = self.last_ref if self.last_ref is not None else ref

        t_list: list[float] = []
        y_list: list[float] = []
        rf_list: list[float] = []
        v_list: list[float] = []
        th_list: list[float] = []
        thr_list: list[float] = []
        has_theta = theta is not None and theta_ref is not None
        for tm, yi, rf, vv, th, thr in self.pre:
            dt = (tm - t_ms) / 1000.0
            if -PRE_STEP_S <= dt < 0.0:
                t_list.append(dt)
                y_list.append(yi)
                rf_list.append(rf)
                v_list.append(vv)
                if has_theta and th is not None and thr is not None:
                    th_list.append(th)
                    thr_list.append(thr)
                else:
                    has_theta = False
        n_pre = len(t_list)
        t_list.append(0.0)
        y_list.append(y)
        rf_list.append(ref)
        v_list.append(voltage)
        if has_theta and theta is not None and theta_ref is not None:
            th_list.append(theta)
            thr_list.append(theta_ref)

        self.step = {
            "index": self.next_index,
            "r0": r0,
            "r1": ref,
            "y0": y0,
            "t0_ms": t_ms,
            "t": t_list,
            "y": y_list,
            "ref": rf_list,
            "v": v_list,
            "theta": th_list if has_theta else None,
            "theta_ref": thr_list if has_theta else None,
            "n_pre": n_pre,
            "started_wall": datetime.now(),
        }
        self.next_index += 1
        self.phase = "live"
        self.end_reason = None
        self.saved = False
        self.capped = False
        self.pre.clear()

    def _append(
        self,
        t_ms: float,
        y: float,
        ref: float,
        voltage: float,
        theta: float | None = None,
        theta_ref: float | None = None,
    ) -> bool:
        t_rel = (t_ms - self.step["t0_ms"]) / 1000.0
        if t_rel > 120.0:
            return False
        self.step["t"].append(t_rel)
        self.step["y"].append(y)
        self.step["ref"].append(ref)
        self.step["v"].append(voltage)
        if self.step["theta"] is not None:
            if theta is not None and theta_ref is not None:
                self.step["theta"].append(theta)
                self.step["theta_ref"].append(theta_ref)
            else:
                self.step["theta"] = None
                self.step["theta_ref"] = None
        return True

    def push(
        self,
        t_ms: float,
        y: float,
        ref: float,
        voltage: float,
        theta: float | None = None,
        theta_ref: float | None = None,
    ) -> str | None:
        jump = self.last_t_ms is not None and t_ms < self.last_t_ms - 500.0
        if jump and self.phase == "live" and self.step is not None and len(self.step["t"]) > self.step["n_pre"]:
            self.last_t_ms = t_ms
            self.last_ref = ref
            self.preview.clear()
            self.preview.append((t_ms, y, ref, voltage))
            self.dwell.clear()
            self.ready = False
            self.phase = "frozen"
            self.end_reason = "board time jumped"
            return "end"
        if jump:
            self.pre.clear()
            self.preview.clear()
            self.dwell.clear()
            self.last_ref = None
            self.ready = False
            if self.phase == "wait":
                self.skipped_first = False
                self.hold_note = "board reset, skipping the first step"

        self.last_t_ms = t_ms
        self.preview.append((t_ms, y, ref, voltage))
        self._trim_preview(t_ms)

        if self.phase == "frozen":
            self.last_ref = ref
            return None

        if self.phase == "wait":
            return self._push_wait(t_ms, y, ref, voltage, theta, theta_ref)

        if abs(ref - self.step["r1"]) >= self.spec.step_delta:
            self.phase = "frozen"
            self.end_reason = "reference stepped away"
            self.last_ref = ref
            return "end"
        if not self._append(t_ms, y, ref, voltage, theta, theta_ref) and not self.capped:
            self.capped = True
            print("\nCapture reached 120 s and is holding the buffer. Press M to freeze.")
        self.last_ref = ref
        return "sample"

    def _near(self, ref: float, target: float) -> bool:
        return abs(ref - target) <= self.spec.ref_tol

    def _target_is_steady(self, target: float) -> tuple[bool, str]:
        """True once the recent dwell at target is flat and the ball is there."""
        spec = self.spec
        if len(self.dwell) < 10:
            return False, f"holding {target:+.3f} m, collecting samples"
        t1 = self.dwell[-1][0]
        tail_ms = spec.tail_s * 1000.0
        tail = [(tm, yi) for tm, yi in self.dwell if tm >= t1 - tail_ms]
        prev = [(tm, yi) for tm, yi in self.dwell if t1 - 2.0 * tail_ms <= tm < t1 - tail_ms]
        if len(tail) < 5 or len(prev) < 5 or tail[-1][0] - tail[0][0] < 0.6 * tail_ms:
            return False, f"holding {target:+.3f} m, waiting for the ball to settle"

        def stats(pts: list[tuple[float, float]]) -> tuple[float, float, float]:
            ts = [(p[0] - pts[0][0]) / 1000.0 for p in pts]
            ys = [p[1] for p in pts]
            return _mean(ys), _stdev(ys), _slope(ts, ys)

        tail_mean, tail_std, tail_slope = stats(tail)
        prev_mean, _, _ = stats(prev)
        flat = (
            tail_std <= spec.ss_std_max
            and abs(tail_slope) <= spec.ss_slope_max
            and abs(tail_mean - prev_mean) <= spec.ss_drift_max
        )
        if not flat:
            return False, f"at {target:+.3f} m, ball still moving"
        if abs(tail_mean - target) > spec.at_target_m:
            return False, f"flat at {tail_mean:+.3f} m, waiting until the ball is near {target:+.3f} m"
        return True, f"steady at {target:+.3f} m"

    def _push_wait(
        self,
        t_ms: float,
        y: float,
        ref: float,
        voltage: float,
        theta: float | None = None,
        theta_ref: float | None = None,
    ) -> str | None:
        changed = self.last_ref is not None and abs(ref - self.last_ref) >= self.spec.step_delta
        allowed = self._allowed_starts()

        if changed and not self.skipped_first:
            self.skipped_first = True
            self.ready = False
            self.dwell_target = None
            self.dwell.clear()
            self.pre.clear()
            self.last_ref = ref
            self.hold_note = f"skipped the first step; {self._wait_prompt()}"
            return "skip"

        if changed and self.ready and self.dwell_target is not None:
            for start_t, end_t in allowed:
                if self._near(self.last_ref, start_t) and self._near(ref, end_t):
                    self._begin(t_ms, y, ref, voltage, theta, theta_ref)
                    self.last_ref = ref
                    self.ready = False
                    self.dwell_target = None
                    self.dwell.clear()
                    self.hold_note = "recording"
                    return "start"

        if changed:
            self.ready = False
            self.dwell_target = None
            self.dwell.clear()
            self.pre.clear()

        matched = next(((s, e) for s, e in allowed if self._near(ref, s)), None)
        if matched is not None:
            start_t, end_t = matched
            if self.dwell_target is not None and abs(self.dwell_target - start_t) > 1e-9:
                self.dwell.clear()
                self.pre.clear()
                self.ready = False
            self.dwell_target = start_t
            self.pre.append((t_ms, y, ref, voltage, theta, theta_ref))
            self._trim_pre(t_ms)
            self.dwell.append((t_ms, y))
            dwell_ms = max(500.0, 2.5 * self.spec.tail_s * 1000.0)
            while self.dwell and t_ms - self.dwell[0][0] > dwell_ms:
                self.dwell.popleft()
            steady, why = self._target_is_steady(start_t)
            self.ready = self.skipped_first and steady
            if not self.skipped_first:
                self.hold_note = "skipping the first reference step"
            elif self.ready:
                self.hold_note = f"steady at {start_t:+.3f} m; recording starts on step to {end_t:+.3f} m"
            else:
                self.hold_note = why
        else:
            self.ready = False
            self.dwell_target = None
            self.dwell.clear()
            self.pre.clear()
            if not self.skipped_first:
                self.hold_note = "skipping the first reference step"
            else:
                want = allowed[0][0]
                self.hold_note = f"y_ref {ref:+.3f} m; waiting for {want:+.3f} m"

        self.last_ref = ref
        return None


# ---------------------------------------------------------------------------
# Report files and clipboard
# ---------------------------------------------------------------------------

def _log_dir() -> str:
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    os.makedirs(path, exist_ok=True)
    return path


class SessionLog:
    """Every accepted serial sample, so the run can be drawn again with --file."""

    def __init__(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(_log_dir(), f"full_{stamp}.csv")
        self._f = open(self.path, "w", encoding="utf-8", newline="\n")
        self._has_theta: bool | None = None
        self._lock = threading.Lock()
        self._n = 0
        self._warned = False

    def write(
        self,
        t_ms: float,
        y: float,
        ref: float,
        volt: float,
        theta: float | None = None,
        theta_ref: float | None = None,
    ) -> None:
        with self._lock:
            if self._f is None:
                return
            try:
                if self._has_theta is None:
                    self._has_theta = theta is not None and theta_ref is not None
                    if self._has_theta:
                        self._f.write("millis_ms,ball_pos_m,ball_ref_m,voltage_V,theta_rad,theta_ref_rad\n")
                    else:
                        self._f.write("millis_ms,ball_pos_m,ball_ref_m,voltage_V\n")
                if self._has_theta and theta is not None and theta_ref is not None:
                    line = f"{t_ms:.0f},{y:.5f},{ref:.4f},{volt:.4f},{theta:.5f},{theta_ref:.5f}\n"
                else:
                    line = f"{t_ms:.0f},{y:.5f},{ref:.4f},{volt:.4f}\n"
                self._f.write(line)
                self._n += 1
                if self._n % 50 == 0:
                    self._f.flush()
            except OSError as exc:
                if not self._warned:
                    print(f"Could not write the full log: {exc}")
                    self._warned = True

    def close(self) -> None:
        with self._lock:
            if self._f is None:
                return
            try:
                self._f.flush()
                self._f.close()
            except OSError:
                pass
            self._f = None
        print(f"Full log: {self.path}  ({self._n} samples)")


def _cell(row: list[str], index: int) -> float:
    return float(row[index])


def load_samples(path: str) -> list[tuple[float, float, float, float]]:
    """Rows of (millis_ms, ball_pos_m, ball_ref_m, voltage_V). time_s columns become ms."""
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    if not rows:
        return []
    first = [c.strip() for c in rows[0]]
    start = 0
    idx = (0, 1, 2, 3)
    scale_t = 1.0
    synth_t = False
    try:
        [_cell(first, i) for i in range(len(first)) if first[i]]
        headerless = True
    except ValueError:
        headerless = False
    if headerless:
        ncols = len([c for c in first if c])
        if ncols == 5:
            # Raw 5-col Arduino serial stream: theta, theta_ref, voltage, ball_pos, ball_ref
            idx = (0, 3, 4, 2)
            synth_t = True
        elif ncols == 6:
            # 6-col stream with leading millis: millis, theta, theta_ref, voltage, ball_pos, ball_ref
            idx = (0, 4, 5, 3)
    else:
        names = [c.lower() for c in first]
        start = 1

        def col(*options: str) -> int | None:
            for name in options:
                if name in names:
                    return names.index(name)
            return None

        t_i = col("millis_ms", "time_ms", "millis", "time_s")
        y_i = col("ball_pos_m", "ball_position_m", "y_m", "ball_pos", "y")
        r_i = col("ball_ref_m", "ball_pos_ref_m", "y_ref_m", "ball_ref", "y_ref", "ref_m")
        v_i = col("voltage_v", "voltage")
        if None in (t_i, y_i, r_i, v_i):
            raise ValueError(f"Unrecognized columns in {path}: {', '.join(first)}")
        idx = (t_i, y_i, r_i, v_i)
        if names[t_i] == "time_s":
            scale_t = 1000.0
    samples: list[tuple[float, float, float, float]] = []
    for i, row in enumerate(rows[start:]):
        if not row or not any(c.strip() for c in row):
            continue
        try:
            t_ms = (i * 2.0) if synth_t else (_cell(row, idx[0]) * scale_t)
            samples.append((
                t_ms,
                _cell(row, idx[1]),
                _cell(row, idx[2]),
                _cell(row, idx[3]),
            ))
        except (ValueError, IndexError):
            continue
    return samples


def resolve_log_path(path: str) -> str:
    if os.path.isfile(path):
        return path
    alt = os.path.join(_log_dir(), os.path.basename(path))
    if os.path.isfile(alt):
        return alt
    raise FileNotFoundError(path)


def _write_step_files(step: dict, body: str) -> tuple[str, str]:
    stamp = step["started_wall"].strftime("%Y%m%d_%H%M%S")
    base = os.path.join(_log_dir(), f"step_{step['index']:02d}_{stamp}")
    csv_path = base + ".csv"
    txt_path = base + ".txt"
    has_theta = step.get("theta") is not None and step.get("theta_ref") is not None
    with open(csv_path, "w", encoding="utf-8", newline="\n") as f:
        if has_theta:
            f.write("time_s,ball_pos_m,ball_ref_m,voltage_V,theta_rad,theta_ref_rad\n")
            for ti, yi, ri, vi, th, thr in zip(
                step["t"], step["y"], step["ref"], step["v"], step["theta"], step["theta_ref"]
            ):
                f.write(f"{ti:.4f},{yi:.5f},{ri:.4f},{vi:.4f},{th:.5f},{thr:.5f}\n")
        else:
            f.write("time_s,ball_pos_m,ball_ref_m,voltage_V\n")
            for ti, yi, ri, vi in zip(step["t"], step["y"], step["ref"], step["v"]):
                f.write(f"{ti:.4f},{yi:.5f},{ri:.4f},{vi:.4f}\n")
    with open(txt_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(body)
        if not body.endswith("\n"):
            f.write("\n")
    return csv_path, txt_path


def _copy_text(text: str) -> bool:
    if sys.platform == "win32":
        try:
            import subprocess
            proc = subprocess.Popen(["clip"], stdin=subprocess.PIPE, text=True, close_fds=True)
            proc.communicate(input=text)
            return proc.returncode == 0
        except Exception:
            pass
    try:
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
        root.clipboard_clear()
        root.clipboard_append(text)
        root.update()
        root.destroy()
        return True
    except Exception:
        return False


def finalize_capture(capture: StepCapture, title: str) -> None:
    if capture.saved or capture.step is None:
        return
    m = capture.metrics()
    if m is None:
        return
    capture.saved = True
    body = format_report(m, capture.spec, capture.end_reason, capture.step["index"])
    emit_report(title, body)
    try:
        csv_path, txt_path = _write_step_files(capture.step, body)
        print(f"Saved {txt_path}")
        print(f"Saved {csv_path}")
    except OSError as exc:
        print(f"Could not write the step log: {exc}")
    if _copy_text(body):
        print("Report copied to the clipboard.")
    else:
        print("Could not copy the report to the clipboard.")


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

POS_C = "#1f77b4"
REF_C = "#ff7f0e"
SS_C = "#2ca02c"
SETTLE_C = "#6a3d9a"
PEAK_C = "#d62728"
VOLT_C = "#17becf"
LIMIT_C = "#d62728"
MUTED = "#444444"
PASS_C = "#1a7f37"
FAIL_C = "#b42318"
PEND_C = "#9a6700"


def _verdict_color(flag: bool | None) -> str:
    if flag is True:
        return PASS_C
    if flag is False:
        return FAIL_C
    return PEND_C


def _panel_rows(m: Metrics | None, phase: str, end_reason: str | None, current_ref: float | None, note: str = "", spec: Spec | None = None):
    rows: list[tuple[str, str, int, str]] = []

    def add(text, color=MUTED, size=8, weight="normal"):
        rows.append((text, color, size, weight))

    def gap():
        add("", MUTED, 4)

    def verdict_line(label: str, flag: bool | None):
        word = _word(flag)
        pad = max(1, 34 - len(label) - len(word))
        add(label + (" " * pad) + word, _verdict_color(flag), 8, "bold")

    if m is None:
        add("WAITING", PEND_C, 13, "bold")
        gap()
        if current_ref is None:
            add("No samples yet.", MUTED, 8)
        else:
            add(f"y_ref = {current_ref:+.3f} m", MUTED, 8)
        for line in textwrap.wrap(note, width=36)[:4]:
            add(line, MUTED, 8)
        gap()
        add("[Space] release port", MUTED, 8)
        return rows, "#fff8ec"

    name = overall(m)
    color = {"ALL PASS": PASS_C, "SPEC FAIL": FAIL_C, "MEASURING": PEND_C}[name]
    face = {"ALL PASS": "#eef8f0", "SPEC FAIL": "#fdf2f2", "MEASURING": "#fff8ec"}[name]
    phase_word = {"live": "LIVE", "frozen": "FROZEN", "wait": "WAIT"}.get(phase, phase.upper())
    add(name, color, 12, "bold")
    add(phase_word, MUTED, 8, "bold")
    ref_line = f"{m.r0:+.3f} -> {m.r1:+.3f} m"
    if not m.lab_step:
        ref_a = LAB_REF_A_M if spec is None else spec.ref_a
        ref_b = LAB_REF_B_M if spec is None else spec.ref_b
        ref_line += f"  (not {ref_a:.2f}->{ref_b:.2f})"
    add(ref_line, "#222222", 8)
    dt = "" if m.dt_ms is None else f"   dt {m.dt_ms:.2f} ms"
    dt_color = FAIL_C if (m.dt_ms is not None and (m.dt_ms > 15.0 or m.dt_ms < 0.4)) else MUTED
    add(f"t={m.elapsed:.3f}s  N={m.n_post}{dt}", dt_color, 8)
    if end_reason and phase == "frozen":
        add(textwrap.shorten(end_reason, width=36, placeholder="..."), MUTED, 8)
    gap()
    add(f"y(0-) = {m.y0:+.4f} m", "#222222", 8)
    gap()
    verdict_line("Stability", m.pass_stable)
    for line in textwrap.wrap(m.stability_reason, width=36)[:2]:
        add(line, MUTED, 8)
    gap()
    if m.e_ss is None:
        verdict_line("e_ss", None)
        for line in textwrap.wrap(m.ss.reason, width=36)[:2]:
            add(line, MUTED, 8)
    else:
        verdict_line(f"e_ss {m.e_ss:+.4f} m", m.pass_ess)
        add(f"y_ss = {m.y_ss:+.4f} m", "#222222", 8)
    gap()
    if m.ts is not None:
        verdict_line(f"t_s {m.ts:.3f}s", m.pass_ts)
        add(f"[{m.r1 - m.band_half:.3f}, {m.r1 + m.band_half:.3f}] m", MUTED, 8)
    else:
        verdict_line("t_s", m.pass_ts)
        for line in textwrap.wrap(m.ts_note, width=36)[:2]:
            add(line, MUTED, 8)
    gap()
    if m.overshoot_pct is not None:
        verdict_line(f"OS {m.overshoot_pct:.1f}%", m.pass_os)
        add(f"peak {m.peak_y:+.4f} @ {m.peak_t:.3f}s", MUTED, 8)
    else:
        verdict_line("overshoot", m.pass_os)
        for line in textwrap.wrap(m.os_note, width=36)[:2]:
            add(line, MUTED, 8)
    gap()
    verdict_line("voltage", m.pass_v)
    if m.v_max is not None:
        add(f"max {m.v_max:+.2f} @ {m.v_max_t:.3f}s", MUTED, 8)
        add(f"min {m.v_min:+.2f} @ {m.v_min_t:.3f}s", MUTED, 8)
    if m.viol_t:
        add(f"{len(m.viol_t)} samples past the rail", FAIL_C, 8)
    gap()
    if phase == "frozen":
        add("[N] next step", MUTED, 8)
    else:
        add("[M] freeze", MUTED, 8)
    if spec is not None:
        add(f"[D] dir: {spec.step_dir}", MUTED, 8)
    add("[Space] release port", MUTED, 8)
    return rows, face


def _paint_panel(ax, rows, face) -> None:
    ax.cla()
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.set_facecolor(face)
    axes_h_in = ax.get_position().height * ax.figure.get_size_inches()[1]
    if axes_h_in < 0.5:
        axes_h_in = 6.0
    y = 0.985
    for text, color, size, weight in rows:
        ax.text(
            0.05,
            y,
            text if text else " ",
            color=color,
            fontsize=size,
            fontweight=weight,
            fontfamily="DejaVu Sans Mono",
            va="top",
            ha="left",
            transform=ax.transAxes,
            clip_on=True,
        )
        y -= ((size + 3.2) / 72.0) / axes_h_in


def _label_bbox(edge: str) -> dict:
    return dict(boxstyle="round,pad=0.18", fc="white", ec=edge, lw=0.4, alpha=0.94)


def _style_pos(ax) -> None:
    ax.set_ylabel("ball position (m)")
    ax.set_title("Ball Position", loc="left", fontsize=10, color="#333333")
    ax.grid(True, linestyle="--", alpha=0.55)
    ax.tick_params(labelbottom=False)


def _style_voltage(ax) -> None:
    ax.set_ylabel("logged voltage (V)")
    ax.set_xlabel("time from reference step (s)")
    ax.set_title("Logged voltage", loc="left", fontsize=10, color="#333333")
    ax.grid(True, linestyle="--", alpha=0.55)


def draw_waiting(
    ax_pos, ax_v, ax_info, preview, current_ref, suptitle: str, note: str = "",
    v_pos: float = LAB_V_POS_V, v_neg: float = LAB_V_NEG_V,
) -> None:
    fig = ax_pos.figure
    ax_pos.cla()
    ax_v.cla()
    _style_pos(ax_pos)
    _style_voltage(ax_v)
    fig.suptitle(suptitle, fontsize=11)
    if preview:
        t_now = preview[-1][0]
        ts = [(p[0] - t_now) / 1000.0 for p in preview]
        y = [p[1] for p in preview]
        ref = [p[2] for p in preview]
        volt = [p[3] for p in preview]
        ax_pos.plot(ts, y, color=POS_C, lw=1.6, label=r"$y$")
        ax_pos.plot(ts, ref, color=REF_C, lw=1.4, drawstyle="steps-post", label=r"$y_{ref}$")
        ax_pos.legend(loc="upper left", fontsize=8, framealpha=0.92)
        ax_v.plot(ts, volt, color=VOLT_C, lw=1.4, label="logged voltage")
        ax_v.axhline(v_pos, color=LIMIT_C, ls="--", lw=1.0, label=f"+{v_pos:.1f} V")
        ax_v.axhline(-v_neg, color=LIMIT_C, ls="--", lw=1.0, label=f"-{v_neg:.1f} V")
        ax_v.legend(loc="upper left", fontsize=8, framealpha=0.92)
        ax_pos.set_xlim(min(ts), 0.0)
        ax_v.set_xlim(min(ts), 0.0)
        ymin = min(-7.5, min(volt) - 0.4)
        ymax = max(7.5, max(volt) + 0.4)
        ax_v.set_ylim(ymin, ymax)
        ax_v.set_xlabel("time (s), right edge is now")
    rows, face = _panel_rows(None, "wait", None, current_ref, note)
    _paint_panel(ax_info, rows, face)


def draw_step(
    ax_pos, ax_v, ax_info, step: dict, m: Metrics, phase: str, suptitle: str,
    spec: Spec | None = None, end_reason: str | None = None,
) -> None:
    fig = ax_pos.figure
    ax_pos.cla()
    ax_v.cla()
    _style_pos(ax_pos)
    _style_voltage(ax_v)
    fig.suptitle(suptitle, fontsize=11)

    t = step["t"]
    y = step["y"]
    ref = step["ref"]
    volt = step["v"]
    y0 = step["y0"]

    ax_pos.axvline(0.0, color="#888888", lw=0.9, zorder=0)
    ax_pos.plot(t, y, color=POS_C, lw=1.8, label=r"$y$", zorder=3)
    ax_pos.plot(t, ref, color=REF_C, lw=1.5, drawstyle="steps-post", label=r"$y_{ref}$", zorder=2)
    ax_pos.axhline(y0, color="#555555", ls=":", lw=1.1, label=r"$y(0^-)$", zorder=1)
    ax_pos.text(
        0.0,
        0.03,
        " step",
        transform=ax_pos.get_xaxis_transform(),
        fontsize=8,
        color="#666666",
        va="bottom",
        ha="left",
    )

    y_vals = list(y) + [y0, step["r1"], step["r0"]]
    t_end = max(t[-1], 0.0)
    if m.band_half is not None:
        lo = step["r1"] - m.band_half
        hi = step["r1"] + m.band_half
        ax_pos.fill_between(
            [0.0, t_end], [lo, lo], [hi, hi],
            color=SS_C, alpha=0.16, label=f"[{lo:.3f}, {hi:.3f}] m band", zorder=0,
        )
        ax_pos.plot([0.0, t_end], [lo, lo], color=SS_C, ls=":", lw=0.8, zorder=1)
        ax_pos.plot([0.0, t_end], [hi, hi], color=SS_C, ls=":", lw=0.8, zorder=1)
        y_vals.extend([lo, hi])
    if m.ss_confident and m.y_ss is not None:
        ax_pos.plot([0.0, t_end], [m.y_ss, m.y_ss], color=SS_C, ls="--", lw=1.4, label=r"$y_{ss}$", zorder=2)
        y_vals.append(m.y_ss)
        if m.ts is not None and m.ts_y is not None:
            ax_pos.axvline(m.ts, color=SETTLE_C, ls="--", lw=1.2, zorder=2)
            ax_pos.plot([m.ts], [m.ts_y], marker="s", ms=7, color=SETTLE_C, markeredgecolor="white", markeredgewidth=0.6, zorder=5)
            span = max(t[-1] - t[0], 1e-3)
            on_right = m.ts > t[0] + 0.72 * span
            ax_pos.text(
                m.ts,
                1.01,
                f"t_s={m.ts:.3f}s",
                transform=ax_pos.get_xaxis_transform(),
                fontsize=8,
                color=SETTLE_C,
                va="bottom",
                ha="right" if on_right else "left",
                clip_on=False,
                bbox=_label_bbox(SETTLE_C),
            )
        if m.peak_t is not None and m.peak_y is not None:
            ax_pos.plot(
                [m.peak_t],
                [m.peak_y],
                marker="o",
                ms=8,
                color=PEAK_C,
                markeredgecolor="white",
                markeredgewidth=0.7,
                zorder=6,
            )

    for i, (gt, gy) in enumerate(m.growth_marks):
        ax_pos.plot(
            [gt],
            [gy],
            marker="x",
            ms=8,
            color=PEAK_C,
            zorder=6,
            label="growing error" if i == 0 else "_",
        )
        y_vals.append(gy)

    y_lo = min(y_vals)
    y_hi = max(y_vals)
    pad = max(0.02, 0.22 * (y_hi - y_lo))
    ax_pos.set_ylim(y_lo - pad, y_hi + pad)

    if m.peak_t is not None and m.peak_y is not None:
        # Callout sits off the trace, on the far side of the peak.
        above = m.y_ss is None or m.peak_y >= m.y_ss
        ax_pos.annotate(
            f"OS {m.overshoot_pct:.1f}%",
            xy=(m.peak_t, m.peak_y),
            xytext=(-12, 12 if above else -12),
            textcoords="offset points",
            fontsize=8,
            color=PEAK_C,
            ha="right",
            va="bottom" if above else "top",
            arrowprops=dict(arrowstyle="-", color=PEAK_C, lw=0.6),
            bbox=_label_bbox(PEAK_C),
            zorder=7,
        )

    # Empty corner: a rising step leaves the lower right open, a falling step the upper right.
    loc = "lower right" if step["r1"] >= y0 else "upper right"
    ax_pos.legend(loc=loc, fontsize=8, framealpha=0.92)

    ax_v.plot(t, volt, color=VOLT_C, lw=1.5, label="logged voltage", zorder=3)
    v_pos = LAB_V_POS_V if spec is None else spec.v_pos
    v_neg = LAB_V_NEG_V if spec is None else spec.v_neg
    ax_v.axhline(v_pos, color=LIMIT_C, ls="--", lw=1.1, label=f"+{v_pos:.1f} V", zorder=1)
    ax_v.axhline(-v_neg, color=LIMIT_C, ls="--", lw=1.1, label=f"-{v_neg:.1f} V", zorder=1)
    if m.viol_t:
        ax_v.scatter(
            m.viol_t, m.viol_v, s=22, color=PEAK_C, zorder=4,
            label=f"outside +{v_pos:.1f}/-{v_neg:.1f} V",
        )
    v_lo = min(-8.0, min(volt) - 1.4)
    v_hi = max(8.0, max(volt) + 1.4)
    ax_v.set_ylim(v_lo, v_hi)

    def mark_extremum(t_mark, v_mark, marker):
        rail = v_pos if v_mark >= 0 else v_neg
        past = v_mark > v_pos or v_mark < -v_neg
        color = PEAK_C if past else "#1f4e79"
        ax_v.plot(
            [t_mark], [v_mark], marker=marker, ms=8, color=color,
            markeredgecolor="white", zorder=5,
        )
        # A point on the rail needs the label off the flat trace. A point near
        # zero stays beside the sample, so it is not read as a fault callout.
        on_rail = abs(v_mark) > 0.5 * rail
        if v_mark >= 0:
            dy = -28 if on_rail else 12
        else:
            dy = -12 if on_rail else 12
        name = "Vmax" if marker == "^" else "Vmin"
        ax_v.annotate(
            f"{name} {v_mark:+.2f} V @ {t_mark:.3f}s",
            xy=(t_mark, v_mark),
            xytext=(12, dy),
            textcoords="offset points",
            fontsize=8,
            color=color,
            va="top" if dy < 0 else "bottom",
            ha="left",
            arrowprops=dict(arrowstyle="-", color=color, lw=0.6),
            bbox=_label_bbox(color),
            zorder=7,
        )

    if m.v_max is not None:
        mark_extremum(m.v_max_t, m.v_max, "^")
    if m.v_min is not None:
        mark_extremum(m.v_min_t, m.v_min, "v")
    ax_v.legend(loc="upper right", fontsize=8, framealpha=0.92)

    left = min(min(t), -0.002)
    right = max(max(t), 0.05)
    span = max(right - left, 0.05)
    ax_pos.set_xlim(left - 0.03 * span, right + 0.06 * span)
    ax_v.set_xlim(ax_pos.get_xlim())

    rows, face = _panel_rows(m, phase, end_reason, None, spec=spec)
    _paint_panel(ax_info, rows, face)


def make_figure(plt):
    fig = plt.figure(figsize=(12.8, 7.4))
    gs = fig.add_gridspec(
        2, 2, width_ratios=[4.4, 1.85], height_ratios=[2.15, 1.05], wspace=0.22, hspace=0.32
    )
    ax_pos = fig.add_subplot(gs[0, 0])
    ax_v = fig.add_subplot(gs[1, 0])
    ax_info = fig.add_subplot(gs[:, 1])
    fig.subplots_adjust(left=0.055, right=0.985, top=0.90, bottom=0.08)
    fig.patch.set_facecolor("white")
    return fig, ax_pos, ax_v, ax_info


# ---------------------------------------------------------------------------
# Serial
# ---------------------------------------------------------------------------

def find_default_port(fallback: str = "COM5") -> str:
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "arduino" in desc or "uno" in desc or "usb serial" in desc or "vid:pid=2341" in hwid:
            return p.device
    if ports:
        return ports[0].device
    return fallback


class SerialReader:
    def __init__(self, port: str, baud: int, log: SessionLog | None = None):
        self.port = port
        self.baud = baud
        self.log = log
        self.queue: queue.Queue = queue.Queue()
        self.running = True
        self.paused = False
        self.status = "Starting..."
        self._conn = None
        self._bad = 0
        self._clock: str | None = None
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self.running = False
        self._close()

    def toggle_pause(self) -> None:
        self.paused = not self.paused
        if self.paused:
            self._close()
            self.status = "PAUSED — port released. Press Space to reconnect."
            print("\nPort released for upload. Press Space in the plot window to reconnect.")
        else:
            self.status = "Reconnecting..."
            print("\nReconnecting...")

    def _close(self) -> None:
        conn = self._conn
        self._conn = None
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass

    def _note_bad(self, line: str) -> None:
        self._bad += 1
        if self._bad <= 8:
            print(f"[serial] {line}")
        elif self._bad == 9:
            print("[serial] further non-numeric lines suppressed")

    def _handle_values(self, vals: list[float]) -> None:
        theta: float | None = None
        theta_ref: float | None = None
        if len(vals) == 6:
            t_ms, theta, theta_ref, volt, y, ref = vals
            clock = "device"
        elif len(vals) == 5:
            theta, theta_ref, volt, y, ref = vals
            t_ms = time.perf_counter() * 1000.0
            clock = "host"
        elif len(vals) == 4:
            t_ms, y, ref, volt = vals
            clock = "device"
        elif len(vals) == 3:
            y, ref, volt = vals
            t_ms = time.perf_counter() * 1000.0
            clock = "host"
        else:
            return
        if self._clock is None:
            self._clock = clock
            if clock == "host":
                print("Serial lines have no millis. Stamping each sample with the host clock.")
        if self.log is not None:
            self.log.write(t_ms, y, ref, volt, theta, theta_ref)
        self.queue.put(("sample", t_ms, y, ref, volt, theta, theta_ref))

    def _loop(self) -> None:
        import serial
        import serial.tools.list_ports

        while self.running:
            if self.paused:
                self._close()
                time.sleep(0.1)
                continue
            if self._conn is None or not self._conn.is_open:
                active = [p.device for p in serial.tools.list_ports.comports()]
                target = self.port if self.port in active else find_default_port(self.port)
                try:
                    self.status = f"Connecting to {target}..."
                    conn = serial.Serial(target, self.baud, timeout=0.05)
                    try:
                        conn.dtr = False
                        time.sleep(0.05)
                        conn.dtr = True
                        conn.reset_input_buffer()
                    except Exception:
                        pass
                    self._conn = conn
                    self.port = target
                    self._bad = 0
                    self.status = f"CONNECTED {target} @ {self.baud}"
                    print(f"\nConnected to {target} @ {self.baud} baud.")
                except (serial.SerialException, OSError, PermissionError):
                    self.status = f"WAITING FOR BOARD ({target})"
                    self._close()
                    time.sleep(0.5)
                    continue
            try:
                raw = self._conn.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                parts = [p.strip() for p in line.split(",") if p.strip()]
                vals: list[float] = []
                numeric = True
                for p in parts:
                    try:
                        vals.append(float(p))
                    except ValueError:
                        numeric = False
                        break
                if not numeric or not vals or not all(math.isfinite(v) for v in vals):
                    self._note_bad(line)
                    continue
                if len(vals) not in (3, 4, 5, 6):
                    self._note_bad(line)
                    continue
                self._bad = 0
                self._handle_values(vals)
            except (serial.SerialException, OSError, PermissionError) as exc:
                print(f"\nBoard disconnected ({exc}).")
                self._close()
                self.status = "DISCONNECTED — waiting for the board"
                self.queue.put(("disconnect",))
                time.sleep(0.5)


def print_banner(port: str, baud: int, spec: Spec, log_path: str) -> None:
    print(f"Step analyzer  {port} @ {baud}")
    print(f"Full log: {log_path}")
    print("Line: theta_rad, theta_ref_rad, voltage_V, ball_pos_m, ball_ref_m   (leading millis optional)")
    print(
        f"Skip the first step, then record {spec.ref_a:+.3f} <-> {spec.ref_b:+.3f} m "
        f"(dir={spec.step_dir}) once steady."
    )
    print(
        f"Checks: bounded, [{spec.ref_a - spec.ess_tol_a:.3f},{spec.ref_a + spec.ess_tol_a:.3f}] / "
        f"[{spec.ref_b - spec.ess_tol_b:.3f},{spec.ref_b + spec.ess_tol_b:.3f}] m, "
        f"t_s<={spec.ts_max:.3f}s, OS<{spec.os_max_pct:.0f}%, "
        f"V within +{spec.v_pos:.1f}/-{spec.v_neg:.1f}"
    )
    print("Keys: M freeze   N next step   D dir (both/pos/neg)   Space release port (DTR reset on connect)")
    print()


def replay_file(path: str, spec: Spec) -> None:
    import matplotlib.pyplot as plt

    samples = load_samples(path)
    print(f"Replaying {len(samples)} samples from {path}")
    if not samples:
        return
    capture = StepCapture(spec)
    for t_ms, y, ref, volt in samples:
        if capture.push(t_ms, y, ref, volt) == "end":
            break
    if capture.phase == "live":
        capture.freeze("end of file")
    if capture.step is not None:
        finalize_capture(capture, "REPLAY")
    else:
        print(capture.hold_note)

    fig, ax_pos, ax_v, ax_info = make_figure(plt)
    try:
        fig.canvas.manager.set_window_title("Step response replay")
    except Exception:
        pass
    m = capture.metrics()
    if capture.step is not None and m is not None:
        draw_step(
            ax_pos, ax_v, ax_info, capture.step, m, "frozen",
            f"REPLAY  {os.path.basename(path)}", spec, capture.end_reason,
        )
    else:
        draw_waiting(
            ax_pos, ax_v, ax_info, list(capture.preview), capture.last_ref,
            f"REPLAY  {os.path.basename(path)}", capture.hold_note, spec.v_pos, spec.v_neg,
        )
    plt.show()


def run_plotter(port: str, baud: int, spec: Spec) -> None:
    import matplotlib.pyplot as plt
    from matplotlib import animation

    log = SessionLog()
    print_banner(port, baud, spec, log.path)
    reader = SerialReader(port, baud, log)
    capture = StepCapture(spec)
    reader.start()

    fig, ax_pos, ax_v, ax_info = make_figure(plt)
    try:
        fig.canvas.manager.set_window_title("Step response analyzer")
    except Exception:
        pass

    state = {"sig": None, "last_line": 0.0}

    def suptitle() -> str:
        conn = reader.status
        if capture.phase == "live":
            return f"LIVE ({spec.step_dir})    {conn}    [M] freeze    [Space] release port"
        if capture.phase == "frozen":
            why = capture.end_reason or "frozen"
            return f"FROZEN ({why})    [N] next step    [D] dir:{spec.step_dir}    [Space] release port"
        return f"[{spec.step_dir}] {capture.hold_note}    {conn}"

    def update(_frame):
        try:
            ended = False
            started = None
            while True:
                try:
                    item = reader.queue.get_nowait()
                except queue.Empty:
                    break
                if item[0] == "disconnect":
                    if capture.freeze("serial disconnected"):
                        ended = True
                    continue
                ev = capture.push(item[1], item[2], item[3], item[4], item[5], item[6])
                if ev == "start" and capture.step is not None:
                    started = (capture.step["index"], capture.step["r0"], capture.step["r1"])
                elif ev == "end":
                    ended = True

            if started is not None:
                idx, r0, r1 = started
                print(f"\nStep {idx} started: y_ref {r0:+.3f} -> {r1:+.3f} m")
                state["sig"] = None
            if ended:
                finalize_capture(capture, "FINAL")
                state["sig"] = None

            m = capture.metrics()
            title = suptitle()
            if capture.phase == "wait" or capture.step is None or m is None:
                if state.get("note") != capture.hold_note:
                    print("\n" + capture.hold_note)
                    state["note"] = capture.hold_note
                draw_waiting(
                    ax_pos, ax_v, ax_info, list(capture.preview), capture.last_ref,
                    title, capture.hold_note, spec.v_pos, spec.v_neg,
                )
            else:
                draw_step(
                    ax_pos, ax_v, ax_info, capture.step, m, capture.phase, title, spec, capture.end_reason
                )
                if capture.phase == "live":
                    now = time.monotonic()
                    if m.ss_confident:
                        sig = _signature(m)
                        if sig != state["sig"]:
                            emit_report("LIVE  (these numbers can still move)", format_report(
                                m, spec, "still collecting", capture.step["index"]
                            ))
                            state["sig"] = sig
                            state["last_line"] = now
                    if now - state["last_line"] > 0.2:
                        sys.stdout.write("\r" + oneline(m).ljust(118))
                        sys.stdout.flush()
                        state["last_line"] = now
        except Exception:
            traceback.print_exc()
        return []

    def on_key(event):
        if event.key == " ":
            reader.toggle_pause()
        elif event.key in ("m", "M"):
            if capture.freeze("frozen by user"):
                finalize_capture(capture, "FINAL  frozen by user")
            else:
                print("\nNothing in progress to freeze.")
        elif event.key in ("n", "N"):
            if capture.phase == "live":
                print("\nPress M to freeze this step before capturing another.")
                return
            capture.arm()
            state["sig"] = None
            state["note"] = capture.hold_note
            print("\n" + capture.hold_note)
        elif event.key in ("d", "D"):
            order = ("both", "pos", "neg")
            spec.step_dir = order[(order.index(spec.step_dir) + 1) % len(order)]
            print(f"\nStep direction: {spec.step_dir}")
            if capture.phase == "wait":
                capture.dwell.clear()
                capture.pre.clear()
                capture.ready = False
                capture.dwell_target = None
                capture.hold_note = capture._wait_prompt()
                state["note"] = capture.hold_note

    def on_close(_event):
        if capture.phase == "live":
            capture.freeze("window closed")
            finalize_capture(capture, "FINAL  window closed")
        reader.stop()

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("close_event", on_close)
    ani = animation.FuncAnimation(fig, update, interval=33, blit=False, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        reader._thread.join(timeout=1.0)
        log.close()
        del ani


def main() -> None:
    parser = argparse.ArgumentParser(description="Live one-step response analyzer for the ball-beam outer loop.")
    parser.add_argument("--port", default=None, help="Serial port (default: auto-detect, else COM5)")
    parser.add_argument("--file", default=None, help="replay a full log CSV from logs/ instead of the serial port")
    parser.add_argument("--baud", type=int, default=230400, help="Baud rate (default: 230400)")
    parser.add_argument("--ref-a", type=float, default=LAB_REF_A_M, help="m, low step reference position")
    parser.add_argument("--ref-b", type=float, default=LAB_REF_B_M, help="m, high step reference position")
    parser.add_argument(
        "--step-dir",
        choices=("both", "pos", "neg"),
        default="both",
        help="which step direction to capture (default: both)",
    )
    parser.add_argument("--ess-tol", type=float, default=None, help="m, override steady-state band half-width for both targets")
    parser.add_argument("--ts-max", type=float, default=LAB_TS_MAX_S, help="seconds, target-band settling-time limit")
    parser.add_argument("--os-max", type=float, default=LAB_OS_MAX_PCT, help="percent overshoot limit")
    parser.add_argument("--v-pos", type=float, default=None, help="positive voltage rail, volts")
    parser.add_argument("--v-neg", type=float, default=None, help="negative voltage rail magnitude, volts")
    parser.add_argument("--v-limit", type=float, default=None, help="set both voltage rails")
    parser.add_argument("--ss-std", type=float, default=SS_STD_MAX_M, help="m, max tail std to call steady state")
    parser.add_argument("--tail", type=float, default=TAIL_S, help="seconds, steady-state window")
    parser.add_argument("--self-test", action="store_true", help="check the metric definitions and exit")
    args = parser.parse_args()

    if args.self_test:
        from step_response_selftest import self_test
        self_test()
        return

    spec = Spec(
        ref_a=args.ref_a,
        ref_b=args.ref_b,
        step_dir=args.step_dir,
        ess_tol_a=args.ess_tol if args.ess_tol is not None else LAB_ESS_TOL_A_M,
        ess_tol_b=args.ess_tol if args.ess_tol is not None else LAB_ESS_TOL_B_M,
        ts_max=args.ts_max,
        os_max_pct=args.os_max,
        v_pos=args.v_pos if args.v_pos is not None else (args.v_limit if args.v_limit is not None else LAB_V_POS_V),
        v_neg=args.v_neg if args.v_neg is not None else (args.v_limit if args.v_limit is not None else LAB_V_NEG_V),
        ss_std_max=args.ss_std,
        tail_s=args.tail,
    )
    if args.file:
        try:
            import matplotlib.pyplot  # noqa: F401
        except ImportError:
            print("This script needs matplotlib:  pip install matplotlib")
            raise
        try:
            replay_file(resolve_log_path(args.file), spec)
        except (FileNotFoundError, ValueError, OSError) as exc:
            print(exc)
            raise SystemExit(1) from exc
        return
    try:
        import serial  # noqa: F401
        import matplotlib.pyplot  # noqa: F401
    except ImportError:
        print("This script needs pyserial and matplotlib:  pip install pyserial matplotlib")
        raise
    port = args.port if args.port else find_default_port()
    run_plotter(port, args.baud, spec)


if __name__ == "__main__":
    main()
