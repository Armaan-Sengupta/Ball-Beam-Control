"""Live one-step analyzer for the auto-tuner firmware.

Select the PlatformIO environment "auto_tuner", then:

    python step_response_analyzer.py
    python step_response_analyzer.py --a1 -0.914329 --b1 -2.702603 --b0 2.270448
    python step_response_analyzer.py --file logs/full_YYYYMMDD_HHMMSS.csv

Each trial sends "C a1 b1 b0" and the moves on the already-open port, so
another trial does not reset the board. Press R in the plot window to run it
again. The sequence is: move to -0.7, wait for it to settle, start recording,
wait a short beat, then command +0.7.

a1 is the coefficient of u[k-1] (the sweep sketch's a0_d):
    u[k] = b1*e[k] + b0*e[k-1] - a1*u[k-1]
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

# Lab 2 inner-loop defaults. Override from the command line.
LAB_TS_MAX_S = 0.25
LAB_OS_MAX_PCT = 5.0
LAB_V_LIMIT_V = 6.0
LAB_ESS_TOL_RAD = 0.01  # tail-mean uncertainty is ~0.001 rad, so this is above the noise
LAB_REF_A_RAD = -0.7
LAB_REF_B_RAD = 0.7

# Sweep-sketch discrete controller. a1 multiplies u[k-1] (that file's a0_d).
DEFAULT_A1 = -0.914329121458805
DEFAULT_B1 = -2.702602561764911
DEFAULT_B0 = 2.270448497190859
SETTLE_S = 1.0
PRE_DELAY_S = 0.10

# Knobs the pot noise and the 500 ms dwell actually move. Edit these first.
#
# Sheet "MTE 484 Lab 1 Data":
#   "0(deg)", motor still, 120 samples: std 0.0092 rad (25 counts),
#     peak-to-peak 0.056 rad. One count is 0.00036 rad. One sample of 120
#     sat outside +/-0.028 rad of the mean. A 5-sample median cuts the
#     peak-to-peak to 0.022 rad.
#   "kp18 (2ms)", settled at +/-0.1 rad: std 0.006 rad, peak-to-peak 0.036 rad.
#     Over 100 ms the std stays <= 0.0066, |slope| <= 0.056 rad/s, and the
#     drift between adjacent windows <= 0.0026 rad. The mean is within
#     0.001 rad of the reference. No sample left +/-0.028 rad of the mean.
# Each target is held for 500 ms, so two flat tails have to fit after the
# transient. 0.08 s + 0.08 s leaves about 0.09 s once a 0.25 s settle is done.
TAIL_S = 0.08
SS_STD_MAX_RAD = 0.020       # above the 0.009 rad static std; a 0.03 rad swing still fails
SS_SLOPE_MAX = 0.30          # rad/s; static noise on an 80 ms fit is ~0.2 rad/s
SS_DRIFT_MAX_RAD = 0.012     # closed-loop window-to-window drift was <= 0.003 rad
AT_TARGET_RAD = 0.03         # tail mean this close to -0.7 counts as arrived
SETTLE_MEDIAN_N = 5          # 10 ms; drops the single-count spikes in the 2% test


@dataclass
class Spec:
    settle_frac: float = 0.02
    ts_max: float = LAB_TS_MAX_S
    os_max_pct: float = LAB_OS_MAX_PCT
    v_limit: float = LAB_V_LIMIT_V
    ess_tol: float = LAB_ESS_TOL_RAD
    tail_s: float = TAIL_S
    ss_std_max: float = SS_STD_MAX_RAD
    ss_slope_max: float = SS_SLOPE_MAX
    ss_drift_max: float = SS_DRIFT_MAX_RAD
    at_target_rad: float = AT_TARGET_RAD
    settle_median_n: int = SETTLE_MEDIAN_N
    min_move_frac: float = 0.10
    min_move_rad: float = 0.02
    still_time_s: float = 0.30
    step_delta: float = 0.02
    ref_a: float = LAB_REF_A_RAD
    ref_b: float = LAB_REF_B_RAD
    ref_tol: float = 0.05
    runaway_abs_rad: float = 2.5
    growth_ratio: float = 1.15
    growth_floor_rad: float = 0.05
    bucket_s: float = 0.10


@dataclass
class SteadyState:
    confident: bool
    moved: bool
    reason: str
    theta_ss: float | None = None
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
    def theta_ss(self) -> float | None:
        return self.ss.theta_ss


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


def _voltage(t, v, limit):
    imax = 0
    imin = 0
    viol_t: list[float] = []
    viol_v: list[float] = []
    for i, vi in enumerate(v):
        if vi > v[imax]:
            imax = i
        if vi < v[imin]:
            imin = i
        if vi > limit or vi < -limit:
            viol_t.append(t[i])
            viol_v.append(vi)
    return v[imin], t[imin], v[imax], t[imax], viol_t, viol_v


def _stability(t, theta, y0, r1, spec: Spec):
    if not t:
        return "INCONCLUSIVE", "no samples", []
    step_amp = abs(r1 - y0)
    for ti, yi in zip(t, theta):
        if abs(yi) > spec.runaway_abs_rad or abs(yi - r1) > step_amp + 1.0:
            return (
                "UNSTABLE",
                f"runaway: theta={yi:.3f} rad at t={ti:.3f} s",
                [],
            )
    span = t[-1] - t[0]
    if span < spec.bucket_s:
        return "INCONCLUSIVE", "less than 0.10 s of step data", []

    n_buckets = int(math.floor(span / spec.bucket_s + 1e-9))
    bucket_peak: list[float | None] = []
    bucket_mark: list[tuple[float, float] | None] = []
    for b in range(n_buckets):
        lo = t[0] + b * spec.bucket_s
        hi = lo + spec.bucket_s
        best_e = None
        best = None
        for ti, yi in zip(t, theta):
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
                and last[2] > spec.growth_floor_rad
            )
            if growing:
                nums = ", ".join(f"{p:.3f}" for p in last)
                reason = f"tracking-error peaks growing over the last 0.30 s ({nums} rad)"
                return "UNSTABLE", reason, [m for m in marks if m is not None]
    return "STABLE", "bounded; tracking-error peaks are not growing", []


def _steady_state(t, theta, y0, r1, spec: Spec) -> SteadyState:
    elapsed = t[-1] - t[0] if t else 0.0
    need = 2.0 * spec.tail_s
    if not t or elapsed < need:
        return SteadyState(False, False, f"need {need:.2f} s before the tail can be called flat")

    t_end = t[-1]
    tail_t, tail_y = _window(t, theta, t_end - spec.tail_s, t_end, right_open=False)
    prev_t, prev_y = _window(t, theta, t_end - 2 * spec.tail_s, t_end - spec.tail_s, right_open=True)
    if len(tail_y) < 5 or len(prev_y) < 5:
        return SteadyState(False, False, "not enough samples in the tail")
    if tail_t[-1] - tail_t[0] < 0.6 * spec.tail_s:
        return SteadyState(False, False, "tail window is too short")

    theta_ss = _mean(tail_y)
    std = _stdev(tail_y)
    slope = _slope(tail_t, tail_y)
    drift = abs(theta_ss - _mean(prev_y))
    flat = std <= spec.ss_std_max and abs(slope) <= spec.ss_slope_max and drift <= spec.ss_drift_max
    move = abs(theta_ss - y0)
    min_move = max(spec.min_move_rad, spec.min_move_frac * abs(r1 - y0))
    moved = move >= min_move
    base = SteadyState(
        confident=False,
        moved=moved,
        reason="",
        theta_ss=theta_ss,
        std=std,
        slope=slope,
        t0=tail_t[0],
        t1=tail_t[-1],
    )
    if not flat:
        base.reason = (
            f"tail not flat (std {std:.4f} rad, slope {slope:+.3f} rad/s, "
            f"drift {drift:.4f} rad)"
        )
        base.moved = False
        return base
    if not moved and elapsed < spec.still_time_s:
        base.reason = "still at the initial angle"
        base.moved = False
        return base
    base.confident = True
    base.moved = moved
    if moved:
        base.reason = "flat tail"
    else:
        base.reason = "flat, but the angle never left the initial value"
    return base


def _trailing_median(theta, n: int) -> list[float]:
    """Median of the last n samples at each index. n <= 1 returns the raw series."""
    if n <= 1:
        return list(theta)
    out: list[float] = []
    for i in range(len(theta)):
        out.append(_median(theta[max(0, i - n + 1) : i + 1]))
    return out


def _settling(t, theta, theta_ss, half, median_n: int = 1):
    """(t_s, median_at_t_s, ends_outside). A trailing median ignores one-sample spikes."""
    judged = _trailing_median(theta, median_n)
    last_out = None
    for i, yi in enumerate(judged):
        if abs(yi - theta_ss) > half:
            last_out = i
    if last_out is None:
        return t[0], judged[0], False
    if last_out == len(judged) - 1:
        return None, None, True
    i = last_out + 1
    return t[i], judged[i], False


def _overshoot(t, theta, y0, theta_ss):
    move = theta_ss - y0
    if move == 0:
        return None
    sigma = 1.0 if move > 0 else -1.0
    best_i = 0
    best = -1e300
    for i, yi in enumerate(theta):
        exc = sigma * (yi - theta_ss)
        if exc > best:
            best = exc
            best_i = i
    excursion = max(0.0, best)
    pct = excursion / abs(move) * 100.0
    return pct, t[best_i], theta[best_i]


def compute_metrics(t, theta, voltage, y0, r0, r1, spec: Spec) -> Metrics | None:
    if not t:
        return None
    v_min, v_min_t, v_max, v_max_t, viol_t, viol_v = _voltage(t, voltage, spec.v_limit)
    stability, stability_reason, growth_marks = _stability(t, theta, y0, r1, spec)
    ss = _steady_state(t, theta, y0, r1, spec)

    e_ss = None
    band_half = None
    ts = ts_y = None
    outside_now = False
    ts_note = "waiting for a flat steady state"
    os_pct = peak_t = peak_y = None
    os_note = "waiting for a flat steady state"
    pass_ess = pass_ts = pass_os = None

    if ss.confident and ss.theta_ss is not None:
        e_ss = r1 - ss.theta_ss
        pass_ess = abs(e_ss) <= spec.ess_tol
        if ss.moved:
            move = abs(ss.theta_ss - y0)
            band_half = spec.settle_frac * move
            ts, ts_y, outside_now = _settling(
                t, theta, ss.theta_ss, band_half, spec.settle_median_n
            )
            elapsed = t[-1] - t[0]
            if outside_now:
                ts_note = f"still outside the band at t={elapsed:.3f} s"
                pass_ts = False if elapsed > spec.ts_max else None
            elif spec.settle_median_n > 1:
                ts_note = (
                    f"first sample whose trailing {spec.settle_median_n}-sample "
                    "median is inside the band and stays inside"
                )
                pass_ts = ts is not None and ts <= spec.ts_max
            else:
                ts_note = "first sample inside the band that stays inside"
                pass_ts = ts is not None and ts <= spec.ts_max
            ov = _overshoot(t, theta, y0, ss.theta_ss)
            if ov is not None:
                os_pct, peak_t, peak_y = ov
                os_note = "peak past theta_ss, in the direction of the step"
                pass_os = os_pct <= spec.os_max_pct
        else:
            ts_note = "angle never left the initial value"
            os_note = "angle never left the initial value"

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
        r(m.e_ss, 3),
        r(m.ts, 3),
        r(m.overshoot_pct, 1),
        r(m.theta_ss, 3),
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
        f"t={m.elapsed:5.3f}s {stab} {_word(m.pass_stable)}  "
        f"ess={ess} {_word(m.pass_ess)}  "
        f"Ts={ts:>7} {_word(m.pass_ts)}  "
        f"OS={osp} {_word(m.pass_os)}  "
        f"V={vmax}/{vmin} {_word(m.pass_v)}"
    )


def format_report(m: Metrics, spec: Spec, end_reason: str | None, step_index: int) -> str:
    dt = "unknown" if m.dt_ms is None else f"{m.dt_ms:.2f} ms"
    lines = [
        f"Step {step_index}: theta_ref {m.r0:+.4f} -> {m.r1:+.4f} rad"
        + ("" if m.lab_step else "  (not -0.7 -> +0.7)"),
        f"theta(0-) = {m.y0:+.4f} rad    {m.elapsed:.3f} s, {m.n_post} samples, dt {dt}",
    ]
    if m.dt_ms is not None and (m.dt_ms > 15.0 or m.dt_ms < 0.4):
        lines.append("WARNING: sample interval is far from the 2 ms loop. Check baud 230400.")
    if end_reason:
        lines.append(f"Capture: {end_reason}")
    lines.append(f"Stability: {m.stability} {_word(m.pass_stable)}  {m.stability_reason}")
    if m.e_ss is None:
        lines.append(f"e_ss: {_word(m.pass_ess)}  {m.ss.reason}")
    else:
        lines.append(
            f"e_ss = {m.e_ss:+.4f} rad {_word(m.pass_ess)}  "
            f"theta_ss={m.theta_ss:+.4f} over [{m.ss.t0:.3f}, {m.ss.t1:.3f}] s  "
            f"std {m.ss.std:.4f} slope {m.ss.slope:+.3f}  |e_ss|<={spec.ess_tol:.3f}"
        )
    if m.ts is not None and m.band_half is not None:
        lines.append(
            f"t_s = {m.ts:.3f} s {_word(m.pass_ts)}  "
            f"band +/-{m.band_half:.4f} rad  <= {spec.ts_max:.3f} s  {m.ts_note}"
        )
    else:
        lines.append(f"t_s: {_word(m.pass_ts)}  {m.ts_note}  <= {spec.ts_max:.3f} s")
    if m.overshoot_pct is not None:
        lines.append(
            f"OS = {m.overshoot_pct:.2f}% {_word(m.pass_os)}  "
            f"peak {m.peak_y:+.4f} at {m.peak_t:.3f} s  <= {spec.os_max_pct:.1f}%"
        )
    else:
        lines.append(f"OS: {_word(m.pass_os)}  {m.os_note}  <= {spec.os_max_pct:.1f}%")
    voltage = f"Voltage: {_word(m.pass_v)}  outside +/-{spec.v_limit:.1f} V: {len(m.viol_t)}"
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
    """Turns a stream of (t_ms, theta, ref, voltage) into one step record."""

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
        # a capture starts only from a settled -0.7 dwell into +0.7.
        self.skipped_first = False
        self.ready = False
        self.commanded = False
        self.dwell: deque = deque()
        self.hold_note = "waiting for the auto-tuner to prime -0.7"

    def command_arm(self) -> None:
        """Record the next -0.7 -> +0.7 edge. The host sends that edge."""
        self.phase = "wait"
        self.step = None
        self.pre.clear()
        self.dwell.clear()
        self.ready = True
        self.skipped_first = True
        self.commanded = True
        self.end_reason = None
        self.saved = False
        self.capped = False
        self.hold_note = (
            f"recording; step to {self.spec.ref_b:+.3f} after a short delay"
        )

    def arm(self) -> None:
        """Arm another -0.7 -> +0.7 capture without skipping an extra step."""
        self.phase = "wait"
        self.step = None
        self.pre.clear()
        self.preview.clear()
        self.dwell.clear()
        self.ready = False
        self.skipped_first = True
        self.commanded = False
        self.end_reason = None
        self.saved = False
        self.capped = False
        self.hold_note = (
            f"waiting for a steady {self.spec.ref_a:+.3f}, "
            f"then the step to {self.spec.ref_b:+.3f}"
        )

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
            self.step["theta"][n:],
            self.step["v"][n:],
            self.step["y0"],
            self.step["r0"],
            self.step["r1"],
            self.spec,
        )

    def _trim_preview(self, t_ms: float) -> None:
        while self.preview and t_ms - self.preview[0][0] > 1500.0:
            self.preview.popleft()

    def _trim_pre(self, t_ms: float) -> None:
        while self.pre and t_ms - self.pre[0][0] > 80.0:
            self.pre.popleft()

    def _begin(self, t_ms: float, theta: float, ref: float, voltage: float) -> None:
        cands = [th for tm, th, _rf, _vv in self.pre if 0.0 < (t_ms - tm) <= 40.0]
        if not cands and self.pre:
            cands = [self.pre[-1][1]]
        y0 = _median(cands) if cands else theta
        r0 = self.last_ref if self.last_ref is not None else ref

        t_list: list[float] = []
        th_list: list[float] = []
        rf_list: list[float] = []
        v_list: list[float] = []
        for tm, th, rf, vv in self.pre:
            dt = (tm - t_ms) / 1000.0
            if -0.05 <= dt < 0.0:
                t_list.append(dt)
                th_list.append(th)
                rf_list.append(rf)
                v_list.append(vv)
        n_pre = len(t_list)
        t_list.append(0.0)
        th_list.append(theta)
        rf_list.append(ref)
        v_list.append(voltage)

        self.step = {
            "index": self.next_index,
            "r0": r0,
            "r1": ref,
            "y0": y0,
            "t0_ms": t_ms,
            "t": t_list,
            "theta": th_list,
            "ref": rf_list,
            "v": v_list,
            "n_pre": n_pre,
            "started_wall": datetime.now(),
        }
        self.next_index += 1
        self.phase = "live"
        self.end_reason = None
        self.saved = False
        self.capped = False
        self.pre.clear()

    def _append(self, t_ms: float, theta: float, ref: float, voltage: float) -> bool:
        t_rel = (t_ms - self.step["t0_ms"]) / 1000.0
        if t_rel > 120.0:
            return False
        self.step["t"].append(t_rel)
        self.step["theta"].append(theta)
        self.step["ref"].append(ref)
        self.step["v"].append(voltage)
        return True

    def push(self, t_ms: float, theta: float, ref: float, voltage: float) -> str | None:
        jump = self.last_t_ms is not None and t_ms < self.last_t_ms - 500.0
        if jump and self.phase == "live" and self.step is not None and len(self.step["t"]) > self.step["n_pre"]:
            self.last_t_ms = t_ms
            self.last_ref = ref
            self.preview.clear()
            self.preview.append((t_ms, theta, ref, voltage))
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
        self.preview.append((t_ms, theta, ref, voltage))
        self._trim_preview(t_ms)

        if self.phase == "frozen":
            self.last_ref = ref
            return None

        if self.phase == "wait":
            return self._push_wait(t_ms, theta, ref, voltage)

        if abs(ref - self.step["r1"]) >= self.spec.step_delta:
            self.phase = "frozen"
            self.end_reason = "reference stepped away"
            self.last_ref = ref
            return "end"
        if not self._append(t_ms, theta, ref, voltage) and not self.capped:
            self.capped = True
            print("\nCapture reached 120 s and is holding the buffer. Press M to freeze.")
        self.last_ref = ref
        return "sample"

    def _near(self, ref: float, target: float) -> bool:
        return abs(ref - target) <= self.spec.ref_tol

    def _low_is_steady(self) -> tuple[bool, str]:
        """True once the recent dwell at ref_a is flat and the angle is there."""
        spec = self.spec
        target = spec.ref_a
        if len(self.dwell) < 10:
            return False, f"holding {target:+.3f}, collecting samples"
        t1 = self.dwell[-1][0]
        tail_ms = spec.tail_s * 1000.0
        tail = [(tm, th) for tm, th in self.dwell if tm >= t1 - tail_ms]
        prev = [(tm, th) for tm, th in self.dwell if t1 - 2.0 * tail_ms <= tm < t1 - tail_ms]
        if len(tail) < 5 or len(prev) < 5 or tail[-1][0] - tail[0][0] < 0.6 * tail_ms:
            return False, f"holding {target:+.3f}, waiting for the angle to settle"

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
            return False, f"at {target:+.3f}, angle still moving"
        if abs(tail_mean - target) > spec.at_target_rad:
            return False, f"flat at {tail_mean:+.3f}, waiting until the angle is at {target:+.3f}"
        return True, f"steady at {target:+.3f}"

    def _push_wait(self, t_ms: float, theta: float, ref: float, voltage: float) -> str | None:
        changed = self.last_ref is not None and abs(ref - self.last_ref) >= self.spec.step_delta
        low = self.spec.ref_a
        high = self.spec.ref_b

        if self.commanded:
            if (
                changed
                and self.ready
                and self._near(self.last_ref, low)
                and self._near(ref, high)
            ):
                self._begin(t_ms, theta, ref, voltage)
                self.last_ref = ref
                self.ready = False
                self.commanded = False
                self.hold_note = "recording"
                return "start"
            if self._near(ref, low):
                self.pre.append((t_ms, theta, ref, voltage))
                self._trim_pre(t_ms)
                self.ready = True
                self.hold_note = f"recording at {low:+.3f}; stepping to {high:+.3f} shortly"
            else:
                self.hold_note = f"recording armed; theta_ref is {ref:+.3f}"
            self.last_ref = ref
            return None

        if changed and not self.skipped_first:
            self.skipped_first = True
            self.ready = False
            self.dwell.clear()
            self.pre.clear()
            self.last_ref = ref
            self.hold_note = (
                f"skipped the first step; waiting for a steady {low:+.3f} "
                f"before recording {low:+.3f} -> {high:+.3f}"
            )
            return "skip"

        if (
            changed
            and self.ready
            and self._near(self.last_ref, low)
            and self._near(ref, high)
        ):
            self._begin(t_ms, theta, ref, voltage)
            self.last_ref = ref
            self.ready = False
            self.dwell.clear()
            self.hold_note = "recording"
            return "start"

        if changed:
            self.ready = False
            self.dwell.clear()
            self.pre.clear()

        if self._near(ref, low):
            self.pre.append((t_ms, theta, ref, voltage))
            self._trim_pre(t_ms)
            self.dwell.append((t_ms, theta))
            while self.dwell and t_ms - self.dwell[0][0] > 500.0:
                self.dwell.popleft()
            steady, why = self._low_is_steady()
            self.ready = self.skipped_first and steady
            if not self.skipped_first:
                self.hold_note = "skipping the first reference step"
            elif self.ready:
                self.hold_note = f"steady at {low:+.3f}; recording starts on the step to {high:+.3f}"
            else:
                self.hold_note = why
        else:
            self.ready = False
            self.dwell.clear()
            self.pre.clear()
            if not self.skipped_first:
                self.hold_note = "skipping the first reference step"
            elif self._near(ref, high):
                self.hold_note = f"at {high:+.3f}; waiting for the return to {low:+.3f}"
            else:
                self.hold_note = f"theta_ref {ref:+.3f}; waiting for {low:+.3f}"

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
        self._f.write("millis_ms,theta_rad,theta_ref_rad,voltage_V\n")
        self._lock = threading.Lock()
        self._n = 0
        self._warned = False

    def write(self, t_ms: float, theta: float, ref: float, volt: float) -> None:
        line = f"{t_ms:.0f},{theta:.5f},{ref:.4f},{volt:.4f}\n"
        with self._lock:
            if self._f is None:
                return
            try:
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
    """Rows of (millis_ms, theta, theta_ref, voltage). time_s columns become ms."""
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    if not rows:
        return []
    first = [c.strip() for c in rows[0]]
    start = 0
    idx = (0, 1, 2, 3)
    scale_t = 1.0
    try:
        [_cell(first, i) for i in range(len(first)) if first[i]]
        headerless = True
    except ValueError:
        headerless = False
    if not headerless:
        names = [c.lower() for c in first]
        start = 1

        def col(*options: str) -> int | None:
            for name in options:
                if name in names:
                    return names.index(name)
            return None

        t_i = col("millis_ms", "time_ms", "millis", "time_s")
        th_i = col("theta_rad", "theta")
        r_i = col("theta_ref_rad", "theta_ref")
        v_i = col("voltage_v", "voltage")
        if None in (t_i, th_i, r_i, v_i):
            raise ValueError(f"Unrecognized columns in {path}: {', '.join(first)}")
        idx = (t_i, th_i, r_i, v_i)
        if names[t_i] == "time_s":
            scale_t = 1000.0
    samples: list[tuple[float, float, float, float]] = []
    for row in rows[start:]:
        if not row or not any(c.strip() for c in row):
            continue
        try:
            samples.append((
                _cell(row, idx[0]) * scale_t,
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
    with open(csv_path, "w", encoding="utf-8", newline="\n") as f:
        f.write("time_s,theta_rad,theta_ref_rad,voltage_V\n")
        for ti, yi, ri, vi in zip(step["t"], step["theta"], step["ref"], step["v"]):
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

ANGLE_C = "#1f77b4"
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


def _panel_rows(m: Metrics | None, phase: str, end_reason: str | None, current_ref: float | None, note: str = ""):
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
            add(f"theta_ref = {current_ref:+.3f} rad", MUTED, 8)
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
    ref_line = f"{m.r0:+.3f} -> {m.r1:+.3f} rad"
    if not m.lab_step:
        ref_line += "  (not +/-0.7)"
    add(ref_line, "#222222", 8)
    dt = "" if m.dt_ms is None else f"   dt {m.dt_ms:.2f} ms"
    dt_color = FAIL_C if (m.dt_ms is not None and (m.dt_ms > 15.0 or m.dt_ms < 0.4)) else MUTED
    add(f"t={m.elapsed:.3f}s  N={m.n_post}{dt}", dt_color, 8)
    if end_reason and phase == "frozen":
        add(textwrap.shorten(end_reason, width=36, placeholder="..."), MUTED, 8)
    gap()
    add(f"theta(0-) = {m.y0:+.4f}", "#222222", 8)
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
        verdict_line(f"e_ss {m.e_ss:+.4f}", m.pass_ess)
        add(f"theta_ss = {m.theta_ss:+.4f}", "#222222", 8)
    gap()
    if m.ts is not None:
        verdict_line(f"t_s {m.ts:.3f}s", m.pass_ts)
        add(f"+/-{m.band_half:.4f} rad", MUTED, 8)
    else:
        verdict_line("t_s", m.pass_ts)
        for line in textwrap.wrap(m.ts_note, width=36)[:2]:
            add(line, MUTED, 8)
    gap()
    if m.overshoot_pct is not None:
        verdict_line(f"OS {m.overshoot_pct:.1f}%", m.pass_os)
        add(f"peak {m.peak_y:+.3f} @ {m.peak_t:.3f}s", MUTED, 8)
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


def _style_angle(ax) -> None:
    ax.set_ylabel("angle (rad)")
    ax.set_title("Angle", loc="left", fontsize=10, color="#333333")
    ax.grid(True, linestyle="--", alpha=0.55)
    ax.tick_params(labelbottom=False)


def _style_voltage(ax) -> None:
    ax.set_ylabel("logged voltage (V)")
    ax.set_xlabel("time from reference step (s)")
    ax.set_title("Logged voltage", loc="left", fontsize=10, color="#333333")
    ax.grid(True, linestyle="--", alpha=0.55)


def draw_waiting(ax_ang, ax_v, ax_info, preview, current_ref, suptitle: str, note: str = "") -> None:
    fig = ax_ang.figure
    ax_ang.cla()
    ax_v.cla()
    _style_angle(ax_ang)
    _style_voltage(ax_v)
    fig.suptitle(suptitle, fontsize=11)
    if preview:
        t_now = preview[-1][0]
        ts = [(p[0] - t_now) / 1000.0 for p in preview]
        theta = [p[1] for p in preview]
        ref = [p[2] for p in preview]
        volt = [p[3] for p in preview]
        ax_ang.plot(ts, theta, color=ANGLE_C, lw=1.6, label=r"$\theta$")
        ax_ang.plot(ts, ref, color=REF_C, lw=1.4, drawstyle="steps-post", label=r"$\theta_{ref}$")
        ax_ang.legend(loc="upper left", fontsize=8, framealpha=0.92)
        ax_v.plot(ts, volt, color=VOLT_C, lw=1.4, label="logged voltage")
        ax_v.axhline(LAB_V_LIMIT_V, color=LIMIT_C, ls="--", lw=1.0, label=r"$\pm$6 V limit")
        ax_v.axhline(-LAB_V_LIMIT_V, color=LIMIT_C, ls="--", lw=1.0)
        ax_v.legend(loc="upper left", fontsize=8, framealpha=0.92)
        ax_v.set_xlim(min(ts), 0.0)
        ymin = min(-7.5, min(volt) - 0.4)
        ymax = max(7.5, max(volt) + 0.4)
        ax_v.set_ylim(ymin, ymax)
        ax_v.set_xlabel("time (s), right edge is now")
    rows, face = _panel_rows(None, "wait", None, current_ref, note)
    _paint_panel(ax_info, rows, face)


def draw_step(
    ax_ang, ax_v, ax_info, step: dict, m: Metrics, phase: str, suptitle: str,
    spec: Spec | None = None, end_reason: str | None = None,
) -> None:
    fig = ax_ang.figure
    ax_ang.cla()
    ax_v.cla()
    _style_angle(ax_ang)
    _style_voltage(ax_v)
    fig.suptitle(suptitle, fontsize=11)

    t = step["t"]
    theta = step["theta"]
    ref = step["ref"]
    volt = step["v"]
    y0 = step["y0"]

    ax_ang.axvline(0.0, color="#888888", lw=0.9, zorder=0)
    ax_ang.plot(t, theta, color=ANGLE_C, lw=1.8, label=r"$\theta$", zorder=3)
    ax_ang.plot(t, ref, color=REF_C, lw=1.5, drawstyle="steps-post", label=r"$\theta_{ref}$", zorder=2)
    ax_ang.axhline(y0, color="#555555", ls=":", lw=1.1, label=r"$\theta(0^-)$", zorder=1)
    ax_ang.text(
        0.0,
        0.03,
        " step",
        transform=ax_ang.get_xaxis_transform(),
        fontsize=8,
        color="#666666",
        va="bottom",
        ha="left",
    )

    y_vals = list(theta) + [y0, step["r1"], step["r0"]]
    if m.ss_confident and m.theta_ss is not None:
        t_end = max(t[-1], 0.0)
        ax_ang.plot([0.0, t_end], [m.theta_ss, m.theta_ss], color=SS_C, ls="--", lw=1.4, label=r"$\theta_{ss}$", zorder=2)
        y_vals.append(m.theta_ss)
        if m.band_half is not None and m.moved:
            lo = m.theta_ss - m.band_half
            hi = m.theta_ss + m.band_half
            ax_ang.fill_between([0.0, t_end], [lo, lo], [hi, hi], color=SS_C, alpha=0.16, label=r"$\pm$2% band", zorder=0)
            ax_ang.plot([0.0, t_end], [lo, lo], color=SS_C, ls=":", lw=0.8, zorder=1)
            ax_ang.plot([0.0, t_end], [hi, hi], color=SS_C, ls=":", lw=0.8, zorder=1)
            y_vals.extend([lo, hi])
        if m.ts is not None and m.ts_y is not None:
            ax_ang.axvline(m.ts, color=SETTLE_C, ls="--", lw=1.2, zorder=2)
            ax_ang.plot([m.ts], [m.ts_y], marker="s", ms=7, color=SETTLE_C, markeredgecolor="white", markeredgewidth=0.6, zorder=5)
            span = max(t[-1] - t[0], 1e-3)
            on_right = m.ts > t[0] + 0.72 * span
            ax_ang.text(
                m.ts,
                1.01,
                f"t_s={m.ts:.3f}s",
                transform=ax_ang.get_xaxis_transform(),
                fontsize=8,
                color=SETTLE_C,
                va="bottom",
                ha="right" if on_right else "left",
                clip_on=False,
                bbox=_label_bbox(SETTLE_C),
            )
        if m.peak_t is not None and m.peak_y is not None:
            ax_ang.plot(
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
        ax_ang.plot(
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
    pad = max(0.08, 0.22 * (y_hi - y_lo))
    ax_ang.set_ylim(y_lo - pad, y_hi + pad)

    if m.peak_t is not None and m.peak_y is not None:
        # Callout sits off the trace, on the far side of the peak.
        above = m.theta_ss is None or m.peak_y >= m.theta_ss
        ax_ang.annotate(
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
    ax_ang.legend(loc=loc, fontsize=8, framealpha=0.92)

    ax_v.plot(t, volt, color=VOLT_C, lw=1.5, label="logged voltage", zorder=3)
    limit = LAB_V_LIMIT_V if spec is None else spec.v_limit
    ax_v.axhline(limit, color=LIMIT_C, ls="--", lw=1.1, label=r"$\pm$6 V limit", zorder=1)
    ax_v.axhline(-limit, color=LIMIT_C, ls="--", lw=1.1, zorder=1)
    if m.viol_t:
        ax_v.scatter(m.viol_t, m.viol_v, s=16, color=PEAK_C, zorder=4, label="exceeds $\\pm$6 V")
    v_lo = min(-8.0, min(volt) - 1.4)
    v_hi = max(8.0, max(volt) + 1.4)
    ax_v.set_ylim(v_lo, v_hi)
    if m.v_max is not None:
        ax_v.plot([m.v_max_t], [m.v_max], marker="^", ms=8, color=PEAK_C, markeredgecolor="white", zorder=5)
        # The logged rail is flat through the marker, so a positive peak's
        # label drops into the open area under the trace instead of sitting on it.
        vmax_below = m.v_max >= 0.0
        ax_v.annotate(
            f"Vmax {m.v_max:+.2f} V @ {m.v_max_t:.3f}s",
            xy=(m.v_max_t, m.v_max),
            xytext=(14, -32 if vmax_below else 10),
            textcoords="offset points",
            fontsize=8,
            color=PEAK_C,
            va="top" if vmax_below else "bottom",
            ha="left",
            arrowprops=dict(arrowstyle="-", color=PEAK_C, lw=0.6),
            bbox=_label_bbox(PEAK_C),
            zorder=7,
        )
    if m.v_min is not None:
        ax_v.plot([m.v_min_t], [m.v_min], marker="v", ms=8, color="#1f4e79", markeredgecolor="white", zorder=5)
        vmin_below = m.v_min <= 0.0
        ax_v.annotate(
            f"Vmin {m.v_min:+.2f} V @ {m.v_min_t:.3f}s",
            xy=(m.v_min_t, m.v_min),
            xytext=(8, -8 if vmin_below else 8),
            textcoords="offset points",
            fontsize=8,
            color="#1f4e79",
            va="top" if vmin_below else "bottom",
            ha="left",
            arrowprops=dict(arrowstyle="-", color="#1f4e79", lw=0.6),
            bbox=_label_bbox("#1f4e79"),
            zorder=7,
        )
    ax_v.legend(loc="upper right", fontsize=8, framealpha=0.92)

    left = min(min(t), -0.002)
    right = max(max(t), 0.05)
    span = max(right - left, 0.05)
    ax_ang.set_xlim(left - 0.03 * span, right + 0.06 * span)
    ax_v.set_xlim(ax_ang.get_xlim())

    rows, face = _panel_rows(m, phase, end_reason, None)
    _paint_panel(ax_info, rows, face)


def make_figure(plt):
    fig = plt.figure(figsize=(12.8, 7.4))
    gs = fig.add_gridspec(
        2, 2, width_ratios=[4.4, 1.85], height_ratios=[2.15, 1.05], wspace=0.22, hspace=0.32
    )
    ax_ang = fig.add_subplot(gs[0, 0])
    ax_v = fig.add_subplot(gs[1, 0])
    ax_info = fig.add_subplot(gs[:, 1])
    fig.subplots_adjust(left=0.055, right=0.985, top=0.90, bottom=0.08)
    fig.patch.set_facecolor("white")
    return fig, ax_ang, ax_v, ax_info


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
        self.status_queue: queue.Queue = queue.Queue()
        self._outbox: queue.Queue = queue.Queue()
        self.running = True
        self.paused = False
        self.status = "Starting..."
        self._conn = None
        self._bad = 0
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def send(self, line: str) -> None:
        self._outbox.put(line.strip())

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

    def _flush_outbox(self) -> None:
        conn = self._conn
        if conn is None:
            return
        while True:
            try:
                line = self._outbox.get_nowait()
            except queue.Empty:
                return
            try:
                conn.write((line + "\n").encode("ascii"))
                conn.flush()
            except OSError as exc:
                print(f"\nCould not send {line!r} ({exc}).")
                self._close()
                return

    def _note_bad(self, line: str) -> None:
        self._bad += 1
        if self._bad <= 8:
            print(f"[serial] {line}")
        elif self._bad == 9:
            print("[serial] further non-numeric lines suppressed")

    def _handle_values(self, vals: list[float]) -> None:
        if len(vals) != 4:
            return
        t_ms, theta, ref, volt = vals
        if self.log is not None:
            self.log.write(t_ms, theta, ref, volt)
        self.queue.put(("sample", t_ms, theta, ref, volt))

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
                self._flush_outbox()
                raw = self._conn.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                if line.startswith("#"):
                    self.status_queue.put(line)
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
                if len(vals) != 4:
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


def print_banner(port: str, baud: int, spec: Spec, log_path: str, gains: tuple[float, float, float], settle_s: float, pre_delay_s: float) -> None:
    a1, b1, b0 = gains
    print(f"Auto-tuner  {port} @ {baud}")
    print(f"Full log: {log_path}")
    print(f"Gains: a1={a1:.6g}  b1={b1:.6g}  b0={b0:.6g}")
    print(
        f"Trial: M {spec.ref_a:+.3f}, wait {settle_s:.2f}s, record, "
        f"wait {pre_delay_s:.2f}s, M {spec.ref_b:+.3f}"
    )
    print(
        f"Checks: bounded, |e_ss|<={spec.ess_tol:.3f} rad, "
        f"t_s<={spec.ts_max:.3f}s, OS<={spec.os_max_pct:.0f}%, |V|<={spec.v_limit:.0f}"
    )
    print("Keys: R repeat trial (no board reset)   M freeze   Space release port")
    print()


def replay_file(path: str, spec: Spec) -> None:
    import matplotlib.pyplot as plt

    samples = load_samples(path)
    print(f"Replaying {len(samples)} samples from {path}")
    if not samples:
        return
    capture = StepCapture(spec)
    for t_ms, theta, ref, volt in samples:
        if capture.push(t_ms, theta, ref, volt) == "end":
            break
    if capture.phase == "live":
        capture.freeze("end of file")
    if capture.step is not None:
        finalize_capture(capture, "REPLAY")
    else:
        print(capture.hold_note)

    fig, ax_ang, ax_v, ax_info = make_figure(plt)
    try:
        fig.canvas.manager.set_window_title("Step response replay")
    except Exception:
        pass
    m = capture.metrics()
    if capture.step is not None and m is not None:
        draw_step(
            ax_ang, ax_v, ax_info, capture.step, m, "frozen",
            f"REPLAY  {os.path.basename(path)}", spec, capture.end_reason,
        )
    else:
        draw_waiting(
            ax_ang, ax_v, ax_info, list(capture.preview), capture.last_ref,
            f"REPLAY  {os.path.basename(path)}", capture.hold_note,
        )
    plt.show()


def run_plotter(
    port: str,
    baud: int,
    spec: Spec,
    gains: tuple[float, float, float],
    settle_s: float,
    pre_delay_s: float,
) -> None:
    import matplotlib.pyplot as plt
    from matplotlib import animation

    log = SessionLog()
    print_banner(port, baud, spec, log.path, gains, settle_s, pre_delay_s)
    reader = SerialReader(port, baud, log)
    capture = StepCapture(spec)
    reader.start()

    fig, ax_ang, ax_v, ax_info = make_figure(plt)
    try:
        fig.canvas.manager.set_window_title("Auto-tuner")
    except Exception:
        pass

    state = {"sig": None, "last_line": 0.0}
    trial = {"phase": "boot", "t_conn": None, "t_mark": None}

    def suptitle() -> str:
        conn = reader.status
        phase = {
            "boot": "waiting for READY",
            "send_c": "sending gains",
            "wait_c": "sending gains",
            "wait_m": f"moving to {spec.ref_a:+.3f}",
            "settle": f"settling at {spec.ref_a:+.3f}",
            "preroll": "recording, step pending",
            "record": "step sent",
        }.get(trial["phase"], trial["phase"])
        if capture.phase == "live":
            return f"LIVE    {phase}    {conn}    [M] freeze    [R] repeat"
        if capture.phase == "frozen":
            why = capture.end_reason or "frozen"
            return f"FROZEN ({why})    [R] repeat trial    [Space] release port"
        return f"{capture.hold_note}    {phase}    {conn}"

    def on_status(line: str) -> None:
        print("\n" + line)
        phase = trial["phase"]
        if line.startswith("# READY") and phase == "boot":
            trial["phase"] = "send_c"
        elif line.startswith("# OK C") and phase == "wait_c":
            reader.send(f"M {spec.ref_a:.6f}")
            print(f"\nMove to {spec.ref_a:+.3f}")
            trial["phase"] = "wait_m"
        elif line.startswith("# OK M") and phase == "wait_m":
            trial["phase"] = "settle"
            trial["t_mark"] = time.monotonic()
            print(f"\nHolding {spec.ref_a:+.3f} for {settle_s:.2f} s")
        elif line.startswith("# ERR"):
            print("\nBoard rejected the last command. Press R to try the trial again.")

    def pump_trial() -> None:
        now = time.monotonic()
        while True:
            try:
                on_status(reader.status_queue.get_nowait())
            except queue.Empty:
                break

        connected = "CONNECTED" in reader.status
        if trial["phase"] == "boot":
            if not connected:
                trial["t_conn"] = None
                return
            if trial["t_conn"] is None:
                trial["t_conn"] = now
            elif now - trial["t_conn"] > 3.0:
                print("\nNo READY line yet; sending gains anyway.")
                trial["phase"] = "send_c"
        if trial["phase"] == "send_c":
            if not connected:
                return
            a1, b1, b0 = gains
            reader.send(f"C {a1:.12g} {b1:.12g} {b0:.12g}")
            print(f"\nGains  a1={a1:.6g}  b1={b1:.6g}  b0={b0:.6g}")
            trial["phase"] = "wait_c"
        elif trial["phase"] == "settle" and trial["t_mark"] is not None and now - trial["t_mark"] >= settle_s:
            capture.command_arm()
            trial["phase"] = "preroll"
            trial["t_mark"] = now
            print(f"\nRecording. Commanding {spec.ref_b:+.3f} in {pre_delay_s:.2f} s.")
        elif trial["phase"] == "preroll" and trial["t_mark"] is not None and now - trial["t_mark"] >= pre_delay_s:
            reader.send(f"M {spec.ref_b:.6f}")
            print(f"\nMove to {spec.ref_b:+.3f}")
            trial["phase"] = "record"

    def repeat_trial() -> None:
        if trial["phase"] in ("send_c", "wait_c", "wait_m", "settle", "preroll"):
            print("\nTrial already in progress.")
            return
        if capture.phase == "live" and capture.freeze("repeat"):
            finalize_capture(capture, "FINAL  repeat")
        trial["phase"] = "send_c"
        trial["t_mark"] = None
        print("\nRepeating trial on the open port.")

    def update(_frame):
        try:
            pump_trial()
            ended = False
            started = None
            while True:
                try:
                    item = reader.queue.get_nowait()
                except queue.Empty:
                    break
                if item[0] == "disconnect":
                    trial["phase"] = "boot"
                    trial["t_conn"] = None
                    trial["t_mark"] = None
                    if capture.freeze("serial disconnected"):
                        ended = True
                    continue
                ev = capture.push(item[1], item[2], item[3], item[4])
                if ev == "start" and capture.step is not None:
                    started = (capture.step["index"], capture.step["r0"], capture.step["r1"])
                elif ev == "end":
                    ended = True

            if started is not None:
                idx, r0, r1 = started
                print(f"\nStep {idx} started: theta_ref {r0:+.3f} -> {r1:+.3f} rad")
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
                    ax_ang, ax_v, ax_info, list(capture.preview), capture.last_ref, title, capture.hold_note
                )
            else:
                draw_step(
                    ax_ang, ax_v, ax_info, capture.step, m, capture.phase, title, spec, capture.end_reason
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
        elif event.key in ("r", "R", "n", "N"):
            state["sig"] = None
            repeat_trial()

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
    parser = argparse.ArgumentParser(description="Live one-step response analyzer for the ball-beam inner loop.")
    parser.add_argument("--port", default=None, help="Serial port (default: auto-detect, else COM5)")
    parser.add_argument("--file", default=None, help="replay a full log CSV from logs/ instead of the serial port")
    parser.add_argument("--baud", type=int, default=230400, help="Baud rate (default: 230400)")
    parser.add_argument("--ess-tol", type=float, default=LAB_ESS_TOL_RAD, help="rad, |e_ss| at or below this counts as zero")
    parser.add_argument("--ts-max", type=float, default=LAB_TS_MAX_S, help="seconds, 2%% settling-time limit")
    parser.add_argument("--os-max", type=float, default=LAB_OS_MAX_PCT, help="percent overshoot limit")
    parser.add_argument("--v-limit", type=float, default=LAB_V_LIMIT_V, help="volt limit, symmetric")
    parser.add_argument("--ss-std", type=float, default=SS_STD_MAX_RAD, help="rad, max tail std to call steady state")
    parser.add_argument("--tail", type=float, default=TAIL_S, help="seconds, steady-state window")
    parser.add_argument("--self-test", action="store_true", help="check the metric definitions and exit")
    parser.add_argument("--a1", type=float, default=DEFAULT_A1, help="u[k-1] coefficient (sweep sketch a0_d)")
    parser.add_argument("--b1", type=float, default=DEFAULT_B1, help="e[k] coefficient")
    parser.add_argument("--b0", type=float, default=DEFAULT_B0, help="e[k-1] coefficient")
    parser.add_argument("--settle", type=float, default=SETTLE_S, help="seconds to hold -0.7 before recording")
    parser.add_argument("--pre-delay", type=float, default=PRE_DELAY_S, help="seconds to record at -0.7 before commanding +0.7")
    args = parser.parse_args()

    if args.self_test:
        from step_response_selftest import self_test
        self_test()
        return

    spec = Spec(
        ess_tol=args.ess_tol,
        ts_max=args.ts_max,
        os_max_pct=args.os_max,
        v_limit=args.v_limit,
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
    run_plotter(port, args.baud, spec, (args.a1, args.b1, args.b0), args.settle, args.pre_delay)


if __name__ == "__main__":
    main()
