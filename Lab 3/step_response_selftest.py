"""Metric checks for step_response_analyzer. Run with the analyzer's --self-test flag."""

from __future__ import annotations

import math
import os
import tempfile

from step_response_analyzer import (
    SerialReader,
    SessionLog,
    Spec,
    StepCapture,
    _trailing_median,
    compute_metrics,
    draw_step,
    load_samples,
    make_figure,
)


def _underdamped(t: float, y0: float, yf: float, wn: float, zeta: float) -> float:
    if t <= 0.0:
        return y0
    wd = wn * math.sqrt(1.0 - zeta * zeta)
    phi = math.acos(zeta)
    env = math.exp(-zeta * wn * t) / math.sqrt(1.0 - zeta * zeta)
    s = 1.0 - env * math.sin(wd * t + phi)
    return y0 + (yf - y0) * s


def _analytical_os(zeta: float) -> float:
    return math.exp(-zeta * math.pi / math.sqrt(1.0 - zeta * zeta))


def _ready_for_rise(cap: StepCapture, t0: float = 0.0) -> tuple[float, str | None]:
    """Skip one edge, dwell steady at 0.10 m, and return the time of the next sample.

    The next push with y_ref = 0.25 m is the recorded step.
    """
    for i in range(10):
        cap.push(t0 + i * 2.0, 0.25, 0.25, 0.0)
    t_skip = t0 + 20.0
    skipped = cap.push(t_skip, 0.20, 0.10, 0.0)
    for i in range(600):
        cap.push(t_skip + 2.0 + i * 2.0, 0.10, 0.10, 0.0)
    return t_skip + 2.0 + 600 * 2.0, skipped


def _feed_step(cap: StepCapture, y0, yf, r0, r1, wn, zeta, n, dt_ms, t0_ms, volt_fn, noise=0.0, rng=None):
    events = []
    for i in range(n):
        t_ms = t0_ms + i * dt_ms
        t_rel = i * dt_ms / 1000.0
        y = _underdamped(t_rel, y0, yf, wn, zeta)
        if noise and rng is not None:
            y += rng.gauss(0.0, noise)
        ref = r1
        ev = cap.push(t_ms, y, ref, volt_fn(r1, y))
        if ev:
            events.append(ev)
    return events


def _check(name: str, cond: bool, detail: str = "") -> None:
    if not cond:
        raise AssertionError(f"{name} failed {detail}")
    print(f"  ok  {name}")


def self_test() -> None:
    import random

    spec = Spec()
    print("formula")
    zeta, wn = 0.35, 1.8
    os_frac = _analytical_os(zeta)
    tp = math.pi / (wn * math.sqrt(1.0 - zeta * zeta))
    y_peak = _underdamped(tp, 0.10, 0.25, wn, zeta)
    expect = 0.10 + 0.15 * (1.0 + os_frac)
    _check("underdamped peak matches the OS formula", abs(y_peak - expect) < 1e-9, f"{y_peak} vs {expect}")

    print("clean positive step")
    cap = StepCapture(spec)
    t_rise, skipped = _ready_for_rise(cap)
    _check("startup edge was skipped", skipped == "skip")
    _check("dwell at 0.10 m armed the rising step", cap.ready)

    def volt_clipped(r, y):
        return max(-6.0, min(6.0, 20.0 * (r - y)))

    events = _feed_step(cap, 0.10, 0.25, 0.10, 0.25, wn, zeta, 5000, 2.0, t_rise, volt_clipped)
    _check("step started on the reference edge", events[0] == "start")
    end = cap.push(t_rise + 5000 * 2.0, 0.25, 0.10, 0.0)
    _check("next edge freezes the step", end == "end" and cap.phase == "frozen")
    _check("end reason is the reference change", cap.end_reason == "reference stepped away")
    m = cap.metrics()
    assert m is not None
    _check("lab step recognized", m.lab_step)
    _check("y(0-) is the pre-step position", abs(m.y0 - 0.10) < 1e-9, f"y0={m.y0}")
    _check("steady state locked", m.ss_confident and m.moved)
    _check("steady state is the final position", abs(m.y_ss - 0.25) < 1e-3, f"ss={m.y_ss}")
    _check("tracking error passes", m.pass_ess is True and abs(m.e_ss) < 1e-3)
    _check("overshoot near 30.9%", abs(m.overshoot_pct - os_frac * 100.0) < 0.3, f"OS={m.overshoot_pct}")
    _check("overshoot passes 45%", m.pass_os is True)
    _check("peak is after the rise and past y_ss", m.peak_t > 0.5 and m.peak_y > m.y_ss)
    _check("settling sample is after the peak (OS > 2%)", m.ts is not None and m.ts > m.peak_t)
    _check("settling passes 8.5 s", m.pass_ts is True and m.ts <= spec.ts_max)
    # Independent restatement of the settling definition, on the same median.
    n = cap.step["n_pre"]
    t_post = cap.step["t"][n:]
    y_post = _trailing_median(cap.step["y"][n:], spec.settle_median_n)
    last_out = None
    for i, yi in enumerate(y_post):
        if abs(yi - m.y_ss) > m.band_half:
            last_out = i
    exp_ts = t_post[0] if last_out is None else t_post[last_out + 1]
    _check("t_s matches the last-exit definition", abs(m.ts - exp_ts) < 1e-12, f"{m.ts} vs {exp_ts}")
    idx = next(i for i, ti in enumerate(t_post) if abs(ti - m.ts) < 1e-12)
    _check("settling sample is inside the band", abs(y_post[idx] - m.y_ss) <= m.band_half)
    _check("sample before t_s is outside the band", abs(y_post[idx - 1] - m.y_ss) > m.band_half)
    _check(
        "every later sample stays inside",
        all(abs(yi - m.y_ss) <= m.band_half for yi in y_post[idx:]),
    )
    _check("voltage on the rail passes", m.pass_v is True and m.v_max <= 6.0 + 1e-9)
    _check("Vmax is at step start", m.v_max_t < 0.05)
    _check("stable", m.stability == "STABLE" and m.pass_stable is True)
    _check("post-step count excludes the pre-roll and the next edge", m.n_post == 5000)

    print("overshoot inside the 2% band settles before the peak")
    t = [i * 0.002 for i in range(5000)]
    z2, wn2 = 0.9, 2.0
    y = [_underdamped(ti, 0.10, 0.25, wn2, z2) for ti in t]
    v = [max(-6.0, min(6.0, 20.0 * (0.25 - yi))) for yi in y]
    m2 = compute_metrics(t, y, v, 0.10, 0.10, 0.25, spec)
    _check("small OS is under 2%", m2.overshoot_pct < 2.0, f"OS={m2.overshoot_pct}")
    _check("t_s is before that peak", m2.ts < m2.peak_t, f"ts={m2.ts} peak={m2.peak_t}")

    print("negative step")
    t = [i * 0.002 for i in range(5000)]
    y = [_underdamped(ti, 0.25, 0.10, wn, zeta) for ti in t]
    v = [0.0 for _ in t]
    mn = compute_metrics(t, y, v, 0.25, 0.25, 0.10, spec)
    _check("negative-step overshoot is still a positive percent", abs(mn.overshoot_pct - os_frac * 100.0) < 0.3)
    _check("peak is beyond y_ss in the negative direction", mn.peak_y < mn.y_ss)
    _check("negative lab step recognized", mn.lab_step and mn.pass_os is True and mn.pass_ess is True)

    print("voltage past the rail")
    v_bad = list(v)
    over_pos = spec.v_pos + 0.5
    over_neg = spec.v_neg + 0.5
    v_bad[3] = over_pos
    v_bad[20] = -over_neg
    mv = compute_metrics(t, y, v_bad, 0.25, 0.25, 0.10, spec)
    _check("logged over-voltage fails", mv.pass_v is False and len(mv.viol_t) == 2)
    _check("Vmax marker is the over-limit sample", abs(mv.v_max - over_pos) < 1e-9 and abs(mv.v_max_t - t[3]) < 1e-12)
    _check("Vmin marker is the under-limit sample", abs(mv.v_min + over_neg) < 1e-9)

    print("nonzero steady-state error")
    y_off = [_underdamped(ti, 0.10, 0.22, wn, zeta) for ti in t]
    me = compute_metrics(t, y_off, [0.0] * len(t), 0.10, 0.10, 0.25, spec)
    _check("offset fails the zero-error check", me.pass_ess is False and abs(me.e_ss - 0.03) < 1e-3, f"ess={me.e_ss}")

    print("no motion")
    t_hold = [i * 0.002 for i in range(1200)]
    mh = compute_metrics(t_hold, [0.10] * len(t_hold), [0.0] * len(t_hold), 0.10, 0.10, 0.25, spec)
    _check("sitting still is a flat steady state", mh.ss_confident and not mh.moved)
    _check("sitting still fails tracking and does not invent a settling time", mh.pass_ess is False and mh.ts is None and mh.pass_ts is None)
    _check("sitting still is bounded", mh.stability == "STABLE")

    print("short record")
    ms = compute_metrics([0.0, 0.002, 0.004], [0.10, 0.101, 0.102], [1.0, 1.0, 1.0], 0.10, 0.10, 0.25, spec)
    _check("short record does not lock a steady state", ms.ss_confident is False and ms.pass_ess is None and ms.stability == "INCONCLUSIVE")

    print("growing error")
    tg = [i * 0.002 for i in range(1250)]
    yg = [0.25 + 0.02 * math.exp(1.0 * ti) for ti in tg]
    mg = compute_metrics(tg, yg, [0.0] * len(tg), 0.10, 0.10, 0.25, spec)
    _check("exponential growth is unstable", mg.stability == "UNSTABLE" and mg.pass_stable is False, mg.stability_reason)
    _check("growth markers are on the rising trace", len(mg.growth_marks) == 3)

    print("runaway position")
    tr = [i * 0.002 for i in range(500)]
    yr = [0.25 + 1.0 * ti for ti in tr]
    mr = compute_metrics(tr, yr, [0.0] * len(tr), 0.10, 0.10, 0.25, spec)
    _check("large position is unstable", mr.stability == "UNSTABLE")

    print("capture gating")
    gate = StepCapture(spec)
    for i in range(700):
        gate.push(i * 2.0, 0.10, 0.10, 0.0)
    ev = gate.push(1400.0, 0.10, 0.25, 1.0)
    _check("first rise is skipped even from a steady 0.10", ev == "skip" and gate.phase == "wait")
    for i in range(100):
        gate.push(1402.0 + i * 2.0, 0.10, 0.10, 0.0)
    ev = gate.push(1610.0, 0.10, 0.25, 1.0)
    _check("a short dwell does not start the record", ev is None and gate.phase == "wait")
    t_rise, _ = _ready_for_rise(gate, t0=2000.0)
    _check("return from 0.25 settles and arms", gate.ready)
    ev = gate.push(t_rise, 0.102, 0.25, 2.0)
    _check("settled 0.10 to 0.25 starts recording", ev == "start" and gate.phase == "live")
    gate.arm()
    for i in range(600):
        gate.push(t_rise + 10.0 + i * 2.0, 0.10, 0.10, 0.0)
    ev = gate.push(t_rise + 10.0 + 600 * 2.0, 0.10, 0.25, 1.0)
    _check("the next capture does not skip another step", ev == "start")

    print("5-col serial stream and CSV log")
    sr = SerialReader("COM0", 230400)
    sr._handle_values([0.12, -0.35, 1.5, 0.14, 0.25])
    item = sr.queue.get_nowait()
    _check("5-col stream extracts ball position and ref", item[2] == 0.14 and item[3] == 0.25 and item[4] == 1.5)
    _check("5-col stream retains theta and theta_ref", item[5] == 0.12 and item[6] == -0.35)
    tmp_csv = os.path.join(tempfile.gettempdir(), "test_ball_pos_5col.csv")
    with open(tmp_csv, "w", encoding="utf-8") as f:
        f.write("0.10,0.20,1.2,0.11,0.10\n0.15,0.25,1.4,0.16,0.25\n")
    loaded = load_samples(tmp_csv)
    os.remove(tmp_csv)
    _check("5-col headerless CSV loads ball pos and ref", len(loaded) == 2 and loaded[1][1] == 0.16 and loaded[1][2] == 0.25 and loaded[1][3] == 1.4)

    print("board time jump ends the live step")
    cap2 = StepCapture(spec)
    t_rise, _ = _ready_for_rise(cap2, t0=2000.0)
    cap2.push(t_rise, 0.10, 0.25, 1.0)
    cap2.push(t_rise + 2.0, 0.12, 0.25, 1.0)
    ev = cap2.push(10.0, 0.15, 0.25, 0.0)
    _check("millis reset freezes the step", ev == "end" and cap2.end_reason == "board time jumped")

    print("noisy tail still locks")
    rng = random.Random(1)
    tn = [i * 0.002 for i in range(5000)]
    yn = [_underdamped(ti, 0.10, 0.25, wn, zeta) + rng.gauss(0.0, 0.0005) for ti in tn]
    mnse = compute_metrics(tn, yn, [0.0] * len(tn), 0.10, 0.10, 0.25, spec)
    _check("0.5 mm noise still finds steady state", mnse.ss_confident and abs(mnse.y_ss - 0.25) < 0.002, f"ss={mnse.y_ss}")

    print("figure")
    import matplotlib
    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    fig, ax_pos, ax_v, ax_info = make_figure(plt)
    cap3 = StepCapture(spec)
    t_rise, _ = _ready_for_rise(cap3)

    def volt_stiction(r, y):
        u = max(-6.0, min(6.0, 60.0 * (r - y)))
        extra = spec.v_pos - 6.0 + 0.17
        if u > 0.0:
            return u + extra
        if u < 0.0:
            return u - extra
        return 0.0

    _feed_step(cap3, 0.10, 0.25, 0.10, 0.25, wn, zeta, 5000, 2.0, t_rise, volt_stiction)
    cap3.freeze("preview")
    mviz = cap3.metrics()
    draw_step(ax_pos, ax_v, ax_info, cap3.step, mviz, "frozen", "preview", spec, "preview")
    texts = [tx.get_text() for tx in list(ax_pos.texts) + list(ax_v.texts)]
    _check("settling time is labeled on the ball position plot", any("t_s=" in s for s in texts), str(texts))
    _check("overshoot is labeled on the ball position plot", any("OS " in s for s in texts))
    _check("Vmax is labeled on the voltage plot", any("Vmax" in s for s in texts))
    _check("preview voltage fails because of the stiction offset", mviz.pass_v is False and mviz.v_max > spec.v_pos)
    path = os.path.join(tempfile.gettempdir(), "step_response_analyzer_preview.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
    print(f"  wrote {path}")
    print("self-test passed")


if __name__ == "__main__":
    self_test()
