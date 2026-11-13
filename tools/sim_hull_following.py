"""Closed-loop hull-following simulation (old vs new distance code and controller).

No ROS needed. Plots:  python3 tools/sim_plot_trajectory.py OUT_DIR   and   python3 tools/sim_plot_return.py OUT_DIR
The controller logic copies guided_controller.control_loop; the wall normal is assumed known.
"""
import math, sys
import numpy as np
import os; _here = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(_here, "..", "hull_guider", "core")); sys.path.insert(0, _here)
from hull_distance import HullDistanceEstimator, default_beam_dirs
from replay_dvl_bag import OldDistance
U = default_beam_dirs(); DT, DVL_DT, T_END, TARGET = 0.05, 1 / 4.8, 200.0, 0.8
EV = {"start": (0, 6), "turn": (45, 75), "push": (120, 124)}

def scenario(seed=7):
    rng = np.random.default_rng(seed); n = int(T_END / DVL_DT) + 5
    t = np.arange(0, T_END + 1, DT)
    yaw = 3 * np.sin(2 * np.pi * t / 23) + np.cumsum(rng.normal(0, 0.05, len(t))) * 0.3
    a, b = EV["turn"]; yaw += np.where((t > a) & (t < b), 18 * np.clip(np.sin(np.pi * (t - a) / (b - a)), 0, None) ** 0.5, 0)
    push = np.where((t >= EV["push"][0]) & (t < EV["push"][1]), 0.3, 0.0)
    sc = dict(t=t, yaw=yaw, push=push, drop=rng.random((n, 4)) < 0.15,
              noise=rng.normal(0, 0.01, (n, 4)), refl=rng.random((n, 4)) < 0.02, refl_val=rng.uniform(1, 3, (n, 4)))
    # drawn after the rest so the scenario is the same as in the earlier figure
    sc["low_ok"] = rng.random((n, 4)) < 0.3          # lower beam still valid during a top-beam gap (bags: ~30%)
    sc["vnoise"] = rng.normal(0, 0.01, (n, 2))
    return sc

def reading(D, yaw_deg, tm, k, sc, v_toward):
    a = math.radians(yaw_deg); n = np.array([0, math.sin(a), math.cos(a)])
    r = np.full(4, -1.0); v = np.zeros(4, bool)
    gap = EV["start"][0] <= tm < EV["start"][1] or EV["push"][0] - 0.5 <= tm < EV["push"][1] + 0.5
    for i in range(4):
        d = D / float(n @ U[i]) + sc["noise"][k, i]
        if sc["refl"][k, i]: d = sc["refl_val"][k, i]
        lost = (i in (2, 3)) if gap else False
        if gap and i in (0, 1) and not sc["low_ok"][k, i]: lost = True
        if sc["drop"][k, i] or lost or d < 0.3 or d > 3.0: continue
        r[i], v[i] = d, True
    vel = None
    if v.sum() >= 3:                                   # bottom-lock velocity needs 3 beams
        tdir = np.array([0, math.cos(a), -math.sin(a)])
        vel = (v_toward + sc["vnoise"][k, 0]) * n + (0.3 + sc["vnoise"][k, 1]) * tdir
    return r, v, vel

CFG = {
    "old":    dict(est="old", deadband=0.15, k_i=0.0),
    "new_v1": dict(est=dict(timeout=1.0, max_range=5.0, backup_beams=()), use_vel=False, deadband=0.15, k_i=0.0),
    "new":    dict(est=dict(), use_vel=True, deadband=0.05, k_i=0.01, i_max=0.1),
}

def run(method, sc, detail=None):
    c = CFG[method]
    old = OldDistance(); new = None if c["est"] == "old" else HullDistanceEstimator(**c["est"])
    x, y = 0.0, -TARGET; vel = np.zeros(2); last = np.zeros(2); integ = 0.0
    filt = 0.0 if method == "old" else None
    cur, ok_cur, nxt, k, log = None, False, 0.0, 0, []
    nh, td = np.array([0.0, -1.0]), np.array([1.0, 0.0])
    for j, tm in enumerate(sc["t"][sc["t"] <= T_END]):
        D = -y
        if tm >= nxt:
            r, v, dvel = reading(D, sc["yaw"][j], tm, k, sc, v_toward=vel[1] - sc["push"][j])
            if new is None: cur, ok_cur = old.update(r, v), True
            else: cur, ok_cur = new.update(r, v, DVL_DT, velocity=dvel if c["use_vel"] else None)
            if detail is not None:
                info = new.last if new is not None else {}
                detail.setdefault("msg", []).append((tm, D, info.get("raw", old.raw(r, v)), info.get("n_beams", int(v[2]) + int(v[3])),
                                                      int(bool(info.get("rejected"))), cur, int(ok_cur), int(dvel is not None)))
            k += 1; nxt += DVL_DT
        if new is not None and (cur is None or not ok_cur):
            cmd = np.zeros(2); last = cmd
        else:
            filt = cur if filt is None else filt
            filt = 0.15 * cur + 0.85 * filt
            e = filt - TARGET; db = c["deadband"]
            se = 0.0 if abs(e) < db else math.copysign(abs(e) - db, e)
            if c["k_i"]:
                if abs(e) < 0.3: integ += e * DT
                lim = c.get("i_max", 0.1) / c["k_i"]; integ = max(-lim, min(lim, integ))
            vn = -0.5 * se - c["k_i"] * integ - 0.8 * float(last @ nh); vt = 0.3 * (0.3 if abs(e) > 0.8 else 1.0)
            last = 0.3 * (vn * nh + vt * td) + 0.7 * last; cmd = last
        vel += (cmd - vel) * (DT / 0.6)
        x += vel[0] * DT; y += (vel[1] - sc["push"][j]) * DT; y = min(y, -0.05)
        if detail is not None:
            detail.setdefault("ctl", []).append((tm, -float(cmd @ nh) if new is None or ok_cur else 0.0, -vel[1] + sc["push"][j], integ))
        log.append((tm, x, -y, np.nan if cur is None else cur, int(ok_cur), sc["yaw"][j]))
    return np.array(log)

def stats(d):
    D = d[:, 2]; post = d[d[:, 0] > 150, 2]
    return dict(rms=np.sqrt(np.mean((D - .8) ** 2)) * 100, band=np.mean(np.abs(D - .8) < .15) * 100,
                band05=np.mean(np.abs(D - .8) < .05) * 100, far30=D[d[:, 0] < 30].max(), post=np.median(post),
                pmax=D[(d[:, 0] > 118) & (d[:, 0] < 150)].max(), hold=np.mean(d[:, 4] == 0) * 100, travel=d[-1, 1],
                back=next((t for t, dd in zip(d[:, 0], D) if t > 124 and abs(dd - .8) < .15), np.nan))
if __name__ == "__main__":
    sc = scenario()
    for m in CFG:
        print(m, {k: round(float(v), 2) for k, v in stats(run(m, sc)).items()})
