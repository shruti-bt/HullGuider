import sys, numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle; from matplotlib.lines import Line2D
from sim_hull_following import *
S = sys.argv[1]; sc = scenario()
det = {}; d = run("new", sc, det); m = np.array(det["msg"], float); c = np.array(det["ctl"])
do = run("old", sc); dv1 = run("new_v1", sc)
T0, T1 = 115, 165
G, R, GREY = "#15803d", "#b91c1c", "#9ca3af"
SHAPE = [(-1.6, 0.6), (1.6, 0.6), (1.6, -0.6), (0.4, -0.6), (0, -1.2), (-0.4, -0.6), (-1.6, -0.6)]
inv_t = m[(m[:, 6] == 0) & (m[:, 0] > T0), 0]
hold = (inv_t.min(), inv_t.max() + DVL_DT)
acc = m[(m[:, 0] > 124) & (m[:, 6] == 1), 0].min()
PH = [(119.5, "1", "119.5 s  Top beams lost, then pushed away at 0.3 m/s. The few lower-beam readings jump too fast and are rejected. The code\n            keeps predicting 0.80 m and still calls it valid (last good reading < 3 s ago), so the controller does nothing."),
      (hold[0], "2", "122.5 s  No reading accepted for 3 s → distance marked invalid → the ROV holds (zero velocity). The push continues until 124 s."),
      (acc, "3", "125.2 s  Top beams back at 124.6 s. The readings near 2.0 m agree with each other for 1.5 s → accepted as the real distance."),
      (127, "4", "127 s     The controller drives back toward the wall, up to 0.26 m/s, slowing as the error shrinks."),
      (132.4, "5", "132 s     Back inside ±0.15 m of the target (8 s after the push ended)."),
      (145, "6", "145 s     The integral term removes the last few cm: 0.80 m at 145 s, then a slow swing to 0.74 m and back.")]
fig = plt.figure(figsize=(14, 15))
gs = fig.add_gridspec(4, 1, height_ratios=[1.35, 1.5, 0.9, 0.5], hspace=0.38, top=0.825)
for k, (tp, num, txt) in enumerate(PH):
    yy = 0.948 - k * 0.0175 - (0.006 if k > 0 else 0)
    fig.text(0.07, yy, num, fontsize=10, weight="bold", color="white", va="top", bbox=dict(boxstyle="circle,pad=0.2", fc="#1f2937"))
    fig.text(0.09, yy, txt, fontsize=9.5, va="top", color="#1f2937", family="monospace" if False else None)

# ---- A: top view
ax = fig.add_subplot(gs[0])
w = (d[:, 0] >= T0) & (d[:, 0] <= T1)
ax.axhspan(0.75, 0.85, color="#dbeafe", lw=0); ax.axhline(0.8, color="#60a5fa", ls=":", lw=1)
ax.fill_between([30, 50], -0.12, 0, color="#9ca3af", hatch="////", ec="#6b7280", lw=0); ax.axhline(0, color="#374151", lw=2.5)
ax.text(47.8, 0.03, "wall / hull", ha="right", va="bottom", fontsize=9, color="#374151")
wo = (do[:, 0] >= T0) & (do[:, 0] <= T1)
ax.plot(do[wo, 1], do[wo, 2], color=R, lw=1.6, ls="--", alpha=0.7, label="old version (never comes back)")
ax.plot(d[w, 1], d[w, 2], color=G, lw=2.8, label="new version")
wh = w & (d[:, 0] >= hold[0]) & (d[:, 0] <= hold[1])
ax.plot(d[wh, 1], d[wh, 2], color="#6b7280", lw=5, alpha=0.6, solid_capstyle="round", label="holding (distance invalid)")
for tm in range(T0, T1 + 1, 5):
    i = np.argmin(np.abs(d[:, 0] - tm))
    ax.plot([d[i, 1]], [d[i, 2]], marker=MarkerStyle(SHAPE).rotated(deg=d[i, 5]), ms=20, mfc="white", mec=G, mew=1.5, ls="none", zorder=5)
OFF = {"1": (-0.6, 0.3), "2": (-0.8, 0.15), "3": (0.5, 0.2), "4": (0.6, 0.0), "5": (0.5, 0.25), "6": (0.0, 0.3)}
for tp, num, _ in PH:
    i = np.argmin(np.abs(d[:, 0] - tp)); ox, oy = OFF[num]
    ax.annotate(num, xy=(d[i, 1], d[i, 2]), xytext=(d[i, 1] + ox, d[i, 2] + oy), fontsize=10, weight="bold", color="white",
                bbox=dict(boxstyle="circle,pad=0.25", fc="#1f2937"), arrowprops=dict(arrowstyle="-", color="#1f2937"), zorder=7)
ax.set_xlim(34.5, 48); ax.set_ylim(-0.12, 2.4); ax.set_ylabel("distance from wall (m)"); ax.set_xlabel("position along the wall (m)")
ax.text(34.7, 2.33, "ROV every 5 s, nose toward the wall. The ROV keeps moving along the wall while it comes back.", fontsize=9, va="top", color="#4b5563")
ax.legend(loc="center right", fontsize=9); ax.grid(alpha=0.25)
ax.set_title("A. Top view, zoomed on the push (115–165 s)", loc="left", fontsize=12, weight="bold")

# ---- B: distance vs time
ax = fig.add_subplot(gs[1]); ax.sharex
for a in (ax,):
    a.axvspan(*EV["push"], color="#fde68a", alpha=0.5, lw=0); a.axvspan(*hold, color="#e5e7eb", lw=0)
ax.axhspan(0.65, 0.95, color="#dbeafe", lw=0, alpha=0.7); ax.axhline(0.8, color="#60a5fa", ls=":", lw=1)
ax.plot(do[:, 0], do[:, 2], color=R, lw=1.4, ls="--", alpha=0.6)
ax.plot(dv1[:, 0], dv1[:, 2], color=GREY, lw=1.6, ls="--")
ax.plot(d[:, 0], d[:, 2], color=G, lw=2.8)
ax.plot(d[:, 0], d[:, 3], color="#065f46", lw=1.6, ls=(0, (3, 2)))
wm = (m[:, 0] > T0) & (m[:, 0] < T1)
mk = {2: ("o", "#2563eb", "both top beams"), 1: ("o", "#93c5fd", "one top beam"), -1: ("s", "#f59e0b", "lower beams only")}
for nb, (sym, col, lab) in mk.items():
    ok = wm & (m[:, 3] == nb) & (m[:, 4] == 0); rj = wm & (m[:, 3] == nb) & (m[:, 4] == 1)
    ax.plot(m[ok, 0], m[ok, 2], sym, color=col, ms=4, ls="none")
    ax.plot(m[rj, 0], m[rj, 2], sym, mfc="none", mec=col, mew=1.5, ms=7, ls="none")
for tp, num, txt in PH:
    ax.axvline(tp, color="#1f2937", lw=0.8, ls=":")
    ax.text(tp + 0.25, 2.55, num, fontsize=10, weight="bold", color="white", va="top", bbox=dict(boxstyle="circle,pad=0.25", fc="#1f2937"))
ax.set_xlim(T0, T1); ax.set_ylim(0, 2.65); ax.set_ylabel("distance (m)"); ax.grid(alpha=0.25)
ax.set_title("B. Distance over time: what really happened vs what the code reported", loc="left", fontsize=12, weight="bold")
ax.legend(handles=[Line2D([], [], color=G, lw=2.8, label="new: actual distance"),
                   Line2D([], [], color="#065f46", lw=1.6, ls=(0, (3, 2)), label="new: distance reported to the controller"),
                   Line2D([], [], color=GREY, lw=1.6, ls="--", label="new before these changes (actual)"),
                   Line2D([], [], color=R, lw=1.4, ls="--", label="old (actual)"),
                   Line2D([], [], marker="o", color="#2563eb", ls="none", ms=5, label="DVL reading, both top beams"),
                   Line2D([], [], marker="o", color="#93c5fd", ls="none", ms=5, label="DVL reading, one top beam"),
                   Line2D([], [], marker="s", color="#f59e0b", ls="none", ms=5, label="DVL reading, lower beams only"),
                   Line2D([], [], marker="o", mfc="none", mec="#374151", ls="none", ms=7, label="hollow = reading rejected"),
                   plt.Rectangle((0, 0), 1, 1, fc="#fde68a", alpha=0.6, label="push (0.3 m/s away)"),
                   plt.Rectangle((0, 0), 1, 1, fc="#e5e7eb", label="distance invalid → hold")],
          loc="center right", fontsize=8, ncol=2, bbox_to_anchor=(1.0, 0.6))

# ---- C: speed toward wall
ax = fig.add_subplot(gs[2])
ax.axvspan(*EV["push"], color="#fde68a", alpha=0.5, lw=0); ax.axvspan(*hold, color="#e5e7eb", lw=0)
ax.axhline(0, color="#374151", lw=0.8)
ax.plot(c[:, 0], c[:, 1], color=G, lw=2, label="commanded by the controller")
ax.plot(c[:, 0], -c[:, 2], color="#7c3aed", lw=1.4, label="actual (includes the push)")
for tp, num, _ in PH: ax.axvline(tp, color="#1f2937", lw=0.8, ls=":")
ax.set_xlim(T0, T1); ax.set_ylim(-0.35, 0.35); ax.set_ylabel("speed toward\nthe wall (m/s)"); ax.grid(alpha=0.25)
ax.legend(fontsize=9, loc="lower right")
ax.text(T0 + 0.3, -0.33, "negative = moving away from the wall", fontsize=8, color="#4b5563")
ax.set_title("C. Speed toward the wall (new version)", loc="left", fontsize=12, weight="bold")

# ---- D: beams
ax = fig.add_subplot(gs[3])
nbv = m[wm, 3]; tt = m[wm, 0]
for nb, y, col, lab in ((2, 2, "#2563eb", "both top"), (1, 1.5, "#93c5fd", "one top"), (-1, 1, "#f59e0b", "lower only"), (0, 0.5, "#6b7280", "none")):
    sel = nbv == nb; ax.scatter(tt[sel], np.full(sel.sum(), y), marker="|", s=120, color=col)
ax.set_yticks([2, 1.5, 1, 0.5]); ax.set_yticklabels(["both top", "one top", "lower only", "none"], fontsize=8)
ax.set_xlim(T0, T1); ax.set_ylim(0.2, 2.3); ax.set_xlabel("time (s)"); ax.grid(alpha=0.25, axis="x")
for tp, num, _ in PH: ax.axvline(tp, color="#1f2937", lw=0.8, ls=":")
ax.set_title("D. Beams the code used for each DVL message", loc="left", fontsize=12, weight="bold")
fig.suptitle("How the ROV comes back to the wall after the push (simulation, new version)", fontsize=14, weight="bold", y=1.0)
fig.text(0.5, 0.972, "Same run as the full trajectory figure: pushed 0.3 m/s away for 4 s (120–124 s), top beams lost 119.5–124.5 s, DVL data on time.", ha="center", fontsize=10, color="#4b5563")
fig.savefig(f"{S}/nav_return_after_push.png", dpi=100, bbox_inches="tight")
print("hold", hold, "accepted", acc)
