import sys, numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle; from matplotlib.lines import Line2D
from sim_hull_following import *
S = sys.argv[1]; sc = scenario(); L = {m: run(m, sc) for m in CFG}; ST = {m: stats(L[m]) for m in CFG}
COL = {"old": "#b91c1c", "new": "#15803d", "new_v1": "#9ca3af"}
SHAPE = [(-1.6, 0.6), (1.6, 0.6), (1.6, -0.6), (0.4, -0.6), (0, -1.2), (-0.4, -0.6), (-1.6, -0.6)]
def rov(ax, x, y, yaw, col):
    ax.plot([x], [y], marker=MarkerStyle(SHAPE).rotated(deg=yaw), ms=22, mfc="white", mec=col, mew=1.6, zorder=5, ls="none")
fig = plt.figure(figsize=(15, 13.5))
gs = fig.add_gridspec(4, 1, height_ratios=[1.25, 1.25, 1.0, 0.55], hspace=0.42)
TITLE = {"old": "A. OLD version (old distance code, old controller)", "new": "B. NEW version (new distance code + latest changes, updated controller)"}
def at(d, t): i = np.argmin(np.abs(d[:, 0] - t)); return d[i, 1], d[i, 2]
for row, m in enumerate(("old", "new")):
    ax = fig.add_subplot(gs[row]); d = L[m]
    db = CFG[m]["deadband"]
    ax.axhspan(0.8 - db, 0.8 + db, color="#dbeafe", lw=0, zorder=0)
    ax.axhline(0.8, color="#60a5fa", lw=1, ls=":", zorder=1)
    ax.fill_between([-5, 70], -0.12, 0, color="#9ca3af", hatch="////", ec="#6b7280", lw=0, zorder=1)
    ax.axhline(0, color="#374151", lw=2.5, zorder=2)
    ax.plot(d[:, 1], d[:, 3], color=COL[m], lw=1.3, alpha=0.8, ls=(0, (2, 2)), zorder=3)
    ax.plot(d[:, 1], d[:, 2], color=COL[m], lw=2.6, zorder=4)
    for tm in range(0, int(T_END) + 1, 20):
        i = np.argmin(np.abs(d[:, 0] - tm)); rov(ax, d[i, 1], d[i, 2], d[i, 5], COL[m])
        ax.text(d[i, 1], d[i, 2] - 0.3, f"{tm}s", ha="center", fontsize=8, color="#374151", zorder=6)
    inv = d[:, 4] == 0
    if m != "old": ax.plot(d[inv, 1], d[inv, 2], ".", color="#6b7280", ms=3, zorder=4)
    for key, lab in (("start", "1  start, top beams\n   not locked"), ("turn", "2  ROV turned up to 18°,\n   beams dropping out"), ("push", "3  pushed away, no top\n   beams for 5 s")):
        a, b = EV[key]; xa = np.interp(a, d[:, 0], d[:, 1]); xb = np.interp(b, d[:, 0], d[:, 1])
        ax.axvspan(xa, max(xb, xa + 0.4), color="#fde68a", alpha=0.45, lw=0, zorder=0)
        ax.text(xa + 0.2, 2.45, lab, fontsize=8.5, va="top", color="#92400e")
    ax.set_xlim(-1, 62); ax.set_ylim(-0.12, 2.5); ax.set_ylabel("distance from wall (m)")
    ax.set_title(TITLE[m], loc="left", fontsize=12, color=COL[m], weight="bold")
    ax.text(60.5, 0.03, "wall / hull", ha="right", va="bottom", fontsize=9, color="#374151", zorder=6)
    kw = dict(fontsize=8.5, color=COL[m], arrowprops=dict(arrowstyle="->", color=COL[m]))
    if m == "old":
        ax.annotate("reports 0.01 m before the beams lock\n→ backs away to 1.8 m", xy=at(d, 8), xytext=(4.5, 1.55), **kw)
        ax.annotate("stuck: reports 0.88 m while the ROV is at 2.1 m\n(new readings > 0.5 m from the history → all rejected)", xy=at(d, 160), xytext=(39, 1.25), **kw)
    else:
        ax.annotate("start: lower beams give the distance\n→ starts moving at once, no hold", xy=at(d, 3), xytext=(2.5, 1.35), **kw)
        ax.annotate("push not seen while it happens (readings jump\ntoo fast); invalid ~2 s → holds; new distance\naccepted → comes back", xy=at(d, 125.5), xytext=(17.5, 1.3), **kw)
        ax.annotate(f"back near 0.8 m (median {ST[m]['post']:.2f} m,\nslow ±5 cm swing from the integral term)", xy=at(d, 175), xytext=(46, 1.25), **kw)
    ax.grid(alpha=0.25)
    if row == 1: ax.set_xlabel("position along the wall (m)")
ax = fig.add_subplot(gs[2])
ax.axhspan(0.65, 0.95, color="#dbeafe", lw=0)
lab = {"old": "old version", "new_v1": "new, before these changes (shown last time)", "new": "new version"}
for m in ("old", "new_v1", "new"):
    ax.plot(L[m][:, 0], L[m][:, 2], color=COL[m], lw=1.6 if m == "new_v1" else 2.2, ls="--" if m == "new_v1" else "-", label=lab[m], zorder=3 if m == "new" else 2)
for key in EV: ax.axvspan(*EV[key], color="#fde68a", alpha=0.45, lw=0)
ax.set_xlabel("time (s)"); ax.set_ylabel("distance (m)"); ax.set_ylim(0, 2.5); ax.grid(alpha=0.25); ax.legend(fontsize=9, loc="upper left")
ax.set_title("C. Actual distance from the wall over time (target 0.8 m, blue band = ±0.15 m)", loc="left", fontsize=12)
axt = fig.add_subplot(gs[3]); axt.axis("off")
cells = [["", "error RMS", "within ±0.15 m", "within ±0.05 m", "time holding\n(distance invalid)", "back within ±0.15 m\nafter the push", "distance\n150-200 s", "distance travelled"]]
for m in ("old", "new_v1", "new"):
    s = ST[m]; w = L[m][:, 0] > 150; s["postmin"], s["postmax"] = L[m][w, 2].min(), L[m][w, 2].max()
    cells.append([lab[m].replace(" (shown last time)", ""), f"{s['rms']:.0f} cm", f"{s['band']:.0f}%", f"{s['band05']:.0f}%", f"{s['hold']:.1f}%",
                  "never" if np.isnan(s["back"]) else f"{s['back'] - EV['push'][1]:.0f} s after", (f"{s['postmin']:.2f}-{s['postmax']:.2f} m" if s['postmax'] - s['postmin'] > 0.02 else f"{s['post']:.2f} m"), f"{s['travel']:.0f} m"])
tb = axt.table(cellText=cells, loc="center", cellLoc="center"); tb.auto_set_font_size(False); tb.set_fontsize(10); tb.scale(1, 2.1)
fill = ["#fef2f2", "#f3f4f6", "#f0fdf4"]
for j in range(8):
    tb[0, j].set_text_props(weight="bold"); tb[0, j].set_height(0.32)
    for r in (1, 2, 3): tb[r, j].set_facecolor(fill[r - 1])
tb.auto_set_column_width(list(range(8)))
fig.legend(handles=[Line2D([], [], color="#111827", lw=2.6, label="actual ROV path"),
                    Line2D([], [], color="#111827", lw=1.3, ls=(0, (2, 2)), label="distance the code reported to the controller"),
                    Line2D([], [], color="#6b7280", marker=".", ls="none", ms=8, label="new: distance invalid → hold"),
                    Line2D([], [], color="#6b7280", marker=MarkerStyle(SHAPE), mfc="white", ls="none", ms=14, label="ROV every 20 s (nose toward the wall)"),
                    plt.Rectangle((0, 0), 1, 1, fc="#dbeafe", label="target 0.8 m ± deadband (old 0.15 m, new 0.05 m)"),
                    plt.Rectangle((0, 0), 1, 1, fc="#fde68a", alpha=0.6, label="event")],
           loc="upper center", ncol=3, fontsize=9, bbox_to_anchor=(0.5, 0.955), frameon=False)
fig.text(0.5, 0.965, "Same scenario as before: DVL data on time, same turn, dropouts, reflections and push. During top-beam gaps each lower beam is valid 30% of the time (as in the bags).", ha="center", fontsize=10, color="#4b5563")
fig.suptitle("Simulated hull following: OLD vs NEW version", fontsize=14, weight="bold", y=0.995)
fig.savefig(f"{S}/nav_trajectory_old_vs_new_v2.png", dpi=100, bbox_inches="tight")
for r in cells: print(r)
