"""Draws docs/distance_current_vs_proposed.svg (old vs new method).

    python3 tools/draw_method_diagram.py docs/distance_current_vs_proposed.svg
    (PNG: pip install cairosvg; cairosvg docs/distance_current_vs_proposed.svg -o docs/distance_current_vs_proposed.png -s 2)
"""
import math, sys
out = sys.argv[1]
W, H = 1200, 1130
a = math.radians(22.5); s = math.sin(a)/math.sqrt(2); c = math.cos(a)
psi = math.radians(15); Hm = 0.8
# beams 2,3 horizontal comps (y=-s / +s); normal dot u for yawed ROV
nu = {2: c*math.cos(psi) - s*math.sin(psi), 3: c*math.cos(psi) + s*math.sin(psi)}
d = {k: Hm/v for k, v in nu.items()}
cur = {k: d[k]*c for k in d}; cur_both = (cur[2]+cur[3])/2
print({k: round(v,3) for k,v in d.items()}, {k: round(v,3) for k,v in cur.items()}, round(cur_both,3))
el = []
def t(x, y, txt, size=11.5, w=400, fill="#4b5563", anchor="middle", extra=""):
    el.append(f'<text x="{x}" y="{y}" text-anchor="{anchor}" font-size="{size}" font-weight="{w}" fill="{fill}" {extra}>{txt}</text>')
def box(x, y, w, h, title, lines, kind="plain"):
    st = {"plain": ('#ffffff', '#6b7280', 1.5), "bad": ('#fef2f2', '#b91c1c', 2.2),
          "new": ('#f0fdf4', '#15803d', 2.2), "out": ('#eef2ff', '#4338ca', 1.8)}[kind]
    el.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="{st[0]}" stroke="{st[1]}" stroke-width="{st[2]}"/>')
    tc = {"bad": "#7f1d1d", "new": "#14532d", "out": "#312e81"}.get(kind, "#111827")
    t(x+w/2, y+22, title, 14, 700, tc)
    for i, l in enumerate(lines):
        t(x+w/2, y+40+i*16, l)
def arrow(x, y1, y2):
    el.append(f'<line x1="{x}" y1="{y1}" x2="{x}" y2="{y2}" stroke="#4b5563" stroke-width="1.6" marker-end="url(#ah)"/>')
BW, BH, GAP = 520, 62, 18
def column(x0, y0, header, items):
    t(x0+BW/2, y0, header, 16, 700, "#111827")
    y = y0+16
    for i, (title, lines, kind) in enumerate(items):
        box(x0, y, BW, BH, title, lines, kind)
        if i < len(items)-1: arrow(x0+BW/2, y+BH+1, y+BH+GAP-1)
        y += BH+GAP
    return y
L, R = 40, 640
cur_items = [
 ("DVL message /dvl/data", ["4 beams, each with distance + valid flag"], "plain"),
 ("Keep beams 2 and 3 only", ["hardcoded use_top_beams_only = True (line 134)"], "plain"),
 ("Drop invalid beams", ["leaves 2, 1 or 0 beams, and this changes from message to message"], "plain"),
 ("Distance = mean of  d × cos 22.5°", ["assumes the ROV is square to the hull",
   "(the yaw correction is passed in but has no effect)"], "bad"),
 ("No valid beam  →  distance = 0.01 m", ["controller sees 1 cm, far from the 0.8 m target"], "bad"),
 ("Jump filter: reject if > 0.5 m from last-5 mean", ["5–20 cm jumps pass straight through",
   "rejected values never enter the history, so it can lock"], "bad"),
 ("Guided controller", ["EMA α = 0.15, target 0.8 m, deadband 0.15 m"], "out"),
]
new_items = [
 ("DVL message /dvl/data", ["4 beams, each with distance + valid flag"], "plain"),
 ("Top beams 2 and 3; lower beams 0 and 1 as backup", ["drop invalid beams and readings ≥ 2.9 m (DVL range limit)"], "new"),
 ("Both top valid  →  hull angle from the two hit points", ["the line through the two hits is the hull; no compass needed",
   "angle > 35° means the beams disagree  →  keep the consistent one"], "new"),
 ("Perpendicular distance per beam:  D = d × (n · u)", ["n = hull normal, u = beam direction",
   "both beams now give the same D"], "new"),
 ("One top beam  →  use it with the last hull angle", ["no top beam  →  lower beams with the last hull angle"], "new"),
 ("Gate: reject if > 0.15 m from the estimate", ["accept a change that stays consistent for 1.5 s,",
   "or at once if the DVL velocity confirms it  →  cannot lock"], "new"),
 ("Small Kalman filter (distance + rate)", ["nothing accepted for 3 s  →  distance marked invalid (no 0.01 m)"], "new"),
 ("Guided controller (updated)", ["holds still when invalid; deadband 0.05 m + small integral term"], "out"),
]
yc = column(L, 70, "Old  (main branch)", cur_items)
yn = column(R, 70, "New  (fix/hull-distance branch)", new_items)
gy = max(yc, yn) + 10
def geom(x0, proposed):
    GH = 305
    el.append(f'<rect x="{x0}" y="{gy}" width="{BW}" height="{GH}" rx="10" fill="#f9fafb" stroke="#d1d5db" stroke-width="1.2"/>')
    t(x0+BW/2, gy+22, "Top view: ROV turned 15° from the hull, true standoff 0.80 m", 12.5, 700, "#374151")
    sc = 190; hy = gy+60; ox = x0+200; oy = hy + Hm*sc
    el.append(f'<rect x="{x0+20}" y="{hy-14}" width="{BW-40}" height="14" fill="url(#hatch)" stroke="none"/>')
    el.append(f'<line x1="{x0+20}" y1="{hy}" x2="{x0+BW-20}" y2="{hy}" stroke="#374151" stroke-width="3"/>')
    t(x0+BW-30, hy-20, "hull", 11.5, 700, "#374151", "end")
    # ROV body rotated by psi (clockwise in screen = turned right)
    el.append(f'<g transform="translate({ox},{oy}) rotate({math.degrees(psi)})"><rect x="-26" y="-6" width="52" height="40" rx="6" fill="#e5e7eb" stroke="#6b7280" stroke-width="1.5"/><circle cx="0" cy="0" r="4" fill="#111827"/></g>')
    th = math.atan2(s, c)  # horizontal beam angle from boresight
    hits = {}
    for k, sign in ((2, -1), (3, +1)):
        ang = psi + sign*th  # from hull normal, + = right
        # beam 2 is the longer one: larger angle from normal
        hits[k] = ang
    # make beam 2 the larger |angle|
    a2, a3 = (psi+th, psi-th)
    for k, ang in ((2, a2), (3, a3)):
        hx = ox + Hm*math.tan(ang)*sc
        el.append(f'<line x1="{ox}" y1="{oy}" x2="{hx:.1f}" y2="{hy}" stroke="#2563eb" stroke-width="2"/>')
        el.append(f'<circle cx="{hx:.1f}" cy="{hy}" r="4" fill="#2563eb"/>')
        lx = hx + (14 if k == 2 else -10)
        t(lx, hy+42 if k==2 else hy+30, f"beam {k}: d = {d[k]:.2f} m", 11, 700, "#1d4ed8", "start" if k==2 else "end")
    if not proposed:
        bx = ox + math.sin(psi)*Hm*1.05*sc; by = oy - math.cos(psi)*Hm*1.05*sc
        el.append(f'<line x1="{ox}" y1="{oy}" x2="{bx:.1f}" y2="{by:.1f}" stroke="#b91c1c" stroke-width="1.8" stroke-dasharray="6 4"/>')
        el.append(f'<line x1="{x0+30}" y1="{hy+112}" x2="{x0+60}" y2="{hy+112}" stroke="#b91c1c" stroke-width="1.8" stroke-dasharray="6 4"/>')
        t(x0+66, hy+108, "direction the code", 11, 700, "#b91c1c", "start")
        t(x0+66, hy+122, "treats as perpendicular", 11, 700, "#b91c1c", "start")
        rows = [("both beams", cur_both), ("beam 2 only", cur[2]), ("beam 3 only", cur[3])]
        col = "#b91c1c"; head = "Reported distance (true 0.80 m):"
        foot = f"one beam dropping out  →  about {abs(cur[2]-cur_both)*100:.0f} cm jump, more at larger angles"
    else:
        el.append(f'<line x1="{ox}" y1="{oy}" x2="{ox}" y2="{hy}" stroke="#15803d" stroke-width="2.4" stroke-dasharray="7 4"/>')
        el.append(f'<path d="M{ox} {hy+12} h12 v-12" fill="none" stroke="#15803d" stroke-width="1.5"/>')
        t(ox-10, hy+95, "D = 0.80 m", 12, 700, "#15803d", "end")
        rows = [("both beams", Hm), ("beam 2 only", Hm), ("beam 3 only", Hm)]
        col = "#15803d"; head = "Reported distance (true 0.80 m):"
        foot = "one beam dropping out  →  no jump (uses the last hull angle)"
    tx = x0+340; ty = gy+150
    t(tx, ty, head, 11.5, 700, "#374151", "start")
    for i, (n, v) in enumerate(rows):
        t(tx, ty+20+i*18, n, 11.5, 400, "#374151", "start")
        t(tx+150, ty+20+i*18, f"{v:.2f} m", 12, 700, col, "end")
    t(x0+BW/2, gy+GH-14, foot, 11.5, 700, col)
geom(L, False); geom(R, True)
ly = gy + 305 + 26
el.append(f'<rect x="{L}" y="{ly}" width="36" height="20" rx="5" fill="#fef2f2" stroke="#b91c1c" stroke-width="2.2"/>')
t(L+46, ly+15, "step that causes or lets through the jumps", 12, 400, "#374151", "start")
el.append(f'<rect x="{R}" y="{ly}" width="36" height="20" rx="5" fill="#f0fdf4" stroke="#15803d" stroke-width="2.2"/>')
t(R+46, ly+15, "new or changed step", 12, 400, "#374151", "start")
H = ly + 44
svg = f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-family="Helvetica, Arial, sans-serif">
<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="#4b5563"/></marker>
<pattern id="hatch" width="8" height="8" patternUnits="userSpaceOnUse" patternTransform="rotate(45)"><line x1="0" y1="0" x2="0" y2="8" stroke="#9ca3af" stroke-width="2"/></pattern></defs>
<rect width="{W}" height="{H}" fill="#ffffff"/>
<text x="{W/2}" y="34" text-anchor="middle" font-size="20" font-weight="700" fill="#111827">DVL distance to the hull: old vs new method</text>
{chr(10).join(el)}
</svg>'''
open(out, "w").write(svg)
