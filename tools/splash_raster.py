# Python mirror of metal_hammer's splash film v3: goth wizard shredding on a storm mountain, lightning, logo.
# Grey render -> hard contrast -> Bayer 4x4 + grain, same pipeline as the firmware.
import math, random
import numpy as np
from PIL import Image, ImageDraw

W = 128
G = np.zeros((W, W), np.float32)
cam = dict(s=1.0, x=64.0, y=64.0, sx=0, sy=0)
pnt = dict(b=1.0, gx=0.0, gy=0.0, ox=64.0, oy=64.0, chrome=False, top=0.0, h=1.0)
fx = dict(lift=0.0, fade=1.0)                 # lift: lightning flash on everything; fade: logo fade-in
LGX, LGY = 0.010, -0.008

def paint(b, gx=0.0, gy=0.0, ox=64.0, oy=64.0): pnt.update(b=b, gx=gx, gy=gy, ox=ox, oy=oy, chrome=False)
def lit(b, k=1.0): paint(b, LGX * k, LGY * k, 60, 60)
def chrome(top, h): pnt.update(chrome=True, top=top, h=h)

def EX(x): return 64 + (x - cam['x']) * cam['s'] + cam['sx']
def EY(y): return 64 + (y - cam['y']) * cam['s'] + cam['sy']
def TX(x): return EX(x + 0.5)
def TY(y): return EY(y + 0.5)
def WX(sx): return cam['x'] + (sx + 0.5 - 64 - cam['sx']) / cam['s'] - 0.5
def WY(sy): return cam['y'] + (sy + 0.5 - 64 - cam['sy']) / cam['s'] - 0.5

def chromeFn(v):
    if v < 0.5: return 1.1 - v * 0.9
    return 0.5 + (v - 0.5) * 1.2

def hspan(xa, xb, y):
    if y < 0 or y > 127: return
    if xa > xb: xa, xb = xb, xa
    x0 = math.ceil(xa - 0.5); x1 = math.floor(xb - 0.5)
    if x1 < x0: x0 = x1 = math.floor((xa + xb) * 0.5)
    if x1 < 0 or x0 > 127: return
    x0 = max(x0, 0); x1 = min(x1, 127); n = x1 - x0 + 1
    wy = WY(y)
    if pnt['chrome']:
        G[y, x0:x1 + 1] = chromeFn((wy - pnt['top']) / pnt['h']) * fx['fade']
        return
    wx0 = WX(x0); d = 1 / cam['s']
    g0 = pnt['b'] + pnt['gx'] * (wx0 - pnt['ox']) + pnt['gy'] * (wy - pnt['oy'])
    if pnt['b'] > 0: g0 += fx['lift']
    G[y, x0:x1 + 1] = g0 + pnt['gx'] * d * np.arange(n)

def rTri(ax, ay, bx, by, cx, cy):
    (ay, ax), (by, bx), (cy, cx) = sorted([(ay, ax), (by, bx), (cy, cx)])
    for y in range(max(0, math.ceil(ay - 0.5)), min(127, math.floor(cy - 0.5)) + 1):
        yc = y + 0.5; xa = ax + (cx - ax) * (yc - ay) / max(cy - ay, 1e-4)
        xb = ax + (bx - ax) * (yc - ay) / max(by - ay, 1e-4) if yc < by else bx + (cx - bx) * (yc - by) / max(cy - by, 1e-4)
        hspan(xa, xb, y)
def rDisc(cx, cy, r):
    R = r + 0.5
    for y in range(max(0, math.floor(cy - R)), min(127, math.ceil(cy + R)) + 1):
        dy = y + 0.5 - cy
        if abs(dy) > R: continue
        dx = math.sqrt(R * R - dy * dy); hspan(cx - dx, cx + dx, y)
def cLine(x0, y0, x1, y1, t=1.0):
    a, b, c, d = TX(x0), TY(y0), TX(x1), TY(y1); w = max(t * cam['s'], 1.0)
    L = math.hypot(c - a, d - b) + 1e-4; nx = -(d - b) / L * w * 0.5; ny = (c - a) / L * w * 0.5
    rTri(a + nx, b + ny, a - nx, b - ny, c + nx, d + ny); rTri(c + nx, d + ny, c - nx, d - ny, a - nx, b - ny)
    if w >= 2: rDisc(a, b, w * 0.5 - 0.5); rDisc(c, d, w * 0.5 - 0.5)
def cBox(x, y, w, h):
    x0, x1, y0, y1 = EX(x), EX(x + w), EY(y), EY(y + h)
    ya = max(0, math.ceil(y0 - 0.5)); yb = min(127, math.floor(y1 - 0.5))
    if yb < ya: ya = yb = math.floor((y0 + y1) * 0.5)
    for yy in range(ya, yb + 1): hspan(x0, x1 - 0.001, yy)
def cPix(x, y): cBox(x, y, 1, 1)
def cDisc(x, y, r): rDisc(TX(x), TY(y), r * cam['s'])
def cTri(*v): rTri(TX(v[0]), TY(v[1]), TX(v[2]), TY(v[3]), TX(v[4]), TY(v[5]))

rng = random.Random(7)
NZ = np.array([[rng.random() for _ in range(32)] for _ in range(32)], np.float32)
def vnoise(x, y):
    xi = np.floor(x).astype(int); yi = np.floor(y).astype(int); fx_ = x - xi; fy = y - yi
    fx_ = fx_ * fx_ * (3 - 2 * fx_); fy = fy * fy * (3 - 2 * fy)
    a = NZ[yi & 31, xi & 31]; b = NZ[yi & 31, (xi + 1) & 31]; c = NZ[(yi + 1) & 31, xi & 31]; d = NZ[(yi + 1) & 31, (xi + 1) & 31]
    return a + (b - a) * fx_ + (c - a) * fy + (a - b - c + d) * fx_ * fy

def ridgeFar(wx): return 97 - 9 * np.abs(np.sin(wx * 0.045)) - 6 * np.sin(wx * 0.13 + 1) - 3 * np.abs(np.sin(wx * 0.37))
def ridgeMid(wx): return 110 - 6 * np.abs(np.sin(wx * 0.06 + 2)) - 4 * np.sin(wx * 0.19) - 2 * np.abs(np.sin(wx * 0.5))

MOON = (64, 50, 27)
def sky(tb, flash):
    sx = np.arange(W); wx = WX(sx)[None, :]; wy = WY(sx)[:, None]
    c = 0.6 * vnoise(wx * 0.035 + tb * 0.25, wy * 0.05 + 3) + 0.4 * vnoise(wx * 0.09 - tb * 0.45, wy * 0.11)
    cl = np.clip((c - 0.42) / 0.35, 0, 1)
    g = 0.06 + 0.26 * np.clip(wy / 100, 0, 1) ** 1.6 + (0.16 + 0.4 * flash) * (cl - 0.35) + 0.25 * flash
    dm = np.hypot(wx - MOON[0], wy - MOON[1])
    crat = vnoise(wx * 0.16 + 5, wy * 0.16 + 9)
    moon = 1.0 - 0.28 * crat ** 2 - 0.2 * (dm / MOON[2]) ** 6 - 0.12 * cl
    g = np.where(dm < MOON[2], moon, g + 0.32 * np.exp(-np.maximum(dm - MOON[2], 0) / 10))
    rf = ridgeFar(wx); rm = ridgeMid(wx); e = 1.2 / cam['s']
    g = np.where(wy > rf, np.where(wy < rf + e, 0.6, 0.16 + 0.15 * flash), g)
    g = np.where(wy > rm, np.where(wy < rm + e, 0.45, 0.05 + 0.06 * flash), g)
    G[:, :] = g

def pylon(x, base, h):
    paint(0.0)
    top = base - h
    cLine(x - 4, base, x - 1, top, 0.8); cLine(x + 4, base, x + 1, top, 0.8)
    for k in range(1, 5):
        yy = base - h * k / 5; w = 4 - 3 * k / 5
        cLine(x - w, yy, x + w, yy - h / 5, 0.5); cLine(x + w, yy, x - w, yy - h / 5, 0.5)
    cLine(x - 6, top + 2, x + 6, top + 2, 0.8); cLine(x - 4, top + 5, x + 4, top + 5, 0.7)
    return (x - 6, top + 2), (x + 6, top + 2)

def cable(a, b, sag):
    paint(0.0)
    px, py = a
    for i in range(1, 11):
        u = i / 10; x = a[0] + (b[0] - a[0]) * u; y = a[1] + (b[1] - a[1]) * u + sag * 4 * u * (1 - u)
        cLine(px, py, x, y, 0.5); px, py = x, y

def world(tb, t, raise_, shred):
    # industrial silhouettes on the mid ridge
    rm = lambda x: float(ridgeMid(np.array(x)))
    l1, r1 = pylon(-22, rm(-22) + 2, 24)
    l2, r2 = pylon(150, rm(150) + 2, 22)
    cable(r1, l2, 9); cable((l1[0], l1[1]), (-90, l1[1] + 8), 6); cable(r2, (220, r2[1] + 6), 6)
    # smokestack + plume + blinking light
    sx_, sb = 14, float(ridgeFar(np.array(14.0))) + 2
    paint(0.0); cTri(sx_ - 3, sb, sx_ + 3, sb, sx_ + 1.5, sb - 26); cTri(sx_ - 3, sb, sx_ + 1.5, sb - 26, sx_ - 1.5, sb - 26)
    for i in range(7):
        ph = math.fmod(tb * 0.25 + i / 7, 1.0)
        paint(0.05 + 0.12 * ph); cDisc(sx_ - ph * 30 + math.sin(tb + i) * 2, sb - 27 - ph * 14, 2 + ph * 6)
    if (t // 500) % 2: paint(1.0); cPix(sx_, sb - 27)
    # the peak: jagged rock, dark, rim-lit on the right edges
    pk = [(-140, 200), (-10, 122), (8, 108), (20, 111), (34, 96), (44, 98), (56, 82), (64, 78), (72, 81), (80, 92), (92, 90), (104, 104), (118, 102), (140, 118), (270, 200)]
    for i in range(len(pk) - 1):
        (ax, ay), (bx, by) = pk[i], pk[i + 1]
        paint(0.06); cTri(ax, ay, bx, by, bx, 210); cTri(ax, ay, bx, 210, ax, 210)
    for i in range(len(pk) - 1):
        (ax, ay), (bx, by) = pk[i], pk[i + 1]
        paint(0.85 if by > ay else 0.45); cLine(ax, ay, bx, by, 0.8)
    paint(0.3); cLine(60, 86, 52, 100, 0.5); cLine(70, 84, 82, 104, 0.5); cLine(30, 100, 24, 112, 0.5)
    return wizard(tb, t, raise_, shred)

def rPoly(P):                                   # scanline fill, even-odd, screen coords
    ys = [p[1] for p in P]
    for y in range(max(0, math.ceil(min(ys) - 0.5)), min(127, math.floor(max(ys) - 0.5)) + 1):
        yc = y + 0.5; xs = []
        for i in range(len(P)):
            (ax, ay), (bx, by) = P[i], P[(i + 1) % len(P)]
            if (ay <= yc < by) or (by <= yc < ay): xs.append(ax + (bx - ax) * (yc - ay) / (by - ay))
        xs.sort()
        for k in range(0, len(xs) - 1, 2): hspan(xs[k], xs[k + 1], y)
def cPoly(P): rPoly([(TX(x), TY(y)) for x, y in P])
def rimPoly(P, dark, rim, dx=-0.8, dy=0.6):     # bright copy, dark copy nudged away from the light = rim light
    paint(0.0); cPoly([(x + 0.7 * (x - sum(p[0] for p in P) / len(P)) / 12, y) for x, y in P]) if False else None
    paint(rim); cPoly(P)
    paint(dark); cPoly([(x + dx, y + dy) for x, y in P])

def wizard(tb, t, raise_, shred):
    wind = math.sin(tb * 2.3) * 0.5 + 0.5
    wv = lambda u, ph: math.sin(tb * 5 + u * 5 + ph)
    # cloak: shoulders -> front -> jagged hem -> billowing back edge
    cloak = [(57, 52), (71, 52), (73, 60), (74.5, 70), (77, 78.5), (73, 77), (70, 79), (66, 77.5), (62, 79.5), (58, 77.5)]
    for i in range(7):
        u = i / 6
        cloak.append((54 - u * 6 - (2 + 7 * u) * (0.65 + 0.35 * wv(u, 0)), 78 - u * 22 + (1.5 if i % 2 else 0) * u))
    cloak.append((55, 55))
    rimPoly(cloak, 0.07, 0.9)
    paint(0.22); cLine(64, 60, 61, 78, 0.5); cLine(67.5, 60, 68.5, 78, 0.5); cLine(60, 62, 54, 76, 0.4)   # folds
    # chains on the belt
    for i in range(9):
        u = i / 8; x = 59 + u * 11; y = 66 + 3 * 4 * u * (1 - u)
        paint(0.8 if i % 2 else 0.45); cDisc(x, y, 0.6 if i % 2 else 0.45)
    # hood: tall point swept back by the wind, open face
    tx_ = 55.5 - wind * 2; ty_ = 33.5 + wind * 0.8
    hood = [(70.8, 53), (70.4, 46), (68.5, 41.5), (65, 38.5), (60.5, 36), (tx_, ty_), (57.5, 37.5), (59, 41), (58.8, 46), (57.5, 53)]
    rimPoly(hood, 0.16, 0.95)
    face = [(69.2, 52.5), (68.8, 46), (66.5, 43.2), (64, 45), (63.6, 52.5)]
    paint(0.0); cPoly(face)
    glow = 0.6 + 0.4 * math.sin(tb * 6)
    for ex in (65.4, 67.9):
        paint(0.5 * glow); cDisc(ex, 47.6, 1.1); paint(1.0); cDisc(ex, 47.6, 0.45)
    # beard: tapered, whipping in the wind, with strands
    sway = lambda u: math.sin(tb * 8 + u * 4) * (0.5 + 2.2 * u) + wind * 3 * u
    left = [(63.8 - sway(i / 6) - i * 0.2, 50 + i * 2.6) for i in range(7)]
    right = [(68.8 - sway(i / 6) - i * 0.75, 50 + i * 2.6) for i in range(7)]
    beard = left + right[::-1]
    paint(0.0); cPoly([(x - 0.6, y) for x, y in beard] + [(x + 0.6, y) for x, y in beard[:0]])
    lit(0.62, 1.2); cPoly(beard)
    for k in range(4):
        paint(0.95 if k % 2 else 0.35)
        u0 = 0.15 + k * 0.2
        cLine(64.8 + k * 1.0 - sway(0.1), 51, 64.5 + k * 0.6 - sway(0.8) - 3, 64, 0.35)
    # spiked, riveted pauldrons
    for (px_, py_, side) in ((58.5, 55, -1), (71, 55, 1)):
        paint(0.0); cDisc(px_, py_, 4.2)
        lit(0.3, 1.5); cDisc(px_, py_, 3.4)
        for a in (-0.5, 0.2, 0.9):
            ang = -math.pi / 2 + side * a
            paint(0.6); cTri(px_ + math.cos(ang - 0.35) * 3, py_ + math.sin(ang - 0.35) * 3,
                              px_ + math.cos(ang + 0.35) * 3, py_ + math.sin(ang + 0.35) * 3,
                              px_ + math.cos(ang) * 5.5, py_ + math.sin(ang) * 5.5)
        paint(0.95); cPix(px_ - 1.5, py_ + 0.5); cPix(px_ + 1, py_ + 1.2)
    # guitar pose: shred (neck -25 deg at the hip) -> raised to the sky (neck straight up, overhead)
    ang = math.radians(-25 + (-90 + 25) * raise_)
    ux, uy = math.cos(ang), math.sin(ang); px, py = -uy, ux
    bx = 62 + (66 - 62) * raise_; by = 69 + (47 - 69) * raise_
    vx = lambda a, sp: bx - ux * a + px * sp
    vy = lambda a, sp: by - uy * a + py * sp
    body = [(-4, 0), (14, 10), (18, 13), (10, 4), (16, -6), (14, -12)]
    pts = [(vx(a, s), vy(a, s)) for a, s in body]
    paint(0.0)
    for i in range(1, len(pts) - 1): cTri(*pts[0], *pts[i], *pts[i + 1])
    for (a, s) in body: cDisc(vx(a, s), vy(a, s), 1.6)
    for i in range(1, len(pts) - 1): paint(0.4, ux * 0.025, uy * 0.025, bx, by); cTri(*pts[0], *pts[i], *pts[i + 1])
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]; paint(0.9); cLine(*a, *b, 0.5)
    nx, ny = bx + ux * 44, by + uy * 44
    paint(0.0); cLine(bx, by, nx, ny, 3.6); lit(0.6); cLine(bx, by, nx, ny, 1.8)
    hs = [(nx, ny), (nx + ux * 8 + px * 3, ny + uy * 8 + py * 3), (nx + ux * 5 - px * 2, ny + uy * 5 - py * 2)]
    paint(0.0); cTri(hs[0][0] - px, hs[0][1] - py, hs[1][0] + ux, hs[1][1] + uy, hs[2][0] - px, hs[2][1] - py)
    paint(0.9); cTri(*hs[0], *hs[1], *hs[2])
    if cam['s'] > 2:
        for i in (-1, 1):
            o = i * 0.5 + math.sin(tb * 80 + i) * 0.15 * shred
            paint(0.95 if shred else 0.2); cLine(bx + px * o, by + py * o, nx + px * o, ny + py * o, 0.3)
    # arms (sleeves) + bony hands
    fret = 26 + math.sin(tb * 7) * 8 * shred
    hx1, hy1 = bx + ux * fret, by + uy * fret
    hx2, hy2 = bx - ux * 1 + px * 2 + (math.sin(tb * 40) * 1.5 * shred), by - uy * 1 + py * 2
    for (sx_, sy_, ex, ey, hx_, hy_) in ((71, 56, 74, 63, hx1, hy1), (58, 56, 56, 64, hx2, hy2)):
        if raise_ > 0.5: ex, ey = (sx_ + hx_) / 2 + 3, (sy_ + hy_) / 2
        paint(0.0); cLine(sx_, sy_, ex, ey, 4.4); cLine(ex, ey, hx_, hy_, 4.4)
        paint(0.12); cLine(sx_, sy_, ex, ey, 2.6); cLine(ex, ey, hx_, hy_, 2.6)
        paint(0.85); cLine(sx_ + 0.8, sy_ - 0.6, ex + 0.8, ey - 0.6, 0.5); cLine(ex + 0.8, ey - 0.6, hx_ + 0.8, hy_ - 0.6, 0.5)
        paint(0.0); cDisc(hx_, hy_, 2); lit(0.75); cDisc(hx_, hy_, 1.3)
    # energy arcs off the strings while shredding
    if shred:
        r = random.Random(t // 40)
        for k in range(3):
            a = r.uniform(4, 40); x0, y0 = bx + ux * a, by + uy * a
            x1, y1 = x0 + r.uniform(-9, 9), y0 + r.uniform(-9, 3)
            px0, py0 = x0, y0
            for s in range(1, 5):
                u = s / 4; x = x0 + (x1 - x0) * u + r.uniform(-1.5, 1.5); y = y0 + (y1 - y0) * u + r.uniform(-1.5, 1.5)
                paint(1.0); cLine(px0, py0, x, y, 0.4); px0, py0 = x, y
    return (hs[1][0], hs[1][1])

def rain(t):
    r = random.Random(3)
    for i in range(28):
        x0 = r.uniform(0, 160); sp = r.uniform(140, 220); y = (r.uniform(0, 160) + t * 0.001 * sp) % 160 - 16
        x = (x0 - y * 0.35) % 140 - 6
        for k in range(5):
            xx, yy = int(x - k * 0.35), int(y + k)
            if 0 <= xx < W and 0 <= yy < W: G[yy, xx] = max(G[yy, xx], 0.42)

def boltPath(seed, x0, y0, x1, y1):
    r = random.Random(seed); pts = [(x0, y0), (x1, y1)]
    for lvl in range(5):
        npts = [pts[0]]
        for i in range(len(pts) - 1):
            (ax, ay), (bx, by) = pts[i], pts[i + 1]; d = math.hypot(bx - ax, by - ay)
            npts += [((ax + bx) / 2 + r.uniform(-0.25, 0.25) * d, (ay + by) / 2 + r.uniform(-0.1, 0.1) * d), (bx, by)]
        pts = npts
    return pts, r

def drawBolt(seed, x0, y0, x1, y1, core=1.0):
    pts, r = boltPath(seed, x0, y0, x1, y1)
    for i in range(len(pts) - 1): paint(0.55); cLine(*pts[i], *pts[i + 1], 7)
    for i in range(len(pts) - 1): paint(core); cLine(*pts[i], *pts[i + 1], 2.4)
    for b in range(4):                                                      # branches
        i = r.randrange(4, len(pts) - 6); (ax, ay) = pts[i]
        bp, _ = boltPath(seed * 7 + b, ax, ay, ax + r.uniform(-30, 30), ay + r.uniform(12, 30))
        for k in range(min(len(bp) - 1, 20)): paint(0.85); cLine(*bp[k], *bp[k + 1], 1.2)

# ---- the METAL HAMMER logo: spiky chrome metal-band letters ----
L = {
    'M': [(0, 14, 0, 0), (0, 0, 5, 8), (5, 8, 10, 0), (10, 0, 10, 14)],
    'E': [(0, 0, 0, 14), (0, 0, 9, 0), (0, 7, 7, 7), (0, 14, 9, 14)],
    'T': [(-1, 0, 11, 0), (5, 0, 5, 14)],
    'A': [(0, 14, 5, 0), (5, 0, 10, 14), (2.3, 9, 7.7, 9)],
    'L': [(0, 0, 0, 14), (0, 14, 10, 14)],
    'H': [(0, 0, 0, 14), (10, 0, 10, 14), (0, 7, 10, 7)],
    'R': [(0, 0, 0, 14), (0, 0, 8, 0), (8, 0, 10, 3.5), (10, 3.5, 8, 7), (8, 7, 0, 7), (4, 7, 10, 14)],
}
SPIKES = {  # word, letter index: list of (x0,y0 base1, x1,y1 base2, tipx,tipy) in letter units
    ('METAL', 0): [(-1.3, 12, 1.3, 12, -3, 22)],
    ('METAL', 4): [(8, 12.7, 8, 15.3, 16, 14)],
    ('METAL', 2): [(-1, -1.3, -1, 1.3, -5, 0), (11, -1.3, 11, 1.3, 15, 0)],
    ('HAMMER', 0): [(-1.3, 2, 1.3, 2, -3, -8)],
    ('HAMMER', 5): [(9, 12, 11, 15, 16, 22)],
}
def logoStrokes(word, x0, y0, sc):
    out = []
    for i, ch in enumerate(word):
        lx = x0 + i * 14 * sc
        for (a, b, c, d) in L[ch]: out.append(('l', lx + a * sc, y0 + b * sc, lx + c * sc, y0 + d * sc))
        for sp in SPIKES.get((word, i), []):
            out.append(('t',) + tuple(lx + sp[k] * sc if k % 2 == 0 else y0 + sp[k] * sc for k in range(6)))
    return out

def logo(tb, t, fade):
    cam.update(s=1.0, x=64, y=64, sx=cam['sx'], sy=cam['sy'])
    sc = 1.28; lines = [('METAL', 64 - (5 * 14 - 4) * sc / 2, 27), ('HAMMER', 64 - (6 * 14 - 4) * sc / 2, 62)]
    strokes = [(s, y0) for (w, x0, y0) in lines for s in logoStrokes(w, x0, y0, sc)]
    fx['fade'] = fade
    for s, y0 in strokes:                                                    # black outlines first
        paint(0.0)
        if s[0] == 'l': cLine(*s[1:], 2.5 * sc + 2.4)
        else:
            cx = (s[1] + s[3] + s[5]) / 3; cy = (s[2] + s[4] + s[6]) / 3
            g = lambda x, y: (x + (x - cx) * 0.35, y + (y - cy) * 0.35)
            cTri(*g(s[1], s[2]), *g(s[3], s[4]), *g(s[5], s[6]))
    for s, y0 in strokes:                                                    # chrome fill
        chrome(y0 - 2, 14 * sc + 4)
        if s[0] == 'l': cLine(*s[1:], 2.5 * sc)
        else: cTri(*s[1:])
    # blade between the lines
    paint(0.0); cTri(4, 54.5, 124, 54.5, 64, 50); cTri(4, 54.5, 124, 54.5, 64, 59)
    chrome(50, 9); cTri(8, 54.5, 120, 54.5, 64, 51.5); cTri(8, 54.5, 120, 54.5, 64, 57.5)
    fx['fade'] = 1.0
    # electricity crawling over the letters
    r = random.Random(t // 60)
    for k in range(2):
        s, _ = strokes[r.randrange(len(strokes))]; s2, _ = strokes[r.randrange(len(strokes))]
        a = (s[1], s[2]); b = (s2[3], s2[4])
        if math.hypot(b[0] - a[0], b[1] - a[1]) > 40: continue
        pts, _ = boltPath(t // 60 * 13 + k, *a, *b)
        for i in range(len(pts) - 1): paint(1.0 * fade); cLine(*pts[i], *pts[i + 1], 0.5)

B4 = np.array([[0, 8, 2, 10], [12, 4, 14, 6], [3, 11, 1, 9], [15, 7, 13, 5]], np.float32)
def dither(t):
    g = np.clip((G - 0.45) * 1.5 + 0.45, 0, 1)
    thr = (np.tile(B4, (32, 32)) + 0.5) / 16
    grain = np.random.default_rng(t).uniform(-0.07, 0.07, (W, W))
    return g > np.maximum(thr + grain, 0.03)

def ease(x): x = min(max(x, 0), 1); return 1 - (1 - x) ** 3
def smooth(x): x = min(max(x, 0), 1); return x * x * (3 - 2 * x)

def frame(u, T0=10000):
    t = T0 + int(u * 1000); tb = t * 0.001
    cam.update(sx=0, sy=0); fx.update(lift=0.0, fade=1.0)
    raise_ = 0.0; shred = False; flash = 0.0; letterbox = True; logoOn = False; white = None
    # distant lightning flickers in the establishing shot
    if 1.0 < u < 1.12 or 1.2 < u < 1.26: flash = 0.6
    if u < 2.2:   k = u / 2.2; cam.update(s=0.6 + 0.8 * smooth(k), x=64, y=76 - 18 * smooth(k))
    elif u < 3.6: k = (u - 2.2) / 1.4; cam.update(s=3.3 + 0.5 * k, x=65, y=50)
    elif u < 4.8: k = (u - 3.6) / 1.2; shred = True; cam.update(s=3.4, x=72 + 8 * k, y=62 - 5 * k)
    elif u < 6.0:
        k = (u - 4.8) / 1.2; raise_ = ease(k / 0.7)
        cam.update(s=2.2 - 1.4 * smooth(k), x=64, y=56 - 32 * smooth(k))
    elif u < 6.7:
        letterbox = False; raise_ = 1.0; cam.update(s=0.8, x=64, y=24); k = u - 6.0
        if k < 0.05 or 0.24 < k < 0.32: white = 1.0
        elif k < 0.24: flash = 1.0 - k * 2; fx['lift'] = 0.35 * (1 - k * 3)
        else: white = max(0.0, 1 - (k - 0.32) / 0.38)
        if k < 0.45:
            a = int(4 * (1 - k / 0.45)) + 1; rr = (t * 2654435761) & 0xffffffff
            cam['sx'] = (rr >> 8) % (2 * a + 1) - a; cam['sy'] = (rr >> 20) % (2 * a + 1) - a
    else: logoOn = True; letterbox = False
    if logoOn:
        k = u - 6.7
        sx = np.arange(W); d = np.hypot(sx[None, :] - 64, (sx[:, None] - 56) * 1.4)
        G[:, :] = 0.3 * np.clip(1 - d / 75, 0, 1) * smooth(k / 0.6)
        logo(tb, t, smooth(k / 0.7))
    else:
        sky(tb, flash)
        tip = world(tb, t, raise_, shred)
        if 6.05 <= u < 6.24:
            drawBolt(42, tip[0] - 22, -70, tip[0], tip[1])
        rain(t)
        if white is not None: G[:, :] = np.maximum(G, white) if white >= 1 else G * (1 - white) + white
    img = Image.fromarray((dither(t) * 255).astype(np.uint8)).convert('1')
    d = ImageDraw.Draw(img)
    if letterbox and white is None:
        d.rectangle((0, 0, 127, 10), fill=0); d.rectangle((0, 117, 127, 127), fill=0)
    if logoOn and u - 6.7 > 1.2:
        d.rectangle((22, 104, 105, 112), fill=0); d.text((24, 103), "MARK HELLAR + CLAUDE", fill=1)
    return img.resize((256, 256), Image.NEAREST)

if __name__ == '__main__':
    import sys
    us = [float(a) for a in sys.argv[1:]] or [0.3, 1.05, 2.0, 2.8, 4.0, 5.3, 5.9, 6.1, 6.5, 8.5]
    cols = 5; rows = (len(us) + cols - 1) // cols
    out = Image.new('1', (cols * 264, rows * 264), 0)
    for i, u in enumerate(us): out.paste(frame(u), ((i % cols) * 264, (i // cols) * 264))
    out.save('film3_preview.png')
