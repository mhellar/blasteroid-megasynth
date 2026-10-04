# Python mirror of OHM's boot splash: generative geometry (two random scenes, sometimes particles), glitch cuts,
# particles converge into an Omega, logo. Uses film3's grey rasterizer + dither.
import math, random, sys
import numpy as np
from PIL import Image, ImageDraw
import splash_raster as R
from splash_raster import paint, cLine, cDisc, cPoly, cTri

W = 128
SC_TIMES, SC_RINGS, SC_PART, SC_TUNNEL, SC_LISSA = range(5)

class Film:
    def __init__(s, seed):
        s.r = random.Random(seed)
        r = s.r
        a = r.choice([SC_TIMES, SC_RINGS, SC_TUNNEL, SC_LISSA])
        b = SC_PART if r.random() < 0.5 else r.choice([x for x in (SC_TIMES, SC_RINGS, SC_TUNNEL, SC_LISSA) if x != a])
        s.scenes = [a, b]
        s.k0 = r.uniform(2, 4); s.kv = r.uniform(0.25, 0.6)
        s.sides = [r.randint(3, 8) for _ in range(5)]; s.rsp = [r.uniform(-1.2, 1.2) for _ in range(5)]
        s.la, s.lb, s.lc = r.choice([1, 2, 3]), r.choice([2, 3, 4, 5]), r.choice([1, 3, 5])
        s.P = [[r.uniform(0, 128), r.uniform(0, 128)] for _ in range(150)]
        s.t = 0.0
        s.omega = omegaPoints(150)

    def step(s, dt):                                  # advance + draw one frame into R.G (persistent)
        s.t += dt; u = s.t; r = s.r
        decay = 0.5
        if u < 6.0:
            sc = s.scenes[0] if u < 3.0 else s.scenes[1]
            decay = 0.86 if sc == SC_PART else 0.55
        elif u < 7.4: decay = 0.8
        else: decay = 0.6
        R.G *= decay
        R.cam.update(s=1.0, x=64.0, y=64.0, sx=0, sy=0); R.fx.update(lift=0.0, fade=1.0)
        if u < 6.0:
            lt = u if u < 3.0 else u - 3.0
            fade = min(1.0, lt / 0.5)
            [s.times, s.rings, s.particles, s.tunnel, s.lissa][sc](lt, fade, dt)
            if 2.85 < u < 3.15: glitch(r)
        elif u < 7.4:
            s.converge((u - 6.0) / 1.4, dt)
        else:
            s.logo(u - 7.4)

    def times(s, t, fade, dt):
        N = 72; k = s.k0 + s.kv * t; rot = t * 0.2
        for i in range(N):
            a1 = rot + i * 2 * math.pi / N; a2 = rot + i * k * 2 * math.pi / N
            paint(0.62 * fade); cLine(64 + 54 * math.cos(a1), 64 + 54 * math.sin(a1), 64 + 54 * math.cos(a2), 64 + 54 * math.sin(a2), 0.5)
        for i in range(0, N, 6):
            a1 = rot + i * 2 * math.pi / N; paint(1.0 * fade); cDisc(64 + 54 * math.cos(a1), 64 + 54 * math.sin(a1), 0.8)

    def rings(s, t, fade, dt):
        for j in range(5):
            rad = 12 + j * 11; n = s.sides[j]; rot = t * s.rsp[j] + j
            pts = [(64 + rad * math.cos(rot + 2 * math.pi * i / n), 64 + rad * math.sin(rot + 2 * math.pi * i / n)) for i in range(n)]
            if j == 0: paint(0.18 * fade); cPoly(pts)
            for i in range(n):
                paint((1.0 if j % 2 == 0 else 0.5) * fade); cLine(*pts[i], *pts[(i + 1) % n], 1.0 if j % 2 == 0 else 0.6)
            ang = -t * s.rsp[j] * 2 + j * 1.7; paint(fade); cDisc(64 + rad * math.cos(ang), 64 + rad * math.sin(ang), 1.3)

    def particles(s, t, fade, dt):
        for p in s.P:
            a = R.vnoise(np.array(p[0] * 0.025), np.array(p[1] * 0.025 + t * 0.15)) * 4 * math.pi
            p[0] += math.cos(a) * 40 * dt; p[1] += math.sin(a) * 40 * dt
            if not (0 <= p[0] < 128 and 0 <= p[1] < 128): p[0], p[1] = s.r.uniform(0, 128), s.r.uniform(0, 128)
            paint(0.95 * fade); cDisc(p[0], p[1], 0.6)

    def tunnel(s, t, fade, dt):
        for i in range(12):
            z = ((i - t * 2.2) % 12) + 0.6
            size = 70 / z; rot = z * 0.35 + t * 0.6
            pts = [(64 + size * math.cos(rot + math.pi / 4 + i2 * math.pi / 2), 64 + size * math.sin(rot + math.pi / 4 + i2 * math.pi / 2)) for i2 in range(4)]
            g = min(1.0, 1.6 / z) * fade
            for i2 in range(4): paint(g); cLine(*pts[i2], *pts[(i2 + 1) % 4], max(0.5, 2.5 / z))

    def lissa(s, t, fade, dt):
        prev = None; ry = t * 0.7; rx = 0.4 + 0.3 * math.sin(t * 0.5)
        for i in range(181):
            q = i / 180 * 2 * math.pi
            x, y, z = math.sin(s.la * q + t * 0.3), math.sin(s.lb * q), math.sin(s.lc * q + 1.0)
            x, z = x * math.cos(ry) + z * math.sin(ry), -x * math.sin(ry) + z * math.cos(ry)
            y, z = y * math.cos(rx) - z * math.sin(rx), y * math.sin(rx) + z * math.cos(rx)
            px, py = 64 + x * 46, 64 + y * 46
            if prev: paint((0.45 + 0.5 * (z + 1) / 2) * fade); cLine(*prev, px, py, 0.6 + 0.8 * (z + 1) / 2)
            prev = (px, py)

    def converge(s, k, dt):
        e = 1 - (1 - min(1.0, k)) ** 3
        if k > 0.7: drawOmega((k - 0.7) / 0.3 * 0.6, outline=False)
        for i, p in enumerate(s.P):
            tx, ty = s.omega[i % len(s.omega)]
            x = p[0] + (tx - p[0]) * e; y = p[1] + (ty - p[1]) * e
            paint(1.0); cDisc(x, y, 0.7)

    def logo(s, k):
        sx = np.arange(W); d = np.hypot(sx[None, :] - 64, (sx[:, None] - 54))
        pulse = 0.5 + 0.5 * math.sin(k * 2.2)
        R.G[:, :] = np.maximum(R.G, (0.14 + 0.08 * pulse) * np.clip(1 - d / 52, 0, 1))
        drawOmega(min(1.0, 0.6 + k * 0.8))
        for i, p in enumerate(s.P[:40]):                          # a few motes orbit the symbol
            a = k * 0.4 + i * 0.157; rr = 38 + 6 * math.sin(i * 1.7 + k)
            paint(0.7); cDisc(64 + rr * math.cos(a), 52 + rr * 0.8 * math.sin(a), 0.5)
        if 1.5 < k < 1.6 or 3.2 < k < 3.25: glitch(s.r)

OX, OY, OR = 64, 50, 26
FX_ = OR * math.cos(math.radians(50)); OFY = OY + OR * math.sin(math.radians(50))
def omegaPoints(n):
    pts = []; arc = int(n * 0.72)
    for i in range(arc):
        th = math.radians(-50 + 280 * i / (arc - 1)); pts.append((OX + OR * math.cos(th), OY - OR * math.sin(th)))
    rest = n - arc
    for i in range(rest):
        u = i / max(1, rest // 2 - 1)
        if i < rest // 2: pts.append((OX - FX_ - 14 * min(1, u), OFY))
        else: pts.append((OX + FX_ + 14 * min(1, (i - rest // 2) / max(1, rest - rest // 2 - 1)), OFY))
    return pts

def drawOmega(fade, outline=True):
    th = [math.radians(-50 + 280 * i / 28) for i in range(29)]
    P = [(OX + OR * math.cos(a), OY - OR * math.sin(a)) for a in th]
    if outline:
        for i in range(28): paint(0.0); cLine(*P[i], *P[i + 1], 9)
        paint(0.0); cLine(OX - FX_, OFY, OX - FX_ - 15, OFY, 8); cLine(OX + FX_, OFY, OX + FX_ + 15, OFY, 8)
    for i in range(28):
        w = 4.6 + 2.0 * abs(math.cos(th[i]))                    # thicker at the sides, like a real Omega
        R.chrome(OY - OR - 3, 2 * OR + 8); R.fx['fade'] = fade; cLine(*P[i], *P[i + 1], w)
    R.chrome(OFY - 6, 12); cLine(OX - FX_, OFY, OX - FX_ - 14, OFY, 5); cLine(OX + FX_, OFY, OX + FX_ + 14, OFY, 5)
    R.fx['fade'] = 1.0

def glitch(r):                                                    # horizontal tearing bands
    for _ in range(4):
        y0 = r.randrange(0, 120); h = r.randrange(2, 8); sh = r.randrange(-14, 15)
        R.G[y0:y0 + h, :] = np.roll(R.G[y0:y0 + h, :], sh, axis=1)
        if r.random() < 0.4: R.G[y0:y0 + 1, :] = np.maximum(R.G[y0:y0 + 1, :], 0.6)

def render(film, u, fps=25):
    while film.t < u - 1e-6: film.step(1 / fps)
    img = Image.fromarray((R.dither(int(u * 1000)) * 255).astype(np.uint8)).convert('1')
    d = ImageDraw.Draw(img)
    if u > 7.4 + 0.6:
        d.text((46, 96), "O H M", fill=1)
    return img.resize((256, 256), Image.NEAREST)

if __name__ == '__main__':
    seed = int(sys.argv[1]); us = [float(a) for a in sys.argv[2:]]
    film = Film(seed); R.G[:, :] = 0
    frames = [render(film, u) for u in us]
    out = Image.new('1', (len(frames) * 264, 256))
    for i, f in enumerate(frames): out.paste(f, (i * 264, 0))
    out.save('splash_preview.png')
    print('scenes', film.scenes)
