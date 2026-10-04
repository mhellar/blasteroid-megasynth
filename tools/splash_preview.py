# Python mirror of the BLASTEROID MEGASYNTH boot splash: generative geometry (from film5), particles converge into a
# cyborg cyclops head with wraparound shades, the eye charges, two lasers burn the name in.
import math, random, sys
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import splash_raster as R
from splash_raster import paint, cLine, cDisc, cPoly, cTri, cBox
import splash_scenes as F

W = 128
HEAD = [(44, 11), (52, 5), (76, 5), (84, 11), (87, 22), (86, 36), (80, 46), (72, 51), (56, 51), (48, 46), (42, 36), (41, 22)]
SHADES = [(35, 22), (46, 18.5), (64, 17.5), (82, 18.5), (93, 22), (91, 30), (80, 33.5), (64, 32), (48, 33.5), (37, 30)]
EYE = (64, 25.5)
L1Y, L2Y = 70, 90                                    # text rows (top of the letters)

def outlinePoints(n):
    pts = []
    for poly, share in ((HEAD, 0.6), (SHADES, 0.4)):
        m = int(n * share); per = []
        L = [math.hypot(poly[(i + 1) % len(poly)][0] - poly[i][0], poly[(i + 1) % len(poly)][1] - poly[i][1]) for i in range(len(poly))]
        tot = sum(L)
        for k in range(m):
            d = tot * k / m; i = 0
            while d > L[i]: d -= L[i]; i += 1
            a, b = poly[i], poly[(i + 1) % len(poly)]; u = d / L[i]
            per.append((a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u))
        pts += per
    while len(pts) < n: pts.append(EYE)
    return [scr(x, y) for x, y in pts]

HS, HCY, HSY = 1.2, 27, 30                            # the head is drawn 1.2x around (64, 27) -> screen y 30
def scr(x, y): return (64 + (x - 64) * HS, HSY + (y - HCY) * HS)
EYE_S = scr(*EYE)

def drawHead(fade, eye, t):
    R.cam.update(s=HS, x=64.0, y=HCY + (64 - HSY) / HS)
    # shoulders / collar: dark, rim-lit
    paint(0.0); cPoly([(46, 49), (82, 49), (100, 58), (28, 58)])
    paint(0.3 * fade); cPoly([(48, 50), (80, 50), (97, 57.5), (31, 57.5)])
    paint(0.9 * fade); cLine(80, 50, 97, 57.5, 0.7); cLine(31, 57.5, 48, 50, 0.5)
    # head: bold, bright, graphic
    paint(0.0); cPoly([(x + (x - 64) * 0.07, y + (y - 28) * 0.07) for x, y in HEAD])
    paint(0.95 * fade); cPoly(HEAD)
    paint(0.55 * fade); cPoly([(42, 36), (41, 22), (46.5, 22), (48, 37), (52, 48), (48, 46)])   # shadow side
    # forehead seam + rivets
    paint(0.0); cLine(52, 10.5, 76, 10.5, 0.5)
    # nose, grim mouth, chin
    paint(0.0); cLine(64.5, 33.5, 62, 38.8, 0.7); cLine(62, 38.8, 65.5, 39.2, 0.7)
    cLine(56, 43, 72, 43, 1.6); cLine(56, 43, 54, 44.8, 1.2); cLine(72, 43, 74, 44.8, 1.2)
    cLine(61, 48.6, 67, 48.6, 0.7)
    # wraparound shades: solid black, one hard reflection line, a glint
    paint(0.0); cPoly([(x + (x - 64) * 0.05, y + (1.4 if y > 25 else -1.4)) for x, y in SHADES])
    SH2 = [(x + (x - 64) * 0.05, y + (1.4 if y > 25 else -1.4)) for x, y in SHADES]
    paint(0.85 * fade)                                 # hard white rim so the shades read against the black
    for i in range(len(SH2)): cLine(*SH2[i], *SH2[(i + 1) % len(SH2)], 0.55)
    paint(1.0 * fade); cLine(38, 28.5, 90, 28.5, 0.6)
    g = math.fmod(t * 0.6, 1.6)
    if g < 1: paint(1.0); cLine(40 + g * 46, 19.5, 37 + g * 46, 30, 0.9)
    # the single eye blazing through the lens
    if eye > 0:
        paint(0.3 + 0.35 * eye); cDisc(*EYE, 2 + 3.5 * eye)
        paint(1.0); cDisc(*EYE, 0.8 + 1.4 * eye)
    R.cam.update(s=1.0, x=64.0, y=64.0)

def textMask(font):
    img = Image.new('L', (W, W), 0); d = ImageDraw.Draw(img)
    for s, y in (("BLASTEROID", L1Y), ("MEGASYNTH", L2Y)):
        w = d.textlength(s, font=font); d.text(((W - w) / 2, y), s, font=font, fill=255)
    return np.array(img) > 128

class Film(F.Film):
    def __init__(s, seed):
        super().__init__(seed)
        s.face = outlinePoints(150)
    def step(s, dt):
        u = s.t + dt
        if u < 6.0: return F.Film.step(s, dt)        # the generative scenes are unchanged
        s.t = u
        R.G *= 0.8 if u < 7.2 else 0.55
        R.cam.update(s=1.0, x=64.0, y=64.0, sx=0, sy=0); R.fx.update(lift=0.0, fade=1.0)
        if u < 7.2:                                   # everything flies into the head
            k = (u - 6.0) / 1.2; e = 1 - (1 - min(1.0, k)) ** 3
            if k > 0.6: drawHead((k - 0.6) / 0.4 * 0.7, 0, u)
            for i, p in enumerate(s.P):
                tx, ty = s.face[i % len(s.face)]
                paint(1.0); cDisc(p[0] + (tx - p[0]) * e, p[1] + (ty - p[1]) * e, 0.7)
        else:
            k = u - 7.2
            eye = min(1.0, k / 0.6) if k < 2.4 else 0.55 + 0.25 * math.sin(k * 5)
            if k > 0.5 and k < 0.7: R.cam['sx'] = random.randint(-2, 2)
            drawHead(min(1.0, 0.7 + k), eye, u)
            s.hit = None
            if 0.6 < k < 2.4:                         # the lasers
                q = (k - 0.6) / 1.8
                x1 = 8 + q * 112; x2 = 120 - q * 112
                for hx, hy in ((x1, L1Y + 7), (x2, L2Y + 7)):
                    paint(0.55); cLine(EYE_S[0], EYE_S[1], hx, hy, 3.2)
                    paint(1.0); cLine(EYE_S[0], EYE_S[1], hx, hy, 1.3)
                    paint(0.6); cDisc(hx, hy, 4); paint(1.0); cDisc(hx, hy, 1.6)
                    for j in range(6):
                        a = random.uniform(0, 2 * math.pi); rr = random.uniform(2, 9)
                        paint(0.9); cDisc(hx + math.cos(a) * rr, hy + math.sin(a) * rr * 0.6, 0.5)
                s.hit = (x1, x2)
            # scorch glow under the burned letters
            sx = np.arange(W)
            for y0, rev in ((L1Y, False), (L2Y, True)):
                band = np.exp(-((sx[:, None] - (y0 + 7)) / 7.0) ** 2) * 0.16
                if s.hit and k < 2.4:
                    lim = s.hit[1] if rev else s.hit[0]
                    band = band * ((sx[None, :] > lim) if rev else (sx[None, :] < lim))
                R.G[:, :] = np.maximum(R.G, band)

FONT = ImageFont.truetype("arialbd.ttf", 14)
MASK = textMask(FONT)
def render(film, u, fps=25):
    while film.t < u - 1e-6: film.step(1 / fps)
    bits = R.dither(int(u * 1000))
    img = Image.fromarray((bits * 255).astype(np.uint8)).convert('1')
    if u >= 7.8:                                      # burned-in text (crisp, revealed up to the laser hit)
        k = u - 7.2; arr = np.array(img).astype(bool); m = MASK.copy()
        if k < 2.4:
            q = (k - 0.6) / 1.8; x1 = 8 + q * 112; x2 = 120 - q * 112
            xs = np.arange(W)[None, :]; rows = np.arange(W)[:, None]
            keep = ((rows < L2Y - 2) & (xs < x1)) | ((rows >= L2Y - 2) & (xs > x2))
            m &= keep
        outline = np.zeros_like(m)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1): outline |= np.roll(np.roll(m, dy, 0), dx, 1)
        arr[outline] = False; arr[m] = True
        img = Image.fromarray((arr * 255).astype(np.uint8)).convert('1')
    return img.resize((256, 256), Image.NEAREST)

if __name__ == '__main__':
    seed = int(sys.argv[1]); us = [float(a) for a in sys.argv[2:]]
    film = Film(seed); R.G[:, :] = 0
    frames = [render(film, u) for u in us]
    out = Image.new('1', (len(frames) * 264, 256))
    for i, f in enumerate(frames): out.paste(f, (i * 264, 0))
    out.save('splash_preview.png')
