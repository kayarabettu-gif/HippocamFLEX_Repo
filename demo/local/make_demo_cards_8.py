"""
Render the 8 demo cards (cards_demo8.pdf plus a preview PNG) from clean
MNIST samples of 1, 2, 3 and 7.

Deck order: BLUE/D1, BLUE/D2, BLUE/D3, RED/D7, GREEN/D2, MAGENTA/D1,
MAGENTA/D3, WHITE/D3.
"""
import struct
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# set these to your MNIST raw folder (outputs/data/MNIST/raw) and output folder
DATA_DIR = Path("/sessions/peaceful-trusting-cerf/mnt/demo_related_mnist/outputs/data/MNIST/raw")
OUT_DIR  = Path("/sessions/peaceful-trusting-cerf/mnt/demo_related_mnist")
PDF_OUT  = OUT_DIR / "cards_demo8.pdf"
PNG_OUT  = OUT_DIR / "cards_demo8.png"

COLOURS_RGB = {
    "RED":     (230,  40,  50),
    "GREEN":   ( 40, 200,  80),
    "BLUE":    ( 40,  90, 220),
    "MAGENTA": (220,  40, 200),
    "WHITE":   (245, 245, 245),
}

DECK = [
    ("BLUE",    1),
    ("BLUE",    2),
    ("BLUE",    3),
    ("RED",     7),
    ("GREEN",   2),
    ("MAGENTA", 1),
    ("MAGENTA", 3),
    ("WHITE",   3),
]


def load_mnist(images_path, labels_path):
    with open(images_path, "rb") as f:
        magic, n, rows, cols = struct.unpack(">IIII", f.read(16))
        images = np.frombuffer(f.read(), dtype=np.uint8).reshape(n, rows, cols)
    with open(labels_path, "rb") as f:
        struct.unpack(">II", f.read(8))
        labels = np.frombuffer(f.read(), dtype=np.uint8)
    return images, labels


def cleanliness_score(img, digit):
    """Lower is cleaner, None rejects the sample. Limits on ink, width and
    aspect are tuned per digit so each one reads clearly on a card."""
    ink = img > 80
    ink_frac = ink.mean()
    if digit == 1:
        if ink_frac < 0.05 or ink_frac > 0.18:
            return None
    else:
        if ink_frac < 0.10 or ink_frac > 0.30:
            return None
    ys, xs = np.where(ink)
    if ys.size == 0:
        return None
    cy, cx = ys.mean(), xs.mean()
    h = ys.max() - ys.min()
    w = xs.max() - xs.min()
    if digit == 1:
        if w < 7 or w > 16:
            return None
        target = 1.7
    elif digit == 2:
        if w < 14 or h < 16:
            return None
        # a 2 needs a flat bottom stroke (>= 55% of the width) and ink at the top
        bot = ink[ys.max()-3:ys.max()+1, :]
        if not bot.any():
            return None
        bot_xs = np.where(bot.any(axis=0))[0]
        if bot_xs.size == 0 or (bot_xs.max() - bot_xs.min()) < 0.55 * w:
            return None
        if not ink[ys.min():ys.min() + 5, :].any():
            return None
        target = 1.10
    elif digit == 3:
        if w < 13:
            return None
        target = 1.20
    elif digit == 7:
        if w < 13:
            return None
        target = 1.25
    else:
        target = 1.4
    dist = abs(cy - 13.5) + abs(cx - 13.5)
    aspect_pen = abs((h / max(w, 1)) - target)
    return dist + 2.5 * aspect_pen


def pick_top_exemplars(images, labels, digit, k=4, search=8000):
    idx = np.where(labels == digit)[0][:search]
    scored = []
    for i in idx:
        s = cleanliness_score(images[i], digit)
        if s is not None:
            scored.append((s, i))
    scored.sort()
    return [images[i].copy() for _, i in scored[:k]]


FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf",
]


def _font(size_px):
    for c in FONT_CANDIDATES:
        if Path(c).exists():
            return ImageFont.truetype(c, size_px)
    return ImageFont.load_default()


def render_card(colour_name, digit_int, glyph_28, size_px=1200):
    """Black digit centred on the coloured card, no label."""
    bg = COLOURS_RGB[colour_name]
    card = Image.new("RGB", (size_px, size_px), bg)
    dsize = int(size_px * 0.55)
    dx = (size_px - dsize) // 2
    dy = (size_px - dsize) // 2
    glyph = Image.fromarray(glyph_28, mode="L").resize((dsize, dsize), Image.BICUBIC)
    glyph_np = np.asarray(glyph)
    mask_np = (glyph_np > 70).astype(np.uint8) * 255
    mask = Image.fromarray(mask_np, mode="L")
    black_layer = Image.new("RGB", (dsize, dsize), (0, 0, 0))
    card.paste(black_layer, (dx, dy), mask)
    return card


def main():
    print("[mnist] loading IDX files...")
    images, labels = load_mnist(
        DATA_DIR / "train-images-idx3-ubyte",
        DATA_DIR / "train-labels-idx1-ubyte",
    )
    print(f"[mnist] {len(images)} training images loaded")
    pools = {d: pick_top_exemplars(images, labels, d, k=4) for d in (1, 2, 3, 7)}
    for d, p in pools.items():
        print(f"[mnist] digit {d}: {len(p)} clean exemplars")

    pool_idx = {d: 0 for d in pools}
    cards = []
    for colour, digit in DECK:
        pool = pools[digit]
        glyph = pool[pool_idx[digit] % len(pool)]
        pool_idx[digit] += 1
        print(f"[deck] {colour}/D{digit}  exemplar #{pool_idx[digit]-1}")
        cards.append(render_card(colour, digit, glyph))

    cards[0].save(PDF_OUT, save_all=True, append_images=cards[1:])
    print(f"[out] wrote {PDF_OUT}  ({len(cards)} pages)")

    thumb_w = thumb_h = 320
    margin = 20
    grid_w = thumb_w * 4 + margin * 5
    grid_h = thumb_h * 2 + margin * 3 + 50
    grid = Image.new("RGB", (grid_w, grid_h), (25, 25, 25))
    draw = ImageDraw.Draw(grid)
    draw.text((margin, 12), "HippocamFLEX MNIST demo: 8 fully-predictive cards",
              fill=(255, 255, 255), font=_font(28))
    for i, card in enumerate(cards):
        row, col = i // 4, i % 4
        x = margin + col * (thumb_w + margin)
        y = 50 + margin + row * (thumb_h + margin)
        thumb = card.resize((thumb_w, thumb_h), Image.LANCZOS)
        grid.paste(thumb, (x, y))
    grid.save(PNG_OUT)
    print(f"[out] wrote {PNG_OUT}")


if __name__ == "__main__":
    main()
