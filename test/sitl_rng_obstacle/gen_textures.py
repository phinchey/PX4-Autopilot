#!/usr/bin/env python3
"""Generate the high-contrast textures used by the indoor furniture world.

The Gazebo optical-flow plugin runs a real KLT feature tracker on a rendered
camera image, so every surface the drone may fly over (floor, table tops,
couch, bed, step) needs visible texture; a uniform colour yields zero flow
quality and the EKF loses its horizontal aiding.  This script writes the PNG
textures into models/indoor_house/materials/textures/.  It only needs to be
re-run if you want different textures; the generated files are committed.
"""

import os
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "models", "indoor_house", "materials", "textures")


def blobs(size, rng, n, r_min, r_max):
    """Random filled discs on a 2D float canvas (values 0..1)."""
    yy, xx = np.mgrid[0:size, 0:size]
    img = np.zeros((size, size), dtype=np.float32)
    for _ in range(n):
        cx, cy = rng.uniform(0, size, 2)
        r = rng.uniform(r_min, r_max)
        img[(xx - cx) ** 2 + (yy - cy) ** 2 < r * r] = rng.uniform(0.0, 1.0)
    return img


def floor_texture(size, rng):
    """Speckled stone-like floor: coarse checker + random blobs.

    No per-pixel noise on purpose: it makes the PNG incompressible and the
    blobs already give the KLT tracker plenty of corners."""
    tile = size // 8
    yy, xx = np.mgrid[0:size, 0:size]
    checker = (((xx // tile) + (yy // tile)) % 2).astype(np.float32)
    img = 0.45 + 0.25 * checker
    img += 0.5 * (blobs(size, rng, 1200, 3, 18) - 0.5)
    img = np.clip(img, 0.05, 0.95)
    rgb = np.stack([img * 0.85, img * 0.82, img * 0.75], axis=-1)
    return rgb


def wood_texture(size, rng):
    """Brown wood grain: wavy stripes plus knots."""
    yy, xx = np.mgrid[0:size, 0:size].astype(np.float32)
    grain = np.sin(xx / 6.0 + 4.0 * np.sin(yy / 90.0)) * 0.5 + 0.5
    img = 0.35 + 0.35 * grain
    img += 0.4 * (blobs(size, rng, 120, 2, 8) - 0.5)
    img = np.clip(img, 0.05, 0.95)
    rgb = np.stack([img * 0.75, img * 0.5, img * 0.28], axis=-1)
    return rgb


def fabric_texture(size, rng, tint):
    """Coarse woven fabric with random darker/lighter patches."""
    yy, xx = np.mgrid[0:size, 0:size]
    weave = (((xx // 4) + (yy // 4)) % 2).astype(np.float32)
    img = 0.4 + 0.3 * weave
    img += 0.6 * (blobs(size, rng, 250, 3, 14) - 0.5)
    img = np.clip(img, 0.05, 0.95)
    return np.stack([img * t for t in tint], axis=-1)


def save(name, rgb):
    os.makedirs(OUT, exist_ok=True)
    data = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)
    Image.fromarray(data, "RGB").save(os.path.join(OUT, name), optimize=True)
    print("wrote", os.path.join(OUT, name))


if __name__ == "__main__":
    rng = np.random.default_rng(42)
    save("floor.png", floor_texture(1024, rng))
    save("wood.png", wood_texture(256, rng))
    save("fabric_blue.png", fabric_texture(256, rng, (0.35, 0.45, 0.95)))
    save("fabric_red.png", fabric_texture(256, rng, (0.95, 0.35, 0.3)))
    save("concrete.png", floor_texture(256, rng) * np.array([0.7, 0.8, 0.7]))
