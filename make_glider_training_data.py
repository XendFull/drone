"""Render the actual Glide350 geometry onto split-specific photo backgrounds."""
import json
from pathlib import Path
import random

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageOps

from glider_design.prepare_glide350 import load_airframe

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'glider_training'
SIZE = 224


def geometry():
    airframe = load_airframe()
    triangles, groups = [], []
    for name, mesh, _ in airframe.assembly_meshes():
        triangles.extend(mesh.v[mesh.f])
        groups.extend([0 if 'wing_' in name else 1 if 'tailplane' in name else 2] * len(mesh.f))
    triangles = np.asarray(triangles)
    triangles -= np.array([155., 0., 0.])
    normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-9)
    return triangles, np.asarray(groups), normals


def background(path, rng):
    with Image.open(path) as opened:
        image = opened.convert('RGB')
    scale = rng.uniform(0.6, 1.0)
    w, h = image.size
    side = max(1, int(min(w, h)*scale))
    x, y = rng.randrange(w-side+1), rng.randrange(h-side+1)
    return image.crop((x, y, x+side, y+side)).resize((SIZE, SIZE))


def render_glider(image, mesh, seed):
    triangles, groups, normals = mesh
    rng = random.Random(seed)
    azimuth = rng.uniform(0, 2*np.pi)
    elevation = np.deg2rad(rng.choice((-1, 1))*rng.uniform(8, 75))
    eye = np.array([np.cos(azimuth)*np.cos(elevation), np.sin(azimuth)*np.cos(elevation), np.sin(elevation)])
    right = np.array([-np.sin(azimuth), np.cos(azimuth), 0])
    up = np.cross(eye, right)
    projected = np.stack((triangles @ right, -(triangles @ up)), axis=-1)
    roll = np.deg2rad(rng.uniform(-55, 55))
    rotation = np.array([[np.cos(roll), -np.sin(roll)], [np.sin(roll), np.cos(roll)]])
    projected = projected @ rotation.T
    lo, hi = projected.min((0, 1)), projected.max((0, 1))
    scale = rng.uniform(0.5, 0.9)*SIZE/max(hi-lo)
    center = np.array([SIZE/2+rng.uniform(-10, 10), SIZE/2+rng.uniform(-10, 10)])
    points = np.rint(((projected-(lo+hi)/2)*scale+center)*2).astype(np.int32)
    wing = rng.choice([(245, 115, 25), (255, 175, 35), (240, 235, 215)])
    tail = wing if rng.random() < 0.6 else (255, 190, 40)
    body = wing if rng.random() < 0.25 else (20, 85, 95)
    palette = np.asarray([wing, tail, body], dtype=float)
    light = np.array([rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(0.3, 1)])
    light /= np.linalg.norm(light)
    shading = 0.65+0.35*np.abs(normals @ light)
    colors = np.clip(palette[groups]*shading[:, None], 0, 255).astype(np.uint8)
    foreground = np.zeros((SIZE*2, SIZE*2, 3), dtype=np.uint8)
    alpha = np.zeros((SIZE*2, SIZE*2), dtype=np.uint8)
    depth = (triangles @ eye).mean(axis=1)
    for i in np.argsort(depth):
        polygon = points[i]
        cv2.fillConvexPoly(foreground, polygon, tuple(int(c) for c in colors[i]))
        cv2.fillConvexPoly(alpha, polygon, 255)
    foreground = Image.fromarray(foreground).resize((SIZE, SIZE), Image.Resampling.LANCZOS)
    mask = Image.fromarray(alpha).resize((SIZE, SIZE), Image.Resampling.LANCZOS)
    result = Image.composite(foreground, image, mask)
    # Apply the same sensor effects to both classes after compositing.
    return result, {'azimuth': azimuth, 'elevation': float(elevation), 'roll': float(roll),
                    'wing_rgb': list(wing), 'body_rgb': list(body),
                    'bbox': list(mask.getbbox()),
                    'synthetic_mask_pixels': int((np.asarray(mask) > 127).sum())}


def nuisance_negative(image, rng):
    # Ordinary coloured shapes prevent "orange patch = glider" shortcuts.
    draw = ImageDraw.Draw(image)
    x, y = rng.randint(25, 120), rng.randint(25, 120)
    color = rng.choice([(240, 110, 20), (255, 180, 30), (15, 95, 100)])
    if rng.random() < 0.5:
        draw.ellipse((x, y, x+rng.randint(35, 90), y+rng.randint(20, 70)), fill=color)
    else:
        draw.rectangle((x, y, x+rng.randint(30, 90), y+rng.randint(20, 70)), fill=color)
    return image


def main():
    OUT.mkdir(exist_ok=True)
    mesh = geometry()
    sources = json.loads((ROOT/'training_data/training_manifest.json').read_text(encoding='utf-8'))
    manifest = {}
    for split, number, seed in (('train', 600, 221100), ('validation', 120, 331100), ('test', 120, 441100)):
        folder = OUT/split
        folder.mkdir(exist_ok=True)
        rng = random.Random(seed)
        rows = []
        for index in range(number):
            source = rng.choice(sources[split])
            base = background(source['path'], rng)
            rendered, pose = render_glider(base, mesh, seed+index)
            # Match the tracker: classify a padded moving-region crop, rather
            # than requiring the network to find a small object in a full scene.
            left, top, right, bottom = pose['bbox']
            pad = max(8, round(.15*max(right-left, bottom-top)))
            crop = (max(0,left-pad),max(0,top-pad),min(SIZE,right+pad),min(SIZE,bottom+pad))
            for positive in (False, True):
                image = rendered.copy() if positive else base.copy()
                image = image.crop(crop).resize((SIZE,SIZE))
                if positive:
                    image_pose = pose
                elif index % 3 == 0:
                    image = nuisance_negative(image, rng)
                if rng.random() < 0.7:
                    image = image.filter(ImageFilter.GaussianBlur(rng.uniform(0.1, 0.7)))
                label = 'glider' if positive else 'background'
                path = folder/f'{label}_{index:04d}.jpg'
                image.save(path, quality=rng.randint(65, 94))
                rows.append({'path': str(path), 'label': label, 'source_photo': source['source'],
                             'source_photo_path': source['path'], 'render_seed': seed+index if positive else None,
                             'pose': pose if positive else None, 'region_crop':list(crop),
                             'synthetic_positive': positive})
            if (index+1) % 100 == 0:
                print('RENDER', split, index+1, '/', number, flush=True)
        manifest[split] = rows
    source_sets = {split: {row['source_photo'] for row in rows} for split, rows in manifest.items()}
    assert not source_sets['train'] & (source_sets['validation'] | source_sets['test'])
    assert not source_sets['validation'] & source_sets['test']
    (OUT/'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding='utf-8')
    examples = manifest['train'][:32]
    sheet = Image.new('RGB', (896, 944), 'white')
    draw = ImageDraw.Draw(sheet)
    for i, row in enumerate(examples):
        with Image.open(row['path']) as image:
            x, y = i % 4*224, i//4*118
            image.thumbnail((220, 100))
            sheet.paste(image, (x, y))
            draw.text((x, y+101), row['label'], fill='black')
    sheet.save(OUT/'render_examples.jpg')
    print('Synthetic dataset ready:', {split: len(rows) for split, rows in manifest.items()}, flush=True)


if __name__ == '__main__':
    main()
