"""Download attributed candidate negatives for visual review, without training."""
from concurrent.futures import ThreadPoolExecutor
from html import unescape
from io import BytesIO
import json
from pathlib import Path
import random
import re
import sys
import time

import numpy as np
from PIL import Image, ImageDraw
import requests

ROOT = Path(__file__).resolve().parent / 'training_data'
API = 'https://commons.wikimedia.org/w/api.php'
HEADERS = {'User-Agent': 'SchoolVisionDemo/1.0 (educational photo classifier; small attributed collection)'}
QUERIES = [
    ('hands', 'intitle:"hand" intitle:"palm" filetype:bitmap', 20),
    ('hands', 'intitle:"hand" intitle:"gesture" filetype:bitmap', 20),
    ('hands', 'intitle:"thumbs" filetype:bitmap', 20),
    ('hands', 'intitle:"hands" filetype:bitmap', 24),
    ('tablets', 'intitle:"iPad" filetype:bitmap', 28),
    ('tablets', 'intitle:"Samsung Galaxy Tab" filetype:bitmap', 20),
    ('tablets', 'intitle:"tablet computer" filetype:bitmap', 20),
    ('phones', 'intitle:"smartphone" filetype:bitmap', 16),
    ('bottles', 'intitle:"water bottle" filetype:bitmap', 16),
    ('desks', 'intitle:"computer desk" filetype:bitmap', 16),
]
HAND_CATEGORIES = [
    ('hands', 'incategory:"Palm (hand)" filetype:bitmap', 30),
    ('hands', 'incategory:"Back of the hand" filetype:bitmap', 30),
    ('hands', 'incategory:"Human hands" filetype:bitmap', 30),
]
EXCLUDE = re.compile(r'missile|rocket|weapon|gun|injur|wound|disease|surgery|cadaver|blood|fracture|syndrome|deform|amput|x-ray|radiograph|painting|drawing|diagram|logo|icon|screenshot|screen shot|map\b|svg\b|statue|sculpture|woodcut|engraving|murder|sex|nude|naked|porn', re.I)


def dhash(image):
    gray = np.asarray(image.convert('L').resize((9, 8)), dtype=np.int16)
    return sum(int(bit) << i for i, bit in enumerate((gray[:, 1:] > gray[:, :-1]).flat))


def get(url, **kwargs):
    for attempt in range(3):
        response = requests.get(url, headers=HEADERS, timeout=25, **kwargs)
        if response.status_code not in (429, 503):
            response.raise_for_status()
            return response
        time.sleep(min(15, 3 * (attempt + 1)))
    response.raise_for_status()


def download(task):
    group, page = task
    info = page['imageinfo'][0]
    meta = info.get('extmetadata', {})
    license_name = meta.get('LicenseShortName', {}).get('value', '')
    if not any(value in license_name for value in ('CC BY', 'CC0', 'Public domain', 'PDM')):
        return None
    url = info.get('thumburl', info['url']).split('?')[0]
    try:
        # Standard cached thumbnails avoid generating a new size on the server.
        response = requests.get(url, headers=HEADERS, timeout=12)
        response.raise_for_status()
        data = response.content
        image = Image.open(BytesIO(data)).convert('RGB')
        if min(image.size) < 150:
            return None
        return {
            'group': group, 'title': page['title'], 'pageid': page['pageid'],
            'source': info['descriptionurl'], 'download_url': url,
            'artist': unescape(re.sub('<[^>]+>', '', meta.get('Artist', {}).get('value', ''))),
            'license': license_name, 'license_url': meta.get('LicenseUrl', {}).get('value', ''),
            'dhash': str(dhash(image)), 'image': image,
        }
    except (requests.RequestException, OSError) as error:
        print('SKIP', page['title'][:60], type(error).__name__, flush=True)
        return None


def main():
    ROOT.mkdir(exist_ok=True)
    (ROOT / 'candidates').mkdir(exist_ok=True)
    catalog_path = ROOT / 'candidates.json'
    rows = json.loads(catalog_path.read_text(encoding='utf-8')) if catalog_path.exists() else []
    known_ids = {row['pageid'] for row in rows}
    hashes = [int(row['dhash']) for row in rows]
    original = Path('data/missile_dataset')
    manifest = json.loads((original / 'manifest.json').read_text(encoding='utf-8'))
    old_sources = {row['source'] for split in manifest['splits'].values() for row in split}
    for image_path in original.rglob('*.jpg'):
        with Image.open(image_path) as image:
            hashes.append(dhash(image))
    queries = HAND_CATEGORIES if '--hands-only' in sys.argv else QUERIES
    for number, (group, query, limit) in enumerate(queries):
        response = get(API, params={
            'action': 'query', 'format': 'json', 'generator': 'search',
            'gsrsearch': query, 'gsrnamespace': 6, 'gsrlimit': 80,
            'prop': 'imageinfo', 'iiprop': 'url|extmetadata', 'iiurlwidth': 330,
        })
        pages = list(response.json().get('query', {}).get('pages', {}).values())
        random.Random(100 + number).shuffle(pages)
        pages = [page for page in pages if page.get('imageinfo')
                 and re.search(r'\.jpe?g$', page['title'], re.I)
                 and not EXCLUDE.search(page['title'])
                 and page['pageid'] not in known_ids
                 and page['imageinfo'][0]['descriptionurl'] not in old_sources]
        added = 0
        with ThreadPoolExecutor(max_workers=3) as pool:
            for result in pool.map(download, [(group, page) for page in pages[:limit + 8]]):
                if result is None or added >= limit:
                    continue
                fingerprint = int(result['dhash'])
                if result['pageid'] in known_ids or any((fingerprint ^ previous).bit_count() <= 5 for previous in hashes):
                    continue
                image = result.pop('image')
                identifier = f'N{len(rows) + 1:03d}'
                path = ROOT / 'candidates' / f'{identifier}.jpg'
                image.save(path, quality=92)
                rows.append({**result, 'id': identifier, 'path': str(path)})
                known_ids.add(result['pageid'])
                hashes.append(fingerprint)
                added += 1
        catalog_path.write_text(json.dumps(rows, indent=2, ensure_ascii=False), encoding='utf-8')
        print('COLLECTED', group, added, 'TOTAL', len(rows), flush=True)
        time.sleep(1)
    for offset in range(0, len(rows), 40):
        sheet = Image.new('RGB', (1200, 1000), 'white')
        draw = ImageDraw.Draw(sheet)
        for i, row in enumerate(rows[offset:offset + 40]):
            with Image.open(row['path']) as image:
                image.thumbnail((232, 98))
                x, y = i % 5 * 240, i // 5 * 125
                sheet.paste(image, (x, y))
                draw.text((x, y + 100), row['id'] + ' ' + row['group'], fill='black')
        sheet.save(ROOT / f'review_{offset // 40 + 1}.jpg')
    print('Review contact sheets before adding candidates to training.', flush=True)


if __name__ == '__main__':
    main()
