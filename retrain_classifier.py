"""Retrain the photo-classification head with reviewed everyday-object negatives.

The original train/validation/test separation is preserved. The backbone is
frozen; four augmented views per training photo are cached before head training.
The current checkpoint is backed up and a report is written before deployment.
"""
from collections import Counter
import copy
import hashlib
import json
from pathlib import Path
import random
import shutil
import time

import numpy as np
from PIL import Image, ImageOps
import torch
from torch import nn
import torch.nn.functional as F
from torchvision import transforms

from cnn import load_model

ROOT = Path(__file__).resolve().parent
DATA = ROOT / 'training_data'
ORIGINAL = Path('data/missile_dataset')
DEVICE = 'cpu'
SEED = 1611


def tablet_view(image):
    """Simulate a photo displayed on a tablet, preserving its existing label."""
    canvas = Image.new('RGB', (320, 240), (110, 110, 110))
    canvas.paste((18, 18, 20), (15, 22, 305, 218))
    screen = ImageOps.contain(image.convert('RGB'), (264, 170))
    canvas.paste(screen, ((320 - screen.width) // 2, (240 - screen.height) // 2))
    return canvas


def rows_for_splits():
    manifest = json.loads((ORIGINAL / 'manifest.json').read_text(encoding='utf-8'))
    splits = {}
    for split, rows in manifest['splits'].items():
        splits[split] = [
            {'path': str(ORIGINAL / row['relative_path']), 'label': row['label'],
             'kind': 'original', 'source': row['source'], 'group': row['scene_group']}
            for row in rows
        ]
    candidates = json.loads((DATA / 'candidates.json').read_text(encoding='utf-8'))
    reviewed = set(json.loads((DATA / 'accepted_ids.json').read_text(encoding='utf-8')))
    added = [row for row in candidates if row['id'] in reviewed]
    assert len(added) == len(reviewed), 'Review list contains unknown IDs'
    groups = {}
    for row in added:
        group = row['group'] + '|' + row['artist']
        groups.setdefault(group, []).append(row)
    # Split each object category by photographer groups, not augmented views.
    assigned = {}
    for category in sorted({row['group'] for row in added}):
        keys = sorted(key for key in groups if key.startswith(category + '|'))
        random.Random(SEED + sum(map(ord, category))).shuffle(keys)
        n = len(keys)
        n_test = max(1, round(n * 0.15)) if n >= 3 else 0
        n_val = max(1, round(n * 0.15)) if n >= 3 else 0
        for index, key in enumerate(keys):
            assigned[key] = ('test' if index < n_test else
                             'validation' if index < n_test + n_val else 'train')
    for group, rows in groups.items():
        split = assigned[group]
        for row in rows:
            splits[split].append({**row, 'label': 'not_missiles', 'kind': row['group'],
                                  'group': group})
    # Exact duplicate check across splits, including original images.
    seen = {}
    for split, rows in splits.items():
        for row in rows:
            digest = hashlib.sha256(Path(row['path']).read_bytes()).hexdigest()
            assert digest not in seen or seen[digest] == split, 'Cross-split duplicate'
            seen[digest] = split
    return splits


def extract(model, rows, transform, train=False):
    features, labels, records = [], [], []
    augmentation = transforms.Compose([
        transforms.RandomAffine(degrees=8, translate=(0.03, 0.03), scale=(0.9, 1.05)),
        transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.15),
    ])
    views = range(4) if train else range(2)
    total = len(rows) * len(views)
    buffer, buffer_labels, buffer_records = [], [], []

    def flush():
        if not buffer:
            return
        with torch.inference_mode():
            embedding = F.normalize(model.backbone(torch.stack(buffer)), dim=1)
        features.append(embedding.clone())
        labels.extend(buffer_labels)
        records.extend(buffer_records)
        buffer.clear()
        buffer_labels.clear()
        buffer_records.clear()

    for index, row in enumerate(rows):
        with Image.open(row['path']) as opened:
            original = opened.convert('RGB')
        for view in views:
            image = original.copy()
            screen = (view % 2 == 1)
            if screen:
                image = tablet_view(image)
            if train and view >= 2:
                # Random crops are safe for negatives. Positive crops could
                # accidentally remove the pictured object and corrupt labels.
                if row['label'] == 'not_missiles':
                    image = transforms.RandomResizedCrop(224, scale=(0.5, 1.0))(image)
                image = augmentation(image)
            buffer.append(transform(image))
            buffer_labels.append(0 if row['label'] == 'missiles' else 1)
            buffer_records.append({**row, 'view': 'tablet' if screen else 'photo'})
            if len(buffer) == 16:
                flush()
        if (index + 1) % 50 == 0:
            print('FEATURES', index + 1, '/', len(rows), 'photos;',
                  (index + 1) * len(views), '/', total, 'views', flush=True)
    flush()
    return torch.cat(features), torch.tensor(labels), records


def stats(probabilities, labels, threshold=0.5):
    positive = labels == 0
    predicted = probabilities[:, 0] >= threshold
    tp = int((predicted & positive).sum())
    fp = int((predicted & ~positive).sum())
    tn = int((~predicted & ~positive).sum())
    fn = int((~predicted & positive).sum())
    return {'true_positive': tp, 'false_positive': fp, 'true_negative': tn,
            'false_negative': fn, 'false_positive_rate': fp / max(1, fp + tn),
            'recall': tp / max(1, tp + fn),
            'binary_alert_accuracy': (tp + tn) / max(1, len(labels))}


def choose_threshold(probabilities, labels):
    negative_scores = probabilities[labels == 1, 0]
    # Select on validation only. This is an empirical target, not a guarantee.
    allowed_fp = max(0, int(len(negative_scores) * 0.02))
    thresholds = sorted({0.9, *[float(score) for score in probabilities[:, 0] if score >= 0.9]})
    feasible = []
    for threshold in thresholds:
        result = stats(probabilities, labels, threshold)
        if result['false_positive'] <= allowed_fp:
            feasible.append((result['recall'], -result['false_positive'], -threshold, threshold))
    return max(feasible)[-1] if feasible else 1.0


def predict_policy(probabilities, threshold):
    names = []
    for missile, not_missile in probabilities.tolist():
        names.append('missiles' if missile >= threshold else
                     'not_missiles' if not_missile >= 0.8 else 'uncertain')
    return names


def evaluate(head, features, labels, records, threshold, reject=True):
    head.eval()
    with torch.inference_mode():
        probabilities = head(features).softmax(1)
    masks = {
        'original_photos': [row['kind'] == 'original' and row['view'] == 'photo' for row in records],
        'new_object_photos': [row['kind'] != 'original' and row['view'] == 'photo' for row in records],
        'tablet_simulations': [row['view'] == 'tablet' for row in records],
        'hands': [row['kind'] == 'hands' and row['view'] == 'photo' for row in records],
        'tablets': [row['kind'] == 'tablets' and row['view'] == 'photo' for row in records],
        'tablet_aircraft': [row['view'] == 'tablet' and any(
            name in row['group'] for name in ('Civilian airliners', 'Civilian Cessna'))
            for row in records],
    }
    result = {}
    for name, values in masks.items():
        mask = torch.tensor(values, dtype=torch.bool)
        if mask.any():
            result[name] = stats(probabilities[mask], labels[mask], threshold)
    result['all_views'] = stats(probabilities, labels, threshold)
    names = (predict_policy(probabilities, threshold) if reject else
             [['missiles', 'not_missiles'][int(row.argmax())] for row in probabilities])
    uncertain = names.count('uncertain')
    correct = sum(name == ['missiles', 'not_missiles'][int(label)]
                  for name, label in zip(names, labels))
    result['uncertain_views'] = uncertain
    result['classification_coverage'] = (len(labels) - uncertain) / len(labels)
    result['classification_accuracy_counting_uncertain_as_incorrect'] = correct / len(labels)
    return result, probabilities


def main():
    random.seed(SEED)
    np.random.seed(SEED)
    torch.manual_seed(SEED)
    start = time.monotonic()
    checkpoint_path = ROOT / 'missile_model.pth'
    backup_path = ROOT / 'missile_model_before_hard_negatives.pth'
    if not backup_path.exists():
        shutil.copy2(checkpoint_path, backup_path)
    model, transform, class_names = load_model(backup_path)
    assert class_names == ['missiles', 'not_missiles']
    model.eval()
    model.requires_grad_(False)
    splits = rows_for_splits()
    (DATA / 'training_manifest.json').write_text(json.dumps(splits, indent=2, ensure_ascii=False), encoding='utf-8')
    for split, rows in splits.items():
        print('SPLIT', split, dict(Counter(row['label'] for row in rows)), flush=True)
    cache_path = DATA / 'features.pt'
    manifest_digest = hashlib.sha256((DATA / 'training_manifest.json').read_bytes()).hexdigest()
    if cache_path.exists():
        cached = torch.load(cache_path, weights_only=True)
        assert cached['manifest_sha256'] == manifest_digest, 'Stale feature cache; remove it before rerunning'
    else:
        cached = {'manifest_sha256': manifest_digest}
        for split in ('train', 'validation', 'test'):
            print('EXTRACT', split, flush=True)
            features, labels, records = extract(model, splits[split], transform, train=(split == 'train'))
            cached[split] = {'features': features, 'labels': labels, 'records': records}
        torch.save(cached, cache_path)
    train, validation, test = (cached[split] for split in ('train', 'validation', 'test'))
    original_head = copy.deepcopy(model.head)
    head = copy.deepcopy(model.head).requires_grad_(True)
    optimizer = torch.optim.AdamW(head.parameters(), lr=0.003, weight_decay=0.02)
    frequencies = torch.bincount(train['labels']).float()
    class_weights = frequencies.sum() / (2 * frequencies)
    class_weights[1] *= 1.5
    loss_fn = nn.CrossEntropyLoss(weight=class_weights)
    best_loss, best_epoch, best_state = float('inf'), 0, None
    for epoch in range(1, 201):
        head.train()
        order = torch.randperm(len(train['labels']))
        for indices in order.split(64):
            optimizer.zero_grad()
            loss = loss_fn(head(train['features'][indices]), train['labels'][indices])
            loss.backward()
            optimizer.step()
        head.eval()
        with torch.inference_mode():
            logits = head(validation['features'])
            val_loss = float(loss_fn(logits, validation['labels']))
            metrics = stats(logits.softmax(1), validation['labels'])
        if val_loss < best_loss - 0.0001:
            best_loss, best_epoch, best_state = val_loss, epoch, copy.deepcopy(head.state_dict())
        if epoch % 10 == 0:
            print('EPOCH', epoch, 'validation_loss', round(val_loss, 4),
                  'FP', metrics['false_positive'], 'recall', round(metrics['recall'], 3), flush=True)
        if epoch - best_epoch >= 35:
            break
    head.load_state_dict(best_state)
    head.eval()
    with torch.inference_mode():
        val_probabilities = head(validation['features']).softmax(1)
    threshold = choose_threshold(val_probabilities, validation['labels'])
    report = {
        'seed': SEED, 'backbone_frozen': True, 'augmented_views_per_training_photo': 4,
        'training_photos': len(splits['train']),
        'new_reviewed_photos': sum(row['kind'] != 'original' for rows in splits.values() for row in rows),
        'split_counts': {split: dict(Counter(row['label'] for row in rows)) for split, rows in splits.items()},
        'best_epoch': best_epoch, 'missile_threshold': threshold, 'not_missile_threshold': 0.8,
        'validation_threshold_target': 'at most 2% empirical false positives; minimum acceptance score 0.90',
        'notes': [
            'Test data was excluded from training and threshold selection.',
            'Tablet simulations are derived diagnostic views, not independent photos.',
            'Uncertain output reduces false labels but does not fix a missed detection.',
            'Camera footage was not recorded or evaluated; real-camera improvement remains unmeasured.',
            'Softmax scores are not calibrated probabilities of correctness.',
        ],
    }
    for name, candidate, acceptance in (
        ('before', original_head, 0.5), ('after_raw', head, 0.5), ('after_policy', head, threshold)
    ):
        report[name], probabilities = evaluate(candidate, test['features'], test['labels'], test['records'], acceptance,
                                               reject=(name == 'after_policy'))
        if name == 'after_policy':
            outputs = predict_policy(probabilities, threshold)
            predictions = [{**row, 'missile_score': float(probs[0]), 'output': output}
                           for row, probs, output in zip(test['records'], probabilities, outputs)]
            (DATA / 'test_predictions.json').write_text(json.dumps(predictions, indent=2, ensure_ascii=False), encoding='utf-8')
    report['validation_after_policy'], _ = evaluate(head, validation['features'], validation['labels'], validation['records'], threshold)
    checkpoint = torch.load(backup_path, weights_only=True, map_location='cpu')
    checkpoint['head'] = head.state_dict()
    checkpoint['inference_policy'] = {'missile_threshold': threshold, 'not_missile_threshold': 0.8}
    checkpoint['training_info'] = {'seed': SEED, 'new_reviewed_photos': report['new_reviewed_photos'], 'best_epoch': best_epoch}
    candidate_path = ROOT / 'missile_model_retrained.pth'
    torch.save(checkpoint, candidate_path)
    report['seconds'] = round(time.monotonic() - start, 1)
    (DATA / 'training_report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2), flush=True)
    print('Candidate saved separately; inspect report before replacing the active model.', flush=True)


if __name__ == '__main__':
    main()
