"""Second training round: extra camera-like views, validation-only selection.

Reuse the fixed source-photo splits and the frozen ResNet features from round 1.
No test image participates in training, threshold choice or candidate selection.
Test sources have been evaluated previously and are reported as reused holdouts.
"""
from collections import Counter
import copy
import hashlib
from io import BytesIO
import json
from pathlib import Path
import random
import shutil
import time

import cv2
import numpy as np
from PIL import Image, ImageEnhance, ImageFilter, ImageOps
import torch
from torch import nn
import torch.nn.functional as F
from torchvision import transforms

from cnn import load_model
from retrain_classifier import tablet_view, stats

ROOT = Path(__file__).resolve().parent
DATA = ROOT / 'training_data'
OUT = DATA / 'round2'
SEED = 1711


def camera_view(image, seed, screen=False):
    """Vary exposure, focus, JPEG quality and screen perspective."""
    rng = random.Random(seed)
    image = image.convert('RGB')
    if screen:
        image = tablet_view(image)
        width, height = image.size
        source = np.float32([[0, 0], [width-1, 0], [width-1, height-1], [0, height-1]])
        margins = [rng.uniform(0.01, 0.08) for _ in range(8)]
        target = np.float32([
            [width*margins[0], height*margins[1]],
            [width*(1-margins[2]), height*margins[3]],
            [width*(1-margins[4]), height*(1-margins[5])],
            [width*margins[6], height*(1-margins[7])],
        ])
        matrix = cv2.getPerspectiveTransform(source, target)
        image = Image.fromarray(cv2.warpPerspective(np.asarray(image), matrix,
                               (width, height), borderValue=(100, 100, 100)))
    image = ImageEnhance.Brightness(image).enhance(rng.uniform(0.65, 1.25))
    image = ImageEnhance.Contrast(image).enhance(rng.uniform(0.8, 1.2))
    image = ImageEnhance.Color(image).enhance(rng.uniform(0.75, 1.15))
    image = image.filter(ImageFilter.GaussianBlur(rng.uniform(0.2, 0.8)))
    memory = BytesIO()
    image.save(memory, format='JPEG', quality=rng.randint(50, 85))
    memory.seek(0)
    with Image.open(memory) as compressed:
        return compressed.convert('RGB')


def extra_features(model, transform, rows, training):
    features, labels, records, pending = [], [], [], []
    views = 4 if training else 2
    def flush():
        if pending:
            with torch.inference_mode():
                result = F.normalize(model.backbone(torch.stack(pending)), dim=1)
            features.append(result.clone())
            pending.clear()
    for index, row in enumerate(rows):
        with Image.open(row['path']) as image:
            original = image.convert('RGB')
        for view in range(views):
            seed = SEED + index*31 + view*7919
            image = original.copy()
            # Negative-only crops cannot accidentally remove a positive object.
            if training and view < 2 and row['label'] == 'not_missiles':
                torch.manual_seed(seed)
                image = transforms.RandomResizedCrop(224, scale=(0.45, 1.0))(image)
            screen = (view >= 2) if training else (view == 1)
            image = camera_view(image, seed, screen)
            pending.append(transform(image))
            labels.append(0 if row['label'] == 'missiles' else 1)
            records.append({**row, 'view': 'camera_tablet' if screen else 'camera_photo'})
            if len(pending) == 16:
                flush()
        if (index + 1) % 75 == 0:
            print('EXTRA FEATURES', index + 1, '/', len(rows), flush=True)
    flush()
    return {'features': torch.cat(features), 'labels': torch.tensor(labels), 'records': records}


def outputs(probabilities, threshold):
    return ['missiles' if row[0] >= threshold else
            'not_missiles' if row[1] >= 0.8 else 'uncertain'
            for row in probabilities.tolist()]


def choose_threshold(probabilities, labels, records):
    scores = probabilities[:, 0]
    allowed_fp = int(int((labels == 1).sum()) * 0.02)
    extra_photos = torch.tensor([
        row['kind'] != 'original' and row['view'] == 'photo' for row in records
    ], dtype=torch.bool)
    choices = [0.8, 0.9, 1.00001]
    for score in scores.tolist():
        if score >= 0.8:
            choices.extend((score, float(np.nextafter(np.float32(score), np.float32(np.inf)))))
    best = None
    for threshold in sorted(set(choices)):
        metrics = stats(probabilities, labels, threshold)
        if metrics['false_positive'] > allowed_fp:
            continue
        if int(((scores >= threshold) & extra_photos & (labels == 1)).sum()):
            continue
        uncertain = outputs(probabilities, threshold).count('uncertain')
        rank = (metrics['true_positive'], -metrics['false_positive'], -uncertain, -threshold)
        if best is None or rank > best[0]:
            best = (rank, threshold)
    return best[1]


def score(head, dataset, threshold):
    head.eval()
    with torch.inference_mode():
        probabilities = head(dataset['features']).softmax(1)
    records, labels = dataset['records'], dataset['labels']
    masks = {
        'original_photos': [r['kind'] == 'original' and r['view'] == 'photo' for r in records],
        'extra_photos': [r['kind'] != 'original' and r['view'] == 'photo' for r in records],
        'hands_photos': [r['kind'] == 'hands' and r['view'] == 'photo' for r in records],
        'tablet_photos': [r['kind'] == 'tablets' and r['view'] == 'photo' for r in records],
        'tablet_simulations': [r['view'] == 'tablet' for r in records],
        'camera_like_views': [r['view'].startswith('camera_') for r in records],
    }
    result = {'all_views': stats(probabilities, labels, threshold)}
    for name, values in masks.items():
        mask = torch.tensor(values, dtype=torch.bool)
        if mask.any():
            result[name] = stats(probabilities[mask], labels[mask], threshold)
    names = outputs(probabilities, threshold)
    result['uncertain_views'] = names.count('uncertain')
    result['total_views'] = len(labels)
    return result, probabilities


def fit_head(head, train, validation, name, learning_rate):
    head.requires_grad_(True)
    optimizer = torch.optim.AdamW(head.parameters(), lr=learning_rate, weight_decay=0.04)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=150)
    frequency = torch.bincount(train['labels']).float()
    weights = frequency.sum() / (2*frequency)
    weights[1] *= 1.2
    criterion = nn.CrossEntropyLoss(weight=weights, label_smoothing=0.02)
    best = None
    patience = 0
    for epoch in range(1, 151):
        head.train()
        for indices in torch.randperm(len(train['labels'])).split(64):
            optimizer.zero_grad(set_to_none=True)
            loss = criterion(head(train['features'][indices]), train['labels'][indices])
            loss.backward()
            optimizer.step()
        scheduler.step()
        if epoch % 5 and epoch != 1:
            continue
        head.eval()
        with torch.inference_mode():
            logits = head(validation['features'])
            val_loss = float(criterion(logits, validation['labels']))
            probabilities = logits.softmax(1)
        threshold = choose_threshold(probabilities, validation['labels'], validation['records'])
        metrics = stats(probabilities, validation['labels'], threshold)
        uncertain = outputs(probabilities, threshold).count('uncertain')
        # Test images are never consulted here. Prefer recall within the
        # false-positive constraint, then fewer alerts and uncertain outputs.
        rank = (metrics['true_positive'], -metrics['false_positive'], -uncertain, -val_loss)
        if best is None or rank > best['rank']:
            best = {'rank': rank, 'state': copy.deepcopy(head.state_dict()),
                    'threshold': threshold, 'epoch': epoch, 'validation_loss': val_loss}
            patience = 0
        else:
            patience += 1
        if epoch % 20 == 0:
            print(name, 'epoch', epoch, 'validation TP', metrics['true_positive'],
                  'FP', metrics['false_positive'], 'threshold', round(threshold, 4), flush=True)
        if patience >= 10:
            break
    head.load_state_dict(best['state'])
    head.eval()
    print('BEST', name, 'epoch', best['epoch'], 'rank', best['rank'], flush=True)
    return head, best


def main():
    OUT.mkdir(exist_ok=True)
    random.seed(SEED)
    np.random.seed(SEED)
    torch.manual_seed(SEED)
    start = time.monotonic()
    checkpoint = ROOT / 'missile_model_before_round2.pth'
    if not checkpoint.exists():
        shutil.copy2(ROOT / 'missile_model.pth', checkpoint)
    model, transform, names = load_model(checkpoint)
    assert names == ['missiles', 'not_missiles']
    model.eval().requires_grad_(False)
    original_backbone = torch.load(ROOT / 'missile_model_before_hard_negatives.pth', weights_only=True)['backbone']
    assert all(torch.equal(value, original_backbone[key]) for key, value in model.backbone.state_dict().items())
    old_cache = torch.load(DATA / 'features.pt', weights_only=True)
    manifest_bytes = (DATA / 'training_manifest.json').read_bytes()
    digest = hashlib.sha256(manifest_bytes).hexdigest()
    assert old_cache['manifest_sha256'] == digest
    splits = json.loads(manifest_bytes)
    cache_path = OUT / 'extra_features.pt'
    if cache_path.exists():
        extra = torch.load(cache_path, weights_only=True)
        assert extra['manifest_sha256'] == digest and extra['seed'] == SEED
    else:
        extra = {'manifest_sha256': digest, 'seed': SEED}
        for split in ('train', 'validation', 'test'):
            print('EXTRACT', split, flush=True)
            extra[split] = extra_features(model, transform, splits[split], split == 'train')
        torch.save(extra, cache_path)
    combined = {}
    for split in ('train', 'validation', 'test'):
        combined[split] = {
            'features': torch.cat((old_cache[split]['features'], extra[split]['features'])),
            'labels': torch.cat((old_cache[split]['labels'], extra[split]['labels'])),
            'records': old_cache[split]['records'] + extra[split]['records'],
        }
    train, validation, test = (combined[split] for split in ('train', 'validation', 'test'))
    baseline_threshold = model.inference_policy['missile_threshold']
    before_validation, _ = score(model.head, validation, baseline_threshold)
    candidates = []
    for name, head, learning_rate, head_type in (
        ('mlp_slow', copy.deepcopy(model.head), 0.0004, 'mlp'),
        ('mlp', copy.deepcopy(model.head), 0.001, 'mlp'),
        ('linear', nn.Linear(512, 2), 0.015, 'linear'),
    ):
        torch.manual_seed(SEED)
        head, details = fit_head(head, train, validation, name, learning_rate)
        candidates.append((details['rank'], name, head_type, head, details))
    _, name, head_type, selected, details = max(candidates, key=lambda candidate: candidate[0])
    threshold = details['threshold']
    after_validation, _ = score(selected, validation, threshold)
    previous = before_validation['all_views']
    current = after_validation['all_views']
    deploy_recommended = (current['false_positive'] <= previous['false_positive']
                          and current['true_positive'] > previous['true_positive'])
    report = {
        'round': 2, 'seed': SEED, 'backbone_frozen': True,
        'source_images': sum(len(rows) for rows in splits.values()),
        'training_source_images': len(splits['train']), 'training_views': len(train['labels']),
        'augmentation_views_per_training_photo': 8,
        'added_unique_photos_this_round': 0,
        'selected': name, 'head_type': head_type, 'best_epoch': details['epoch'],
        'missile_threshold': threshold, 'not_missile_threshold': 0.8,
        'selection': 'Validation only, empirical FP target <=2%, zero FP on added validation photos, then maximum recall.',
        'validation_before': before_validation, 'validation_after': after_validation,
        'validation_improvement': deploy_recommended,
        'candidate_summaries': [{'name': candidate[1], 'epoch': candidate[4]['epoch'],
                                 'rank': candidate[4]['rank'], 'threshold': candidate[4]['threshold']}
                                for candidate in candidates],
        'notes': ['Test sources are reused holdouts, not a new independent benchmark.',
                  'Augmented/simulated views are not new independent photographs.',
                  'No physical webcam or tablet footage has been evaluated.',
                  'Scores are not calibrated probabilities of correctness.'],
    }
    # Selection is complete before examining any test performance.
    report['test_before'], _ = score(model.head, test, baseline_threshold)
    report['test_after'], probabilities = score(selected, test, threshold)
    predictions = [{**record, 'missile_score': float(probability[0]), 'output': output}
                   for record, probability, output in zip(test['records'], probabilities, outputs(probabilities, threshold))]
    (OUT / 'test_predictions.json').write_text(json.dumps(predictions, indent=2, ensure_ascii=False), encoding='utf-8')
    saved = torch.load(checkpoint, weights_only=True)
    saved['head_type'] = head_type
    saved['head'] = selected.state_dict()
    saved['inference_policy'] = {'missile_threshold': threshold, 'not_missile_threshold': 0.8}
    saved['training_info'] = {'round': 2, 'seed': SEED, 'source_training_images': len(splits['train']),
                              'training_views': len(train['labels']), 'best_epoch': details['epoch']}
    torch.save(saved, ROOT / 'missile_model_round2.pth')
    report['seconds'] = round(time.monotonic()-start, 1)
    (OUT / 'report.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({key: report[key] for key in ('selected', 'training_views', 'missile_threshold',
                    'validation_improvement', 'validation_before', 'validation_after', 'test_before', 'test_after', 'seconds')}, indent=2), flush=True)
    print('Candidate saved separately; active model has not been replaced.', flush=True)


if __name__ == '__main__':
    main()
