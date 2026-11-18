"""Fine-tune the final residual block and classifier; keep source splits fixed.

Earlier convolutional layers and batch-normalization running statistics stay
fixed. Cached prefix activations make CPU fine-tuning practical. Selection and
threshold calibration use validation only; reused test sources are evaluated
after selecting the best checkpoint.
"""
import copy
import hashlib
import json
from pathlib import Path
import random
import shutil
import time

import numpy as np
from PIL import Image
import torch
from torch import nn
import torch.nn.functional as F
from torchvision import transforms

from cnn import load_model
from retrain_classifier import tablet_view, stats
from train_round2 import camera_view, choose_threshold, outputs

ROOT = Path(__file__).resolve().parent
DATA = ROOT / 'training_data'
OUT = DATA / 'round3'
SEED = 171103


def prefix(model, inputs):
    b = model.backbone
    x = b.maxpool(b.relu(b.bn1(b.conv1(inputs))))
    return b.layer4[0](b.layer3(b.layer2(b.layer1(x))))


def tail(model, features):
    x = model.backbone.layer4[1](features)
    return model.head(F.normalize(model.backbone.avgpool(x).flatten(1), dim=1))


def cache_split(model, transform, rows, training):
    tensors, labels, records, pending = [], [], [], []
    def flush():
        if pending:
            with torch.inference_mode():
                result = prefix(model, torch.stack(pending))
            tensors.append(result.clone())
            pending.clear()
    for index, row in enumerate(rows):
        with Image.open(row['path']) as opened:
            image = opened.convert('RGB')
        for view in range(4):
            seed = 1711 + index*31 + (view-2)*7919
            if view == 0:
                variant, kind = image, 'photo'
            elif view == 1:
                variant, kind = tablet_view(image), 'tablet'
            else:
                variant = image
                if training and view == 2 and row['label'] == 'not_missiles':
                    torch.manual_seed(SEED + index)
                    variant = transforms.RandomResizedCrop(224, scale=(0.45, 1.0))(variant)
                variant = camera_view(variant, seed, screen=(view == 3))
                kind = 'camera_photo' if view == 2 else 'camera_tablet'
            pending.append(transform(variant))
            labels.append(0 if row['label'] == 'missiles' else 1)
            records.append({**row, 'view': kind})
            if len(pending) == 16:
                flush()
        if (index+1) % 75 == 0:
            print('PREFIX', index+1, '/', len(rows), flush=True)
    flush()
    return {'features': torch.cat(tensors), 'labels': torch.tensor(labels), 'records': records}


def probabilities(model, data):
    model.eval()
    batches = []
    with torch.inference_mode():
        for features in data['features'].split(32):
            batches.append(tail(model, features).softmax(1))
    return torch.cat(batches)


def summarize(probs, data, threshold):
    records, labels = data['records'], data['labels']
    result = {'all_views': stats(probs, labels, threshold)}
    masks = {
        'original_photos': [r['kind'] == 'original' and r['view'] == 'photo' for r in records],
        'extra_photos': [r['kind'] != 'original' and r['view'] == 'photo' for r in records],
        'hands_photos': [r['kind'] == 'hands' and r['view'] == 'photo' for r in records],
        'tablet_photos': [r['kind'] == 'tablets' and r['view'] == 'photo' for r in records],
        'tablet_simulations': [r['view'] == 'tablet' for r in records],
        'camera_like_views': [r['view'].startswith('camera_') for r in records],
    }
    for name, values in masks.items():
        mask = torch.tensor(values, dtype=torch.bool)
        result[name] = stats(probs[mask], labels[mask], threshold)
    result['uncertain_views'] = outputs(probs, threshold).count('uncertain')
    result['total_views'] = len(labels)
    return result


def main():
    OUT.mkdir(exist_ok=True)
    random.seed(SEED)
    np.random.seed(SEED)
    torch.manual_seed(SEED)
    started = time.monotonic()
    backup = ROOT / 'missile_model_before_round3.pth'
    if not backup.exists():
        shutil.copy2(ROOT / 'missile_model.pth', backup)
    model, transform, names = load_model(backup)
    assert names == ['missiles', 'not_missiles']
    model.eval().requires_grad_(False)
    model_hash = hashlib.sha256(backup.read_bytes()).hexdigest()
    manifest_bytes = (DATA / 'training_manifest.json').read_bytes()
    manifest_hash = hashlib.sha256(manifest_bytes).hexdigest()
    splits = json.loads(manifest_bytes)
    cache_path = OUT / 'prefix_features.pt'
    if cache_path.exists():
        cache = torch.load(cache_path, weights_only=True)
        assert cache['model_hash'] == model_hash and cache['manifest_hash'] == manifest_hash
    else:
        cache = {'model_hash': model_hash, 'manifest_hash': manifest_hash}
        for name in ('train', 'validation', 'test'):
            print('EXTRACT PREFIX', name, flush=True)
            cache[name] = cache_split(model, transform, splits[name], name == 'train')
        torch.save(cache, cache_path)
    train, validation, test = (cache[name] for name in ('train', 'validation', 'test'))
    reference_probs = probabilities(model, train).detach()
    before_probs = probabilities(model, validation)
    before_threshold = model.inference_policy['missile_threshold']
    before = summarize(before_probs, validation, before_threshold)
    # Verify the cached prefix reproduces the complete model for an ordinary photo.
    with Image.open(splits['validation'][0]['path']) as image:
        tensor = transform(image.convert('RGB')).unsqueeze(0)
    with torch.inference_mode():
        direct = model(tensor).softmax(1)[0]
    assert torch.allclose(direct, before_probs[0], atol=1e-5)
    block = model.backbone.layer4[1]
    block.requires_grad_(True)
    model.head.requires_grad_(True)
    optimizer = torch.optim.AdamW([
        {'params': block.parameters(), 'lr': 0.00001},
        {'params': model.head.parameters(), 'lr': 0.0001},
    ], weight_decay=0.01)
    counts = torch.bincount(train['labels']).float()
    weights = counts.sum() / (2*counts)
    weights[1] *= 1.2
    criterion = nn.CrossEntropyLoss(weight=weights, label_smoothing=0.02)
    trainable = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print('TRAINABLE PARAMETERS', trainable, 'TRAINING VIEWS', len(train['labels']), flush=True)
    baseline = before['all_views']
    best = {
        'epoch': 0, 'block': copy.deepcopy(block.state_dict()),
        'head': copy.deepcopy(model.head.state_dict()), 'threshold': before_threshold,
        'summary': before,
        'rank': (baseline['true_positive'], -baseline['false_positive'], -before['uncertain_views']),
    }
    history = []
    for epoch in range(1, 9):
        model.eval()  # Keep every BN running mean/variance fixed with small batches.
        model.head.train()
        total_loss = 0.0
        order = torch.randperm(len(train['labels']))
        for step, indices in enumerate(order.split(16), 1):
            optimizer.zero_grad(set_to_none=True)
            logits = tail(model, train['features'][indices])
            task_loss = criterion(logits, train['labels'][indices])
            preservation = F.kl_div(logits.log_softmax(1), reference_probs[indices], reduction='batchmean')
            loss = task_loss + 0.15*preservation
            loss.backward()
            nn.utils.clip_grad_norm_([p for p in model.parameters() if p.requires_grad], 1.0)
            optimizer.step()
            total_loss += float(loss.detach())*len(indices)
            if step % 30 == 0:
                print('EPOCH', epoch, 'BATCH', step, '/', (len(order)+15)//16, flush=True)
        probs = probabilities(model, validation)
        threshold = choose_threshold(probs, validation['labels'], validation['records'])
        summary = summarize(probs, validation, threshold)
        metrics = summary['all_views']
        rank = (metrics['true_positive'], -metrics['false_positive'], -summary['uncertain_views'])
        record = {'epoch': epoch, 'loss': total_loss/len(order), 'threshold': threshold,
                  'validation': summary}
        history.append(record)
        if rank > best['rank'] and metrics['false_positive'] <= baseline['false_positive']:
            best.update(epoch=epoch, block=copy.deepcopy(block.state_dict()),
                        head=copy.deepcopy(model.head.state_dict()), threshold=threshold,
                        summary=summary, rank=rank)
        print('EPOCH RESULT', epoch, 'TP', metrics['true_positive'], 'FP', metrics['false_positive'],
              'threshold', round(threshold, 4), 'BEST EPOCH', best['epoch'], flush=True)
        (OUT / 'progress.json').write_text(json.dumps(history, indent=2)+'\n')
        if epoch >= 4 and epoch-best['epoch'] >= 3:
            break
    block.load_state_dict(best['block'])
    model.head.load_state_dict(best['head'])
    model.eval()
    # Select the checkpoint before evaluating any test scores.
    after_probs = probabilities(model, test)
    selected_threshold = best['threshold']
    report = {
        'round': 3, 'training_method': 'final residual block plus classifier fine-tuning',
        'seed': SEED, 'trainable_parameters': trainable, 'training_source_photos': len(splits['train']),
        'training_views': len(train['labels']), 'new_unique_photos_this_round': 0,
        'epochs_run': len(history), 'selected_epoch': best['epoch'],
        'missile_threshold': selected_threshold, 'not_missile_threshold': 0.8,
        'validation_before': before, 'validation_after': best['summary'], 'history': history,
        'validation_improvement': best['epoch'] > 0,
        'test_after': summarize(after_probs, test, selected_threshold),
        'notes': ['Test source photos are reused holdouts, not a fresh benchmark.',
                  'Derived views are not independent source photographs.',
                  'No physical camera or screen was evaluated.',
                  'Earlier layers and all batch-normalization running statistics stayed fixed.',
                  'The active model is not overwritten by this training script.'],
    }
    predictions = [{**row, 'missile_score': float(prob[0]), 'output': output}
                   for row, prob, output in zip(test['records'], after_probs, outputs(after_probs, selected_threshold))]
    (OUT / 'test_predictions.json').write_text(json.dumps(predictions, indent=2, ensure_ascii=False), encoding='utf-8')
    saved = torch.load(backup, weights_only=True)
    saved['backbone'] = model.backbone.state_dict()
    saved['head'] = model.head.state_dict()
    saved['inference_policy'] = {'missile_threshold': selected_threshold, 'not_missile_threshold': 0.8}
    saved['training_info'] = {'round': 3, 'method': report['training_method'], 'seed': SEED,
                              'selected_epoch': best['epoch'], 'trainable_parameters': trainable}
    torch.save(saved, ROOT / 'missile_model_round3.pth')
    report['seconds'] = round(time.monotonic()-started, 1)
    (OUT / 'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({key: report[key] for key in ('epochs_run', 'selected_epoch', 'trainable_parameters',
                    'validation_improvement', 'validation_before', 'validation_after', 'test_after', 'seconds')}, indent=2), flush=True)


if __name__ == '__main__':
    main()
