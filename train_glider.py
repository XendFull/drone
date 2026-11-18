"""Train a separate, explicitly synthetic-only Glide350 recognition prototype."""
import copy
import hashlib
import json
from pathlib import Path
import random
import time

import numpy as np
from PIL import Image
import torch
from torch import nn
import torch.nn.functional as F

import cnn

ROOT = Path(__file__).resolve().parent
DATA = ROOT/'glider_training'
SEED = 3501711


def extract(model, transform, rows):
    vectors = []
    for start in range(0, len(rows), 16):
        batch = []
        for row in rows[start:start+16]:
            with Image.open(row['path']) as image:
                batch.append(transform(image.convert('RGB')))
        with torch.inference_mode():
            feature = F.normalize(model.backbone(torch.stack(batch)), dim=1)
        vectors.append(feature.clone())
        if start and start % 160 == 0:
            print('FEATURES', start, '/', len(rows), flush=True)
    return torch.cat(vectors), torch.tensor([0 if row['label']=='glider' else 1 for row in rows])


def stats(probs, labels, threshold):
    predicted = probs[:, 0] >= threshold
    positive = labels == 0
    return {'true_positive':int((predicted & positive).sum()),
            'false_positive':int((predicted & ~positive).sum()),
            'false_negative':int((~predicted & positive).sum()),
            'true_negative':int((~predicted & ~positive).sum())}


def threshold_for(probs, labels):
    allowed = int(int((labels==1).sum())*.01)
    candidates = [0.9, 1.00001]
    for score in probs[:, 0].tolist():
        if score >= 0.9:
            candidates.extend((score, float(np.nextafter(np.float32(score), np.float32(np.inf)))))
    valid = []
    for threshold in candidates:
        metrics = stats(probs, labels, threshold)
        if metrics['false_positive'] <= allowed:
            valid.append((metrics['true_positive'], -metrics['false_positive'], -threshold, threshold))
    return max(valid)[-1]


def main():
    random.seed(SEED);np.random.seed(SEED);torch.manual_seed(SEED)
    started = time.monotonic()
    manifest_bytes = (DATA/'manifest.json').read_bytes()
    manifest = json.loads(manifest_bytes)
    digest = hashlib.sha256(manifest_bytes).hexdigest()
    model, transform, _ = cnn.load_model(ROOT/'missile_model.pth')
    model.eval().requires_grad_(False)
    cache_path = DATA/f'features_{digest[:12]}.pt'
    if cache_path.exists():
        cached = torch.load(cache_path, weights_only=True)
        assert cached['manifest_hash'] == digest
    else:
        cached = {'manifest_hash':digest}
        for split, rows in manifest.items():
            print('EXTRACT', split, flush=True)
            features, labels = extract(model, transform, rows)
            cached[split] = {'features':features, 'labels':labels}
        torch.save(cached, cache_path)
    head = nn.Sequential(nn.Linear(512,128),nn.ReLU(),nn.Dropout(.5),nn.Linear(128,2))
    optimizer = torch.optim.AdamW(head.parameters(),lr=.003,weight_decay=.02)
    loss_fn = nn.CrossEntropyLoss(label_smoothing=.02)
    train, validation, test = (cached[name] for name in ('train','validation','test'))
    best, stale = None, 0
    for epoch in range(1,151):
        head.train()
        for indices in torch.randperm(len(train['labels'])).split(64):
            optimizer.zero_grad()
            loss = loss_fn(head(train['features'][indices]),train['labels'][indices])
            loss.backward();optimizer.step()
        head.eval()
        with torch.inference_mode():
            logits = head(validation['features'])
            val_loss = float(loss_fn(logits,validation['labels']))
        if best is None or val_loss < best['loss']-.0001:
            best = {'epoch':epoch,'loss':val_loss,'head':copy.deepcopy(head.state_dict())}
            stale = 0
        else:
            stale += 1
        if epoch%10==0:
            print('EPOCH',epoch,'validation_loss',round(val_loss,4),flush=True)
        if stale >= 25:
            break
    head.load_state_dict(best['head']);head.eval()
    with torch.inference_mode():
        val_probs = head(validation['features']).softmax(1)
        test_probs = head(test['features']).softmax(1)
    threshold = threshold_for(val_probs, validation['labels'])
    policy = {'class_thresholds':{'glider':threshold,'background':.8}}
    saved = torch.load(ROOT/'missile_model.pth',weights_only=True)
    saved['head_type']='mlp';saved['head']=head.state_dict()
    saved['class_names']=['glider','background'];saved['inference_policy']=policy
    saved['training_info']={'object':'Glide350','positive_data':'CAD renderings only',
                            'physical_validation':False,'best_epoch':best['epoch'],'seed':SEED}
    torch.save(saved,ROOT/'glider_model.pth')
    outputs = cnn.classify_probabilities(test_probs,saved['class_names'],policy)
    report={'object':'Glide350','training_source':'Synthetic views of the actual 3D geometry composited on split-specific photo backgrounds',
            'physical_glider_photos':0,'physical_glider_tests':0,'best_epoch':best['epoch'],'epochs_run':epoch,
            'split_image_counts':{split:len(rows) for split,rows in manifest.items()},
            'source_backgrounds_disjoint_between_splits':True,
            'glider_threshold':threshold,'validation':stats(val_probs,validation['labels'],threshold),
            'test':stats(test_probs,test['labels'],threshold),
            'test_uncertain':sum(label=='uncertain' for label,score in outputs),
            'limitations':['Render-only results do not establish real-world recognition.',
                           'The physical print, chosen finish, motion blur and actual camera require separate testing.',
                           'Softmax scores are not calibrated probabilities of correctness.'],
            'seconds':round(time.monotonic()-started,1)}
    (DATA/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    predictions=[{**row,'output':label,'score':score} for row,(label,score) in zip(manifest['test'],outputs)]
    (DATA/'test_predictions.json').write_text(json.dumps(predictions,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    main()
