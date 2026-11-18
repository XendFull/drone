"""Explicit CNN architecture for the school missile-photo classifier.

ResNet18 backbone: ImageNet pretrained weights, plus the binary head trained on
the reviewed missile/not-missile collection. Weights: missile_model.pth.
No training or downloads occur when this file is imported.
"""
from pathlib import Path
import cv2
from PIL import Image
import torch
from torch import nn
import torch.nn.functional as F
from torchvision import transforms


class ResidualBlock(nn.Module):
    def __init__(self, input_channels, output_channels, stride=1):
        super().__init__()
        self.conv1 = nn.Conv2d(input_channels, output_channels, 3, stride, 1, bias=False)
        self.bn1 = nn.BatchNorm2d(output_channels)
        self.relu = nn.ReLU(inplace=True)
        self.conv2 = nn.Conv2d(output_channels, output_channels, 3, padding=1, bias=False)
        self.bn2 = nn.BatchNorm2d(output_channels)
        self.downsample = None
        if stride != 1 or input_channels != output_channels:
            self.downsample = nn.Sequential(
                nn.Conv2d(input_channels, output_channels, 1, stride, bias=False),
                nn.BatchNorm2d(output_channels))

    def forward(self, x):
        shortcut = x if self.downsample is None else self.downsample(x)
        x = self.relu(self.bn1(self.conv1(x)))
        x = self.bn2(self.conv2(x))
        return self.relu(x + shortcut)


class ResNet18Backbone(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(3, 64, 7, 2, 3, bias=False)
        self.bn1 = nn.BatchNorm2d(64)
        self.relu = nn.ReLU(inplace=True)
        self.maxpool = nn.MaxPool2d(3, 2, 1)
        self.layer1 = nn.Sequential(ResidualBlock(64, 64), ResidualBlock(64, 64))
        self.layer2 = nn.Sequential(ResidualBlock(64, 128, 2), ResidualBlock(128, 128))
        self.layer3 = nn.Sequential(ResidualBlock(128, 256, 2), ResidualBlock(256, 256))
        self.layer4 = nn.Sequential(ResidualBlock(256, 512, 2), ResidualBlock(512, 512))
        self.avgpool = nn.AdaptiveAvgPool2d(1)

    def forward(self, x):
        x = self.maxpool(self.relu(self.bn1(self.conv1(x))))
        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)
        x = self.layer4(x)
        return self.avgpool(x).flatten(1)


class MissileCNN(nn.Module):
    def __init__(self, head_type='mlp'):
        super().__init__()
        self.backbone = ResNet18Backbone()
        self.head = (nn.Linear(512, 2) if head_type == 'linear' else
                     nn.Sequential(nn.Linear(512, 128), nn.ReLU(), nn.Dropout(0.5), nn.Linear(128, 2)))

    def forward(self, x):
        return self.head(F.normalize(self.backbone(x), dim=1))


def load_model(model_path=None):
    torch.set_num_threads(2)
    path = Path(model_path) if model_path else Path(__file__).with_name('missile_model.pth')
    saved = torch.load(path, map_location='cpu', weights_only=True)
    model = MissileCNN(saved['head_type'])
    model.backbone.load_state_dict(saved['backbone'])
    model.head.load_state_dict(saved['head'])
    model.eval()
    model.inference_policy = saved.get('inference_policy', {})
    resize = ([transforms.Resize((224, 224))] if saved['preprocess'] == 'square'
              else [transforms.Resize(256), transforms.CenterCrop(224)])
    transform = transforms.Compose(resize + [transforms.ToTensor(),
        transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225))])
    return model, transform, saved['class_names']


def classify_probabilities(probabilities, class_names, policy=None):
    """Allow rejection instead of forcing a class for an unfamiliar crop.

    Thresholds are stored with the retrained checkpoint. Legacy checkpoints
    retain their previous argmax behavior for reproducible comparisons.
    """
    policy = policy or {}
    results = []
    for row in probabilities:
        if policy.get('class_thresholds'):
            index = int(row.argmax())
            label, score = class_names[index], float(row[index])
            required = policy['class_thresholds'].get(label, 1.000001)
            results.append((label if score >= required else 'uncertain', score))
        elif policy and 'missiles' in class_names and 'not_missiles' in class_names:
            missile_score = float(row[class_names.index('missiles')])
            negative_score = float(row[class_names.index('not_missiles')])
            if missile_score >= policy['missile_threshold']:
                results.append(('missiles', missile_score))
            elif negative_score >= policy['not_missile_threshold']:
                results.append(('not_missiles', negative_score))
            else:
                results.append(('uncertain', float(row.max())))
        else:
            index = int(row.argmax())
            results.append((class_names[index], float(row[index])))
    return results


def predict_frames(frames, model, transform, class_names):
    if not frames:
        return []
    inputs = torch.stack([transform(Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))) for frame in frames])
    with torch.inference_mode():
        probabilities = model(inputs).softmax(1)
    return classify_probabilities(probabilities, class_names,
                                  getattr(model, 'inference_policy', None))


if __name__ == '__main__':
    model, transform, class_names = load_model()
    print(model)
