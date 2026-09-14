"""Loop-retrieval labels, metrics, descriptors, and MobileNetV3 models.

Ground-truth poses are used by this module only to construct supervised labels
and to evaluate retrieval.  They are never an input to descriptor inference or
to the RGB-D odometry estimate.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
from PIL import Image

from .metrics import invert_pose, rotation_angle_degrees


@dataclass(frozen=True)
class LoopProtocol:
    """Fixed geometric definition used by training, validation, and testing."""

    min_frame_separation: int = 30
    positive_translation_m: float = 0.20
    positive_rotation_deg: float = 25.0
    negative_translation_m: float = 0.50
    negative_rotation_deg: float = 60.0
    evaluation_step: int = 5

    def validate(self) -> None:
        if self.min_frame_separation < 1 or self.evaluation_step < 1:
            raise ValueError("Frame separation and evaluation step must be positive")
        if not 0 < self.positive_translation_m < self.negative_translation_m:
            raise ValueError("Translation thresholds must satisfy 0 < positive < negative")
        if not 0 < self.positive_rotation_deg < self.negative_rotation_deg <= 180:
            raise ValueError("Rotation thresholds must satisfy 0 < positive < negative <= 180")

    def to_dict(self) -> dict[str, int | float]:
        return asdict(self)


def pose_separation(first: np.ndarray, second: np.ndarray) -> tuple[float, float]:
    """Return translation and rotation separating two camera-to-world poses."""
    relative = invert_pose(np.asarray(first, dtype=float)) @ np.asarray(second, dtype=float)
    return float(np.linalg.norm(relative[:3, 3])), rotation_angle_degrees(relative[:3, :3])


def is_positive_pair(first: np.ndarray, second: np.ndarray, protocol: LoopProtocol) -> bool:
    translation, rotation = pose_separation(first, second)
    return translation <= protocol.positive_translation_m and rotation <= protocol.positive_rotation_deg


def is_negative_pair(first: np.ndarray, second: np.ndarray, protocol: LoopProtocol) -> bool:
    translation, rotation = pose_separation(first, second)
    return translation >= protocol.negative_translation_m or rotation >= protocol.negative_rotation_deg


def make_training_triplets(
    poses: list[np.ndarray],
    protocol: LoopProtocol,
    *,
    triplets_per_anchor: int = 4,
    seed: int = 0,
    negative_sampling: str = "random",
    hard_negative_fraction: float = 0.25,
) -> list[tuple[int, int, int]]:
    """Build deterministic (anchor, positive, negative) supervision triples."""
    protocol.validate()
    if triplets_per_anchor < 1:
        raise ValueError("triplets_per_anchor must be positive")
    if negative_sampling not in {"random", "geometric_hard"}:
        raise ValueError("negative_sampling must be 'random' or 'geometric_hard'")
    if not 0 < hard_negative_fraction <= 1:
        raise ValueError("hard_negative_fraction must be in (0, 1]")
    rng = np.random.default_rng(seed)
    triplets: list[tuple[int, int, int]] = []
    for anchor in range(len(poses)):
        eligible = [
            candidate
            for candidate in range(len(poses))
            if abs(candidate - anchor) >= protocol.min_frame_separation
        ]
        positives = [candidate for candidate in eligible if is_positive_pair(poses[anchor], poses[candidate], protocol)]
        negatives = [candidate for candidate in eligible if is_negative_pair(poses[anchor], poses[candidate], protocol)]
        if not positives or not negatives:
            continue
        if negative_sampling == "geometric_hard":
            negatives = sorted(
                negatives,
                key=lambda candidate: (
                    pose_separation(poses[anchor], poses[candidate])[0] / protocol.negative_translation_m
                    + pose_separation(poses[anchor], poses[candidate])[1] / protocol.negative_rotation_deg
                ),
            )[:max(1, int(np.ceil(len(negatives) * hard_negative_fraction)))]
        for _ in range(triplets_per_anchor):
            positive = int(positives[int(rng.integers(len(positives)))])
            negative = int(negatives[int(rng.integers(len(negatives)))])
            triplets.append((anchor, positive, negative))
    rng.shuffle(triplets)
    return triplets


def build_frozen_mobilenet_descriptor(*, pretrained: bool = True):
    """Return pooled MobileNetV3-Small features without task-specific training."""
    import torch
    from torch import nn
    from torchvision.models import MobileNet_V3_Small_Weights, mobilenet_v3_small

    weights = MobileNet_V3_Small_Weights.IMAGENET1K_V1 if pretrained else None
    backbone = mobilenet_v3_small(weights=weights)

    class FrozenMobileNetDescriptor(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.features = backbone.features
            self.pool = nn.AdaptiveAvgPool2d(1)
            for parameter in self.parameters():
                parameter.requires_grad_(False)

        def forward(self, images):
            features = self.pool(self.features(images)).flatten(1)
            return torch.nn.functional.normalize(features, p=2, dim=1)

    return FrozenMobileNetDescriptor()


def evaluation_pairs(
    poses: list[np.ndarray], protocol: LoopProtocol
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return historical query/candidate indices and their GT evaluation labels."""
    protocol.validate()
    queries: list[int] = []
    candidates: list[int] = []
    labels: list[bool] = []
    start = protocol.min_frame_separation
    for query in range(start, len(poses), protocol.evaluation_step):
        for candidate in range(0, query - protocol.min_frame_separation + 1):
            queries.append(query)
            candidates.append(candidate)
            labels.append(is_positive_pair(poses[query], poses[candidate], protocol))
    return np.asarray(queries, dtype=np.int64), np.asarray(candidates, dtype=np.int64), np.asarray(labels, dtype=bool)


def _binary_metrics(scores: np.ndarray, labels: np.ndarray, threshold: float) -> dict[str, float | int]:
    predictions = np.asarray(scores) >= threshold
    labels = np.asarray(labels, dtype=bool)
    true_positive = int(np.count_nonzero(predictions & labels))
    false_positive = int(np.count_nonzero(predictions & ~labels))
    false_negative = int(np.count_nonzero(~predictions & labels))
    precision = true_positive / (true_positive + false_positive) if true_positive + false_positive else 0.0
    recall = true_positive / (true_positive + false_negative) if true_positive + false_negative else 0.0
    f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0
    return {
        "threshold": float(threshold),
        "true_positive": true_positive,
        "false_positive": false_positive,
        "false_negative": false_negative,
        "precision": float(precision),
        "recall": float(recall),
        "f1": float(f1),
    }


def select_f1_threshold(scores: np.ndarray, labels: np.ndarray) -> dict[str, float | int]:
    """Choose a cosine-similarity threshold by maximum F1 on validation data."""
    scores = np.asarray(scores, dtype=float)
    labels = np.asarray(labels, dtype=bool)
    if scores.ndim != 1 or scores.size == 0 or labels.shape != scores.shape or not np.any(labels):
        raise ValueError("Non-empty one-dimensional scores with at least one positive are required")
    order = np.argsort(-scores, kind="stable")
    sorted_scores = scores[order]
    sorted_labels = labels[order]
    cumulative_tp = np.cumsum(sorted_labels)
    cumulative_fp = np.cumsum(~sorted_labels)
    group_ends = np.flatnonzero(np.r_[sorted_scores[1:] != sorted_scores[:-1], True])
    tp = cumulative_tp[group_ends].astype(float)
    fp = cumulative_fp[group_ends].astype(float)
    fn = float(np.count_nonzero(labels)) - tp
    precision = tp / np.maximum(tp + fp, 1.0)
    recall = tp / np.maximum(tp + fn, 1.0)
    f1 = 2.0 * precision * recall / np.maximum(precision + recall, np.finfo(float).eps)
    best_candidates = np.flatnonzero(f1 == np.max(f1))
    # When F1 ties, prefer higher precision, then the stricter threshold.
    best = int(best_candidates[np.argmax(precision[best_candidates])])
    return _binary_metrics(scores, labels, float(sorted_scores[group_ends[best]]))


def evaluate_descriptors(
    descriptors: np.ndarray,
    poses: list[np.ndarray],
    protocol: LoopProtocol,
    *,
    threshold: float,
) -> tuple[dict[str, float | int], list[dict[str, int | float | bool]]]:
    """Evaluate historical loop retrieval and pair classification."""
    descriptors = np.asarray(descriptors, dtype=np.float32)
    if descriptors.ndim != 2 or descriptors.shape[0] != len(poses):
        raise ValueError("Descriptor rows must match the number of poses")
    norms = np.linalg.norm(descriptors, axis=1, keepdims=True)
    if np.any(norms <= 0):
        raise ValueError("Descriptors must have non-zero norm")
    descriptors = descriptors / norms
    query_indices, candidate_indices, labels = evaluation_pairs(poses, protocol)
    scores = np.sum(descriptors[query_indices] * descriptors[candidate_indices], axis=1)
    pair_metrics = _binary_metrics(scores, labels, threshold)

    rows: list[dict[str, int | float | bool]] = []
    recall_1_hits = 0
    recall_5_hits = 0
    evaluable_queries = 0
    for query in np.unique(query_indices):
        mask = query_indices == query
        query_scores = scores[mask]
        query_candidates = candidate_indices[mask]
        query_labels = labels[mask]
        if not np.any(query_labels):
            continue
        evaluable_queries += 1
        order = np.argsort(-query_scores, kind="stable")
        top1 = int(order[0])
        top5 = order[: min(5, len(order))]
        hit1 = bool(query_labels[top1])
        hit5 = bool(np.any(query_labels[top5]))
        recall_1_hits += int(hit1)
        recall_5_hits += int(hit5)
        rows.append({
            "query_index": int(query),
            "top1_index": int(query_candidates[top1]),
            "top1_score": float(query_scores[top1]),
            "top1_correct": hit1,
            "top5_correct": hit5,
            "positive_candidates": int(np.count_nonzero(query_labels)),
            "eligible_candidates": int(len(query_labels)),
        })
    metrics = {
        **pair_metrics,
        "pairs": int(len(scores)),
        "positive_pairs": int(np.count_nonzero(labels)),
        "evaluable_queries": evaluable_queries,
        "recall_at_1": recall_1_hits / evaluable_queries if evaluable_queries else 0.0,
        "recall_at_5": recall_5_hits / evaluable_queries if evaluable_queries else 0.0,
    }
    return metrics, rows


def validation_scores(
    descriptors: np.ndarray, poses: list[np.ndarray], protocol: LoopProtocol
) -> tuple[np.ndarray, np.ndarray]:
    """Return cosine scores and labels for validation-only threshold fitting."""
    descriptors = np.asarray(descriptors, dtype=np.float32)
    descriptors = descriptors / np.linalg.norm(descriptors, axis=1, keepdims=True)
    query_indices, candidate_indices, labels = evaluation_pairs(poses, protocol)
    scores = np.sum(descriptors[query_indices] * descriptors[candidate_indices], axis=1)
    return scores, labels


def hsv_histogram_descriptor(path: str | Path, bins: int = 8) -> np.ndarray:
    """Compute a Hellinger-normalized global HSV histogram baseline."""
    if bins < 2:
        raise ValueError("bins must be at least two")
    with Image.open(path) as image:
        hsv = np.asarray(image.convert("RGB").resize((160, 120)).convert("HSV"), dtype=np.uint8)
    histogram, _ = np.histogramdd(
        hsv.reshape(-1, 3), bins=(bins, bins, bins), range=((0, 256), (0, 256), (0, 256)),
    )
    histogram = histogram.reshape(-1).astype(np.float32)
    histogram /= max(float(np.sum(histogram)), np.finfo(np.float32).eps)
    histogram = np.sqrt(histogram)
    histogram /= max(float(np.linalg.norm(histogram)), np.finfo(np.float32).eps)
    return histogram


def build_mobilenet_descriptor(*, with_se: bool, descriptor_dim: int = 128, pretrained: bool = True):
    """Build MobileNetV3-Small descriptor with an optional SE ablation."""
    import torch
    from torch import nn
    from torchvision.models import MobileNet_V3_Small_Weights, mobilenet_v3_small
    from torchvision.ops.misc import SqueezeExcitation

    weights = MobileNet_V3_Small_Weights.IMAGENET1K_V1 if pretrained else None
    backbone = mobilenet_v3_small(weights=weights)
    if not with_se:
        def remove_se(module: nn.Module) -> None:
            for name, child in list(module.named_children()):
                if isinstance(child, SqueezeExcitation):
                    setattr(module, name, nn.Identity())
                else:
                    remove_se(child)
        remove_se(backbone.features)
    feature_dim = int(backbone.classifier[0].in_features)

    class MobileNetDescriptor(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.features = backbone.features
            self.pool = nn.AdaptiveAvgPool2d(1)
            self.projection = nn.Linear(feature_dim, descriptor_dim)

        def forward(self, images):
            features = self.pool(self.features(images)).flatten(1)
            return torch.nn.functional.normalize(self.projection(features), p=2, dim=1)

    return MobileNetDescriptor()
