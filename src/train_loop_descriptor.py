"""Train and evaluate loop descriptors on sequence-isolated TUM RGB-D data."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import platform
import random
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

from .loop_learning import (
    LoopProtocol,
    build_mobilenet_descriptor,
    evaluate_descriptors,
    hsv_histogram_descriptor,
    make_training_triplets,
    select_f1_threshold,
    validation_scores,
)
from .runtime import process_rss_bytes
from .tum import RgbdFrame, load_tum_rgbd_frames


def percentile(values: list[float], value: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=float), value))


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    import torch

    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True


def make_transforms():
    from torchvision import transforms

    normalize = transforms.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225))
    training = transforms.Compose(
        [
            transforms.Resize(256),
            transforms.RandomResizedCrop(224, scale=(0.75, 1.0)),
            transforms.ColorJitter(brightness=0.20, contrast=0.20, saturation=0.15, hue=0.03),
            transforms.ToTensor(),
            normalize,
        ]
    )
    evaluation = transforms.Compose(
        [transforms.Resize(232), transforms.CenterCrop(224), transforms.ToTensor(), normalize]
    )
    return training, evaluation


class TripletImageDataset:
    def __init__(self, frames: list[RgbdFrame], triplets: list[tuple[int, int, int]], transform) -> None:
        self.frames = frames
        self.triplets = triplets
        self.transform = transform

    def __len__(self) -> int:
        return len(self.triplets)

    def _read(self, index: int):
        with Image.open(self.frames[index].rgb_path) as image:
            return self.transform(image.convert("RGB"))

    def __getitem__(self, index: int):
        anchor, positive, negative = self.triplets[index]
        return self._read(anchor), self._read(positive), self._read(negative)


class FrameImageDataset:
    def __init__(self, frames: list[RgbdFrame], transform) -> None:
        self.frames = frames
        self.transform = transform

    def __len__(self) -> int:
        return len(self.frames)

    def __getitem__(self, index: int):
        with Image.open(self.frames[index].rgb_path) as image:
            return self.transform(image.convert("RGB")), index


def extract_model_descriptors(model, frames: list[RgbdFrame], transform, device, batch_size: int):
    import torch
    from torch.utils.data import DataLoader

    loader = DataLoader(
        FrameImageDataset(frames, transform),
        batch_size=batch_size,
        shuffle=False,
        num_workers=0,
        pin_memory=device.type == "cuda",
    )
    model.eval()
    parts: list[np.ndarray] = []
    start = time.perf_counter()
    with torch.inference_mode():
        for images, _indices in loader:
            images = images.to(device, non_blocking=True)
            parts.append(model(images).cpu().numpy())
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    elapsed = time.perf_counter() - start
    return np.concatenate(parts, axis=0), elapsed


def benchmark_model(model, transform, frames: list[RgbdFrame], device, samples: int) -> dict[str, float | int | str]:
    import torch

    sample_count = min(samples, len(frames))
    tensors = []
    for frame in frames[:sample_count]:
        with Image.open(frame.rgb_path) as image:
            tensors.append(transform(image.convert("RGB")).unsqueeze(0).to(device))
    model.eval()
    with torch.inference_mode():
        for tensor in tensors[: min(20, len(tensors))]:
            model(tensor)
    if device.type == "cuda":
        torch.cuda.synchronize(device)
        torch.cuda.reset_peak_memory_stats(device)
    latencies: list[float] = []
    with torch.inference_mode():
        for tensor in tensors:
            start = time.perf_counter()
            model(tensor)
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            latencies.append((time.perf_counter() - start) * 1000.0)
    result: dict[str, float | int | str] = {
        "scope": "model forward only; batch=1; preprocessed tensor already on device",
        "samples": len(latencies),
        "mean_latency_ms": float(np.mean(latencies)),
        "median_latency_ms": float(np.median(latencies)),
        "p95_latency_ms": percentile(latencies, 95),
        "fps_from_mean": 1000.0 / float(np.mean(latencies)),
    }
    if device.type == "cuda":
        result["peak_gpu_allocated_bytes"] = int(torch.cuda.max_memory_allocated(device))
    return result


def evaluate_split(descriptors: np.ndarray, frames: list[RgbdFrame], protocol: LoopProtocol, threshold: float):
    poses = [frame.ground_truth for frame in frames]
    return evaluate_descriptors(descriptors, poses, protocol, threshold=threshold)


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def train_variant(
    name: str,
    *,
    with_se: bool,
    train_frames: list[RgbdFrame],
    validation_frames: list[RgbdFrame],
    triplets: list[tuple[int, int, int]],
    protocol: LoopProtocol,
    output: Path,
    epochs: int,
    batch_size: int,
    learning_rate: float,
    descriptor_dim: int,
    margin: float,
    seed: int,
    device,
    pretrained: bool,
):
    import torch
    from torch import nn
    from torch.utils.data import DataLoader

    set_seed(seed)
    train_transform, eval_transform = make_transforms()
    dataset = TripletImageDataset(train_frames, triplets, train_transform)
    generator = torch.Generator().manual_seed(seed)
    loader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=True,
        generator=generator,
        num_workers=0,
        pin_memory=device.type == "cuda",
        drop_last=False,
    )
    model = build_mobilenet_descriptor(with_se=with_se, descriptor_dim=descriptor_dim, pretrained=pretrained).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=learning_rate, weight_decay=1e-4)
    loss_function = nn.TripletMarginLoss(margin=margin, p=2)
    use_amp = device.type == "cuda"
    scaler = torch.amp.GradScaler(device.type, enabled=use_amp)
    history: list[dict] = []
    best_rank = (-1.0, -1.0, -1.0)
    best_state = None
    training_start = time.perf_counter()
    for epoch in range(1, epochs + 1):
        model.train()
        losses: list[float] = []
        epoch_start = time.perf_counter()
        for anchor, positive, negative in loader:
            anchor = anchor.to(device, non_blocking=True)
            positive = positive.to(device, non_blocking=True)
            negative = negative.to(device, non_blocking=True)
            optimizer.zero_grad(set_to_none=True)
            with torch.autocast(device_type=device.type, dtype=torch.float16, enabled=use_amp):
                combined = model(torch.cat([anchor, positive, negative], dim=0))
                anchor_descriptor, positive_descriptor, negative_descriptor = torch.chunk(combined, 3, dim=0)
                loss = loss_function(anchor_descriptor, positive_descriptor, negative_descriptor)
            scaler.scale(loss).backward()
            scaler.step(optimizer)
            scaler.update()
            losses.append(float(loss.detach().cpu()))
        validation_descriptors, validation_elapsed = extract_model_descriptors(
            model, validation_frames, eval_transform, device, batch_size,
        )
        validation_scores_array, validation_labels = validation_scores(
            validation_descriptors, [frame.ground_truth for frame in validation_frames], protocol,
        )
        selected = select_f1_threshold(validation_scores_array, validation_labels)
        validation_metrics, _ = evaluate_split(
            validation_descriptors, validation_frames, protocol, float(selected["threshold"]),
        )
        row = {
            "variant": name,
            "epoch": epoch,
            "train_loss": float(np.mean(losses)),
            "epoch_runtime_s": time.perf_counter() - epoch_start,
            "validation_extraction_s": validation_elapsed,
            "validation_threshold": selected["threshold"],
            "validation_precision": validation_metrics["precision"],
            "validation_recall": validation_metrics["recall"],
            "validation_f1": validation_metrics["f1"],
            "validation_recall_at_1": validation_metrics["recall_at_1"],
            "validation_recall_at_5": validation_metrics["recall_at_5"],
        }
        history.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
        rank = (
            float(validation_metrics["recall_at_5"]),
            float(validation_metrics["recall_at_1"]),
            float(validation_metrics["f1"]),
        )
        if rank > best_rank:
            best_rank = rank
            best_state = {key: value.detach().cpu().clone() for key, value in model.state_dict().items()}
    if best_state is None:
        raise RuntimeError("Training did not produce a checkpoint")
    model.load_state_dict(best_state)
    checkpoint = output / f"{name}.pth"
    torch.save(
        {
            "variant": name,
            "with_se": with_se,
            "descriptor_dim": descriptor_dim,
            "pretrained_imagenet": pretrained,
            "protocol": protocol.to_dict(),
            "state_dict": best_state,
        },
        checkpoint,
    )
    checkpoint_sha256 = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    return model, eval_transform, history, {
        "training_runtime_s": time.perf_counter() - training_start,
        "checkpoint": str(checkpoint),
        "checkpoint_size_bytes": checkpoint.stat().st_size,
        "checkpoint_sha256": checkpoint_sha256,
        "parameters": int(sum(parameter.numel() for parameter in model.parameters())),
        "trainable_parameters": int(sum(parameter.numel() for parameter in model.parameters() if parameter.requires_grad)),
    }


def plot_comparison(path: Path, result_rows: list[dict]) -> None:
    names = [str(row["method"]) for row in result_rows]
    measures = ["test_recall_at_1", "test_recall_at_5", "test_precision", "test_recall"]
    labels = ["Recall@1", "Recall@5", "Pair precision", "Pair recall"]
    x = np.arange(len(names))
    width = 0.19
    figure, axis = plt.subplots(figsize=(11, 5.5))
    for index, (measure, label) in enumerate(zip(measures, labels)):
        values = [float(row[measure]) for row in result_rows]
        axis.bar(x + (index - 1.5) * width, values, width, label=label)
    axis.set_xticks(x, names)
    axis.set_ylim(0, 1.05)
    axis.set_ylabel("Score")
    axis.set_title("TUM fr1/xyz loop-retrieval test (threshold chosen on fr1/desk2)")
    axis.grid(axis="y", alpha=0.25)
    axis.legend(ncols=2)
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def plot_retrieval_cases(path: Path, frames: list[RgbdFrame], rows: list[dict]) -> None:
    if not rows:
        return
    correct = [row for row in rows if row["top1_correct"]]
    incorrect = [row for row in rows if not row["top1_correct"]]
    chosen = []
    if correct:
        chosen.append(max(correct, key=lambda row: row["top1_score"]))
    if incorrect:
        chosen.append(max(incorrect, key=lambda row: row["top1_score"]))
    elif len(correct) > 1:
        chosen.append(min(correct, key=lambda row: row["top1_score"]))
    figure, axes = plt.subplots(len(chosen), 2, figsize=(10, 4 * len(chosen)), squeeze=False)
    for row_index, row in enumerate(chosen):
        query = int(row["query_index"])
        match = int(row["top1_index"])
        for column, frame_index, title in (
            (0, query, f"query frame {query}"),
            (1, match, f"top-1 frame {match}; similarity={float(row['top1_score']):.3f}"),
        ):
            with Image.open(frames[frame_index].rgb_path) as image:
                axes[row_index, column].imshow(image.convert("RGB"))
            axes[row_index, column].set_title(title)
            axes[row_index, column].axis("off")
        axes[row_index, 0].set_ylabel("correct" if row["top1_correct"] else "false match")
    figure.tight_layout()
    figure.savefig(path, dpi=170)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train-dataset", type=Path, default=Path("data/rgbd_dataset_freiburg1_desk"))
    parser.add_argument("--validation-dataset", type=Path, default=Path("data/rgbd_dataset_freiburg1_desk2"))
    parser.add_argument("--test-dataset", type=Path, default=Path("data/rgbd_dataset_freiburg1_xyz"))
    parser.add_argument("--output", type=Path, default=Path("artifacts/loop_learning"))
    parser.add_argument("--epochs", type=int, default=5)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--descriptor-dim", type=int, default=128)
    parser.add_argument("--triplets-per-anchor", type=int, default=4)
    parser.add_argument("--margin", type=float, default=0.3)
    parser.add_argument("--seed", type=int, default=20260724)
    parser.add_argument("--device", choices=("auto", "cuda", "cpu"), default="auto")
    parser.add_argument("--no-pretrained", action="store_true")
    parser.add_argument("--benchmark-samples", type=int, default=100)
    args = parser.parse_args()
    if args.epochs < 1 or args.batch_size < 1 or args.descriptor_dim < 2:
        parser.error("epochs, batch-size, and descriptor-dim must be positive")

    import torch

    args.output.mkdir(parents=True, exist_ok=True)
    if args.device == "cuda" and not torch.cuda.is_available():
        parser.error("CUDA was requested but is unavailable")
    device = torch.device("cuda" if args.device == "auto" and torch.cuda.is_available() else args.device)
    if device.type == "auto":
        device = torch.device("cpu")
    protocol = LoopProtocol()
    protocol.validate()
    set_seed(args.seed)
    total_start = time.perf_counter()
    frames = {
        "train": load_tum_rgbd_frames(args.train_dataset),
        "validation": load_tum_rgbd_frames(args.validation_dataset),
        "test": load_tum_rgbd_frames(args.test_dataset),
    }
    poses = {name: [frame.ground_truth for frame in sequence] for name, sequence in frames.items()}
    triplets = make_training_triplets(
        poses["train"], protocol, triplets_per_anchor=args.triplets_per_anchor, seed=args.seed,
    )
    if not triplets:
        raise RuntimeError("No valid training triplets were produced")
    protocol_record = {
        "split_policy": "sequence-isolated: train=fr1/desk, validation=fr1/desk2, test=fr1/xyz",
        "ground_truth_scope": "training labels, validation threshold selection, and final evaluation only",
        "candidate_policy": "each evaluation query retrieves only older frames separated by min_frame_separation",
        "threshold_policy": "maximize pair F1 on validation; freeze before test",
        "protocol": protocol.to_dict(),
        "frame_counts": {name: len(sequence) for name, sequence in frames.items()},
        "training_triplets": len(triplets),
        "training_anchors": len({anchor for anchor, _, _ in triplets}),
    }
    (args.output / "protocol.json").write_text(
        json.dumps(protocol_record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8",
    )

    result_rows: list[dict] = []
    retrieval_rows_by_method: dict[str, list[dict]] = {}
    histories: list[dict] = []

    print("Computing HSV histogram baseline...", flush=True)
    hsv_descriptors: dict[str, np.ndarray] = {}
    hsv_test_latencies: list[float] = []
    for split, sequence in frames.items():
        descriptors = []
        for frame in sequence:
            start = time.perf_counter()
            descriptors.append(hsv_histogram_descriptor(frame.rgb_path))
            if split == "test":
                hsv_test_latencies.append((time.perf_counter() - start) * 1000.0)
        hsv_descriptors[split] = np.asarray(descriptors)
    hsv_scores, hsv_labels = validation_scores(hsv_descriptors["validation"], poses["validation"], protocol)
    hsv_threshold = select_f1_threshold(hsv_scores, hsv_labels)
    hsv_validation, _ = evaluate_split(
        hsv_descriptors["validation"], frames["validation"], protocol, float(hsv_threshold["threshold"]),
    )
    hsv_test, hsv_retrieval_rows = evaluate_split(
        hsv_descriptors["test"], frames["test"], protocol, float(hsv_threshold["threshold"]),
    )
    retrieval_rows_by_method["hsv_histogram"] = hsv_retrieval_rows
    result_rows.append(
        {
            "method": "hsv_histogram",
            "descriptor_dim": int(hsv_descriptors["test"].shape[1]),
            "with_se": "N/A",
            "model_parameters": "N/A",
            "checkpoint_size_bytes": "N/A",
            "checkpoint_sha256": "N/A",
            "validation_threshold": hsv_threshold["threshold"],
            "validation_f1": hsv_validation["f1"],
            "validation_recall_at_1": hsv_validation["recall_at_1"],
            "validation_recall_at_5": hsv_validation["recall_at_5"],
            "test_precision": hsv_test["precision"],
            "test_recall": hsv_test["recall"],
            "test_f1": hsv_test["f1"],
            "test_recall_at_1": hsv_test["recall_at_1"],
            "test_recall_at_5": hsv_test["recall_at_5"],
            "test_evaluable_queries": hsv_test["evaluable_queries"],
            "inference_scope": "CPU image read + resize + HSV histogram; one frame",
            "inference_mean_ms": float(np.mean(hsv_test_latencies)),
            "inference_median_ms": float(np.median(hsv_test_latencies)),
            "inference_p95_ms": percentile(hsv_test_latencies, 95),
            "fps_from_mean": 1000.0 / float(np.mean(hsv_test_latencies)),
            "peak_gpu_allocated_bytes": "N/A",
        }
    )

    for name, with_se in (("mobilenetv3_no_se", False), ("mobilenetv3_se", True)):
        print(f"Training {name}...", flush=True)
        model, eval_transform, history, metadata = train_variant(
            name,
            with_se=with_se,
            train_frames=frames["train"],
            validation_frames=frames["validation"],
            triplets=triplets,
            protocol=protocol,
            output=args.output,
            epochs=args.epochs,
            batch_size=args.batch_size,
            learning_rate=args.learning_rate,
            descriptor_dim=args.descriptor_dim,
            margin=args.margin,
            seed=args.seed,
            device=device,
            pretrained=not args.no_pretrained,
        )
        histories.extend(history)
        learned_descriptors = {}
        extraction_times = {}
        for split in ("validation", "test"):
            learned_descriptors[split], extraction_times[split] = extract_model_descriptors(
                model, frames[split], eval_transform, device, args.batch_size,
            )
            np.save(args.output / f"{name}_{split}_descriptors.npy", learned_descriptors[split])
        learned_scores, learned_labels = validation_scores(
            learned_descriptors["validation"], poses["validation"], protocol,
        )
        learned_threshold = select_f1_threshold(learned_scores, learned_labels)
        learned_validation, _ = evaluate_split(
            learned_descriptors["validation"], frames["validation"], protocol, float(learned_threshold["threshold"]),
        )
        learned_test, learned_retrieval_rows = evaluate_split(
            learned_descriptors["test"], frames["test"], protocol, float(learned_threshold["threshold"]),
        )
        retrieval_rows_by_method[name] = learned_retrieval_rows
        benchmark = benchmark_model(model, eval_transform, frames["test"], device, args.benchmark_samples)
        result_rows.append(
            {
                "method": name,
                "descriptor_dim": args.descriptor_dim,
                "with_se": with_se,
                "model_parameters": metadata["parameters"],
                "checkpoint_size_bytes": metadata["checkpoint_size_bytes"],
                "checkpoint_sha256": metadata["checkpoint_sha256"],
                "validation_threshold": learned_threshold["threshold"],
                "validation_f1": learned_validation["f1"],
                "validation_recall_at_1": learned_validation["recall_at_1"],
                "validation_recall_at_5": learned_validation["recall_at_5"],
                "test_precision": learned_test["precision"],
                "test_recall": learned_test["recall"],
                "test_f1": learned_test["f1"],
                "test_recall_at_1": learned_test["recall_at_1"],
                "test_recall_at_5": learned_test["recall_at_5"],
                "test_evaluable_queries": learned_test["evaluable_queries"],
                "inference_scope": benchmark["scope"],
                "inference_mean_ms": benchmark["mean_latency_ms"],
                "inference_median_ms": benchmark["median_latency_ms"],
                "inference_p95_ms": benchmark["p95_latency_ms"],
                "fps_from_mean": benchmark["fps_from_mean"],
                "peak_gpu_allocated_bytes": benchmark.get("peak_gpu_allocated_bytes", "N/A"),
            }
        )

    write_csv(args.output / "metrics.csv", result_rows)
    write_csv(args.output / "training_history.csv", histories)
    for method, rows in retrieval_rows_by_method.items():
        write_csv(args.output / f"retrieval_{method}.csv", rows)
    plot_comparison(args.output / "loop_retrieval_comparison.png", result_rows)
    plot_retrieval_cases(
        args.output / "mobilenetv3_se_retrieval_cases.png", frames["test"], retrieval_rows_by_method["mobilenetv3_se"],
    )
    summary = {
        "experiment": "sequence-isolated lightweight loop descriptor comparison",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "protocol": protocol_record,
        "training": {
            "epochs": args.epochs,
            "batch_size": args.batch_size,
            "learning_rate": args.learning_rate,
            "margin": args.margin,
            "descriptor_dim": args.descriptor_dim,
            "seed": args.seed,
            "pretrained_imagenet": not args.no_pretrained,
        },
        "results": result_rows,
        "performance": {
            "total_runtime_s": time.perf_counter() - total_start,
            "peak_process_rss_bytes": process_rss_bytes(),
        },
        "environment": {
            "python": sys.version,
            "torch": torch.__version__,
            "torchvision": __import__("torchvision").__version__,
            "cuda_available": torch.cuda.is_available(),
            "device": str(device),
            "gpu": torch.cuda.get_device_name(device) if device.type == "cuda" else "not used",
            "platform": platform.platform(),
            "logical_cpu_count": os.cpu_count(),
        },
        "truthfulness_notes": [
            "Train, validation, and test are separate TUM sequences.",
            "The decision threshold is chosen independently per method on validation and frozen for test.",
            "Ground truth supplies supervision/evaluation labels only; descriptor inference receives RGB images only.",
            "Recall@K includes only queries that have at least one GT-positive historical candidate.",
            "GPU latency is synchronized batch-1 model forward and excludes image read/preprocessing, as stated in each row.",
        ],
    }
    (args.output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
