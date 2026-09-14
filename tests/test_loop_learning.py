import numpy as np
from scipy.spatial.transform import Rotation

from src.loop_learning import (
    LoopProtocol,
    evaluate_descriptors,
    evaluation_pairs,
    make_training_triplets,
    select_f1_threshold,
)


def pose(x: float, angle_deg: float = 0.0) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("z", angle_deg, degrees=True).as_matrix()
    result[0, 3] = x
    return result


def test_triplets_respect_temporal_and_geometric_rules() -> None:
    poses = [pose(0.0), pose(0.6), pose(1.2), pose(0.05), pose(0.65), pose(1.25)]
    protocol = LoopProtocol(min_frame_separation=3, evaluation_step=1)
    triplets = make_training_triplets(poses, protocol, triplets_per_anchor=2, seed=7)
    assert triplets
    for anchor, positive, negative in triplets:
        assert abs(anchor - positive) >= 3
        assert abs(anchor - negative) >= 3
        assert abs(poses[anchor][0, 3] - poses[positive][0, 3]) <= 0.20
        assert abs(poses[anchor][0, 3] - poses[negative][0, 3]) >= 0.50


def test_geometric_hard_negative_sampling_prefers_nearer_valid_negatives() -> None:
    poses = [
        pose(0.0), pose(0.1), pose(0.2),
        pose(0.05), pose(0.55), pose(1.5), pose(3.0),
    ]
    protocol = LoopProtocol(min_frame_separation=3, evaluation_step=1)
    triplets = make_training_triplets(
        poses,
        protocol,
        triplets_per_anchor=20,
        seed=7,
        negative_sampling="geometric_hard",
        hard_negative_fraction=0.5,
    )
    anchor_zero_negatives = [negative for anchor, _, negative in triplets if anchor == 0]
    assert anchor_zero_negatives
    assert set(anchor_zero_negatives) <= {4, 5}


def test_invalid_negative_sampling_is_rejected() -> None:
    with np.testing.assert_raises_regex(ValueError, "negative_sampling"):
        make_training_triplets(
            [pose(0.0), pose(1.0)],
            LoopProtocol(min_frame_separation=1),
            negative_sampling="unknown",
        )


def test_evaluation_uses_only_older_candidates() -> None:
    poses = [pose(float(index)) for index in range(8)]
    protocol = LoopProtocol(min_frame_separation=3, evaluation_step=2)
    queries, candidates, _ = evaluation_pairs(poses, protocol)
    assert np.all(candidates <= queries - 3)
    assert set(queries) == {3, 5, 7}


def test_validation_threshold_and_retrieval_metrics() -> None:
    labels = np.array([True, True, False, False])
    scores = np.array([0.9, 0.8, 0.7, 0.1])
    selected = select_f1_threshold(scores, labels)
    assert selected["threshold"] == 0.8
    assert selected["f1"] == 1.0

    poses = [pose(0.0), pose(1.0), pose(2.0), pose(0.05), pose(1.05), pose(2.05)]
    descriptors = np.array(
        [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 0, 0], [0, 1, 0], [0, 0, 1]],
        dtype=np.float32,
    )
    protocol = LoopProtocol(min_frame_separation=3, evaluation_step=1)
    metrics, rows = evaluate_descriptors(descriptors, poses, protocol, threshold=0.9)
    assert metrics["recall_at_1"] == 1.0
    assert metrics["recall_at_5"] == 1.0
    assert rows and all(row["top1_correct"] for row in rows)
