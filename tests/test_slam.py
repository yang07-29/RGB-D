import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from src.slam import propose_descriptor_loop_candidates, propose_loop_candidates, select_keyframe_indices


def pose(x: float, angle_deg: float = 0.0) -> np.ndarray:
    value = np.eye(4)
    value[0, 3] = x
    value[:3, :3] = Rotation.from_euler("z", angle_deg, degrees=True).as_matrix()
    return value


class SlamUtilityTests(unittest.TestCase):
    def test_keyframes_use_motion_and_maximum_interval(self) -> None:
        poses = [pose(0.00), pose(0.01), pose(0.02), pose(0.06), pose(0.07), pose(0.08), pose(0.09)]
        indices = select_keyframe_indices(
            poses, min_translation_m=0.05, min_rotation_deg=10.0, max_interval_frames=3,
        )
        self.assertEqual(indices, [0, 3, 6])

    def test_keyframes_include_last_pose(self) -> None:
        indices = select_keyframe_indices(
            [pose(0.0), pose(0.01), pose(0.02)],
            min_translation_m=1.0, min_rotation_deg=90.0, max_interval_frames=10,
        )
        self.assertEqual(indices, [0, 2])

    def test_loop_candidates_are_nonlocal_and_do_not_use_reference(self) -> None:
        poses = [pose(0.00), pose(0.10), pose(0.20), pose(0.30), pose(0.20), pose(0.10), pose(0.01)]
        candidates = propose_loop_candidates(
            poses,
            min_keyframe_separation=3,
            max_estimated_distance_m=0.03,
            max_estimated_rotation_deg=5.0,
            max_candidates=4,
            nonmax_radius_keyframes=0,
        )
        pairs = {(item.source_keyframe_id, item.target_keyframe_id) for item in candidates}
        self.assertIn((0, 6), pairs)
        self.assertTrue(all(target - source >= 3 for source, target in pairs))

    def test_descriptor_candidates_do_not_require_poses(self) -> None:
        descriptors = np.array(
            [[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.99, 0.01], [0.01, 0.99]],
            dtype=np.float32,
        )
        candidates = propose_descriptor_loop_candidates(
            descriptors,
            min_keyframe_separation=3,
            min_similarity=0.95,
            max_candidates=5,
            top_k_per_target=1,
            nonmax_radius_keyframes=0,
        )
        pairs = {(item.source_keyframe_id, item.target_keyframe_id) for item in candidates}
        self.assertEqual(pairs, {(0, 3), (1, 4)})


if __name__ == "__main__":
    unittest.main()
