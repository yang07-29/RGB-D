from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from src.tum import RgbdPair


@pytest.mark.filterwarnings("ignore::DeprecationWarning")
def test_cached_cloud_helper_records_point_count_after_shared_constructor(monkeypatch):
    o3d = pytest.importorskip("open3d")
    from src import open3d_odometry

    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.zeros((3, 3))))
    monkeypatch.setattr(
        open3d_odometry,
        "make_open3d_cloud",
        lambda _frame, _args: (cloud, 0.01, 0.02, 0.03),
    )
    frame = RgbdPair(1.0, Path("rgb.png"), Path("depth.png"), 0.0)
    clouds, rows = open3d_odometry.build_clouds_with_normals([frame], SimpleNamespace(), [])
    assert len(clouds) == 1
    assert rows[0]["point_count"] == 3
    assert rows[0]["preprocessing_runtime_s"] == pytest.approx(0.05)
