import numpy as np
import pytest
from gemini336l_yolo_seg.geometry import (
    depth_to_meters,
    format_position_label,
    project_mask,
    project_pixel,
)


def test_depth_mm_to_meters():
    depth = np.array([[0, 1000, 2500]], dtype=np.uint16)
    np.testing.assert_allclose(
        depth_to_meters(depth, "16UC1"), np.array([[0.0, 1.0, 2.5]])
    )


def test_project_mask_at_principal_point():
    depth = np.full((5, 5), 2.0, dtype=np.float32)
    mask = np.zeros((5, 5), dtype=np.uint8)
    mask[1:4, 1:4] = 1
    result = project_mask(mask, depth, 100.0, 100.0, 2.0, 2.0, 0.1, 8.0, 3)
    assert result is not None
    assert (result.x, result.y, result.z) == pytest.approx((0.0, 0.0, 2.0))
    assert result.valid_pixels == 9


def test_project_mask_rejects_too_few_valid_pixels():
    depth = np.zeros((3, 3), dtype=np.float32)
    depth[1, 1] = 1.0
    mask = np.ones((3, 3), dtype=np.uint8)
    assert project_mask(mask, depth, 100.0, 100.0, 1.0, 1.0, 0.1, 8.0, 2) is None


def test_position_label_contains_xyz_and_frame():
    assert (
        format_position_label(1.234, -0.345, 0.456, "base_link", "base_link")
        == "base x 1.23  y -0.34  z 0.46 m"
    )
    assert (
        format_position_label(
            1.234,
            -0.345,
            0.456,
            "camera_color_optical_frame",
            "base_link",
        )
        == "cam x 1.23  y -0.34  z 0.46 m"
    )


def test_project_pixel_uses_local_median_and_requested_ray():
    depth = np.zeros((5, 5), dtype=np.float32)
    depth[1:4, 1:4] = 2.0
    depth[2, 2] = 20.0  # invalid outlier
    result = project_pixel(depth, 2, 2, 1, 100.0, 100.0, 1.0, 1.0, 0.1, 8.0)
    assert result is not None
    assert (result.x, result.y, result.z) == pytest.approx((0.02, 0.02, 2.0))
    assert result.valid_pixels == 8


def test_project_pixel_rejects_bounds_and_empty_depth():
    depth = np.zeros((3, 4), dtype=np.float32)
    assert project_pixel(depth, 1, 1, 2, 100.0, 100.0, 1.0, 1.0, 0.1, 8.0) is None
    with pytest.raises(ValueError, match="outside"):
        project_pixel(depth, 4, 1, 0, 100.0, 100.0, 1.0, 1.0, 0.1, 8.0)
