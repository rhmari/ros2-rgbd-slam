"""Unit test for the VO core on a synthetic RGB-D scene with known camera motion.

The scene is a textured plane 2 m in front of the camera. Each test frame is rendered by
ray-casting every pixel onto the plane, so the RGB and depth images are geometrically exact.
Run:  python3 -m pytest tests/  (or simply python3 tests/test_vo_core.py)
"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from vo_core import RGBDVisualOdometry, VOParams  # noqa: E402

W, H = 640, 480
K = np.array([[525.0, 0, 319.5], [0, 525.0, 239.5], [0, 0, 1]])
PLANE_Z = 2.0

rng = np.random.default_rng(0)
TEXTURE = cv2.GaussianBlur(rng.integers(0, 255, (1200, 1600), dtype=np.uint8), (0, 0), 1.2)
TEX_SCALE = 400.0  # texture pixels per metre on the plane


def rot(rx, ry, rz):
    return cv2.Rodrigues(np.array([rx, ry, rz], dtype=np.float64))[0]


def render(T_w_c):
    """Grey image and depth of the plane z=PLANE_Z (world frame) seen from camera pose T_w_c."""
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    rays_c = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u, float)], -1)
    R, t = T_w_c[:3, :3], T_w_c[:3, 3]
    rays_w = rays_c @ R.T
    s = (PLANE_Z - t[2]) / rays_w[..., 2]          # ray parameter to hit the plane
    hit = t + rays_w * s[..., None]
    depth = s * 1.0                                # rays_c has z=1, so s is the optical-z depth
    map_x = (hit[..., 0] * TEX_SCALE + TEXTURE.shape[1] / 2).astype(np.float32)
    map_y = (hit[..., 1] * TEX_SCALE + TEXTURE.shape[0] / 2).astype(np.float32)
    gray = cv2.remap(TEXTURE, map_x, map_y, cv2.INTER_LINEAR)
    return gray, depth.astype(np.float32)


def test_tracks_known_motion():
    vo = RGBDVisualOdometry(VOParams())
    errors, path, prev = [], 0.0, np.zeros(3)
    for k in range(25):
        T = np.eye(4)
        T[:3, :3] = rot(0.004 * k, -0.006 * k, 0.003 * k)
        T[:3, 3] = [0.01 * k, -0.006 * k, 0.012 * k]
        gray, depth = render(T)
        T_est, ok = vo.process(gray, depth, K)
        assert ok, f"lost at frame {k}"
        errors.append(np.linalg.norm(T_est[:3, 3] - T[:3, 3]))
        path += np.linalg.norm(T[:3, 3] - prev)
        prev = T[:3, 3].copy()
        ang = np.degrees(np.arccos(np.clip((np.trace(T_est[:3, :3].T @ T[:3, :3]) - 1) / 2, -1, 1)))
        assert ang < 0.5, f"rotation error {ang:.2f} deg at frame {k}"
    drift = max(errors) / path
    print(f"max translation error {max(errors) * 1000:.1f} mm over a {path * 100:.0f} cm path "
          f"= {drift * 100:.1f} % drift")
    # Frame-to-keyframe VO typically drifts 1-4 % of the distance travelled. The exact value
    # depends on the OpenCV version (ORB/RANSAC internals): ~1.4 % with OpenCV 5, ~2.7 % with
    # the 4.5 shipped in Ubuntu 22.04 / ROS Humble.
    assert drift < 0.04, f"drift {drift * 100:.1f} % is too high"


if __name__ == "__main__":
    test_tracks_known_motion()
    print("OK")
