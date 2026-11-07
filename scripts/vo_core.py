"""RGB-D visual odometry core: ORB features + frame-to-keyframe PnP/RANSAC. No ROS in here.

Pipeline per frame
  1. Detect ORB features in the grey image.
  2. Match descriptors against the current keyframe (Hamming, Lowe ratio test).
  3. Keyframe features with valid depth are back-projected to 3D (keyframe camera frame), so each
     match is a 3D-2D correspondence.
  4. solvePnPRansac gives T_cur_kf (keyframe -> current camera); refined with Levenberg-Marquardt
     on the inliers.
  5. World pose: T_w_cur = T_w_kf @ inv(T_cur_kf).
  6. When the inlier count drops below a threshold, the current frame becomes the new keyframe.

Frame-to-keyframe (instead of frame-to-frame) means that while the camera looks at the same part of
the scene, every frame is measured against the same reference, so errors do not accumulate.

Conventions: all poses are 4x4 homogeneous matrices in the camera *optical* frame
(x right, y down, z forward). T_a_b maps points from frame b to frame a.
"""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class VOParams:
    n_features: int = 2000
    ratio: float = 0.8            # Lowe ratio test
    min_depth: float = 0.1        # metres
    max_depth: float = 4.0
    ransac_reproj_px: float = 2.0
    ransac_iters: int = 300
    min_inliers: int = 25         # below this the frame is considered lost
    keyframe_inliers: int = 150   # below this the current frame becomes the new keyframe


@dataclass
class Keyframe:
    keypoints: np.ndarray   # (N, 2) pixel coordinates
    descriptors: np.ndarray
    points3d: np.ndarray    # (N, 3) in the keyframe optical frame, NaN where depth is invalid
    T_w_kf: np.ndarray


def to_T(rvec, tvec):
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = np.asarray(tvec).ravel()
    return T


def backproject(kps, depth_m, K, min_depth, max_depth):
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    u = np.clip(np.round(kps[:, 0]).astype(int), 0, depth_m.shape[1] - 1)
    v = np.clip(np.round(kps[:, 1]).astype(int), 0, depth_m.shape[0] - 1)
    z = depth_m[v, u].astype(np.float64)
    valid = np.isfinite(z) & (z > min_depth) & (z < max_depth)
    pts = np.full((len(kps), 3), np.nan)
    pts[valid, 0] = (kps[valid, 0] - cx) * z[valid] / fx
    pts[valid, 1] = (kps[valid, 1] - cy) * z[valid] / fy
    pts[valid, 2] = z[valid]
    return pts


class RGBDVisualOdometry:
    def __init__(self, params: VOParams = VOParams()):
        self.p = params
        self.orb = cv2.ORB_create(nfeatures=params.n_features)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.kf = None
        self.T_w_c = np.eye(4)
        self.last = {}  # diagnostics of the last frame

    def _detect(self, gray):
        kps, des = self.orb.detectAndCompute(gray, None)
        pts = np.array([k.pt for k in kps], dtype=np.float64).reshape(-1, 2)
        return pts, des

    def _make_keyframe(self, pts, des, depth_m, K):
        p3d = backproject(pts, depth_m, K, self.p.min_depth, self.p.max_depth)
        self.kf = Keyframe(pts, des, p3d, self.T_w_c.copy())

    def process(self, gray, depth_m, K):
        """Feed one frame. Returns (T_w_c, ok). The first frame defines the world frame."""
        pts, des = self._detect(gray)
        if self.kf is None or des is None or self.kf.descriptors is None:
            if des is not None:
                self._make_keyframe(pts, des, depth_m, K)
            self.last = dict(matches=0, inliers=0, new_keyframe=True)
            return self.T_w_c.copy(), self.kf is not None

        # 1-2. match keyframe -> current, ratio test, keep only keyframe points with depth
        knn = self.matcher.knnMatch(self.kf.descriptors, des, k=2)
        good = [m[0] for m in knn if len(m) == 2 and m[0].distance < self.p.ratio * m[1].distance]
        good = [m for m in good if np.isfinite(self.kf.points3d[m.queryIdx, 2])]
        self.last = dict(matches=len(good), inliers=0, new_keyframe=False)
        if len(good) < self.p.min_inliers:
            return self._lost(pts, des, depth_m, K)

        obj = np.array([self.kf.points3d[m.queryIdx] for m in good])
        img = np.array([pts[m.trainIdx] for m in good])

        # 3-4. PnP with RANSAC, then LM refinement on the inliers
        ok, rvec, tvec, inl = cv2.solvePnPRansac(
            obj, img, K, None, iterationsCount=self.p.ransac_iters,
            reprojectionError=self.p.ransac_reproj_px, confidence=0.999,
            flags=cv2.SOLVEPNP_EPNP)
        if not ok or inl is None or len(inl) < self.p.min_inliers:
            return self._lost(pts, des, depth_m, K)
        inl = inl.ravel()
        rvec, tvec = cv2.solvePnPRefineLM(obj[inl], img[inl], K, None, rvec, tvec)
        T_c_kf = to_T(rvec, tvec)

        # 5. compose the world pose
        self.T_w_c = self.kf.T_w_kf @ np.linalg.inv(T_c_kf)
        self.last["inliers"] = len(inl)

        # 6. keyframe management
        if len(inl) < self.p.keyframe_inliers:
            self._make_keyframe(pts, des, depth_m, K)
            self.last["new_keyframe"] = True
        return self.T_w_c.copy(), True

    def _lost(self, pts, des, depth_m, K):
        # Keep the last pose and restart from the current frame: the trajectory gets a jump-free
        # pause instead of a wild pose.
        self._make_keyframe(pts, des, depth_m, K)
        self.last["new_keyframe"] = True
        return self.T_w_c.copy(), False
