"""Aruco marker detection backend.

Uses OpenCV (`cv2.aruco`) to detect fiducial markers in frames grabbed from a
V4L camera and project each marker pose into the robot base frame.

Calibration is loaded from a JSON file (see ``perception/calibration.example.json``)
and must contain:

    camera_matrix    : 3x3 intrinsics (fx, fy, cx, cy)
    dist_coeffs      : distortion coefficients [k1, k2, p1, p2, k3]
    marker_size_mm   : physical side length of the printed marker (mm)
    dictionary       : one of cv2.aruco.DICT_* constants, e.g. "DICT_4X4_50"
    T_base_cam       : 4x4 homogeneous transform from camera frame to robot base
    markers          : {"<marker_id>": "<object_name>"} mapping

The transform ``T_base_cam`` converts a point expressed in the camera frame
(mm) into the robot base frame (mm). It is computed offline via a hand-eye
calibration procedure — outside the scope of this module.
"""

import json
import logging
import os
from dataclasses import dataclass

from perception.base import DetectedObject, PerceptionBackend

logger = logging.getLogger(__name__)


@dataclass
class _ArucoConfig:
    camera_matrix: list
    dist_coeffs: list
    marker_size_mm: float
    dictionary_name: str
    t_base_cam: list
    marker_map: dict


class ArucoPerception(PerceptionBackend):
    """Detect Aruco markers and return their pose in the robot base frame.

    Args:
        config_path:   Path to the JSON calibration file (see module docstring).
        camera_device: V4L index (int) or GStreamer/ffmpeg source string.
    """

    def __init__(
        self,
        config_path: str,
        camera_device: int | str = 0,
    ) -> None:
        try:
            import cv2  # type: ignore
            import numpy as np  # type: ignore
        except ImportError as exc:  # pragma: no cover - import-time guard
            raise ImportError(
                f"Aruco perception requires opencv-contrib-python and numpy: {exc}"
            ) from exc

        self._cv2 = cv2
        self._np = np

        cfg = self._load_config(config_path)
        self._cfg = cfg
        self._camera_matrix = np.asarray(cfg.camera_matrix, dtype=np.float64)
        self._dist_coeffs = np.asarray(cfg.dist_coeffs, dtype=np.float64)
        self._t_base_cam = np.asarray(cfg.t_base_cam, dtype=np.float64)
        if self._t_base_cam.shape != (4, 4):
            raise ValueError(
                f"T_base_cam must be 4x4, got shape {self._t_base_cam.shape}"
            )
        self._marker_size = float(cfg.marker_size_mm)
        self._marker_map = {int(k): v for k, v in cfg.marker_map.items()}

        dict_id = getattr(cv2.aruco, cfg.dictionary_name, None)
        if dict_id is None:
            raise ValueError(
                f"Unknown aruco dictionary {cfg.dictionary_name!r}. "
                "Use a name like 'DICT_4X4_50' or 'DICT_5X5_100'."
            )
        self._dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self._params = cv2.aruco.DetectorParameters()

        # OpenCV 4.7+ exposes the ArucoDetector class; older builds keep the
        # free-function API. Support both to stay portable.
        self._detector = None
        if hasattr(cv2.aruco, "ArucoDetector"):
            self._detector = cv2.aruco.ArucoDetector(self._dict, self._params)

        self._camera = cv2.VideoCapture(camera_device)
        if not self._camera.isOpened():
            raise RuntimeError(
                f"Aruco perception failed to open camera device: {camera_device!r}"
            )

        logger.info(
            "ArucoPerception ready: dict=%s, marker_size=%smm, %d mapped markers",
            cfg.dictionary_name,
            self._marker_size,
            len(self._marker_map),
        )

    # -- configuration -------------------------------------------------------

    @staticmethod
    def _load_config(path: str) -> _ArucoConfig:
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Aruco calibration file not found: {path}")
        with open(path, encoding="utf-8") as f:
            raw = json.load(f)

        required = (
            "camera_matrix",
            "dist_coeffs",
            "marker_size_mm",
            "dictionary",
            "T_base_cam",
            "markers",
        )
        missing = [k for k in required if k not in raw]
        if missing:
            raise ValueError(
                f"Aruco calibration {path} is missing keys: {missing}"
            )

        return _ArucoConfig(
            camera_matrix=raw["camera_matrix"],
            dist_coeffs=raw["dist_coeffs"],
            marker_size_mm=float(raw["marker_size_mm"]),
            dictionary_name=raw["dictionary"],
            t_base_cam=raw["T_base_cam"],
            marker_map=raw["markers"],
        )

    # -- detection pipeline --------------------------------------------------

    def _grab_frame(self):
        ok, frame = self._camera.read()
        if not ok or frame is None:
            raise RuntimeError("Aruco perception: failed to capture a camera frame")
        return frame

    def _detect_markers(self, gray):
        cv2 = self._cv2
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:  # pragma: no cover - legacy OpenCV path
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._dict, parameters=self._params
            )
        return corners, ids

    def _estimate_poses(self, corners):
        """Return (rvecs, tvecs) for each marker in ``corners``.

        Prefers ``estimatePoseSingleMarkers`` when available; otherwise falls
        back to ``solvePnP`` on a per-marker basis. Returns tvecs in the same
        unit as ``marker_size_mm`` (millimetres).
        """
        cv2 = self._cv2
        np = self._np

        if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self._marker_size,
                self._camera_matrix,
                self._dist_coeffs,
            )
            return rvecs, tvecs

        # Manual fallback using solvePnP (OpenCV >= 4.9 removed the helper).
        half = self._marker_size / 2.0
        obj_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )
        rvecs, tvecs = [], []
        for marker_corners in corners:
            ok, rvec, tvec = cv2.solvePnP(
                obj_points,
                marker_corners.reshape(-1, 2).astype(np.float64),
                self._camera_matrix,
                self._dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                rvecs.append(np.zeros((3, 1)))
                tvecs.append(np.zeros((3, 1)))
                continue
            rvecs.append(rvec)
            tvecs.append(tvec)
        return np.asarray(rvecs), np.asarray(tvecs)

    def _transform_to_base(self, tvec):
        np = self._np
        point_cam = np.array(
            [float(tvec[0]), float(tvec[1]), float(tvec[2]), 1.0],
            dtype=np.float64,
        )
        point_base = self._t_base_cam @ point_cam
        return point_base[:3]

    # -- PerceptionBackend ---------------------------------------------------

    def scan(self) -> list[DetectedObject]:
        cv2 = self._cv2

        frame = self._grab_frame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids = self._detect_markers(gray)
        if ids is None or len(ids) == 0:
            logger.debug("Aruco scan: no markers detected")
            return []

        rvecs, tvecs = self._estimate_poses(corners)

        results: list[DetectedObject] = []
        for i, marker_id in enumerate(ids.flatten()):
            marker_id = int(marker_id)
            name = self._marker_map.get(marker_id, f"marker_{marker_id}")
            tvec = tvecs[i].flatten()
            p_base = self._transform_to_base(tvec)
            results.append(
                DetectedObject(
                    name=name,
                    position={
                        "x": float(p_base[0]),
                        "y": float(p_base[1]),
                        "z": float(p_base[2]),
                    },
                    confidence=1.0,
                    source="aruco",
                    metadata={"marker_id": marker_id},
                )
            )

        logger.info("Aruco scan: detected %d marker(s)", len(results))
        return results

    def capabilities(self) -> dict:
        return {
            "perception_backend": "aruco",
            "perception_markers": {
                str(mid): name for mid, name in self._marker_map.items()
            },
            "marker_size_mm": self._marker_size,
        }

    def close(self) -> None:
        try:
            self._camera.release()
        except Exception:  # pragma: no cover - best-effort cleanup
            pass
