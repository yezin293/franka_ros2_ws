#!/usr/bin/env python3
"""Solve and validate FR3 eye-in-hand calibration from captured samples.

The input pairs are:
  - base_T_gripper: fr3_link0 <- fr3_link8 from TF at the RGB timestamp
  - camera_T_target: camera_color_optical_frame <- checkerboard from solvePnP

OpenCV returns gripper_T_camera.  The output names every matrix explicitly to
avoid the common direction ambiguity in hand-eye calibration.

This script requires an OpenCV build containing calibrateHandEye().  On this
machine, for example:
  /home/nuc/miniconda3/envs/dreamgaussian/bin/python \
    scripts/calibrate_handeye.py
"""
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


METHODS = {
    "TSAI": "CALIB_HAND_EYE_TSAI",
    "PARK": "CALIB_HAND_EYE_PARK",
    "HORAUD": "CALIB_HAND_EYE_HORAUD",
    "ANDREFF": "CALIB_HAND_EYE_ANDREFF",
    "DANIILIDIS": "CALIB_HAND_EYE_DANIILIDIS",
}


def quaternion_to_matrix(q):
    x, y, z, w = map(float, q)
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-15:
        raise ValueError("zero quaternion")
    scale = 2.0 / norm
    return np.array([
        [1 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)],
        [scale * (x * y + z * w), 1 - scale * (x * x + z * z), scale * (y * z - x * w)],
        [scale * (x * z - y * w), scale * (y * z + x * w), 1 - scale * (x * x + y * y)],
    ])


def matrix_to_quaternion(matrix):
    matrix = np.asarray(matrix, dtype=float)
    trace = np.trace(matrix)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = [(matrix[2, 1] - matrix[1, 2]) / s,
             (matrix[0, 2] - matrix[2, 0]) / s,
             (matrix[1, 0] - matrix[0, 1]) / s, 0.25 * s]
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            s = math.sqrt(1 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
            q = [0.25 * s, (matrix[0, 1] + matrix[1, 0]) / s,
                 (matrix[0, 2] + matrix[2, 0]) / s, (matrix[2, 1] - matrix[1, 2]) / s]
        elif index == 1:
            s = math.sqrt(1 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
            q = [(matrix[0, 1] + matrix[1, 0]) / s, 0.25 * s,
                 (matrix[1, 2] + matrix[2, 1]) / s, (matrix[0, 2] - matrix[2, 0]) / s]
        else:
            s = math.sqrt(1 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
            q = [(matrix[0, 2] + matrix[2, 0]) / s,
                 (matrix[1, 2] + matrix[2, 1]) / s, 0.25 * s,
                 (matrix[1, 0] - matrix[0, 1]) / s]
    q = np.asarray(q)
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q *= -1
    return q


def transform(rotation, translation):
    result = np.eye(4)
    result[:3, :3] = np.asarray(rotation).reshape(3, 3)
    result[:3, 3] = np.asarray(translation).reshape(3)
    return result


def rotation_angle_deg(rotation):
    cosine = np.clip((np.trace(rotation) - 1) / 2, -1, 1)
    return math.degrees(math.acos(cosine))


def mean_transform(transforms):
    translation = np.mean([item[:3, 3] for item in transforms], axis=0)
    matrix = sum(item[:3, :3] for item in transforms)
    u, _, vt = np.linalg.svd(matrix)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0:
        u[:, -1] *= -1
        rotation = u @ vt
    return transform(rotation, translation)


def load_samples(folder):
    paths = sorted(folder.glob("sample_*/sample.json"))
    if len(paths) < 3:
        raise RuntimeError(f"need at least 3 complete samples, found {len(paths)}")
    base_to_gripper, camera_to_target, metadata = [], [], []
    expected = None
    for path in paths:
        data = json.loads(path.read_text())
        signature = (data["base_frame"], data["gripper_frame"], data["camera_frame"],
                     tuple(data["image_size"]), tuple(data["board"]["inner_corners"]),
                     data["board"]["square_size_m"])
        if expected is None:
            expected = signature
        elif signature != expected:
            raise RuntimeError(f"incompatible sample metadata: {path}")
        if len(data["corners_px"]) != 70:
            raise RuntimeError(f"expected 70 corners: {path}")
        gripper = data["gripper_to_base"]
        base_to_gripper.append(transform(
            quaternion_to_matrix(gripper["quaternion_xyzw"]), gripper["translation_m"]))
        camera_to_target.append(np.asarray(data["target_to_camera_matrix"], dtype=float))
        metadata.append((path, data))
    return paths, base_to_gripper, camera_to_target, metadata, expected


def solve(base_to_gripper, camera_to_target, indices, method):
    rotation, translation = cv2.calibrateHandEye(
        [base_to_gripper[i][:3, :3] for i in indices],
        [base_to_gripper[i][:3, 3] for i in indices],
        [camera_to_target[i][:3, :3] for i in indices],
        [camera_to_target[i][:3, 3] for i in indices],
        method=method,
    )
    return transform(rotation, translation)


def consistency(base_to_gripper, camera_to_target, gripper_to_camera, indices):
    targets = [base_to_gripper[i] @ gripper_to_camera @ camera_to_target[i] for i in indices]
    center = mean_transform(targets)
    translation_errors = np.array([
        np.linalg.norm(item[:3, 3] - center[:3, 3]) * 1000 for item in targets])
    rotation_errors = np.array([
        rotation_angle_deg(center[:3, :3].T @ item[:3, :3]) for item in targets])
    return center, translation_errors, rotation_errors


def leave_one_out(base_to_gripper, camera_to_target, method):
    translation_errors, rotation_errors = [], []
    count = len(base_to_gripper)
    for held_out in range(count):
        training = [i for i in range(count) if i != held_out]
        result = solve(base_to_gripper, camera_to_target, training, method)
        target, _, _ = consistency(base_to_gripper, camera_to_target, result, training)
        prediction = base_to_gripper[held_out] @ result @ camera_to_target[held_out]
        translation_errors.append(np.linalg.norm(prediction[:3, 3] - target[:3, 3]) * 1000)
        rotation_errors.append(rotation_angle_deg(target[:3, :3].T @ prediction[:3, :3]))
    return np.asarray(translation_errors), np.asarray(rotation_errors)


def stats(values):
    return {"mean": float(np.mean(values)), "rms": float(np.sqrt(np.mean(values ** 2))),
            "max": float(np.max(values))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=Path, default=Path("calibration_data/handeye_d435i"))
    parser.add_argument("--method", choices=METHODS, default="PARK")
    parser.add_argument("--out", type=Path, default=Path("scripts/handeye_d435i.json"))
    parser.add_argument(
        "--camera-link-to-optical", type=float, nargs=7,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        help="RealSense camera_link_T_camera_color_optical TF; adds link8_T_camera_link to output",
    )
    args = parser.parse_args()
    if not hasattr(cv2, "calibrateHandEye"):
        raise SystemExit("this OpenCV build has no calibrateHandEye(); use OpenCV 4.x with calib3d")
    paths, base_to_gripper, camera_to_target, metadata, signature = load_samples(args.samples)
    indices = list(range(len(paths)))
    comparisons = {}
    for name, attribute in METHODS.items():
        method = getattr(cv2, attribute)
        result = solve(base_to_gripper, camera_to_target, indices, method)
        _, trans, rot = consistency(base_to_gripper, camera_to_target, result, indices)
        loo_trans, loo_rot = leave_one_out(base_to_gripper, camera_to_target, method)
        comparisons[name] = {
            "all_sample_translation_error_mm": stats(trans),
            "all_sample_rotation_error_deg": stats(rot),
            "leave_one_out_translation_error_mm": stats(loo_trans),
            "leave_one_out_rotation_error_deg": stats(loo_rot),
        }
    method = getattr(cv2, METHODS[args.method])
    result = solve(base_to_gripper, camera_to_target, indices, method)
    target, trans, rot = consistency(base_to_gripper, camera_to_target, result, indices)
    inverse = np.linalg.inv(result)
    sample_residuals = []
    for i, path in enumerate(paths):
        sample_residuals.append({
            "sample": path.parent.name,
            "board_reprojection_rms_px": metadata[i][1]["board_reprojection_rms_px"],
            "fixed_target_translation_error_mm": float(trans[i]),
            "fixed_target_rotation_error_deg": float(rot[i]),
        })
    output = {
        "schema_version": 1,
        "calibration_type": "eye_in_hand",
        "method": args.method,
        "sample_count": len(paths),
        "frames": {"base": signature[0], "gripper": signature[1], "camera": signature[2]},
        "board": {"inner_corners": list(signature[4]), "square_size_m": signature[5]},
        "image_size": list(signature[3]),
        "link8_T_camera_color_optical": {
            "meaning": "Transforms points from camera_color_optical_frame into fr3_link8",
            "matrix": result.tolist(),
            "translation_m": result[:3, 3].tolist(),
            "quaternion_xyzw": matrix_to_quaternion(result[:3, :3]).tolist(),
        },
        "camera_color_optical_T_link8": {
            "meaning": "Inverse transform; transforms points from fr3_link8 into camera_color_optical_frame",
            "matrix": inverse.tolist(),
            "translation_m": inverse[:3, 3].tolist(),
            "quaternion_xyzw": matrix_to_quaternion(inverse[:3, :3]).tolist(),
        },
        "estimated_fixed_board_pose_in_base": {
            "matrix": target.tolist(),
            "translation_m": target[:3, 3].tolist(),
            "quaternion_xyzw": matrix_to_quaternion(target[:3, :3]).tolist(),
        },
        "validation": comparisons[args.method],
        "method_comparison": comparisons,
        "sample_residuals": sample_residuals,
        "interpretation": (
            "Validation errors measure how consistently the fixed board pose is reconstructed. "
            "They are not a direct guarantee of object localization accuracy. Validate on new poses."
        ),
    }
    if args.camera_link_to_optical:
        x, y, z, qx, qy, qz, qw = args.camera_link_to_optical
        camera_link_to_optical = transform(
            quaternion_to_matrix([qx, qy, qz, qw]), [x, y, z])
        link8_to_camera_link = result @ np.linalg.inv(camera_link_to_optical)
        output["camera_link_T_camera_color_optical"] = {
            "meaning": "Factory transform reported by the active RealSense ROS driver",
            "matrix": camera_link_to_optical.tolist(),
            "translation_m": camera_link_to_optical[:3, 3].tolist(),
            "quaternion_xyzw": matrix_to_quaternion(camera_link_to_optical[:3, :3]).tolist(),
        }
        output["link8_T_camera_link"] = {
            "meaning": "TF-tree attachment transform; pose of camera_link in fr3_link8",
            "matrix": link8_to_camera_link.tolist(),
            "translation_m": link8_to_camera_link[:3, 3].tolist(),
            "quaternion_xyzw": matrix_to_quaternion(link8_to_camera_link[:3, :3]).tolist(),
        }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(output, indent=2) + "\n")
    chosen = comparisons[args.method]
    print(f"Saved {args.method} eye-in-hand result: {args.out}")
    print("link8_T_camera translation [m]:", np.round(result[:3, 3], 6))
    print("link8_T_camera quaternion xyzw:", np.round(matrix_to_quaternion(result[:3, :3]), 8))
    print("All-sample fixed-board translation error [mm]:", chosen["all_sample_translation_error_mm"])
    print("Leave-one-out translation error [mm]:", chosen["leave_one_out_translation_error_mm"])
    print("Leave-one-out rotation error [deg]:", chosen["leave_one_out_rotation_error_deg"])


if __name__ == "__main__":
    main()
