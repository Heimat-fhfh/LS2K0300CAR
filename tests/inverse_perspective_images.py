#!/usr/bin/env python3
"""Batch undistort and inverse-perspective transform images.

The inverse-perspective matrix follows the lower-machine convention used by
VisionTransformPipeline: for every output pixel (x, y), the matrix maps back to
the source coordinate to sample.
"""

import argparse
import json
import re
import sys
from pathlib import Path
from typing import NamedTuple, Tuple

import numpy as np


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


class TransformConfig(NamedTuple):
    input_size: Tuple[int, int]
    output_size: Tuple[int, int]
    enable: bool
    undistort_enable: bool
    inverse_perspective_enable: bool
    calibration_file: Path
    matrix: np.ndarray


class CalibrationData(NamedTuple):
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    image_size: Tuple[int, int]


class ProcessResult(NamedTuple):
    written: int
    skipped: int
    output_dir: Path
    map_output: Path


def _require_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    return value


def _read_size(node, name):
    if not isinstance(node, dict):
        raise ValueError(f"{name} must be an object with width/height")
    width = node.get("width")
    height = node.get("height")
    if not isinstance(width, int) or not isinstance(height, int) or width <= 0 or height <= 0:
        raise ValueError(f"{name}.width and {name}.height must be positive integers")
    return width, height


def _resolve_config_path(base_path, value):
    path = Path(value)
    if path.is_absolute():
        return path
    return (base_path.parent / path).resolve()


def load_transform_config(config_path):
    config_path = Path(config_path).resolve()
    with config_path.open("r", encoding="utf-8") as f:
        root = json.load(f)

    inverse_node = root.get("inverse_perspective", {})
    matrix_values = inverse_node.get("matrix")
    if not isinstance(matrix_values, list) or len(matrix_values) != 9:
        raise ValueError("inverse_perspective.matrix must contain exactly 9 numbers")
    matrix = np.array(
        [_require_number(value, f"inverse_perspective.matrix[{index}]") for index, value in enumerate(matrix_values)],
        dtype=np.float64,
    ).reshape((3, 3))

    undistort_node = root.get("undistort", {})
    calibration_file = _resolve_config_path(
        config_path,
        undistort_node.get("calibration_file", "../config/calibration.yaml"),
    )

    return TransformConfig(
        input_size=_read_size(root.get("input_size"), "input_size"),
        output_size=_read_size(root.get("output_size"), "output_size"),
        enable=bool(root.get("enable", False)),
        undistort_enable=bool(undistort_node.get("enable", False)),
        inverse_perspective_enable=bool(inverse_node.get("enable", False)),
        calibration_file=calibration_file,
        matrix=matrix,
    )


def _extract_opencv_matrix(text, key):
    block_match = re.search(rf"^{re.escape(key)}:\s*!!opencv-matrix(?P<body>.*?)(?=^\S|\Z)", text, re.M | re.S)
    if not block_match:
        raise ValueError(f"missing OpenCV matrix node: {key}")

    body = block_match.group("body")
    rows_match = re.search(r"rows:\s*(\d+)", body)
    cols_match = re.search(r"cols:\s*(\d+)", body)
    data_match = re.search(r"data:\s*\[(?P<data>.*?)\]", body, re.S)
    if not rows_match or not cols_match or not data_match:
        raise ValueError(f"incomplete OpenCV matrix node: {key}")

    rows = int(rows_match.group(1))
    cols = int(cols_match.group(1))
    values = [
        float(token)
        for token in re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", data_match.group("data"))
    ]
    if len(values) != rows * cols:
        raise ValueError(f"{key}.data has {len(values)} values, expected {rows * cols}")
    return np.array(values, dtype=np.float64).reshape((rows, cols))


def load_calibration_yaml(yaml_path):
    yaml_path = Path(yaml_path)
    text = yaml_path.read_text(encoding="utf-8")
    camera_matrix = _extract_opencv_matrix(text, "camera_matrix")
    dist_coeffs = _extract_opencv_matrix(text, "dist_coeffs")

    width_match = re.search(r"^image_width:\s*(\d+)", text, re.M)
    height_match = re.search(r"^image_height:\s*(\d+)", text, re.M)
    image_size = (
        int(width_match.group(1)) if width_match else 0,
        int(height_match.group(1)) if height_match else 0,
    )
    return CalibrationData(camera_matrix=camera_matrix, dist_coeffs=dist_coeffs, image_size=image_size)


def build_perspective_map(matrix, input_size, output_size):
    input_width, input_height = input_size
    output_width, output_height = output_size

    grid_x, grid_y = np.meshgrid(
        np.arange(output_width, dtype=np.float64),
        np.arange(output_height, dtype=np.float64),
    )
    denominator = matrix[2, 0] * grid_x + matrix[2, 1] * grid_y + matrix[2, 2]

    map_x = np.full((output_height, output_width), -1.0, dtype=np.float32)
    map_y = np.full((output_height, output_width), -1.0, dtype=np.float32)
    nonzero = np.abs(denominator) >= 1e-9

    source_x = np.empty_like(grid_x)
    source_y = np.empty_like(grid_y)
    source_x[nonzero] = (matrix[0, 0] * grid_x[nonzero] + matrix[0, 1] * grid_y[nonzero] + matrix[0, 2]) / denominator[nonzero]
    source_y[nonzero] = (matrix[1, 0] * grid_x[nonzero] + matrix[1, 1] * grid_y[nonzero] + matrix[1, 2]) / denominator[nonzero]

    valid = (
        nonzero
        & (source_x >= 0.0)
        & (source_y >= 0.0)
        & (source_x < float(input_width - 1))
        & (source_y < float(input_height - 1))
    )
    map_x[valid] = source_x[valid].astype(np.float32)
    map_y[valid] = source_y[valid].astype(np.float32)
    return map_x, map_y


def build_identity_map(input_size, output_size):
    input_width, input_height = input_size
    output_width, output_height = output_size
    map_x = np.full((output_height, output_width), -1.0, dtype=np.float32)
    map_y = np.full((output_height, output_width), -1.0, dtype=np.float32)

    copy_width = min(input_width, output_width)
    copy_height = min(input_height, output_height)
    grid_x, grid_y = np.meshgrid(
        np.arange(copy_width, dtype=np.float32),
        np.arange(copy_height, dtype=np.float32),
    )
    map_x[:copy_height, :copy_width] = grid_x
    map_y[:copy_height, :copy_width] = grid_y
    return map_x, map_y


def build_fused_map(cv2, config, calibration, apply_undistort, apply_inverse_perspective):
    if apply_inverse_perspective:
        perspective_x, perspective_y = build_perspective_map(config.matrix, config.input_size, config.output_size)
    else:
        perspective_x, perspective_y = build_identity_map(config.input_size, config.output_size)

    if not apply_undistort:
        return perspective_x, perspective_y

    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
        calibration.camera_matrix,
        calibration.dist_coeffs,
        config.input_size,
        1.0,
        config.input_size,
    )
    undistort_x, undistort_y = cv2.initUndistortRectifyMap(
        calibration.camera_matrix,
        calibration.dist_coeffs,
        None,
        new_camera_matrix,
        config.input_size,
        cv2.CV_32FC1,
    )

    invalid = (perspective_x < 0.0) | (perspective_y < 0.0)
    fused_x = cv2.remap(
        undistort_x,
        perspective_x,
        perspective_y,
        cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=-1.0,
    )
    fused_y = cv2.remap(
        undistort_y,
        perspective_x,
        perspective_y,
        cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=-1.0,
    )
    fused_x[invalid] = -1.0
    fused_y[invalid] = -1.0
    return fused_x, fused_y


def save_coordinate_map(
    output_path,
    map_x,
    map_y,
    config_path,
    calibration_path,
    input_size,
    output_size,
    apply_undistort,
    apply_inverse_perspective,
):
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    map_x = np.asarray(map_x, dtype=np.float32)
    map_y = np.asarray(map_y, dtype=np.float32)
    if map_x.shape != map_y.shape:
        raise ValueError("map_x and map_y must have the same shape")

    valid_mask = ((map_x >= 0.0) & (map_y >= 0.0)).astype(np.uint8)

    with output_path.open("w", encoding="utf-8") as f:
        f.write("// Generated by test/inverse_perspective_images.py. Do not edit by hand.\n")
        f.write("// Coordinate convention: output pixel (row, col) samples source image at (kMapX[row][col], kMapY[row][col]).\n")
        f.write(f"// Config: {config_path}\n")
        f.write(f"// Calibration: {calibration_path if calibration_path else ''}\n")
        f.write(f"// Undistort: {str(bool(apply_undistort)).lower()}\n")
        f.write(f"// Inverse perspective: {str(bool(apply_inverse_perspective)).lower()}\n\n")
        f.write("#include <cstdint>\n\n")
        f.write("namespace vision_transform_coordinate_map {\n\n")
        f.write(f"constexpr int kInputWidth = {int(input_size[0])};\n")
        f.write(f"constexpr int kInputHeight = {int(input_size[1])};\n")
        f.write(f"constexpr int kOutputWidth = {int(output_size[0])};\n")
        f.write(f"constexpr int kOutputHeight = {int(output_size[1])};\n")
        f.write("constexpr float kInvalidCoordinate = -1.0f;\n\n")
        _write_cpp_float_array(f, "kMapX", map_x)
        f.write("\n")
        _write_cpp_float_array(f, "kMapY", map_y)
        f.write("\n")
        _write_cpp_uint8_array(f, "kValidMask", valid_mask)
        f.write("\n} // namespace vision_transform_coordinate_map\n")
    return output_path


def _write_cpp_float_array(f, name, values):
    f.write(f"extern const float {name}[kOutputHeight][kOutputWidth] = {{\n")
    for row in values:
        formatted = ", ".join(f"{float(value):.6f}f" for value in row)
        f.write(f"    {{{formatted}}},\n")
    f.write("};\n")


def _write_cpp_uint8_array(f, name, values):
    f.write(f"extern const uint8_t {name}[kOutputHeight][kOutputWidth] = {{\n")
    for row in values:
        formatted = ", ".join(f"{int(value)}u" for value in row)
        f.write(f"    {{{formatted}}},\n")
    f.write("};\n")


def iter_images(input_dir):
    for path in sorted(Path(input_dir).rglob("*")):
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES:
            yield path


def imwrite_params(cv2, output_path):
    suffix = output_path.suffix.lower()
    if suffix in {".jpg", ".jpeg"}:
        return [cv2.IMWRITE_JPEG_QUALITY, 95]
    if suffix == ".png":
        return [cv2.IMWRITE_PNG_COMPRESSION, 3]
    return []


def process_images(args):
    try:
        import cv2
    except ModuleNotFoundError as exc:
        raise RuntimeError("Python OpenCV is missing. Install it with: python3 -m pip install opencv-python") from exc

    config = load_transform_config(args.config)
    apply_undistort = True
    apply_inverse_perspective = True
    if args.respect_config_enable:
        apply_undistort = config.enable and config.undistort_enable
        apply_inverse_perspective = config.enable and config.inverse_perspective_enable

    calibration = load_calibration_yaml(config.calibration_file) if apply_undistort else None
    map_x, map_y = build_fused_map(cv2, config, calibration, apply_undistort, apply_inverse_perspective)
    map_output = save_coordinate_map(
        output_path=args.map_output,
        map_x=map_x,
        map_y=map_y,
        config_path=Path(args.config),
        calibration_path=config.calibration_file if apply_undistort else None,
        input_size=config.input_size,
        output_size=config.output_size,
        apply_undistort=apply_undistort,
        apply_inverse_perspective=apply_inverse_perspective,
    )

    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    if args.map_only:
        return ProcessResult(written=0, skipped=0, output_dir=output_dir, map_output=map_output)

    image_paths = list(iter_images(input_dir))
    if args.limit is not None:
        image_paths = image_paths[: args.limit]
    if not image_paths:
        raise RuntimeError(f"no images found under {input_dir}")

    written = 0
    skipped = 0
    for image_path in image_paths:
        src = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
        if src is None:
            print(f"skip unreadable image: {image_path}", file=sys.stderr)
            skipped += 1
            continue

        height, width = src.shape[:2]
        if (width, height) != config.input_size:
            message = f"{image_path}: image size {width}x{height}, expected {config.input_size[0]}x{config.input_size[1]}"
            if not args.resize_input:
                raise RuntimeError(message)
            src = cv2.resize(src, config.input_size, interpolation=cv2.INTER_AREA)

        dst = cv2.remap(
            src,
            map_x,
            map_y,
            args.interpolation,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        output_path = output_dir / image_path.relative_to(input_dir)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(output_path), dst, imwrite_params(cv2, output_path)):
            raise RuntimeError(f"failed to write {output_path}")
        written += 1

    return ProcessResult(written=written, skipped=skipped, output_dir=output_dir, map_output=map_output)


def parse_args(argv=None):
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Batch undistort and inverse-perspective transform images.")
    parser.add_argument("--config", type=Path, default=repo_root / "config" / "vision_transform.json")
    parser.add_argument("--input-dir", type=Path, default=repo_root / "document" / "img" / "20260602_113553")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=repo_root / "document" / "img" / "20260602_113553_undistort_ipm",
    )
    parser.add_argument(
        "--map-output",
        type=Path,
        default=repo_root / "src" / "vision_transform_coordinate_map.cpp",
        help="Path to save the fused coordinate map C++ arrays.",
    )
    parser.add_argument("--map-only", action="store_true", help="Only save the coordinate map; do not process images.")
    parser.add_argument(
        "--respect-config-enable",
        action="store_true",
        help="Respect enable flags in vision_transform.json. By default this script forces undistort + IPM.",
    )
    parser.add_argument("--resize-input", action="store_true", help="Resize images to input_size instead of failing.")
    parser.add_argument("--limit", type=int, help="Only process the first N images.")
    interpolation = parser.add_mutually_exclusive_group()
    interpolation.add_argument("--nearest", action="store_const", const="nearest", dest="interpolation_name")
    interpolation.add_argument("--linear", action="store_const", const="linear", dest="interpolation_name")
    parser.set_defaults(interpolation_name="linear")
    args = parser.parse_args(argv)

    if args.limit is not None and args.limit <= 0:
        parser.error("--limit must be positive")

    try:
        import cv2

        args.interpolation = cv2.INTER_NEAREST if args.interpolation_name == "nearest" else cv2.INTER_LINEAR
    except ModuleNotFoundError:
        args.interpolation = None
    return args


def main(argv=None):
    args = parse_args(argv)
    result = process_images(args)
    if args.map_only:
        print(f"saved coordinate map: {result.map_output}")
    else:
        print(
            f"processed {result.written} images, skipped {result.skipped}, "
            f"output: {result.output_dir}, map: {result.map_output}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
