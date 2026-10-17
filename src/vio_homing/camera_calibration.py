# -*- coding: utf-8 -*-
"""Validate ROS camera_calibration YAML without ROS or OpenCV."""
import math


def validate_calibration(data, width, height):
    """Reject malformed/nonfinite calibration and resolution mismatch."""
    if not isinstance(data, dict):
        raise ValueError("Calibration must be a mapping")
    if (data.get("image_width"), data.get("image_height")) != (width, height):
        raise ValueError("Calibration resolution must match captured image")
    if data.get("distortion_model") != "plumb_bob":
        raise ValueError("Only plumb_bob calibration is supported here")
    result = {}
    for key, rows, cols in (("camera_matrix", 3, 3),
                            ("rectification_matrix", 3, 3),
                            ("projection_matrix", 3, 4)):
        matrix = data.get(key, {})
        values = matrix.get("data", [])
        if (matrix.get("rows"), matrix.get("cols")) != (rows, cols):
            raise ValueError("Invalid matrix dimensions: " + key)
        result[key] = _finite(values, rows * cols)
    distortion = data.get("distortion_coefficients", {})
    if distortion.get("rows") != 1 or distortion.get("cols") != 5:
        raise ValueError("Expected five plumb_bob distortion coefficients")
    result["distortion_coefficients"] = _finite(
        distortion.get("data", []), 5)
    k = result["camera_matrix"]
    projection = result["projection_matrix"]
    if k[0] <= 0 or k[4] <= 0 or abs(k[8] - 1) > 1e-6:
        raise ValueError("Invalid camera focal lengths or homogeneous scale")
    if projection[0] <= 0 or projection[5] <= 0:
        raise ValueError("Invalid projection focal lengths")
    return result


def _finite(values, count):
    if not isinstance(values, (list, tuple)) or len(values) != count:
        raise ValueError("Invalid calibration coefficient count")
    try:
        values = [float(value) for value in values]
    except (ValueError, TypeError, OverflowError):
        raise ValueError("Calibration coefficients must be numeric")
    if any(math.isnan(value) or math.isinf(value) for value in values):
        raise ValueError("Calibration coefficients must be finite")
    return values
