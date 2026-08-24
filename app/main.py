#!/usr/bin/env python3
"""Drive-over undercarriage capture and calibrated inspection mosaic."""

import json
import signal
import time
from datetime import datetime
from pathlib import Path

import adafruit_mlx90640
import board
import busio
import cv2
import numpy as np
from adafruit_tca9548a import TCA9548A
from gpiozero import DigitalInputDevice, DigitalOutputDevice
from picamera2 import Picamera2

RADAR_PIN = 17
LED_PIN = 27
THERMAL_WIDTH = 32
THERMAL_HEIGHT = 24
CAPTURE_DIR = Path(__file__).resolve().parents[1] / "captures"
LAYOUT_PATH = Path(__file__).resolve().parents[1] / "config" / "inspection_layout.json"
CAPTURE_DIR.mkdir(parents=True, exist_ok=True)


class CaptureRejected(Exception):
    """A vehicle pass could not produce a spatially valid inspection image."""


def load_layout():
    with LAYOUT_PATH.open(encoding="utf-8") as layout_file:
        layout = json.load(layout_file)

    required = {
        "canvas", "motion_camera_index", "motion_roi", "min_motion_response",
        "motion_to_canvas", "max_motion_step_pixels", "max_traversal_speed_mps", "max_pair_skew_seconds",
        "capture_interval_seconds", "inspection_roi", "minimum_visible_coverage", "minimum_thermal_coverage",
        "minimum_capture_duration_seconds", "maximum_capture_duration_seconds", "minimum_visible_sharpness",
        "minimum_visible_mean", "maximum_visible_mean", "thermal_range_c", "pairs",
    }
    missing = required - layout.keys()
    if missing:
        raise ValueError(f"Layout is missing required settings: {', '.join(sorted(missing))}")
    if not layout.get("calibrated"):
        raise ValueError("Inspection layout is not calibrated; set calibrated to true only after calibration")

    canvas = layout["canvas"]
    if canvas["width"] <= 0 or canvas["height"] <= 0 or canvas["pixels_per_meter"] <= 0:
        raise ValueError("Canvas dimensions and pixels_per_meter must be positive")
    if layout["max_traversal_speed_mps"] <= 0:
        raise ValueError("max_traversal_speed_mps must be positive")
    if layout["max_pair_skew_seconds"] <= 0:
        raise ValueError("max_pair_skew_seconds must be positive")
    if layout["capture_interval_seconds"] <= 0:
        raise ValueError("capture_interval_seconds must be positive")
    if layout["minimum_capture_duration_seconds"] <= 0 or layout["minimum_capture_duration_seconds"] > layout["maximum_capture_duration_seconds"]:
        raise ValueError("capture duration bounds are invalid")
    if layout["minimum_visible_sharpness"] <= 0 or layout["minimum_visible_mean"] >= layout["maximum_visible_mean"]:
        raise ValueError("visible quality thresholds are invalid")
    if not 0 < layout["minimum_visible_coverage"] <= 1 or not 0 < layout["minimum_thermal_coverage"] <= 1:
        raise ValueError("minimum coverage values must be in the range (0, 1]")
    roi_x, roi_y, roi_width, roi_height = layout["inspection_roi"]
    if roi_width <= 0 or roi_height <= 0 or roi_x < 0 or roi_y < 0 or roi_x + roi_width > canvas["width"] or roi_y + roi_height > canvas["height"]:
        raise ValueError("inspection_roi must be inside the inspection canvas")
    if len(layout["thermal_range_c"]) != 2 or layout["thermal_range_c"][0] >= layout["thermal_range_c"][1]:
        raise ValueError("thermal_range_c must contain increasing minimum and maximum Celsius values")
    layout["motion_to_canvas"] = parse_motion_transform(layout["motion_to_canvas"])

    names = set()
    cameras = set()
    channels = set()
    for pair in layout["pairs"]:
        for key in (
            "name", "visible_camera_index", "thermal_tca_channel", "visible_camera_matrix",
            "visible_distortion_coefficients", "visible_to_canvas", "thermal_camera_matrix",
            "thermal_distortion_coefficients", "thermal_to_canvas",
        ):
            if key not in pair:
                raise ValueError(f"Pair is missing required setting: {key}")
        if pair["name"] in names or pair["visible_camera_index"] in cameras or pair["thermal_tca_channel"] in channels:
            raise ValueError("Pair names, visible camera indexes, and thermal TCA channels must be unique")
        names.add(pair["name"])
        cameras.add(pair["visible_camera_index"])
        channels.add(pair["thermal_tca_channel"])
        pair["visible_camera_matrix"] = parse_camera_matrix(pair["visible_camera_matrix"], f"{pair['name']} visible_camera_matrix")
        pair["visible_distortion_coefficients"] = parse_distortion(pair["visible_distortion_coefficients"], f"{pair['name']} visible_distortion_coefficients")
        pair["visible_to_canvas"] = parse_homography(pair["visible_to_canvas"], f"{pair['name']} visible_to_canvas")
        pair["thermal_camera_matrix"] = parse_camera_matrix(pair["thermal_camera_matrix"], f"{pair['name']} thermal_camera_matrix")
        pair["thermal_distortion_coefficients"] = parse_distortion(pair["thermal_distortion_coefficients"], f"{pair['name']} thermal_distortion_coefficients")
        pair["thermal_to_canvas"] = parse_homography(pair["thermal_to_canvas"], f"{pair['name']} thermal_to_canvas")

    if layout["motion_camera_index"] not in cameras:
        raise ValueError("motion_camera_index must reference a configured visible camera")
    return layout


def parse_homography(values, label):
    matrix = np.asarray(values, dtype=np.float32)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all() or np.isclose(matrix[2, 2], 0):
        raise ValueError(f"{label} must be a finite 3x3 homography")
    return matrix


def parse_camera_matrix(values, label):
    matrix = np.asarray(values, dtype=np.float32)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError(f"{label} must be a finite 3x3 camera matrix with positive focal lengths")
    return matrix


def parse_distortion(values, label):
    coefficients = np.asarray(values, dtype=np.float32).reshape(-1)
    if coefficients.size < 4 or not np.isfinite(coefficients).all():
        raise ValueError(f"{label} must contain at least four finite distortion coefficients")
    return coefficients


def parse_motion_transform(values):
    matrix = np.asarray(values, dtype=np.float32)
    if matrix.shape != (2, 2) or not np.isfinite(matrix).all():
        raise ValueError("motion_to_canvas must be a finite 2x2 calibration matrix")
    return matrix


def setup_gpio():
    return (
        DigitalInputDevice(RADAR_PIN, pull_up=False),
        DigitalOutputDevice(LED_PIN, active_high=True, initial_value=False),
    )


def setup_visible_cameras(pairs):
    cameras = {}
    for pair in pairs:
        index = pair["visible_camera_index"]
        camera = None
        try:
            camera = Picamera2(index)
            camera.configure(camera.create_still_configuration(main={"size": (2304, 1296), "format": "RGB888"}))
            camera.start()
            cameras[index] = camera
            print(f"Visible camera {index} ({pair['name']}) ready")
        except Exception as error:
            if camera is not None:
                camera.close()
            for started_camera in cameras.values():
                started_camera.close()
            raise RuntimeError(f"Visible camera {index} failed to initialize") from error
    return cameras


def setup_thermal_cameras(pairs):
    tca = TCA9548A(busio.I2C(board.SCL, board.SDA))
    thermals = {}
    for pair in pairs:
        channel = pair["thermal_tca_channel"]
        try:
            thermal = adafruit_mlx90640.MLX90640(tca[channel])
            thermal.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_4_HZ
            thermals[channel] = thermal
            print(f"Thermal sensor TCA channel {channel} ({pair['name']}) ready")
        except Exception as error:
            raise RuntimeError(f"Thermal sensor on TCA channel {channel} failed to initialize") from error
    return thermals


def capture_frame_set(pairs, visible_cameras, thermal_sensors):
    visible_frames = {}
    thermal_frames = {}
    for pair in pairs:
        index = pair["visible_camera_index"]
        channel = pair["thermal_tca_channel"]
        # Capture each physical pair together to minimize vehicle-motion skew.
        visible_frames[index] = {
            "frame": cv2.cvtColor(visible_cameras[index].capture_array("main"), cv2.COLOR_RGB2BGR),
            "captured_at": time.monotonic(),
        }
        frame = np.zeros(THERMAL_WIDTH * THERMAL_HEIGHT, dtype=np.float32)
        thermal_sensors[channel].getFrame(frame)
        frame = frame.reshape(THERMAL_HEIGHT, THERMAL_WIDTH)
        thermal_frames[channel] = {"frame": frame, "captured_at": time.monotonic()}
    return visible_frames, thermal_frames


def estimate_motion(previous_frame, current_frame, layout):
    x, y, width, height = layout["motion_roi"]
    previous_gray = cv2.cvtColor(previous_frame, cv2.COLOR_BGR2GRAY)[y:y + height, x:x + width]
    current_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)[y:y + height, x:x + width]
    if previous_gray.shape != (height, width) or current_gray.shape != (height, width):
        raise ValueError("motion_roi is outside the motion camera frame")
    return cv2.phaseCorrelate(np.float32(previous_gray), np.float32(current_gray))


def undistort(image, camera_matrix, distortion_coefficients):
    return cv2.undistort(image, camera_matrix, distortion_coefficients)


def visible_frame_is_valid(frame, layout):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    brightness = float(np.mean(gray))
    sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    if not layout["minimum_visible_mean"] <= brightness <= layout["maximum_visible_mean"]:
        return False, f"visible brightness outside calibrated range ({brightness:.1f})"
    if sharpness < layout["minimum_visible_sharpness"]:
        return False, f"visible frame is too blurred ({sharpness:.1f})"
    return True, None


def validate_event(frame_sets, layout):
    if len(frame_sets) < 2:
        raise CaptureRejected("fewer than two frame sets were captured")

    max_step = layout["max_motion_step_pixels"]
    max_speed = layout["max_traversal_speed_mps"]
    pixels_per_meter = layout["canvas"]["pixels_per_meter"]
    motion_camera = layout["motion_camera_index"]
    minimum_response = layout["min_motion_response"]
    speeds = []
    for frame_set in frame_sets:
        for pair in layout["pairs"]:
            valid, reason = visible_frame_is_valid(frame_set["visible"][pair["visible_camera_index"]]["frame"], layout)
            if not valid:
                raise CaptureRejected(f"{pair['name']} {reason}")
            visible_at = frame_set["visible"][pair["visible_camera_index"]]["captured_at"]
            thermal_at = frame_set["thermal"][pair["thermal_tca_channel"]]["captured_at"]
            if abs(thermal_at - visible_at) > layout["max_pair_skew_seconds"]:
                raise CaptureRejected(f"{pair['name']} visible/thermal skew exceeds max_pair_skew_seconds")

    for previous, current in zip(frame_sets, frame_sets[1:]):
        dx, dy = current["canvas_motion"]
        response = current["motion_response"]
        elapsed = current["motion_captured_at"] - previous["motion_captured_at"]
        if elapsed <= 0:
            raise CaptureRejected("non-increasing frame timestamps")
        speed = np.hypot(dx, dy) / pixels_per_meter / elapsed
        speeds.append(speed)
        if response < minimum_response:
            raise CaptureRejected(f"motion registration confidence too low ({response:.3f})")
        if np.hypot(dx, dy) > max_step or speed > max_speed:
            raise CaptureRejected(f"vehicle moved too far between samples ({speed:.2f} m/s)")
        if current["visible"][motion_camera]["frame"].shape != previous["visible"][motion_camera]["frame"].shape:
            raise CaptureRejected("motion camera frame shape changed during capture")
    return {
        "frame_sets": len(frame_sets),
        "duration_seconds": frame_sets[-1]["captured_at"] - frame_sets[0]["captured_at"],
        "average_speed_mps": float(np.mean(speeds)),
        "maximum_speed_mps": float(np.max(speeds)),
    }


def project_into_canvas(image, homography, offset, canvas_size, interpolation):
    translate = np.array([[1, 0, offset[0]], [0, 1, offset[1]], [0, 0, 1]], dtype=np.float32)
    transform = translate @ homography
    width, height = canvas_size
    projected = cv2.warpPerspective(image, transform, (width, height), flags=interpolation)
    source_mask = np.full(image.shape[:2], 255, dtype=np.uint8)
    mask = cv2.warpPerspective(source_mask, transform, (width, height), flags=cv2.INTER_NEAREST)
    return projected, mask


def blend_visible(canvas, weights, image, mask):
    valid = mask > 0
    canvas[valid] += image[valid]
    weights[valid] += 1


def blend_thermal(canvas, weights, temperatures, mask):
    valid = mask > 0
    canvas[valid] += temperatures[valid]
    weights[valid] += 1


def source_offset(frame_set, source_time):
    return frame_set["canvas_position"] + frame_set["canvas_velocity"] * (source_time - frame_set["motion_captured_at"])


def save_inspection_image(frame_sets, event_id, layout, event_stats):
    width = layout["canvas"]["width"]
    height = layout["canvas"]["height"]
    canvas_size = (width, height)
    visible_sum = np.zeros((height, width, 3), dtype=np.float32)
    visible_weight = np.zeros((height, width), dtype=np.uint16)
    thermal_sum = np.zeros((height, width), dtype=np.float32)
    thermal_weight = np.zeros((height, width), dtype=np.uint16)
    for frame_set in frame_sets:
        for pair in layout["pairs"]:
            visible_source = frame_set["visible"][pair["visible_camera_index"]]
            visible, visible_mask = project_into_canvas(
                undistort(
                    visible_source["frame"], pair["visible_camera_matrix"], pair["visible_distortion_coefficients"]
                ),
                pair["visible_to_canvas"],
                source_offset(frame_set, visible_source["captured_at"]),
                canvas_size,
                cv2.INTER_LINEAR,
            )
            blend_visible(visible_sum, visible_weight, visible, visible_mask)

            thermal_source = frame_set["thermal"][pair["thermal_tca_channel"]]
            thermal, thermal_mask = project_into_canvas(
                undistort(
                    thermal_source["frame"], pair["thermal_camera_matrix"], pair["thermal_distortion_coefficients"]
                ),
                pair["thermal_to_canvas"],
                source_offset(frame_set, thermal_source["captured_at"]),
                canvas_size,
                cv2.INTER_LINEAR,
            )
            blend_thermal(thermal_sum, thermal_weight, thermal, thermal_mask)

    valid_visible = visible_weight > 0
    visible_mosaic = np.zeros_like(visible_sum, dtype=np.uint8)
    visible_mosaic[valid_visible] = (visible_sum[valid_visible] / visible_weight[valid_visible, None]).astype(np.uint8)

    valid_thermal = thermal_weight > 0
    roi_x, roi_y, roi_width, roi_height = layout["inspection_roi"]
    roi = np.s_[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]
    visible_coverage = float(np.mean(valid_visible[roi]))
    thermal_coverage = float(np.mean(valid_thermal[roi]))
    if visible_coverage < layout["minimum_visible_coverage"]:
        raise CaptureRejected(f"visible coverage is incomplete ({visible_coverage:.1%})")
    if thermal_coverage < layout["minimum_thermal_coverage"]:
        raise CaptureRejected(f"thermal coverage is incomplete ({thermal_coverage:.1%})")
    thermal_celsius = np.full((height, width), np.nan, dtype=np.float32)
    thermal_celsius[valid_thermal] = thermal_sum[valid_thermal] / thermal_weight[valid_thermal]
    min_c, max_c = layout["thermal_range_c"]
    normalized = np.clip((np.nan_to_num(thermal_celsius, nan=min_c) - min_c) / (max_c - min_c), 0, 1)
    heatmap = cv2.applyColorMap(np.uint8(normalized * 255), cv2.COLORMAP_INFERNO)
    inspection = visible_mosaic.copy()
    inspection[valid_thermal] = cv2.addWeighted(
        visible_mosaic[valid_thermal], 0.55, heatmap[valid_thermal], 0.45, 0
    )

    roi_inspection = inspection[roi].copy()
    roi_temperatures = thermal_celsius[roi]
    roi_thermal_coverage = valid_thermal[roi]
    legend = np.zeros((60, 320, 3), dtype=np.uint8)
    gradient = np.tile(np.linspace(0, 255, 280, dtype=np.uint8), (20, 1))
    legend[10:30, 20:300] = cv2.applyColorMap(gradient, cv2.COLORMAP_INFERNO)
    cv2.putText(legend, f"{min_c:.0f} C", (20, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(legend, f"{max_c:.0f} C", (265, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    header = np.zeros((60, roi_inspection.shape[1], 3), dtype=np.uint8)
    header[:, :320] = legend
    technician_image = cv2.vconcat([header, roi_inspection])

    output = CAPTURE_DIR / f"{event_id}_inspection.jpg"
    coverage = CAPTURE_DIR / f"{event_id}_thermal_coverage.png"
    temperatures = CAPTURE_DIR / f"{event_id}_thermal_celsius.npy"
    metadata = CAPTURE_DIR / f"{event_id}_metadata.json"
    if not cv2.imwrite(str(output), technician_image):
        raise OSError(f"Could not write inspection image: {output}")
    if not cv2.imwrite(str(coverage), np.uint8(roi_thermal_coverage * 255)):
        raise OSError(f"Could not write thermal coverage mask: {coverage}")
    np.save(temperatures, roi_temperatures)
    with metadata.open("w", encoding="utf-8") as metadata_file:
        json.dump({
            "event_id": event_id,
            "layout": LAYOUT_PATH.name,
            "thermal_range_c": layout["thermal_range_c"],
            "visible_coverage": visible_coverage,
            "thermal_coverage": thermal_coverage,
            **event_stats,
        }, metadata_file, indent=2)
    print(f"Inspection mosaic saved -> {output}")


def capture_event(radar, led, visible_cameras, thermal_sensors, layout):
    event_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    frame_sets = []
    previous_motion_frame = None
    next_capture = time.monotonic()
    led.on()
    try:
        started_at = time.monotonic()
        deadline = started_at + layout["maximum_capture_duration_seconds"]
        while time.monotonic() < deadline:
            captured_at = time.monotonic()
            visible, thermal = capture_frame_set(layout["pairs"], visible_cameras, thermal_sensors)
            motion_pair = next(
                pair for pair in layout["pairs"] if pair["visible_camera_index"] == layout["motion_camera_index"]
            )
            motion_frame = undistort(
                visible[layout["motion_camera_index"]]["frame"],
                motion_pair["visible_camera_matrix"],
                motion_pair["visible_distortion_coefficients"],
            )
            motion_captured_at = visible[layout["motion_camera_index"]]["captured_at"]
            motion = np.zeros(2, dtype=np.float32)
            response = 1.0
            if previous_motion_frame is not None:
                motion, response = estimate_motion(previous_motion_frame, motion_frame, layout)
            canvas_motion = layout["motion_to_canvas"] @ motion
            canvas_position = frame_sets[-1]["canvas_position"] + canvas_motion if frame_sets else np.zeros(2, dtype=np.float32)
            elapsed = motion_captured_at - frame_sets[-1]["motion_captured_at"] if frame_sets else 0
            canvas_velocity = canvas_motion / elapsed if elapsed > 0 else np.zeros(2, dtype=np.float32)
            frame_sets.append({
                "captured_at": captured_at,
                "visible": visible,
                "thermal": thermal,
                "motion": np.asarray(motion, dtype=np.float32),
                "canvas_motion": np.asarray(canvas_motion, dtype=np.float32),
                "canvas_position": np.asarray(canvas_position, dtype=np.float32),
                "canvas_velocity": np.asarray(canvas_velocity, dtype=np.float32),
                "motion_captured_at": motion_captured_at,
                "motion_response": response,
            })
            previous_motion_frame = motion_frame
            if time.monotonic() - started_at >= layout["minimum_capture_duration_seconds"] and not radar.value:
                break
            next_capture += layout["capture_interval_seconds"]
            delay = next_capture - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_capture = time.monotonic()
    finally:
        led.off()
        print(">>> Capture complete. LEDs OFF.")

    event_stats = validate_event(frame_sets, layout)
    save_inspection_image(frame_sets, event_id, layout, event_stats)


def wait_for_radar_clear(radar):
    while radar.value:
        time.sleep(0.05)


def main():
    radar = None
    led = None
    visible_cameras = {}

    def stop_handler(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    try:
        layout = load_layout()
        print("Starting undercarriage system...")
        radar, led = setup_gpio()
        visible_cameras = setup_visible_cameras(layout["pairs"])
        thermal_sensors = setup_thermal_cameras(layout["pairs"])
        while True:
            wait_for_radar_clear(radar)
            print("\nReady. Waiting for vehicle...\n")
            while not radar.value:
                time.sleep(0.05)
            print(">>> Vehicle detected — LEDs ON, capturing...")
            try:
                capture_event(radar, led, visible_cameras, thermal_sensors, layout)
            except CaptureRejected as error:
                print(f">>> Inspection rejected: {error}")
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        if led is not None:
            led.off()
            led.close()
        for camera in visible_cameras.values():
            camera.close()
        if radar is not None:
            radar.close()
        print("Done.")


if __name__ == "__main__":
    main()
