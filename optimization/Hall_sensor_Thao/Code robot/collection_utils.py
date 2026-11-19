"""Shared, fail-closed UART acquisition helpers for robot data collection."""

import argparse
import math
import re
import socket
import struct
import time
from pathlib import Path

import numpy as np


NUM_SENSORS = 25
NUM_AXES = 3
TOTAL_VALUES = NUM_SENSORS * NUM_AXES
FRAME_HEADER = b"\xaa\xbb"
FRAME_TAIL = b"\xcc\xdd"
FRAME_UNIT = "mT"
FRAME_STRUCT = struct.Struct(f"<{TOTAL_VALUES}f")
PAYLOAD_SIZE = FRAME_STRUCT.size


def magnetic_csv_headers():
    """Return CSV field names matching the firmware's sensor-major mT payload."""
    return [
        f"sensor_{sensor_id}_{axis}_{FRAME_UNIT}"
        for sensor_id in range(1, NUM_SENSORS + 1)
        for axis in ("Bx", "By", "Bz")
    ]


def open_sensor_uart(port, baud=115200, timeout=2.0):
    import serial

    ser = serial.Serial(port, baud, timeout=timeout)
    time.sleep(1.0)
    ser.reset_input_buffer()
    print(f"UART connected: {port} @ {baud}")
    return ser


def _read_exact(ser, size, deadline):
    data = bytearray()
    while len(data) < size and time.monotonic() < deadline:
        chunk = ser.read(size - len(data))
        if chunk:
            data.extend(chunk)
    return bytes(data)


def read_sensor_frame(ser, timeout=1.0):
    """Read one firmware frame as 75 sensor-major float32 values in mT.

    The STM32 payload order is S1(Bx, By, Bz), ..., S25(Bx, By, Bz). STM32F1
    stores the transmitted C ``float`` values little-endian, hence the explicit
    ``<`` format in ``FRAME_STRUCT``.
    """
    deadline = time.monotonic() + timeout
    header_state = 0

    while time.monotonic() < deadline:
        byte = ser.read(1)
        if not byte:
            continue

        if header_state == 0:
            header_state = 1 if byte == FRAME_HEADER[:1] else 0
            continue

        if byte == FRAME_HEADER[1:]:
            payload = _read_exact(ser, PAYLOAD_SIZE, deadline)
            tail = _read_exact(ser, 2, deadline)
            header_state = 0
            if len(payload) != PAYLOAD_SIZE or tail != FRAME_TAIL:
                continue
            try:
                values = FRAME_STRUCT.unpack(payload)
            except struct.error:
                continue
            return np.asarray(values, dtype=np.float32)

        header_state = 1 if byte == FRAME_HEADER[:1] else 0

    return None


def acquire_sensor_average(ser, num_samples=5, timeout=2.0):
    """
    Acquire exactly ``num_samples`` fully finite frames and return their mean.

    The function is deliberately fail-closed: a frame containing any NaN/Inf is
    discarded, and the whole robot sample is rejected unless enough valid frames
    arrive before the deadline. This prevents ``nanmean`` from hiding intermittent
    sensor failures.
    """
    if num_samples <= 0:
        raise ValueError("num_samples must be positive")

    ser.reset_input_buffer()
    samples = []
    invalid_frames = 0
    deadline = time.monotonic() + timeout
    previous_serial_timeout = getattr(ser, "timeout", None)
    can_adjust_timeout = hasattr(ser, "timeout")
    if can_adjust_timeout:
        ser.timeout = 0.05

    try:
        while len(samples) < num_samples and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            frame = read_sensor_frame(ser, timeout=min(0.5, max(remaining, 0.0)))
            if frame is None:
                continue
            if frame.shape != (TOTAL_VALUES,) or not np.isfinite(frame).all():
                invalid_frames += 1
                continue
            samples.append(frame)
    finally:
        if can_adjust_timeout:
            ser.timeout = previous_serial_timeout

    if len(samples) != num_samples:
        print(
            "[UART ERROR] Reject sample: "
            f"valid={len(samples)}/{num_samples}, invalid={invalid_frames}"
        )
        return None

    data = np.stack(samples, axis=0)
    frozen_channels = np.ptp(data, axis=0) == 0.0
    frozen_sensors = [
        sensor_id + 1
        for sensor_id in range(NUM_SENSORS)
        if frozen_channels[sensor_id * 3 : sensor_id * 3 + 3].all()
    ]
    if frozen_sensors:
        print(
            "[UART WARNING] All XYZ values repeated across this short window for "
            f"sensor(s): {frozen_sensors}. Verify with a moving-magnet test."
        )

    averaged = data.mean(axis=0, dtype=np.float64).astype(np.float32)
    if not np.isfinite(averaged).all():
        print("[UART ERROR] Reject sample: averaged data is not finite")
        return None
    return averaged


def validate_quaternion(quat):
    if len(quat) != 4 or not all(math.isfinite(v) for v in quat):
        raise ValueError(f"Invalid quaternion: {quat}")
    norm = math.sqrt(sum(v * v for v in quat))
    if norm < 1e-9:
        raise ValueError("Quaternion norm is zero")
    return tuple(v / norm for v in quat)


def clear_socket_input(client):
    """Discard already-buffered robot replies without changing socket timeout."""
    previous_timeout = client.gettimeout()
    client.setblocking(False)
    try:
        while client.recv(1024):
            pass
    except BlockingIOError:
        pass
    finally:
        client.settimeout(previous_timeout)


def wait_for_robot_ack(client, ack_regex, timeout=20.0):
    """Wait for one explicit in-position token; fail on timeout/disconnect."""
    pattern = re.compile(ack_regex, re.IGNORECASE)
    deadline = time.monotonic() + timeout
    buffer = ""

    while time.monotonic() < deadline:
        client.settimeout(max(0.01, deadline - time.monotonic()))
        try:
            chunk = client.recv(1024)
        except socket.timeout:
            return False
        except (ConnectionError, OSError) as exc:
            print(f"[ROBOT ERROR] ACK receive failed: {exc}")
            return False

        if not chunk:
            print("[ROBOT ERROR] Connection closed while waiting for ACK")
            return False
        buffer += chunk.decode(errors="ignore")
        if pattern.search(buffer):
            return True

    return False


def generate_safe_random_orientations(path, reference_quat,
                                      orientation_factory, tcp_factory,
                                      max_step_mm, max_attempts=200):
    """Generate random orientations while bounding consecutive commanded TCPs."""
    if not path:
        return []

    previous_quat = validate_quaternion(reference_quat)
    previous_tcp = np.asarray(tcp_factory(path[0], previous_quat), dtype=float)
    orientations = []

    for point in path:
        accepted = None
        accepted_tcp = None
        for _ in range(max_attempts):
            candidate = validate_quaternion(orientation_factory(reference_quat))
            candidate_tcp = np.asarray(tcp_factory(point, candidate), dtype=float)
            if np.linalg.norm(candidate_tcp - previous_tcp) <= max_step_mm:
                accepted = candidate
                accepted_tcp = candidate_tcp
                break

        if accepted is None:
            # Keeping the previous orientation makes TCP displacement equal the
            # already-validated tooltip displacement.
            accepted = previous_quat
            accepted_tcp = np.asarray(tcp_factory(point, accepted), dtype=float)
            if np.linalg.norm(accepted_tcp - previous_tcp) > max_step_mm + 1e-9:
                raise ValueError(
                    "Cannot satisfy TCP step limit; reduce tooltip spacing or "
                    "increase --max-step-mm after a robot-safety review"
                )

        orientations.append(accepted)
        previous_quat = accepted
        previous_tcp = accepted_tcp

    return orientations


def parse_collection_args(description, *, port, baud, robot_ip, robot_port,
                          coord_file, magnetic_file, samples):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--port", default=port, help="STM32 serial port")
    parser.add_argument("--baud", type=int, default=baud)
    parser.add_argument("--robot-ip", default=robot_ip)
    parser.add_argument("--robot-port", type=int, default=robot_port)
    parser.add_argument("--coord-out", default=str(coord_file))
    parser.add_argument("--magnetic-out", default=str(magnetic_file))
    parser.add_argument("--samples", type=int, default=samples)
    parser.add_argument(
        "--ack-regex",
        default=r"\[(?:DONE|REACHED|IN_POSITION|1)\]",
        help="Regex that uniquely identifies the robot in-position response",
    )
    parser.add_argument("--seed", type=int, default=None,
                        help="Seed for reproducible random paths/orientations")
    parser.add_argument("--dry-run", action="store_true",
                        help="Validate and summarize the path without connecting")
    parser.add_argument("--max-step-mm", type=float, default=60.0,
                        help="Abort when tooltip or commanded TCP steps exceed this distance")
    parser.add_argument("--overwrite", action="store_true",
                        help="Allow replacing existing output CSV files")
    return parser.parse_args()


def validate_and_summarize_path(path, name, max_step_mm=60.0):
    if not path:
        raise ValueError(f"{name} path is empty")
    points = np.asarray(path, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError(f"{name} path contains invalid coordinates")
    steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    summary = {
        "points": len(points),
        "minimum_mm": points.min(axis=0),
        "maximum_mm": points.max(axis=0),
        "max_step_mm": float(steps.max()) if len(steps) else 0.0,
    }
    print(f"{name} path validation: {summary}")
    if summary["max_step_mm"] > max_step_mm:
        raise ValueError(
            f"{name} path max step {summary['max_step_mm']:.3f} mm exceeds "
            f"the {max_step_mm:.3f} mm safety limit"
        )
    return summary


def prepare_output_files(coord_file, magnetic_file, overwrite=False):
    coord_path = Path(coord_file).expanduser()
    magnetic_path = Path(magnetic_file).expanduser()
    for path in (coord_path, magnetic_path):
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.exists() and not overwrite:
            raise FileExistsError(
                f"Refusing to overwrite {path}. Use --overwrite or choose another path."
            )
    return coord_path, magnetic_path
