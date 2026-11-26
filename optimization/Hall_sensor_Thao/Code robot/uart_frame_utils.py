"""UART reader for the STM32 16-sensor Bx/By/Bz text frame."""

import time

import numpy as np


NUM_SENSORS = 16
NUM_AXES = 3
NUM_SAMPLES = 5
AXIS_NAMES = ("Bx", "By", "Bz")
MAX_UART_LINE_BYTES = 512


def magnetic_csv_headers():
    """Headers in the same axis-major order as the firmware UART stream."""
    return [
        f"{axis}_sensor_{sensor_id:02d}_mT"
        for axis in AXIS_NAMES
        for sensor_id in range(1, NUM_SENSORS + 1)
    ]


def open_sensor_uart(port, baud=921600, timeout=2.0):
    """Open the UART and discard startup lines before collection starts."""
    import serial

    ser = serial.Serial(port, baud, timeout=timeout)
    time.sleep(1.0)
    ser.reset_input_buffer()
    print(f"UART connected: {port} @ {baud}")
    return ser


def _read_line(ser, deadline):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        return None

    previous_timeout = getattr(ser, "timeout", None)
    can_adjust_timeout = hasattr(ser, "timeout")
    if can_adjust_timeout:
        ser.timeout = remaining if previous_timeout is None else min(
            float(previous_timeout), remaining
        )

    try:
        line = ser.readline(MAX_UART_LINE_BYTES)
    finally:
        if can_adjust_timeout:
            ser.timeout = previous_timeout

    if not line or not line.endswith(b"\n"):
        return None
    return line.rstrip(b"\r\n")


def _parse_status_line(line):
    try:
        fields = line.decode("ascii").strip().split(",")
    except UnicodeDecodeError:
        return None

    if not fields:
        return None
    if fields[0] == "ERR":
        return ("ERR", ",".join(fields[1:]))
    if fields[0] not in ("ACK", "DONE") or len(fields) != 3:
        return None

    try:
        return fields[0], int(fields[1]), int(fields[2])
    except ValueError:
        return ("ERR", "malformed status line")


def _parse_axis_line(line):
    try:
        fields = line.decode("ascii").strip().split(",")
    except UnicodeDecodeError:
        return None

    axis = fields[0].strip() if fields else ""
    if axis not in AXIS_NAMES:
        return None
    if len(fields) != NUM_SENSORS + 3:
        return axis, None, None, None

    try:
        sample_id = int(fields[1], 10)
        frame_index = int(fields[2], 10)
    except ValueError:
        return axis, None, None, None

    try:
        values = np.asarray([int(value.strip(), 10) for value in fields[3:]],
                            dtype=np.float32)
    except ValueError:
        return axis, sample_id, frame_index, None

    if not np.isfinite(values).all():
        return axis, sample_id, frame_index, None

    # main.c transmits integer microtesla; the CSV uses millitesla.
    return axis, sample_id, frame_index, values * np.float32(0.001)


def _read_sensor_frame(ser, deadline):
    """Read one ordered triplet and return its sample ID, index, and 48 values."""
    axes = {}
    current_sample_id = None
    current_frame_index = None

    while time.monotonic() < deadline:
        line = _read_line(ser, deadline)
        if line is None:
            continue

        parsed = _parse_axis_line(line)
        if parsed is None:
            # Ignore status lines, startup diagnostics, and unrelated text.
            continue
        axis, sample_id, frame_index, values = parsed
        if values is None:
            axes.clear()
            continue

        if axis == "Bx":
            axes = {"Bx": values}
            current_sample_id = sample_id
            current_frame_index = frame_index
        elif (axis == "By" and "Bx" in axes
              and sample_id == current_sample_id
              and frame_index == current_frame_index):
            axes["By"] = values
        elif (axis == "Bz" and "Bx" in axes and "By" in axes
              and sample_id == current_sample_id
              and frame_index == current_frame_index):
            axes["Bz"] = values
            frame = np.concatenate([axes[name] for name in AXIS_NAMES])
            return current_sample_id, current_frame_index, frame
        else:
            # Do not combine lines from different scans.
            axes.clear()

    return None


def acquire_sensor_average(ser, sample_id, num_samples=NUM_SAMPLES, timeout=2.0):
    """Request five scans and average them only after matching ACK/DONE IDs."""
    if num_samples != NUM_SAMPLES:
        raise ValueError(f"Firmware capture count is fixed at {NUM_SAMPLES}")
    if not 0 <= int(sample_id) <= 0xFFFFFFFF:
        raise ValueError("sample_id must fit an unsigned 32-bit integer")

    ser.reset_input_buffer()
    command = f"CAPTURE,{int(sample_id)},{num_samples}\r\n".encode("ascii")
    ser.write(command)
    ser.flush()

    deadline = time.monotonic() + timeout * (num_samples + 2)
    acknowledged = False
    while not acknowledged and time.monotonic() < deadline:
        line = _read_line(ser, deadline)
        if line is None:
            continue
        status = _parse_status_line(line)
        if status is None:
            continue
        if status[0] == "ERR":
            print(f"[UART ERROR] STM32 rejected capture: {status[1]}")
            return None
        if status[0] == "ACK":
            if status[1:] != (int(sample_id), num_samples):
                print(f"[UART ERROR] Unexpected capture ACK: {status}")
                return None
            acknowledged = True

    if not acknowledged:
        print(f"[UART ERROR] No ACK for sample {sample_id}")
        return None

    frames = []
    while len(frames) < num_samples and time.monotonic() < deadline:
        result = _read_sensor_frame(ser, deadline)
        if result is None:
            continue
        received_id, frame_index, frame = result
        expected_index = len(frames) + 1
        if received_id != int(sample_id) or frame_index != expected_index:
            print(
                f"[UART ERROR] Expected sample {sample_id} frame {expected_index}, "
                f"received sample {received_id} frame {frame_index}"
            )
            return None
        if frame.shape != (NUM_SENSORS * NUM_AXES,) or not np.isfinite(frame).all():
            print(f"[UART ERROR] Invalid 48-channel frame {frame_index}")
            return None
        frames.append(frame)

    if len(frames) != num_samples:
        print(
            f"[UART ERROR] Received {len(frames)}/{num_samples} complete "
            "Bx/By/Bz scans; skipping this robot sample."
        )
        return None

    completed = False
    while not completed and time.monotonic() < deadline:
        line = _read_line(ser, deadline)
        if line is None:
            continue
        status = _parse_status_line(line)
        if status is None:
            continue
        if status[0] == "ERR":
            print(f"[UART ERROR] STM32 capture failed: {status[1]}")
            return None
        if status[0] == "DONE":
            if status[1:] != (int(sample_id), num_samples):
                print(f"[UART ERROR] Unexpected capture completion: {status}")
                return None
            completed = True

    if not completed:
        print(f"[UART ERROR] No DONE for sample {sample_id}")
        return None

    return np.mean(np.stack(frames, axis=0), axis=0, dtype=np.float64).astype(
        np.float32
    )
