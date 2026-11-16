#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Đọc dữ liệu 16 cảm biến từ UART và lưu một dòng CSV cho mỗi khung.

Ví dụ:
    python uart_to_csv.py --port COM4 --output data.csv

Cài thư viện cần thiết một lần:
    python -m pip install pyserial

Mỗi khung UART tạo một dòng CSV. Sau cột frame, các cột Bx, By, Bz được
xếp theo từng cảm biến, từ sensor0 đến sensor15. Giá trị giữ nguyên đơn vị uT.
"""

import argparse
import csv
import sys


SENSOR_COUNT = 16


def parse_frame(raw_line):
    """Phân tích một dòng B,... từ firmware; trả về None nếu dòng không hợp lệ."""
    try:
        line = raw_line.decode("ascii", errors="strict").strip()
        fields = line.split(",")

        # 51 trường: dữ liệu B; 69 trường: B cùng dữ liệu nhiệt độ/calib.
        if not fields or fields[0] != "B" or len(fields) not in (51, 69):
            return None

        frame = int(fields[1])
        error_mask = int(fields[2], 16)
        bx = [int(value) for value in fields[3:19]]
        by = [int(value) for value in fields[19:35]]
        bz = [int(value) for value in fields[35:51]]
        if not (len(bx) == len(by) == len(bz) == SENSOR_COUNT):
            return None

        return frame, error_mask, bx, by, bz
    except (UnicodeDecodeError, ValueError):
        return None


def main():
    parser = argparse.ArgumentParser(
        description="Đọc UART từ firmware TMAG5170 và lưu mỗi trục vào một cột CSV."
    )
    parser.add_argument("-p", "--port", required=True,
                        help="cổng UART, ví dụ COM5 trên Windows hoặc /dev/ttyUSB0")
    parser.add_argument("-o", "--output", default="uart_data.csv",
                        help="tên file CSV đầu ra (mặc định: uart_data.csv)")
    parser.add_argument("-b", "--baud", type=int, default=921600,
                        help="baud rate (mặc định: 921600, theo main.c)")
    parser.add_argument("-n", "--frames", type=int, default=0,
                        help="dừng sau N khung hợp lệ; mặc định là đọc đến khi nhấn Ctrl+C")
    args = parser.parse_args()

    if args.frames < 0:
        parser.error("--frames phải lớn hơn hoặc bằng 0")

    try:
        import serial
    except ImportError:
        sys.exit("Thiếu pyserial. Cài bằng lệnh: python -m pip install pyserial")

    frames_written = 0
    print("Đang mở %s ở %d baud; lưu vào %s" % (args.port, args.baud, args.output))
    print("Nhấn Ctrl+C để dừng.")

    try:
        with serial.Serial(args.port, args.baud, timeout=1) as ser:
            with open(args.output, "w", newline="", encoding="utf-8-sig") as csv_file:
                writer = csv.writer(csv_file)
                header = ["frame"]
                for sensor_index in range(SENSOR_COUNT):
                    header.extend([
                        "Bx_uT_sensor%d" % sensor_index,
                        "By_uT_sensor%d" % sensor_index,
                        "Bz_uT_sensor%d" % sensor_index,
                    ])
                header.extend(["error_mask_hex", "sensor15_error"])
                writer.writerow(header)
                csv_file.flush()

                try:
                    while args.frames == 0 or frames_written < args.frames:
                        parsed = parse_frame(ser.readline())
                        if parsed is None:
                            continue

                        frame, error_mask, bx, by, bz = parsed
                        row = [frame]
                        for sensor_index in range(SENSOR_COUNT):
                            row.extend([
                                bx[sensor_index], by[sensor_index], bz[sensor_index],
                            ])
                        row.extend([
                            "0x%04X" % error_mask,
                            int(bool(error_mask & (1 << 15))),
                        ])
                        writer.writerow(row)

                        frames_written += 1
                        # Giữ dữ liệu đã nhận nếu chương trình hoặc máy tính dừng đột ngột.
                        csv_file.flush()
                        if frames_written % 100 == 0:
                            print("Đã lưu %d khung" % frames_written)
                except KeyboardInterrupt:
                    print("\nĐã dừng theo yêu cầu.")
    except serial.SerialException as exc:
        sys.exit("Không mở/đọc được cổng %s: %s" % (args.port, exc))
    except OSError as exc:
        sys.exit("Không ghi được file %s: %s" % (args.output, exc))

    print("Hoàn tất: %d khung trong %s" % (frames_written, args.output))


if __name__ == "__main__":
    main()
