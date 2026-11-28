import socket
import time
import re
import math
import importlib.util
import sys
import numpy as np
import serial
import csv
import os
from scipy.spatial.transform import Rotation as R
from uart_frame_utils import (NUM_SAMPLES, acquire_sensor_average,
                              magnetic_csv_headers, open_sensor_uart)

# ==============================================================================
# ROBOT CONFIG
# ==============================================================================
ROBOT_IP   = "192.168.125.1"
ROBOT_PORT = 2500

# ==============================================================================
# UART CONFIG
# ==============================================================================
UART_PORT    = "COM5"
UART_BAUD    = 921600 #check again
UART_TIMEOUT = 2

# ==============================================================================
# FILE OUTPUT
# ==============================================================================
COORD_FILE  = r"C:\Users\Admin\OneDrive\Documents\Hall_sensor_array\Git\optimization\Hall_sensor_Thao\Code robot\Grid_points_coordinates_2.csv"
SENSOR_FILE = r"C:\Users\Admin\OneDrive\Documents\Hall_sensor_array\Git\optimization\Hall_sensor_Thao\Code robot\Grid_data_2.csv"

# ==============================================================================
# TOOL OFFSET
# ==============================================================================
TOOL_OFFSET_X = -3.0
TOOL_OFFSET_Y = 0.0
TOOL_OFFSET_Z = 98.5

# ==============================================================================
# START TOOL POSITION (vị trí bắt đầu quỹ đạo theo tọa độ của đầu tool tại sensor[0][0])
# chú ý cao hơn mảng sensor một khoảng nhất định để tránh va chạm
# ============================================================================== 
# check lại hết các thông số vị trí
XC = -50 #sensor [0][0]
YC = 690
ZC = -63.5 #cách 2 cm

# ==============================================================================
# GRID CONFIG
# ==============================================================================
GRID_X = 8
GRID_Y = 8
GRID_Z = 6

# Offset spans from the first tooltip pose (mm). X is subtracted so increasing
# grid indices move toward decreasing world X. With the current 8x8x6 grid,
# spacing is about 21.43 mm in X/Y and 5 mm in Z.
DX_MIN, DX_MAX = 0.0, 140.0
DY_MIN, DY_MAX = 0.0, 140.0
DZ_MIN, DZ_MAX = 0.0, 25.0

# Chạy từ đầu
START_POINT = 1

# ==============================================================================
# TIMING
# ==============================================================================
POINT_DELAY = 2.0 #(s)
ANGLE_DELAY = 2.4 #(s)
WARMUP_DELAY = 2 #(s)

# ==============================================================================
# REFERENCE ORIENTATION
# ==============================================================================
REFERENCE_QUAT_ABB = (0.0,0.0,1.0,0.0)

# hướng nam châm trong tool frame, chiều dương 0x trùng cực bắc nam châm
MAGNET_DIR_TOOL = np.array([1.0,0.0,0.0])

# ==============================================================================
# UART FUNCTIONS
# ==============================================================================
def open_uart():
    return open_sensor_uart(UART_PORT, UART_BAUD, UART_TIMEOUT)


def acquire_data_uart(ser, sample_id):
    return acquire_sensor_average(ser, sample_id, NUM_SAMPLES, UART_TIMEOUT)

# ==============================================================================
# MATH
# ==============================================================================
def quat_to_rot_matrix(q):

    qw,qx,qy,qz=q

    norm=math.sqrt(qw*qw+qx*qx+qy*qy+qz*qz)

    qw,qx,qy,qz=qw/norm,qx/norm,qy/norm,qz/norm

    return [
        [1-2*(qy*qy+qz*qz),2*(qx*qy-qz*qw),2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),1-2*(qx*qx+qz*qz),2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),2*(qy*qz+qx*qw),1-2*(qx*qx+qy*qy)]
    ]


def compute_tcp_from_tooltip(x,y,z,quat,ox,oy,oz):

    Rm=quat_to_rot_matrix(quat)

    dx=Rm[0][0]*ox + Rm[0][1]*oy + Rm[0][2]*oz
    dy=Rm[1][0]*ox + Rm[1][1]*oy + Rm[1][2]*oz
    dz=Rm[2][0]*ox + Rm[2][1]*oy + Rm[2][2]*oz

    return x-dx,y-dy,z-dz


# ==============================================================================
# MAGNET ORIENTATION
# ==============================================================================
def magnet_vector_world(quat):

    qw,qx,qy,qz = quat

    r = R.from_quat([qx,qy,qz,qw])

    m_world = r.apply(MAGNET_DIR_TOOL)

    return m_world


def magnet_angles_from_vector(m):

    mx,my,mz = m

    norm = math.sqrt(mx*mx+my*my+mz*mz)

    if norm < 1e-9:
        return 0.0,0.0

    mx,my,mz = mx/norm,my/norm,mz/norm

    pitch = math.degrees(math.asin(mz))
    yaw   = math.degrees(math.atan2(my,mx))

    return pitch,yaw

# ==============================================================================
# TRAJECTORY
# ==============================================================================
def generate_3d_grid():
    """Return every point in the 8x8 grid at each configured Z layer."""
    if GRID_X < 1 or GRID_Y < 1 or GRID_Z < 1:
        raise ValueError("GRID_X, GRID_Y, and GRID_Z must all be positive")

    x_offsets = np.linspace(DX_MIN, DX_MAX, GRID_X)
    y_offsets = np.linspace(DY_MIN, DY_MAX, GRID_Y)
    z_offsets = np.linspace(DZ_MIN, DZ_MAX, GRID_Z)
    path = []

    for iz in range(GRID_Z):
        z = ZC + z_offsets[iz]
        x_indices = range(GRID_X) if iz % 2 == 0 else range(GRID_X - 1, -1, -1)

        for x_sequence, ix in enumerate(x_indices):
            x = XC - x_offsets[ix]
            y_forward = (x_sequence + iz) % 2 == 0
            y_indices = range(GRID_Y) if y_forward else range(GRID_Y - 1, -1, -1)

            for iy in y_indices:
                y = YC + y_offsets[iy]
                # Do not filter edges: include all interior and boundary points.
                path.append((x, y, z))

    expected_points = GRID_X * GRID_Y * GRID_Z
    if len(path) != expected_points:
        raise RuntimeError(
            f"Grid is incomplete: generated {len(path)} of {expected_points} points"
        )

    return path


def _load_trajectory_script(file_name, module_name):
    """Load another trajectory module without entering its robot main loop."""
    source_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), file_name)
    spec = importlib.util.spec_from_file_location(module_name, source_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load trajectory script: {source_path}")

    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def visualize_all_trajectories(seed=2):
    """Plot all four waypoint paths without connecting to robot or UART."""
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "Matplotlib is required for visualization. Install it with: "
            "python -m pip install matplotlib"
        ) from exc

    grid = sys.modules[__name__]
    helix = _load_trajectory_script("robot_ABB_helical.py", "trajectory_helical")
    lissajous = _load_trajectory_script(
        "robot_ABB_Lissajous Curve.py", "trajectory_lissajous"
    )
    random_path = _load_trajectory_script(
        "robot_ABB_random.py", "trajectory_random"
    )

    np.random.seed(seed)
    trajectories = [
        ("Grid", grid.generate_3d_grid()),
        ("Helix", helix.generate_helical_path()),
        ("Lissajous", lissajous.generate_lissajous_path()),
        ("Random", random_path.generate_random_points()),
    ]

    figure = plt.figure(figsize=(13, 10))
    figure.suptitle(f"Robot trajectory waypoints (random seed: {seed})")
    for index, (name, path) in enumerate(trajectories, start=1):
        if not path:
            raise ValueError(f"Trajectory '{name}' contains no points")

        xs, ys, zs = zip(*path)
        axis = figure.add_subplot(2, 2, index, projection="3d")
        line, = axis.plot(xs, ys, zs, linewidth=1.4, label="Path")
        axis.scatter(xs, ys, zs, color=line.get_color(), s=8, alpha=0.55)
        axis.scatter(xs[0], ys[0], zs[0], color="green", marker="^", s=55,
                     edgecolors="black", linewidths=0.5, label="Start")
        axis.scatter(xs[-1], ys[-1], zs[-1], color="red", marker="s", s=45,
                     edgecolors="black", linewidths=0.5, label="End")
        axis.set_title(f"{name} ({len(path)} points)")
        axis.set_xlabel("X (mm)")
        axis.set_ylabel("Y (mm)")
        axis.set_zlabel("Z (mm)")
        axis.view_init(elev=25, azim=-55)
        axis.legend(loc="best", fontsize=8)
        spans = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
        axis.set_box_aspect(tuple(max(span, 1.0) for span in spans))

    figure.tight_layout(rect=(0, 0, 1, 0.96))
    output_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "trajectory_visualization.png"
    )
    figure.savefig(output_path, dpi=180, bbox_inches="tight")
    print(f"Saved trajectory figure: {output_path}")
    plt.show()

# ============================================================================
# ORIENTATION SET
# ============================================================================

def generate_orientations(ref_quat, num_cycles=1):
    w, x, y, z = ref_quat
    r_ref = R.from_quat([x, y, z, w])

    # Cặp pitch/yaw cố định cho mỗi vị trí
    angle_pairs = [
        (-20, -15),
        (-15, -10),
        (-10, -5),
        (-5, 0),
        (0, 5),
        (5, 10),
        (10, 15),
        (15, 20)
    ]

    quats = []

    for _ in range(num_cycles):
        for pitch, yaw in angle_pairs:
            r_pitch = R.from_euler('y', pitch, degrees=True)
            r_yaw = R.from_euler('z', yaw, degrees=True)
            r_final = r_ref * r_pitch * r_yaw

            q = r_final.as_quat()
            quats.append((q[3], q[0], q[1], q[2]))

    return quats

# ==============================================================================
# ROBOT COMMUNICATION
# ==============================================================================
def connect_robot():
    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.settimeout(20)
    print("Connecting robot...")
    s.connect((ROBOT_IP,ROBOT_PORT))
    print("Connected")
    return s


def clear_socket_buffer(client):
    client.setblocking(False)
    try:
        while client.recv(1024):
            pass
    except:
        pass
    client.setblocking(True)


def send_pose(client,x,y,z,quat):
    qw,qx,qy,qz=quat
    msg=f"[{x:.3f},{y:.3f},{z:.3f}];[{qw:.5f},{qx:.5f},{qy:.5f},{qz:.5f}]"
    client.sendall(msg.encode())


def wait_robot(client,timeout=20):

    client.settimeout(timeout)

    buffer=""
    pattern=re.compile(r"\[.*?\]")

    start=time.time()

    while time.time()-start<timeout:

        try:

            buffer+=client.recv(1024).decode(errors="ignore")

            if pattern.search(buffer):
                return True

        except:
            pass

    return False

# ==============================================================================
# MAIN
# ==============================================================================
if __name__=="__main__":

    if "--visualize-trajectories" in sys.argv[1:]:
        visualize_all_trajectories()
        raise SystemExit(0)

    client=None
    ser=None

    try:

        client=connect_robot()
        ser=open_uart()

        path=generate_3d_grid()
        orientations=generate_orientations(REFERENCE_QUAT_ABB)

        total_points = len(path)
        total_orient = len(orientations)
        total_samples = total_points * total_orient

        print("="*70)
        print("SCAN SUMMARY")
        print("Grid points:", total_points)
        print("Orientations per point:", total_orient)
        print("Total samples:", total_samples)
        print(f"Start tooltip: ({XC:.3f}, {YC:.3f}, {ZC:.3f}) mm")
        print(
            f"Relative limits: dX=[{DX_MIN:.1f},{DX_MAX:.1f}], "
            f"dY=[{DY_MIN:.1f},{DY_MAX:.1f}], dZ=[{DZ_MIN:.1f},{DZ_MAX:.1f}] mm"
        )
        print("="*70)

        print("Total points:",len(path))
        print("Orientations per point:",len(orientations))

        os.makedirs(os.path.dirname(COORD_FILE),exist_ok=True)

        # Append hoặc ghi mới tùy thuộc vào START_POINT
        if START_POINT > 1:
            coord_file = open(COORD_FILE, "a", newline="")
            sensor_file = open(SENSOR_FILE, "a", newline="")
            print(f"Appending from sample {START_POINT}...")
        else:
            coord_file = open(COORD_FILE, "w", newline="")
            sensor_file = open(SENSOR_FILE, "w", newline="")

        coord_writer=csv.writer(coord_file)
        sensor_writer=csv.writer(sensor_file)

        coord_writer.writerow(["sample_id","x","y","z","pitch","yaw","mx","my","mz"])
        sensor_writer.writerow(["sample_id"] + magnetic_csv_headers())

        # warmup
        first=path[0]

        tcp=compute_tcp_from_tooltip(
            first[0],first[1],first[2],
            REFERENCE_QUAT_ABB,
            TOOL_OFFSET_X,TOOL_OFFSET_Y,TOOL_OFFSET_Z
        )

        send_pose(client,tcp[0],tcp[1],tcp[2],REFERENCE_QUAT_ABB)

        if wait_robot(client):

            print("Warmup reached")
            time.sleep(WARMUP_DELAY)

        # scanning loop

        sample_id = 0
        for p,(tx,ty,tz) in enumerate(path):

            print("\n"+"="*70)
            print(f"GRID POINT {p+1}/{total_points}")
            print(f"Tooltip position : ({tx:.2f}, {ty:.2f}, {tz:.2f}) mm")
            print("="*70)

            for i,quat in enumerate(orientations):
                sample_id += 1
                if sample_id < START_POINT:
                    continue

                tcp=compute_tcp_from_tooltip(
                    tx,ty,tz,
                    quat,
                    TOOL_OFFSET_X,
                    TOOL_OFFSET_Y,
                    TOOL_OFFSET_Z
                )
                m_world = magnet_vector_world(quat)
                mx,my,mz = m_world
                pitch,yaw = magnet_angles_from_vector(m_world)

                print("\n------------------------------------------------")
                print(f"Sample {sample_id}/{total_samples}")
                print(f"Orientation {i+1}/{total_orient}")

                print(f"TCP position   : ({tcp[0]:.2f}, {tcp[1]:.2f}, {tcp[2]:.2f}) mm")
                print(f"Tooltip fixed  : ({tx:.2f}, {ty:.2f}, {tz:.2f}) mm")

                print(f"Magnet vector  : [{mx:.3f}, {my:.3f}, {mz:.3f}]")
                print(f"Magnet angles  : pitch={pitch:.2f}°, yaw={yaw:.2f}°")

                clear_socket_buffer(client)

                # Measure one pose from command send through both CSV flushes.
                pose_start = time.perf_counter()
                send_pose(client,tcp[0],tcp[1],tcp[2],quat)

                robot_reached = wait_robot(client)
                robot_wait_s = time.perf_counter() - pose_start
                if not robot_reached:

                    print("Robot timeout")
                    print(
                        f"[TIMING] sample_id={sample_id} robot_wait={robot_wait_s:.3f}s "
                        f"total={robot_wait_s:.3f}s status=ROBOT_TIMEOUT"
                    )
                    continue

                settle_start = time.perf_counter()
                if i==0:
                    time.sleep(POINT_DELAY)
                else:
                    time.sleep(ANGLE_DELAY)
                settle_s = time.perf_counter() - settle_start

                # =============================
                # SENSOR DATA
                # =============================

                uart_start = time.perf_counter()
                avg_data=acquire_data_uart(ser, sample_id)
                uart_capture_s = time.perf_counter() - uart_start
                if avg_data is None:
                    total_pose_s = time.perf_counter() - pose_start
                    print(
                        f"[TIMING] sample_id={sample_id} robot_wait={robot_wait_s:.3f}s "
                        f"settle={settle_s:.3f}s uart={uart_capture_s:.3f}s "
                        f"total={total_pose_s:.3f}s status=UART_ERROR"
                    )
                    raise RuntimeError(
                        f"UART capture failed for sample {sample_id}; stopping trajectory"
                    )
                # magnet orientation
                m_world = magnet_vector_world(quat)

                mx,my,mz = m_world

                pitch,yaw = magnet_angles_from_vector(m_world)

                # print("Sensor:",np.round(avg_data,4))

                csv_start = time.perf_counter()
                coord_writer.writerow([
                    sample_id,
                    tx/1000.0, ty/1000.0, tz/1000.0, # chuyển sang mét khi lưu file 
                    pitch,yaw,
                    mx,my,mz
                ])

                sensor_writer.writerow([sample_id, *avg_data])

                coord_file.flush()
                sensor_file.flush()
                csv_write_s = time.perf_counter() - csv_start
                total_pose_s = time.perf_counter() - pose_start
                print(
                    f"[TIMING] sample_id={sample_id} robot_wait={robot_wait_s:.3f}s "
                    f"settle={settle_s:.3f}s uart={uart_capture_s:.3f}s "
                    f"csv={csv_write_s:.3f}s total={total_pose_s:.3f}s"
                )

        print("\nScan finished")

    except Exception as e:

        print("Error:",e)

    finally:

        try: coord_file.close()
        except: pass

        try: sensor_file.close()
        except: pass

        try: client.close()
        except: pass

        try: ser.close()
        except: pass

        print("Disconnected")
