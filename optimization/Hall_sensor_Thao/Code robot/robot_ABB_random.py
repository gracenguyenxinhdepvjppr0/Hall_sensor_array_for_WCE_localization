import socket
import time
import re
import math
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
COORD_FILE  = r"C:\Users\Admin\OneDrive\Documents\Hall_sensor_array\Git\optimization\Hall_sensor_Thao\Code robot\Random_points_coordinates.csv"
SENSOR_FILE = r"C:\Users\Admin\OneDrive\Documents\Hall_sensor_array\Git\optimization\Hall_sensor_Thao\Code robot\Random_data.csv"

# ==============================================================================
# TOOL OFFSET
# ==============================================================================
TOOL_OFFSET_X = -3.0
TOOL_OFFSET_Y = 0.0
TOOL_OFFSET_Z = 98.5

# ==============================================================================
# RANDOM SPACE LIMIT
# ==============================================================================
X_MIN = -45.0
X_MAX = 95.0

Y_MIN = 635.0
Y_MAX = 775.0

Z_MIN = -63.5 #cách 2 cm
Z_MAX = -33.5 #cách 5 cm

TOTAL_RANDOM_POINTS = 150 #check again

# ==============================================================================
# TIMING
# ==============================================================================
POINT_DELAY = 2.5
WARMUP_DELAY = 2

# ==============================================================================
# REFERENCE ORIENTATION
# ==============================================================================
REFERENCE_QUAT_ABB = (0.0,0.0,1.0,0.0)

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
# RANDOM TRAJECTORY
# ==============================================================================
def generate_random_points():

    path=[]

    for _ in range(TOTAL_RANDOM_POINTS):

        x = np.random.uniform(X_MIN,X_MAX)
        y = np.random.uniform(Y_MIN,Y_MAX)
        z = np.random.uniform(Z_MIN,Z_MAX)

        path.append((x,y,z))

    return path

# ==============================================================================
# RANDOM ORIENTATION
# ==============================================================================
def generate_random_orientation(ref_quat):

    w,x,y,z = ref_quat
    r_ref = R.from_quat([x,y,z,w])

    pitch = np.random.uniform(-20,20)
    yaw   = np.random.uniform(-20,20)

    r_pitch = R.from_euler('y',pitch,degrees=True)
    r_yaw   = R.from_euler('z',yaw,degrees=True)

    r_final = r_ref * r_pitch * r_yaw

    q = r_final.as_quat()

    return (q[3],q[0],q[1],q[2])

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

    client=None
    ser=None

    try:

        client=connect_robot()
        ser=open_uart()

        path=generate_random_points()

        total_points = len(path)
        total_orient = 1
        total_samples = total_points

        print("="*70)
        print("SCAN SUMMARY")
        print("Random points:", total_points)
        print("Total samples:", total_samples)
        print("="*70)

        os.makedirs(os.path.dirname(COORD_FILE),exist_ok=True)

        coord_file = open(COORD_FILE, "w", newline="")
        sensor_file = open(SENSOR_FILE, "w", newline="")

        coord_writer=csv.writer(coord_file)
        sensor_writer=csv.writer(sensor_file)

        coord_writer.writerow(["sample_id","x","y","z","pitch","yaw","mx","my","mz"])
        sensor_writer.writerow(["sample_id"] + magnetic_csv_headers())

        # warmup
        first=path[0]

        quat=generate_random_orientation(REFERENCE_QUAT_ABB)

        tcp=compute_tcp_from_tooltip(
            first[0],first[1],first[2],
            quat,
            TOOL_OFFSET_X,TOOL_OFFSET_Y,TOOL_OFFSET_Z
        )

        send_pose(client,tcp[0],tcp[1],tcp[2],quat)

        if wait_robot(client):

            print("Warmup reached")
            time.sleep(WARMUP_DELAY)

        sample_id = 0

        for p,(tx,ty,tz) in enumerate(path):

            quat = generate_random_orientation(REFERENCE_QUAT_ABB)

            sample_id += 1

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
            print(f"Tooltip : ({tx:.2f}, {ty:.2f}, {tz:.2f}) mm")
            print(f"TCP     : ({tcp[0]:.2f}, {tcp[1]:.2f}, {tcp[2]:.2f}) mm")
            print(f"Angles  : pitch={pitch:.2f} yaw={yaw:.2f}")

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
            time.sleep(POINT_DELAY)
            settle_s = time.perf_counter() - settle_start

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

            csv_start = time.perf_counter()
            coord_writer.writerow([
                sample_id,
                tx/1000.0,ty/1000.0,tz/1000.0, # chuyển sang mét khi lưu file
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
