import socket
import time
import re
import math
import numpy as np
import serial
import csv
import os
from scipy.spatial.transform import Rotation as R

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

SENSOR_ROWS = 64
SENSOR_COLS = 5

# ==============================================================================
# FILE OUTPUT
# ==============================================================================
COORD_FILE  = r"C:\Workspace\Documents\2025.2\Final project\Data_8_2026\Data\Helix_points_coordinates_2.csv"
SENSOR_FILE = r"C:\Workspace\Documents\2025.2\Final project\Data_8_2026\Data\Helix_data_2.csv"

# ==============================================================================
# TOOL OFFSET
# ==============================================================================
TOOL_OFFSET_X = -3.0
TOOL_OFFSET_Y = 0.0
TOOL_OFFSET_Z = 98.5

# ==============================================================================
# HELIX CENTER 
# điểm bắt đầu tại tâm mảng cảm biến 
# cách mảng sensor một khoảng nhất định để tránh va chạm
# ==============================================================================
XC = 25.0
YC = 705.0
ZC = -63.5 #cách 2 cm
#(phải cách mảng cảm biến một khoảng nhất định để tránh va chạm)

# ==============================================================================
# HELIX PARAMETERS
# ==============================================================================
R_BAR_BASE   = 70
R_TOP        = 35
HELIX_HEIGHT = 30 # cách mặt sensor 2-5cm 
LOOPS        = 5
N_POINTS     = 150

# Chạy tiếp quỹ đạo
START_POINT = 159  # bắt đầu từ điểm thứ 159 của quỹ đạo

# ==============================================================================
# TIMING
# ==============================================================================
POINT_DELAY = 2.5 #(s)
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

    ser = serial.Serial(UART_PORT,UART_BAUD,timeout=UART_TIMEOUT)
    time.sleep(1)

    print("UART connected")

    return ser


def acquire_data_uart(ser):

    ser.reset_input_buffer()
    ser.write(b"START")

    data_matrix=[[] for _ in range(SENSOR_COLS)]

    current_col=0

    while True:

        line=ser.readline().decode(errors="ignore").strip()

        if line=="END":
            break

        try:
            value=float(line)
        except:
            continue

        if current_col<SENSOR_COLS:

            data_matrix[current_col].append(value)

            if len(data_matrix[current_col])==SENSOR_ROWS:
                current_col+=1

        if current_col==SENSOR_COLS:
            break

    for i,col in enumerate(data_matrix):

        if len(col)!=SENSOR_ROWS:

            print(f"[UART WARNING] column {i+1} received {len(col)} samples")

            return None

    data_matrix=np.array(data_matrix)

    return np.mean(data_matrix,axis=0)

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

    dx=Rm[0][0]*ox+Rm[0][1]*oy+Rm[0][2]*oz
    dy=Rm[1][0]*ox+Rm[1][1]*oy+Rm[1][2]*oz
    dz=Rm[2][0]*ox+Rm[2][1]*oy+Rm[2][2]*oz

    return x-dx,y-dy,z-dz

# ==============================================================================
# MAGNET ORIENTATION
# ==============================================================================
def magnet_vector_world(quat):

    qw,qx,qy,qz=quat

    r=R.from_quat([qx,qy,qz,qw])

    m_world=r.apply(MAGNET_DIR_TOOL)

    return m_world


def magnet_angles_from_vector(m):

    mx,my,mz=m

    norm=math.sqrt(mx*mx+my*my+mz*mz)

    if norm<1e-9:
        return 0.0,0.0

    mx,my,mz=mx/norm,my/norm,mz/norm

    pitch=math.degrees(math.asin(mz))
    yaw=math.degrees(math.atan2(my,mx))

    return pitch,yaw

# ==============================================================================
# HELICAL TRAJECTORY
# ==============================================================================
def generate_helical_path():

    path=[]

    for i in range(N_POINTS):

        t=i/(N_POINTS-1)

        theta=2*math.pi*LOOPS*t

        r=R_BAR_BASE+(R_TOP-R_BAR_BASE)*t

        x=XC+r*math.cos(theta)
        y=YC+r*math.sin(theta)
        z=ZC+HELIX_HEIGHT*t

        path.append((x,y,z))

    return path

# ==============================================================================
# RANDOM ORIENTATION
# ==============================================================================
def random_orientation(ref_quat):

    w,x,y,z=ref_quat

    r_ref=R.from_quat([x,y,z,w])

    pitch=np.random.uniform(-20,20)
    yaw=np.random.uniform(-20,20)

    r_pitch=R.from_euler('y',pitch,degrees=True)
    r_yaw=R.from_euler('z',yaw,degrees=True)

    r_final=r_ref*r_pitch*r_yaw

    q=r_final.as_quat()

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

        path=generate_helical_path()

        # if START_POINT > 1:
        #     print(f"Starting from point {START_POINT} of {len(path)}")
        #     path = path[START_POINT-1:]

        total_points=len(path)
        total_samples=total_points

        print("="*70)
        print("HELICAL SCAN SUMMARY")
        print("Points:",total_points)
        print("="*70)

        os.makedirs(os.path.dirname(COORD_FILE),exist_ok=True)

        coord_file=open(COORD_FILE,"w",newline="")
        sensor_file=open(SENSOR_FILE,"w",newline="")

        coord_writer=csv.writer(coord_file)
        sensor_writer=csv.writer(sensor_file)

        coord_writer.writerow(["x","y","z","pitch","yaw","mx","my","mz"])
        sensor_writer.writerow([f"sensor_{i+1}" for i in range(SENSOR_ROWS)])

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

        sample_id=0

        for p,(tx,ty,tz) in enumerate(path):

            sample_id+=1

            quat=random_orientation(REFERENCE_QUAT_ABB)

            tcp=compute_tcp_from_tooltip(
                tx,ty,tz,
                quat,
                TOOL_OFFSET_X,
                TOOL_OFFSET_Y,
                TOOL_OFFSET_Z
            )

            m_world=magnet_vector_world(quat)

            mx,my,mz=m_world
            pitch,yaw=magnet_angles_from_vector(m_world)

            print("\n------------------------------------------------")
            print(f"Sample {sample_id}/{total_samples}")
            print(f"Tooltip : ({tx:.2f},{ty:.2f},{tz:.2f})")
            print(f"TCP     : ({tcp[0]:.2f},{tcp[1]:.2f},{tcp[2]:.2f})")
            print(f"Angles  : pitch={pitch:.2f} yaw={yaw:.2f}")

            clear_socket_buffer(client)

            send_pose(client,tcp[0],tcp[1],tcp[2],quat)

            if not wait_robot(client):

                print("Robot timeout")
                continue

            time.sleep(POINT_DELAY)
            
            avg_data=acquire_data_uart(ser)

            if avg_data is None:
                print("Sensor read failed")
                continue

            coord_writer.writerow([
                tx/1000.0, ty/1000.0, tz/1000.0, # chuyển sang mét khi lưu file
                pitch,yaw,
                mx,my,mz
            ])

            sensor_writer.writerow(avg_data)

            coord_file.flush()
            sensor_file.flush()

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