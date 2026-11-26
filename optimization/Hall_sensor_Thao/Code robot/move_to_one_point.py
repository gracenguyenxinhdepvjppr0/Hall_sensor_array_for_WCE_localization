import socket
import math
import time
import re

# ---------------- TCP/IP CONFIG ----------------
ROBOT_IP = "192.168.125.1"
ROBOT_PORT = 2500

# ---------------- QUATERNION FUNCTION ----------------
def euler_to_quaternion(rx_deg, ry_deg, rz_deg):
    """Chuyển từ góc Euler (deg) sang quaternion (q0..q3) theo thứ tự quay Rz-Ry-Rx."""
    
    # Convert degrees to radians
    # rx = math.radians(rx_deg)
    # ry = math.radians(ry_deg)
    # rz = math.radians(rz_deg)
    
    # cz2 = math.cos(rz / 2.0)
    # sz2 = math.sin(rz / 2.0)
    # cy2 = math.cos(ry / 2.0)
    # sy2 = math.sin(ry / 2.0)
    # cx2 = math.cos(rx / 2.0)
    # sx2 = math.sin(rx / 2.0)

    # q0 = cz2 * cy2 * cx2 + sz2 * sy2 * sx2
    # q1 = cz2 * cy2 * sx2 - sz2 * sy2 * cx2
    # q2 = cz2 * sy2 * cx2 + sz2 * cy2 * sx2
    # q3 = sz2 * cy2 * cx2 - cz2 * sy2 * sx2

    # fixed rotation quaternion
    q0 =   0.0
    q1 =   0.0
    q2 =   1.0
    q3 =   0.0
    
    return q0, q1, q2, q3

# ---------------- SOCKET UTILS ----------------
def connect_robot(ip, port):
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.settimeout(20)
    print(f"Connecting to robot server {ip}:{port} ...")
    client.connect((ip, port))
    print("Connected to robot server.")
    return client


def clear_socket_buffer(client):
    """Xóa toàn bộ dữ liệu tồn trong buffer trước khi gửi lệnh mới."""
    client.setblocking(False)
    try:
        while True:
            data = client.recv(1024)
            if not data:
                break
    except BlockingIOError:
        pass
    finally:
        client.setblocking(True)

def send_position(client, x, y, z, rx=0, ry=0, rz=0):
    """Gửi tọa độ (x, y, z) và góc quay sang robot"""
    q0, q1, q2, q3 = euler_to_quaternion(rx, ry, rz)
    msg = f"[{x:.3f},{y:.3f},{z:.3f}];[{q0:.3f},{q1:.3f},{q2:.3f},{q3:.3f}]"
    client.sendall(msg.encode())
    print(f"Sent: {msg}")


def wait_for_robot_feedback(client, timeout=10):
    """Chờ robot phản hồi lại dạng [{x},{y},{z}];[{rx},{ry},{rz}]"""
    client.settimeout(timeout)
    buffer = ""
    start = time.time()

    pattern = re.compile(
        r"\[\s*(-?\d+\.\d+)\s*,\s*(-?\d+\.\d+)\s*,\s*(-?\d+\.\d+)\s*\];\[\s*(-?\d+\.\d+)\s*,\s*(-?\d+\.\d+)\s*,\s*(-?\d+\.\d+)\s*\]"
    )

    while True:
        try:
            data = client.recv(1024)
            if not data:
                continue
            buffer += data.decode()

            match = pattern.search(buffer)
            if match:
                x, y, z, rx, ry, rz = map(float, match.groups())
                print(f"[{time.strftime('%H:%M:%S')}] Robot reached position: "
                      f"({x:.3f}, {y:.3f}, {z:.3f}), rotation ({rx:.3f}, {ry:.3f}, {rz:.3f})")
                return True

        except socket.timeout:
            print("Timeout waiting for robot feedback.")
            return False
        except Exception as e:
            print("Receive error:", e)
            return False

        if time.time() - start > timeout:
            print("Feedback wait timeout expired.")
            return False

# ---------------- MAIN ----------------
if __name__ == "__main__":
    try:
        client = connect_robot(ROBOT_IP, ROBOT_PORT)
        print("\n=== Connecting to robot successfully ===")

        while True:
            user_input = input("Nhập tọa độ (x y z rx ry rz) hoặc 'exit' để thoát: ").strip()
            if user_input.lower() == "exit":
                break
 
            x, y, z, rx, ry, rz = map(float, user_input.split())

            # Xóa phản hồi cũ trước khi gửi lệnh mới
            clear_socket_buffer(client)

            print(f"Position: ({x:.3f}, {y:.3f}, {z:.3f}), Rotation: ({rx:.3f}, {ry:.3f}, {rz:.3f}) ...")
            send_position(client, x, y, z, rx, ry, rz)
            

            ok = wait_for_robot_feedback(client, timeout=20)
            if not ok:
                print("Không nhận được phản hồi từ robot, chờ khôi phục kết nối.")
                continue

    except Exception as e:
        print("Error:", e)
    finally:
        try:
            client.close()
        except:
            pass
        print("Đã ngắt kết nối.")

