import numpy as np
import matplotlib.pyplot as plt
import csv
import os

# Note
# z tính từ cạnh dưới mô hình 3D không phải từ capsule, x,y 
# so với phần capsule thò ra ngoài mô hình 3D
# Thực tế tọa độ nam châm: x + 0.0215m (+21.5mm); z - 0.0985m (-98.5mm)

# ==============================================================================
# CONFIGURATION MACROS
# ==============================================================================
# Absolute robot-frame coordinate of Sensor 1 (top-left), in meters. 
# Lấy tọa độ thật nhập vào đây 
SENSOR1_X_ABS = -0.045  # m = -45 mm
SENSOR1_Y_ABS =  0.635  # m = 635 mm
SENSOR1_Z_ABS = -0.0835  # m = -83.5 mm

# New coordinate origin relative to Sensor 1: 30 mm left and 120 mm below.
ORIGIN_OFFSET_X = -0.030  # m
ORIGIN_OFFSET_Y = -0.120  # m
SENSOR1_OFFSET_X = -ORIGIN_OFFSET_X  # Sensor 1 relative to new origin
SENSOR1_OFFSET_Y = -ORIGIN_OFFSET_Y

# Absolute robot-frame coordinate of the new origin, in meters.
ORIGIN_X = SENSOR1_X_ABS + ORIGIN_OFFSET_X
ORIGIN_Y = SENSOR1_Y_ABS + ORIGIN_OFFSET_Y
ORIGIN_Z = SENSOR1_Z_ABS

# Sensor spacing (distance between adjacent sensors)
DX = 0.030  # m (horizontal spacing, X axis)
DY = 0.030  # m (vertical spacing, Y axis)

# Array dimensions
N_ROWS = 4
N_COLS = 4

# Visualization parameters
ANNOTATE = True
CIRCLE_SIZE = 200
VISUALIZE_DX = 0.030  # m
VISUALIZE_DY = 0.030  # m

def generate_sensor_coordinates(
    origin,
    dx=0.030,
    dy=0.030,
    n_rows=4,
    n_cols=4,
    sensor1_offset=(SENSOR1_OFFSET_X, SENSOR1_OFFSET_Y),
):
    """
    origin là tọa độ tuyệt đối của gốc mới trong hệ robot, đơn vị mét.
    sensor1_offset là tọa độ Sensor 1 so với gốc mới, đơn vị mét.
    X tăng sang phải; Y giảm khi đi từ hàng trên xuống hàng dưới.
    Trả về tọa độ tuyệt đối theo thứ tự hàng, rồi cột.
    """
    x0, y0, z0 = origin
    sensor1_x_offset, sensor1_y_offset = sensor1_offset
    coords = []
    for row in range(n_rows):    # row 0 = hàng 1 (trên cùng)
        for col in range(n_cols):# col 0 = cột 1 (trái nhất)
            x = x0 + sensor1_x_offset + col * dx
            y = y0 + sensor1_y_offset - row * dy
            z = z0
            coords.append([x, y, z])
    return np.array(coords)

def reorder_coordinates_by_column(coords, n_rows=4, n_cols=4):
    """
    Sắp xếp lại tọa độ theo thứ tự: cột đầu tiên từ hàng đầu đến hàng cuối,
    sau đó đến cột tiếp theo, lặp lại đến cột cuối cùng.
    """
    reordered = []
    for col in range(n_cols):    # Duyệt theo cột
        for row in range(n_rows): # Duyệt theo hàng trong mỗi cột
            index = row * n_cols + col  # Tính chỉ số trong mảng gốc
            reordered.append(coords[index])
    return np.array(reordered)

def visualize_sensor_array(coords, dx=0.030, dy=0.030, annotate=True, circle_size=200, save_image=True):
    """
    Vẽ sensor array với trục X ngang và trục Y dọc.
    Nếu save_image=True, lưu ảnh vào cùng thư mục với file này.
    """
    xs = coords[:, 0]   # x coordinates (ngang)
    ys = coords[:, 1]   # y coordinates (dọc)

    fig, ax = plt.subplots(figsize=(10, 8))
    
    # X nằm ngang, Y nằm dọc
    ax.scatter(xs, ys, s=circle_size, edgecolor='k', facecolor='orange', zorder=3)

    # Annotate sensor ID (số bắt đầu từ 1)
    if annotate:
        for i, (x, y) in enumerate(zip(xs, ys)):
            ax.text(x, y, str(i+1), ha='center', va='center', fontsize=8, zorder=4)

    # Đặt ticks
    xticks = np.unique(xs)  # trục ngang (X)
    yticks = np.unique(ys)  # trục dọc (Y)

    ax.set_xticks(xticks)
    ax.set_yticks(yticks)

    # Ghi nhãn trục - X là ngang, Y là dọc
    ax.set_xlabel("X (m) - Horizontal")
    ax.set_ylabel("Y (m) - Vertical")

    ax.set_aspect('equal', adjustable='box')

    # vẽ lưới
    for xt in xticks:
        ax.axvline(xt, linestyle='--', linewidth=0.7, alpha=0.5, zorder=0)
    for yt in yticks:
        ax.axhline(yt, linestyle='--', linewidth=0.7, alpha=0.5, zorder=0)

    n_rows = len(np.unique(ys))
    n_cols = len(np.unique(xs))
    ax.set_title(f"Sensor Array {n_rows}x{n_cols} (X: Horizontal, Y: Vertical) - Units: meters")

    plt.tight_layout()
    
    # Lưu ảnh vào cùng thư mục với file này
    if save_image:
        current_dir = os.path.dirname(os.path.abspath(__file__))
        output_path = os.path.join(current_dir, "sensor_array_visualization.png")
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Image saved: {output_path}")
    
    plt.show()


def save_to_csv_by_column(coords, filename="sensor_positions.csv", n_rows=4, n_cols=4):
    """
    Ghi ra file CSV theo thứ tự: cột đầu tiên từ hàng đầu đến hàng cuối,
    sau đó đến cột tiếp theo, lặp lại đến cột cuối cùng.
    """
    # Sắp xếp lại tọa độ theo thứ tự cột
    reordered_coords = reorder_coordinates_by_column(coords, n_rows, n_cols)
    
    with open(filename, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x_mm", "y_mm", "z_mm"])
        for row in reordered_coords:
            writer.writerow([value * 1000.0 for value in row])
    print(f"Saved: {os.path.abspath(filename)}")
    print("Coordinate frame: absolute robot frame; units: mm")
    
    # In thông tin về thứ tự
    print(f"Total sensors: {len(reordered_coords)}")
    print("Order: Column-major (first complete column, then next column)")


# ===== Main execution =====
coords = generate_sensor_coordinates(
    origin=(ORIGIN_X, ORIGIN_Y, ORIGIN_Z),
    dx=DX,
    dy=DY,
    n_rows=N_ROWS,
    n_cols=N_COLS,
)

print(f"Absolute array origin (m): ({ORIGIN_X:.4f}, {ORIGIN_Y:.4f}, {ORIGIN_Z:.4f})")

# In tọa độ tuyệt đối theo thứ tự tạo gốc
print("Absolute Coordinates (row-major order, mm):")
print("Index, x(mm), y(mm), z(mm)")
for i, (x, y, z) in enumerate(coords, start=1):
    print(f"{i:2d}: {x * 1000:8.1f}, {y * 1000:8.1f}, {z * 1000:8.1f}")

# In ra tọa độ sau khi sắp xếp theo cột
reordered_coords = reorder_coordinates_by_column(coords, N_ROWS, N_COLS)
print("\nSensor order (top to bottom, then left to right):")
print("New Index, Original Index, x(mm), y(mm), z(mm)")
for new_idx, (x, y, z) in enumerate(reordered_coords, start=1):
    original_idx = np.where((coords == [x, y, z]).all(axis=1))[0][0] + 1
    print(f"{new_idx:2d} (was {original_idx:2d}): {x * 1000:8.1f}, {y * 1000:8.1f}, {z * 1000:8.1f}")

# Kiểm tra thứ tự
print(f"\nColumn-major order examples:")
print(f"First column (original row-major IDs): {[i+1 for i in range(0, N_ROWS * N_COLS, N_COLS)]}")
print(f"Second column (original row-major IDs): {[i+1 for i in range(1, N_ROWS * N_COLS, N_COLS)]}")
print(f"Last column (original row-major IDs): {[i+1 for i in range(N_COLS-1, N_ROWS * N_COLS, N_COLS)]}")

# Visualize với số thứ tự từ trên xuống, rồi trái sang phải
visualize_sensor_array(reordered_coords, dx=VISUALIZE_DX, dy=VISUALIZE_DY, annotate=ANNOTATE, circle_size=CIRCLE_SIZE)

# Save to CSV theo thứ tự cột
csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Hall_sensor_positions_4x4_absolute_mm.csv")
save_to_csv_by_column(coords, csv_path, N_ROWS, N_COLS)
