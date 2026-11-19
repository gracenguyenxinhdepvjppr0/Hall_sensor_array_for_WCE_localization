# =============================================================================
# inverse_linear.py
#
# Giải bài toán ngược 5-DOF trực tiếp từ từ trường đo:
#   - Position: x, y, z
#   - Orientation: pitch, yaw
#
# Mô hình:
#   B_pred = dipole_field(sensor_pos - magnet_pos, magnetic_moment)
#   B_sensor_pred = B_pred . sensor_dir
#   residual = B_measured - B_sensor_pred
#
# Không dùng offset/gain điện áp hoặc alpha hiệu chuẩn. Tọa độ cảm biến
# được lấy từ file tọa độ; B đầu vào phải là từ trường đã quy đổi sang Tesla.
# =============================================================================

import numpy as np
import pandas as pd
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
import time
from pathlib import Path


# =============================================================================
# PHYSICS MODEL
# =============================================================================

MU0 = 4 * np.pi * 1e-7
MU0_OVER_4PI = 1e-7

# B input is converted to Tesla before fitting.
# Use 1.0 for Tesla, 1e-3 for mT, or 1e-6 for uT.
B_FIELD_SCALE_TO_TESLA = 1.0
# Scale Tesla residuals to microtesla for better optimizer conditioning.
B_RESIDUAL_SCALE = 1e6

# Replace this example value with the magnet's known moment (A*m^2).
Br = 1.4 # Tesla
MAGNET_VOLUME_M3 = 1.57e-6 # m^3
MAGNET_DIPOLE_MOMENT_AM2 = Br * MAGNET_VOLUME_M3 / MU0

SCRIPT_DIR = Path(__file__).resolve().parent
SENSOR_POSITIONS_PATH = (
    SCRIPT_DIR.parent / "Code robot" / "Hall_sensor_positions_4x4_absolute_mm.csv"
)
DATA_DIR = Path(r"C:\Users\Admin\OneDrive\Documents\Hall_sensor_array\Hall_sensor_Thao\Code thuật toán tối ưu")
# B_meas, ground truth, kết quả path 
B_FIELD_DATA_PATH = DATA_DIR / "Helix_B_data.csv"
GT_POSES_PATH = DATA_DIR / "Helix_points_coordinates_2.csv"
ESTIMATED_POSES_PATH = SCRIPT_DIR / "estimated_poses_from_B.csv"


def compute_m_vec(pitch_rad, yaw_rad):
    mx = np.cos(pitch_rad) * np.cos(yaw_rad)
    my = np.cos(pitch_rad) * np.sin(yaw_rad)
    mz = np.sin(pitch_rad)

    if isinstance(mx, np.ndarray):
        return np.column_stack((mx, my, mz))

    return np.array([mx, my, mz])


def dipole_field(r_vec, m_vec):
    r = np.linalg.norm(r_vec, axis=-1, keepdims=True)

    r3 = np.where(r == 0, 1e-12, r**3)
    r5 = np.where(r == 0, 1e-12, r**5)

    mdotr = np.sum(m_vec * r_vec, axis=-1, keepdims=True)
    term2 = m_vec / r3

    term1 = 3.0 * r_vec * mdotr / r5

    return MU0_OVER_4PI * (term1 - term2)


# =============================================================================
# INVERSE MODEL
# =============================================================================

class InverseProblem5DOF:
    def __init__(self, B_data, sensor_positions, magnet_moment_am2):
        self.B = np.asarray(B_data, dtype=float)
        self.sensor_pos = np.asarray(sensor_positions, dtype=float)
        self.magnet_moment_am2 = float(magnet_moment_am2)

        if self.B.ndim != 2:
            raise ValueError("B data phải có dạng (số mẫu, số cảm biến).")
        if self.sensor_pos.ndim != 2 or self.sensor_pos.shape[1] != 3:
            raise ValueError("Tọa độ cảm biến phải có dạng (số cảm biến, 3).")
        if self.B.shape[1] != self.sensor_pos.shape[0]:
            raise ValueError(
                f"B data có {self.B.shape[1]} kênh nhưng tọa độ có "
                f"{self.sensor_pos.shape[0]} cảm biến."
            )
        if self.magnet_moment_am2 <= 0:
            raise ValueError("Moment nam châm phải lớn hơn 0 A*m^2.")

        self.N = self.sensor_pos.shape[0]
        self.M = self.B.shape[0]

        # Hướng nhạy của mỗi cảm biến được giả định cố định theo +Z.
        self.sensor_dir = np.tile(np.array([0.0, 0.0, 1.0]), (self.N, 1))

        print("\n=== Loaded magnetic-field model ===")
        print(f"Number of sensors       : {self.N}")
        print(f"Number of B samples     : {self.M}")
        print(f"Magnet moment           : {self.magnet_moment_am2:.6g} A*m^2")
        print("Sensor direction        : [0, 0, 1]")

    def residuals_single_position(self, state, measured_B):
        """
        state = [x, y, z, pitch, yaw]

        All position quantities are in meters.
        Angles are in radians.
        measured_B contains one B component per sensor, in Tesla.
        """

        magnet_pos = state[:3]
        pitch = state[3]
        yaw = state[4]

        # ---------------------------------------------------------
        # 1. Magnet orientation
        # ---------------------------------------------------------
        m_vec = self.magnet_moment_am2 * compute_m_vec(pitch, yaw)

        # ---------------------------------------------------------
        # 2. Magnetic field at every sensor position
        # ---------------------------------------------------------
        r_vec = self.sensor_pos - magnet_pos

        B_pred_3D = dipole_field(r_vec, m_vec)

        # Project onto fixed Hall sensor direction [0,0,1]
        B_proj = np.sum(
            B_pred_3D * self.sensor_dir,
            axis=-1
        )

        # ---------------------------------------------------------
        # Scale both fields equally for numerical conditioning; the minimum is unchanged.
        return B_RESIDUAL_SCALE * (measured_B - B_proj)


# =============================================================================
# LOAD INPUT DATA
# =============================================================================

def load_sensor_positions(file_path):
    """Load absolute sensor coordinates and return them in meters.

    Accepts either x_mm/y_mm/z_mm columns or x/y/z columns already in meters.
    """
    df = pd.read_csv(file_path, encoding="latin-1")
    df.columns = df.columns.str.strip().str.lower()

    if {"x_mm", "y_mm", "z_mm"}.issubset(df.columns):
        positions = df[["x_mm", "y_mm", "z_mm"]].to_numpy(dtype=float) / 1000.0
    elif {"x", "y", "z"}.issubset(df.columns):
        positions = df[["x", "y", "z"]].to_numpy(dtype=float)
    else:
        raise KeyError(
            "Tọa độ cảm biến cần các cột x_mm,y_mm,z_mm (mm) "
            "hoặc x,y,z (m). "
            f"Các cột hiện có: {df.columns.tolist()}"
        )

    if not np.all(np.isfinite(positions)):
        raise ValueError("File tọa độ cảm biến có giá trị không phải số hữu hạn.")

    return positions


def load_ground_truth_poses(file_path):
    """Load ground-truth x,y,z (meters) and pitch,yaw (degrees)."""
    df = pd.read_csv(
        file_path,
        sep=None,
        engine="python"
    )
    df.columns = df.columns.str.strip()

    required = ["x", "y", "z", "pitch", "yaw"]
    missing = [col for col in required if col not in df.columns]
    if missing:
        raise KeyError(
            f"Ground truth thiếu các cột {missing}; "
            f"các cột hiện có: {df.columns.tolist()}"
        )

    positions = df[["x", "y", "z"]].to_numpy(dtype=float)
    angles_deg = df[["pitch", "yaw"]].to_numpy(dtype=float)
    return positions, angles_deg


def load_magnetic_field_data(csv_path, scale_to_tesla=1.0):
    """Load B measurements with one numeric column per sensor, in Tesla."""
    df = pd.read_csv(csv_path, encoding="latin-1")
    df.columns = df.columns.str.strip()

    try:
        B_data = df.apply(pd.to_numeric, errors="raise").to_numpy(dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            "B CSV phải chỉ có các cột số của cảm biến; hãy bỏ cột ID/thời gian."
        ) from exc

    if B_data.ndim != 2 or B_data.shape[1] == 0:
        raise ValueError("B CSV phải có dạng (số mẫu, số cảm biến).")
    if not np.all(np.isfinite(B_data)):
        raise ValueError("B CSV có giá trị không phải số hữu hạn.")

    return B_data * float(scale_to_tesla)


# =============================================================================
# ANGLE HELPERS
# =============================================================================

def wrap_angle_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def angle_diff_deg(a, b):
    return wrap_angle_deg(a - b)


# =============================================================================
# SOLVER
# =============================================================================

def solve_inverse_problem_5DOF(
    B_data,
    inverse_model,
    gt_positions,
    gt_angles_deg
):
    """
    Solve every measurement independently.

    CURRENT VERSION:
        Uses ground truth as initial guess.

    This is intentionally kept to isolate model errors from
    initialization/symmetry errors.

    For REAL localization later, replace initial_guess by a fixed/previous-state
    estimate.
    """

    M = B_data.shape[0]

    estimated_states = np.zeros(
        (M, 5)
    )

    for pos_idx in range(M):

        measured_B = B_data[pos_idx, :]

        # ---------------------------------------------------------
        # GT initial guess
        # ---------------------------------------------------------
        initial_guess = np.array([
            gt_positions[pos_idx, 0],
            gt_positions[pos_idx, 1],
            gt_positions[pos_idx, 2],
            np.radians(gt_angles_deg[pos_idx, 0]),
            np.radians(gt_angles_deg[pos_idx, 1])
        ])

        def residuals_func(state):
            return inverse_model.residuals_single_position(
                state,
                measured_B
            )

        result = least_squares(
            residuals_func,
            initial_guess,
            method="lm",
            max_nfev=5000
        )

        estimated_states[pos_idx] = result.x

    return estimated_states


# =============================================================================
# PLOTS
# =============================================================================

def plot_rmse_metrics(
    pos_err,
    pitch_err,
    yaw_err
):
    pos_rmse = (
        np.sqrt(
            np.mean(
                pos_err ** 2,
                axis=0
            )
        ) * 1000
    )

    total_pos_rmse = (
        np.sqrt(
            np.mean(
                np.linalg.norm(
                    pos_err,
                    axis=1
                ) ** 2
            )
        ) * 1000
    )

    pitch_rmse = np.sqrt(
        np.mean(pitch_err ** 2)
    )

    yaw_rmse = np.sqrt(
        np.mean(yaw_err ** 2)
    )

    print("\n=== RMSE ===")
    print("Position RMSE X/Y/Z (mm):", pos_rmse)
    print(f"Position RMSE Total      : {total_pos_rmse:.3f} mm")
    print(f"Pitch RMSE               : {pitch_rmse:.3f} deg")
    print(f"Yaw RMSE                 : {yaw_rmse:.3f} deg")

    fig, (ax1, ax2) = plt.subplots(
        1,
        2,
        figsize=(12, 5)
    )

    labels_pos = [
        "X",
        "Y",
        "Z",
        "Total"
    ]

    values_pos = [
        pos_rmse[0],
        pos_rmse[1],
        pos_rmse[2],
        total_pos_rmse
    ]

    bars1 = ax1.bar(
        labels_pos,
        values_pos
    )

    ax1.set_title(
        "Position RMSE (mm)"
    )

    ax1.set_ylabel(
        "Error (mm)"
    )

    for bar in bars1:
        yval = bar.get_height()

        ax1.text(
            bar.get_x()
            + bar.get_width() / 2,
            yval + 0.1,
            f"{yval:.2f}",
            ha="center",
            va="bottom"
        )

    labels_ang = [
        "Pitch",
        "Yaw"
    ]

    values_ang = [
        pitch_rmse,
        yaw_rmse
    ]

    bars2 = ax2.bar(
        labels_ang,
        values_ang,
        width=0.5
    )

    ax2.set_title(
        "Angular RMSE (Degrees)"
    )

    ax2.set_ylabel(
        "Error (deg)"
    )

    for bar in bars2:
        yval = bar.get_height()

        ax2.text(
            bar.get_x()
            + bar.get_width() / 2,
            yval + 0.1,
            f"{yval:.2f}",
            ha="center",
            va="bottom"
        )

    plt.tight_layout()
    plt.show()


def plot_pointwise_errors(
    pos_err,
    pitch_err,
    yaw_err
):
    idx = np.arange(
        len(pos_err)
    )

    fig, (ax1, ax2) = plt.subplots(
        2,
        1,
        figsize=(12, 8),
        sharex=True
    )

    ax1.plot(
        idx,
        np.abs(pos_err[:, 0]) * 1000,
        "--",
        alpha=0.6,
        label="|Ex|"
    )

    ax1.plot(
        idx,
        np.abs(pos_err[:, 1]) * 1000,
        "--",
        alpha=0.6,
        label="|Ey|"
    )

    ax1.plot(
        idx,
        np.abs(pos_err[:, 2]) * 1000,
        "--",
        alpha=0.6,
        label="|Ez|"
    )

    ax1.plot(
        idx,
        np.linalg.norm(
            pos_err,
            axis=1
        ) * 1000,
        "-",
        linewidth=2,
        label="Total Pos Error"
    )

    ax1.set_ylabel(
        "Position Error (mm)"
    )

    ax1.set_title(
        "Absolute Position Errors"
    )

    ax1.legend()
    ax1.grid(
        True,
        alpha=0.3
    )

    ax2.plot(
        idx,
        np.abs(pitch_err),
        "-",
        linewidth=2,
        label="|E_pitch|"
    )

    ax2.plot(
        idx,
        np.abs(yaw_err),
        "-",
        linewidth=2,
        label="|E_yaw|"
    )

    ax2.set_ylabel(
        "Angular Error (deg)"
    )

    ax2.set_xlabel(
        "Point Index"
    )

    ax2.set_title(
        "Absolute Angular Errors"
    )

    ax2.legend()

    ax2.grid(
        True,
        alpha=0.3
    )

    plt.tight_layout()
    plt.show()


def plot_trajectory_5dof(
    gt_pos,
    est_pos,
    gt_angles_deg,
    est_angles_deg
):
    fig = plt.figure(
        figsize=(12, 10)
    )

    ax = fig.add_subplot(
        111,
        projection="3d"
    )

    gt_mm = gt_pos * 1000
    est_mm = est_pos * 1000

    ax.plot(
        gt_mm[:, 0],
        gt_mm[:, 1],
        gt_mm[:, 2],
        label="Ground Truth"
    )

    ax.scatter(
        gt_mm[:, 0],
        gt_mm[:, 1],
        gt_mm[:, 2],
        s=20
    )

    ax.plot(
        est_mm[:, 0],
        est_mm[:, 1],
        est_mm[:, 2],
        "--",
        label="Estimated"
    )

    ax.scatter(
        est_mm[:, 0],
        est_mm[:, 1],
        est_mm[:, 2],
        s=20
    )

    step = max(
        1,
        len(gt_pos) // 20
    )

    gt_rad = np.radians(
        gt_angles_deg
    )

    m_gt = compute_m_vec(
        gt_rad[:, 0],
        gt_rad[:, 1]
    )

    est_rad = np.radians(
        est_angles_deg
    )

    m_est = compute_m_vec(
        est_rad[:, 0],
        est_rad[:, 1]
    )

    arrow_length = 5.0

    ax.quiver(
        gt_mm[::step, 0],
        gt_mm[::step, 1],
        gt_mm[::step, 2],
        m_gt[::step, 0],
        m_gt[::step, 1],
        m_gt[::step, 2],
        length=arrow_length,
        alpha=0.6,
        label="GT Orientation"
    )

    ax.quiver(
        est_mm[::step, 0],
        est_mm[::step, 1],
        est_mm[::step, 2],
        m_est[::step, 0],
        m_est[::step, 1],
        m_est[::step, 2],
        length=arrow_length,
        alpha=0.6,
        label="Est Orientation"
    )

    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")

    ax.set_title(
        "5-DOF Trajectory & Orientation"
    )

    ax.legend()

    ax.set_box_aspect(
        [1, 1, 1]
    )

    plt.tight_layout()
    plt.show()


# =============================================================================
# MAIN
# =============================================================================

def main():
    try:
        print("\n========================================")
        print(" Loading sensor coordinates and B data")
        print("========================================")

        sensor_positions = load_sensor_positions(SENSOR_POSITIONS_PATH)
        B_data = load_magnetic_field_data(
            B_FIELD_DATA_PATH,
            scale_to_tesla=B_FIELD_SCALE_TO_TESLA
        )
        gt_positions, gt_angles_deg = load_ground_truth_poses(GT_POSES_PATH)

        # ---------------------------------------------------------------------
        # CHECK DIMENSIONS
        # ---------------------------------------------------------------------

        if B_data.shape[1] != sensor_positions.shape[0]:
            raise ValueError(
                f"B data có {B_data.shape[1]} kênh nhưng file tọa độ có "
                f"{sensor_positions.shape[0]} cảm biến."
            )

        min_points = min(
            B_data.shape[0],
            gt_positions.shape[0],
            gt_angles_deg.shape[0]
        )

        if min_points == 0:
            raise ValueError("Không có mẫu B/ground truth để tối ưu.")

        B_data = B_data[:min_points]
        gt_positions = gt_positions[:min_points]
        gt_angles_deg = gt_angles_deg[:min_points]

        # ---------------------------------------------------------------------
        # CREATE INVERSE MODEL
        # ---------------------------------------------------------------------

        model = InverseProblem5DOF(
            B_data=B_data,
            sensor_positions=sensor_positions,
            magnet_moment_am2=MAGNET_DIPOLE_MOMENT_AM2
        )

        # ---------------------------------------------------------------------
        # SOLVE
        # ---------------------------------------------------------------------

        print("\n========================================")
        print(" Solving inverse problem")
        print(" Using GT initial guess")
        print("========================================")

        start_time = time.perf_counter()

        est_states = solve_inverse_problem_5DOF(
            B_data,
            model,
            gt_positions,
            gt_angles_deg
        )

        end_time = time.perf_counter()

        total_time = end_time - start_time

        print(
            f"\nTotal optimization time: "
            f"{total_time:.4f} s"
        )

        print(
            f"Average time / point: "
            f"{total_time / min_points * 1000:.4f} ms"
        )

        # ---------------------------------------------------------------------
        # EXTRACT RESULT
        # ---------------------------------------------------------------------

        est_positions = est_states[:, :3]

        est_angles_rad = est_states[:, 3:]

        est_angles_deg = np.degrees(
            est_angles_rad
        )

        est_angles_deg[:, 0] = wrap_angle_deg(
            est_angles_deg[:, 0]
        )

        est_angles_deg[:, 1] = wrap_angle_deg(
            est_angles_deg[:, 1]
        )

        # ---------------------------------------------------------------------
        # ERROR
        # ---------------------------------------------------------------------

        pos_err = (
            est_positions
            - gt_positions
        )

        pitch_err = angle_diff_deg(
            est_angles_deg[:, 0],
            gt_angles_deg[:, 0]
        )

        yaw_err = angle_diff_deg(
            est_angles_deg[:, 1],
            gt_angles_deg[:, 1]
        )

        # ---------------------------------------------------------------------
        # RMSE
        # ---------------------------------------------------------------------

        pos_rmse_mm = (
            np.sqrt(
                np.mean(
                    np.linalg.norm(
                        pos_err,
                        axis=1
                    ) ** 2
                )
            ) * 1000
        )

        pitch_rmse = np.sqrt(
            np.mean(
                pitch_err ** 2
            )
        )

        yaw_rmse = np.sqrt(
            np.mean(
                yaw_err ** 2
            )
        )

        print("\n========================================")
        print(" FINAL RESULTS")
        print("========================================")

        print(
            f"Position RMSE : "
            f"{pos_rmse_mm:.3f} mm"
        )

        print(
            f"Pitch RMSE    : "
            f"{pitch_rmse:.3f} deg"
        )

        print(
            f"Yaw RMSE      : "
            f"{yaw_rmse:.3f} deg"
        )

        # ---------------------------------------------------------------------
        # ESTIMATED RANGES
        # ---------------------------------------------------------------------

        print("\n=== Estimated ranges ===")

        print(
            f"Position X: "
            f"{est_positions[:,0].min():.4f} "
            f"to "
            f"{est_positions[:,0].max():.4f} m"
        )

        print(
            f"Position Y: "
            f"{est_positions[:,1].min():.4f} "
            f"to "
            f"{est_positions[:,1].max():.4f} m"
        )

        print(
            f"Position Z: "
            f"{est_positions[:,2].min():.4f} "
            f"to "
            f"{est_positions[:,2].max():.4f} m"
        )

        print(
            f"Pitch: "
            f"{est_angles_deg[:,0].min():.2f} "
            f"to "
            f"{est_angles_deg[:,0].max():.2f} deg"
        )

        print(
            f"Yaw: "
            f"{est_angles_deg[:,1].min():.2f} "
            f"to "
            f"{est_angles_deg[:,1].max():.2f} deg"
        )

        # ---------------------------------------------------------------------
        # PLOTS
        # ---------------------------------------------------------------------

        print("\n--- Generating plots ---")

        plot_rmse_metrics(
            pos_err,
            pitch_err,
            yaw_err
        )

        plot_pointwise_errors(
            pos_err,
            pitch_err,
            yaw_err
        )

        plot_trajectory_5dof(
            gt_positions,
            est_positions,
            gt_angles_deg,
            est_angles_deg
        )

        # ---------------------------------------------------------------------
        # SAVE ESTIMATED POSES
        # ---------------------------------------------------------------------

        output = np.column_stack([
            est_positions,
            est_angles_deg
        ])

        np.savetxt(
            ESTIMATED_POSES_PATH,
            output,
            delimiter=",",
            header="x,y,z,pitch,yaw",
            comments=""
        )

        print(
            f"\nSaved: {ESTIMATED_POSES_PATH}"
        )

        print("\nDone.")

    except Exception as e:

        print(
            f"\n*** ERROR: {e}"
        )


if __name__ == "__main__":
    main()
