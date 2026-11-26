"""Plot the four robot paths defined by the trajectory collection scripts.

Run from this folder with:
    python visualize_trajectories.py

The figure is saved beside this file as ``trajectory_visualization.png`` and
shown in a Matplotlib window. No robot or UART connection is opened.
"""

import argparse
import importlib.util
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_CANDIDATES = (
    SCRIPT_DIR,
    SCRIPT_DIR / "RobotArm_CollectData" / "RobotArm_CollectData",
)
TRAJECTORY_DIR = next(
    (directory for directory in PROJECT_CANDIDATES
     if (directory / "robot_ABB_grid.py").is_file()),
    SCRIPT_DIR,
)
if str(TRAJECTORY_DIR) not in sys.path:
    sys.path.insert(0, str(TRAJECTORY_DIR))


def load_module(file_name, module_name):
    """Load a trajectory script without executing its hardware entry point."""
    source = TRAJECTORY_DIR / file_name
    spec = importlib.util.spec_from_file_location(module_name, source)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load trajectory source: {source}")

    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def make_trajectories(seed):
    """Use the same path generators and settings as the collection scripts."""
    import numpy as np

    np.random.seed(seed)

    grid = load_module("robot_ABB_grid.py", "trajectory_grid")
    helix = load_module("robot_ABB_helical.py", "trajectory_helix")
    lissajous = load_module(
        "robot_ABB_Lissajous Curve.py", "trajectory_lissajous"
    )
    random_path = load_module("robot_ABB_random.py", "trajectory_random")

    return [
        ("Grid", grid.generate_3d_grid()),
        ("Helix", helix.generate_helical_path()),
        ("Lissajous", lissajous.generate_lissajous_path()),
        ("Random", random_path.generate_random_points()),
    ]


def plot_trajectories(trajectories, output_path, seed, show=True):
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "Missing Matplotlib. Install it with: python -m pip install matplotlib"
        ) from exc

    figure = plt.figure(figsize=(13, 10))
    figure.suptitle(f"Robot trajectory visualization (random seed: {seed})")

    for index, (name, path) in enumerate(trajectories, start=1):
        if not path:
            raise ValueError(f"Trajectory '{name}' contains no points")

        points = [tuple(point) for point in path]
        xs, ys, zs = zip(*points)
        axis = figure.add_subplot(2, 2, index, projection="3d")

        line, = axis.plot(xs, ys, zs, linewidth=1.4, label="Path")
        axis.scatter(xs, ys, zs, color=line.get_color(), s=8, alpha=0.55)
        axis.scatter(
            xs[0], ys[0], zs[0], color="green", marker="^", s=55,
            edgecolors="black", linewidths=0.5, label="Start", zorder=3,
        )
        axis.scatter(
            xs[-1], ys[-1], zs[-1], color="red", marker="s", s=45,
            edgecolors="black", linewidths=0.5, label="End", zorder=3,
        )

        axis.set_title(f"{name} ({len(points)} points)")
        axis.set_xlabel("X (mm)")
        axis.set_ylabel("Y (mm)")
        axis.set_zlabel("Z (mm)")
        axis.view_init(elev=25, azim=-55)
        axis.legend(loc="best", fontsize=8)

        spans = (
            max(xs) - min(xs),
            max(ys) - min(ys),
            max(zs) - min(zs),
        )
        axis.set_box_aspect(tuple(max(span, 1.0) for span in spans))

    figure.tight_layout(rect=(0, 0, 1, 0.96))
    output_path = Path(output_path).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180, bbox_inches="tight")
    print(f"Saved trajectory figure: {output_path}")

    if show:
        plt.show()
    else:
        plt.close(figure)


def main():
    parser = argparse.ArgumentParser(
        description="Visualize Grid, Helix, Lissajous, and Random trajectories."
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=SCRIPT_DIR / "trajectory_visualization.png",
        help="PNG output path (default: trajectory_visualization.png beside this file)",
    )
    parser.add_argument(
        "--seed", type=int, default=42,
        help="Seed used to make the random path repeatable (default: 42)",
    )
    parser.add_argument(
        "--no-show", action="store_true",
        help="Save the PNG without opening a plot window",
    )
    args = parser.parse_args()

    paths = make_trajectories(args.seed)
    plot_trajectories(paths, args.output, args.seed, show=not args.no_show)


if __name__ == "__main__":
    main()
