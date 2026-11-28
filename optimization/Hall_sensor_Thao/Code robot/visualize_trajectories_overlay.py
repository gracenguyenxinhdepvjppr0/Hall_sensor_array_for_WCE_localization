"""Plot Grid, Helix, Lissajous, and Random paths on one 3D coordinate system.

Run from this folder with:
    python visualize_trajectories_overlay.py

The figure is saved beside this file as ``trajectory_overlay.png`` and shown
in a Matplotlib window. No robot or UART connection is opened.
"""

import argparse
from pathlib import Path

from visualize_trajectories import SCRIPT_DIR, make_trajectories


def plot_trajectories_overlay(trajectories, output_path, seed, show=True):
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "Missing Matplotlib. Install it with: python -m pip install matplotlib"
        ) from exc

    figure = plt.figure(figsize=(12, 9))
    axis = figure.add_subplot(1, 1, 1, projection="3d")
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    all_points = []
    path_artists = []

    for index, (name, path) in enumerate(trajectories):
        if not path:
            raise ValueError(f"Trajectory '{name}' contains no points")

        points = [tuple(point) for point in path]
        xs, ys, zs = zip(*points)
        color = colors[index % len(colors)]
        line, = axis.plot(
            xs, ys, zs,
            color=color,
            linewidth=1.2,
            marker=".",
            markersize=2,
            alpha=0.85,
            label=f"{name} ({len(points)} points)",
        )
        start_marker = axis.scatter(
            xs[0], ys[0], zs[0], color=color, marker="^", s=48,
            edgecolors="black", linewidths=0.5, depthshade=False,
        )
        end_marker = axis.scatter(
            xs[-1], ys[-1], zs[-1], color=color, marker="s", s=42,
            edgecolors="black", linewidths=0.5, depthshade=False,
        )
        path_artists.append((line, start_marker, end_marker))
        all_points.extend(points)

    if len(trajectories) == 1:
        title = f"{trajectories[0][0]} trajectory (seed {seed})"
    else:
        title = f"Four robot trajectories in one 3D workspace (seed {seed})"
    axis.set_title(title)
    axis.set_xlabel("X (mm)")
    axis.set_ylabel("Y (mm)")
    axis.set_zlabel("Z (mm)")
    axis.view_init(elev=25, azim=-55)
    axis.grid(True, alpha=0.35)
    legend = axis.legend(
        loc="best", fontsize=9, title="Click a path name to show/hide"
    )
    for index, (legend_line, artists) in enumerate(
        zip(legend.get_lines(), path_artists)
    ):
        legend_line.set_picker(6)
        legend_line._trajectory_artists = artists
        legend_line._trajectory_legend_text = legend.get_texts()[index]

    def toggle_trajectory(event):
        legend_line = event.artist
        artists = getattr(legend_line, "_trajectory_artists", None)
        if artists is None:
            return

        visible = not artists[0].get_visible()
        for artist in artists:
            artist.set_visible(visible)
        legend_line.set_alpha(1.0 if visible else 0.25)
        legend_line._trajectory_legend_text.set_alpha(1.0 if visible else 0.45)
        figure.canvas.draw_idle()

    figure.canvas.mpl_connect("pick_event", toggle_trajectory)

    spans = tuple(
        max(point[dimension] for point in all_points)
        - min(point[dimension] for point in all_points)
        for dimension in range(3)
    )
    axis.set_box_aspect(tuple(max(span, 1.0) for span in spans))

    figure.tight_layout()
    output_path = Path(output_path).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180, bbox_inches="tight")
    print(f"Saved combined trajectory figure: {output_path}")

    if show:
        plt.show()
    else:
        plt.close(figure)


def main():
    parser = argparse.ArgumentParser(
        description="Overlay Grid, Helix, Lissajous, and Random paths on one 3D axis."
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="PNG output path (default name depends on --trajectory)",
    )
    parser.add_argument(
        "--trajectory",
        choices=("all", "grid", "helix", "lissajous", "random"),
        default="all",
        help="Choose one path to view, or 'all' to overlay the four paths",
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

    trajectories = make_trajectories(args.seed)
    selected_name = {
        "grid": "Grid",
        "helix": "Helix",
        "lissajous": "Lissajous",
        "random": "Random",
    }
    if args.trajectory != "all":
        wanted_name = selected_name[args.trajectory]
        trajectories = [
            (name, path) for name, path in trajectories if name == wanted_name
        ]

    if args.output is None:
        output_name = (
            "trajectory_overlay.png"
            if args.trajectory == "all"
            else f"trajectory_{args.trajectory}.png"
        )
        output_path = SCRIPT_DIR / output_name
    else:
        output_path = args.output

    plot_trajectories_overlay(
        trajectories,
        output_path,
        args.seed,
        show=not args.no_show,
    )


if __name__ == "__main__":
    main()
