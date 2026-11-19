"""将第 23 课 C++ 保存的两组模型反馈和理论曲线画到同一张图。"""

import argparse
import csv
from pathlib import Path


def read_rows(path):
    # 首帧只有位置，第二帧才有速度；保留缺测，不把它补成零加速度。
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def series(rows, t_column, value_column):
    pairs = [(float(r[t_column]), float(r[value_column])) for r in rows if r[value_column]]
    return zip(*pairs, strict=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    parser.add_argument("--show", action="store_true", help="同时在 WSL 图形窗口打开")
    args = parser.parse_args()
    import matplotlib

    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    fig, axes = plt.subplots(2, 2, figsize=(11, 6.2), sharex="col", sharey="row")
    colors = {"trapezoid": "#11679c", "smooth": "#bf5310"}
    for name, color in colors.items():
        reference = read_rows(args.folder / f"{name}_reference.csv")
        feedback = read_rows(args.folder / f"{name}_feedback.csv")
        for row, (clock, column) in enumerate((("t_v_s", "v_mm_s"), ("t_a_s", "a_mm_s2"))):
            for ax in axes[row]:
                ts, values = series(reference, "t_s", column)
                ax.plot(
                    ts,
                    values,
                    color=color,
                    linewidth=1.7,
                    drawstyle="steps-post" if name == "trapezoid" and row == 1 else "default",
                )
                ts, values = series(feedback, clock, column)
                ax.plot(ts, values, ".", color=color, markersize=3.4, alpha=0.8)

    for ax in axes.flat:
        ax.grid(alpha=0.2)
    axes[0, 0].set_title("Full 140 mm traverse")
    axes[0, 1].set_title("First 0.70 s")
    axes[0, 1].set_xlim(0, 0.70)
    axes[1, 0].set_xlabel("Time from left endpoint (s)")
    axes[1, 1].set_xlabel("Time from left endpoint (s)")
    axes[0, 0].set_ylabel("X velocity (mm/s)")
    axes[1, 0].set_ylabel("X acceleration (mm/s²)")
    handles = [Line2D([], [], color=c, label=n.capitalize()) for n, c in colors.items()]
    handles += [
        Line2D([], [], color="0.25", label="Analytical"),
        Line2D([], [], color="0.25", linestyle="none", marker=".", label="Model feedback"),
    ]
    fig.legend(handles=handles, loc="upper center", ncol=4, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    output = args.folder / "speed_acceleration.png"
    fig.savefig(output, dpi=160)
    print(output)
    if args.show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
