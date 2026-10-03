"""Render a README chart from a completed, clean release-bundle comparison.

Requires matplotlib; does not run as part of normal mod builds.
"""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

SCENARIOS = ["idle", "supported64", "supported256", "active64", "edits64"]
LABELS = ["Idle", "64 settled bodies", "256 settled bodies", "64 active bodies", "64 bodies + terrain edits"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--version", required=True)
    args = parser.parse_args()
    data = json.loads(args.result.read_text())
    if not (data.get("pass_") and data.get("publicationEligible")
            and not data.get("profiled") and data.get("sourceDirty") is False
            and data.get("nativeMode") == "checked-in-release-bundle"):
        parser.error("Chart requires a successful, clean, unprofiled release-bundle comparison")
    rows = {(row["scenario"], row["engine"]): row for row in data["summary"]}
    if set(rows) != {(scenario, engine) for scenario in SCENARIOS for engine in ("sable", "worldengine")}:
        parser.error("All five paired workloads are required")
    for row in rows.values():
        if row["trials"] < 5:
            parser.error("At least five paired trials are required")
    addon_runs = [run for run in data["runs"] if run["engine"] == "worldengine"]
    if not addon_runs or any(f"worldengine:{args.version}" not in run["mods"] for run in addon_runs):
        parser.error("The chart version must match every measured World Engine runtime")

    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11,
                         "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(1, 3, figsize=(16.8, 7.0), sharey=True)
    fig.patch.set_facecolor("#ffffff")
    metrics = [("tickMs", "Median server tick", "ms", 1.0, 3),
               ("processCpuMsPerTick", "Process CPU per tick", "ms", 1.0, 2),
               ("serverThreadAllocatedBytes", "Java allocation per tick", "KiB", 1024.0, 1)]
    for axis, (metric, title, unit, scale, digits) in zip(axes, metrics):
        highest = 100.0
        for index, scenario in enumerate(SCENARIOS):
            baseline = rows[(scenario, "sable")][metric]["median"]
            addon = rows[(scenario, "worldengine")][metric]["median"]
            relative = addon / baseline * 100
            highest = max(highest, relative)
            axis.barh(index - 0.17, 100, height=0.29, color="#94a3b8", zorder=3)
            axis.barh(index + 0.17, relative, height=0.29,
                      color="#137b66" if relative <= 100 else "#bb403d", zorder=3)
            axis.text(103, index - 0.17, f"{baseline / scale:.{digits}f} {unit}", va="center", fontsize=10)
            axis.text(relative + 3, index + 0.17,
                      f"{addon / scale:.{digits}f} {unit} ({relative - 100:+.0f}%)", va="center", fontsize=10)
        axis.set_title(title, fontsize=15, fontweight="bold", pad=18)
        axis.set_xlim(0, max(170, highest + 75))
        axis.set_xticks([0, 50, 100, 150], ["0%", "50%", "100%", "150%"])
        axis.grid(axis="x", color="#e7ebf0", zorder=0)
        axis.axvline(100, color="#738198", linestyle="--", linewidth=0.8, zorder=1)
        axis.set_xlabel("Relative to stock Sable", labelpad=12)
        axis.spines["left"].set_visible(False)
        axis.spines["bottom"].set_color("#d4dae2")
        axis.tick_params(axis="y", length=0)
    axes[0].set_yticks(range(len(SCENARIOS)), LABELS, fontsize=12)
    axes[0].invert_yaxis()
    fig.suptitle(f"World Engine {args.version} vs stock Sable 2.0.5", x=0.51, y=0.965,
                 fontsize=23, fontweight="bold", color="#172b42")
    fig.text(0.51, 0.905, "Lower is better · Each workload's stock Sable result is 100%", ha="center", fontsize=13)
    fig.legend(handles=[Patch(color="#94a3b8", label="Stock Sable"),
                        Patch(color="#137b66", label=f"World Engine {args.version}")],
               loc="upper center", bbox_to_anchor=(0.52, 0.88), ncol=2, frameon=False, fontsize=12)
    warmup = sorted({run["warmupTicks"] for run in data["runs"]})
    ticks = sorted({run["measuredTicks"] for run in data["runs"]})
    fig.text(0.5, 0.085, f"Minecraft 1.21.1 / NeoForge · {rows[('idle', 'sable')]['trials']} paired trials · "
             f"{warmup[0]} warmup + {ticks[0]} measured ticks per fresh JVM", ha="center", fontsize=10, color="#42566d")
    fig.text(0.5, 0.055, "Medians across trials; CPU includes native workers. Allocation is server-thread bytes, not peak RAM.",
             ha="center", fontsize=10, color="#42566d")
    fig.text(0.5, 0.025, f"Release bundle · source {data['sourceHead'][:12]} · evidence {data['runId']}",
             ha="center", fontsize=9, color="#65758a")
    fig.subplots_adjust(left=0.16, right=0.985, top=0.76, bottom=0.20, wspace=0.20)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160, facecolor=fig.get_facecolor())
    plt.close(fig)


if __name__ == "__main__":
    main()
