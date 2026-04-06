#!/usr/bin/env python3
"""控制波形绘图工具：
1) 读取 build/control_wave.csv
2) 生成多子图 PNG
3) 可选 GUI 窗口显示
"""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def read_csv(path: Path):
    data = {
        "time_s": [],
        "wm": [],
        "omega_ref": [],
        "theta_ref": [],
        "theta_mech": [],
        "epos_live": [],
        "iq_ref": [],
        "id_ref": [],
        "i_alpha": [],
        "i_beta": [],
        "vdc": [],
        "rs": [],
        "mpcJ": [],
        "fw": [],
    }
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            data["time_s"].append(float(row["time_s"]))
            data["wm"].append(float(row["wm"]))
            data["omega_ref"].append(float(row["omega_ref"]))
            data["theta_ref"].append(float(row["theta_ref"]))
            data["theta_mech"].append(float(row["theta_mech"]))
            data["epos_live"].append(float(row["epos_live"]))
            data["iq_ref"].append(float(row["iq_ref"]))
            data["id_ref"].append(float(row["id_ref"]))
            data["i_alpha"].append(float(row["i_alpha"]))
            data["i_beta"].append(float(row["i_beta"]))
            data["vdc"].append(float(row["vdc"]))
            data["rs"].append(float(row["rs"]))
            data["mpcJ"].append(float(row["mpcJ"]))
            data["fw"].append(float(row["fw"]))
    return data


def main():
    parser = argparse.ArgumentParser(description="Plot MPC control waveform CSV")
    parser.add_argument("--input", default="build/control_wave.csv", help="input CSV path")
    parser.add_argument("--save", default="build/control_wave.png", help="output PNG path")
    parser.add_argument("--show", action="store_true", help="show GUI window")
    args = parser.parse_args()

    in_path = Path(args.input)
    out_path = Path(args.save)

    if not in_path.exists():
        raise SystemExit(f"input not found: {in_path}")

    d = read_csv(in_path)
    t = d["time_s"]

    fig, axs = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

    axs[0].plot(t, d["wm"], label="omega_mech")
    axs[0].plot(t, d["omega_ref"], label="omega_ref", linestyle="--")
    axs[0].set_ylabel("rad/s")
    axs[0].set_title("Speed")
    axs[0].grid(True)
    axs[0].legend()

    axs[1].plot(t, d["theta_mech"], label="theta_mech")
    axs[1].plot(t, d["theta_ref"], label="theta_ref", linestyle="--")
    axs[1].plot(t, d["epos_live"], label="e_pos")
    axs[1].set_ylabel("rad")
    axs[1].set_title("Position")
    axs[1].grid(True)
    axs[1].legend()

    axs[2].plot(t, d["iq_ref"], label="iq_ref")
    axs[2].plot(t, d["id_ref"], label="id_ref")
    axs[2].plot(t, d["i_alpha"], label="i_alpha")
    axs[2].plot(t, d["i_beta"], label="i_beta")
    axs[2].set_ylabel("A")
    axs[2].set_title("Current")
    axs[2].grid(True)
    axs[2].legend()

    axs[3].plot(t, d["vdc"], label="vdc")
    axs[3].plot(t, d["rs"], label="Rs")
    axs[3].plot(t, d["mpcJ"], label="MPC cost")
    axs[3].plot(t, d["fw"], label="fw_flag", linestyle=":")
    axs[3].set_xlabel("time (s)")
    axs[3].set_title("Bus/Param/Cost")
    axs[3].grid(True)
    axs[3].legend()

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    print(f"saved plot: {out_path}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
