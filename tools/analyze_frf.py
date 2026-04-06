#!/usr/bin/env python3
"""基于扫频注入日志估计频响（FRF）并绘制 Bode 图。

输入：build/control_wave.csv（需含 iq_inj/frf_freq_hz/wm）
输出：
1) build/frf_result.csv
2) build/frf_bode.png
"""

import argparse
import csv
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def load_csv(path: Path):
    t = []
    u = []
    y = []
    f = []
    with path.open("r", newline="") as fp:
        reader = csv.DictReader(fp)
        for row in reader:
            t.append(float(row["time_s"]))
            u.append(float(row["iq_inj"]))
            y.append(float(row["wm"]))
            f.append(float(row["frf_freq_hz"]))
    return np.asarray(t), np.asarray(u), np.asarray(y), np.asarray(f)


def estimate_frf_lockin(t, u, y, f_traj, f_points, band_hz):
    f_out = []
    mag_db = []
    phase_deg = []
    coh = []

    for f0 in f_points:
        mask = np.abs(f_traj - f0) <= band_hz
        idx = np.where(mask)[0]
        if idx.size < 40:
            continue

        tt = t[idx]
        uu = u[idx]
        yy = y[idx]

        ref = np.exp(-1j * 2.0 * np.pi * f0 * tt)
        U = np.sum(uu * ref)
        Y = np.sum(yy * ref)

        if np.abs(U) < 1.0e-8:
            continue

        H = Y / U
        # 简单相干性代理：输出与输入在该频率的复相关强度
        gamma = np.abs(np.vdot(yy, uu)) / (np.linalg.norm(yy) * np.linalg.norm(uu) + 1.0e-12)

        f_out.append(f0)
        mag_db.append(20.0 * np.log10(np.abs(H) + 1.0e-12))
        phase_deg.append(np.angle(H, deg=True))
        coh.append(gamma)

    return np.asarray(f_out), np.asarray(mag_db), np.asarray(phase_deg), np.asarray(coh)


def main():
    parser = argparse.ArgumentParser(description="Analyze FRF from control_wave.csv")
    parser.add_argument("--input", default="build/control_wave.csv")
    parser.add_argument("--output-csv", default="build/frf_result.csv")
    parser.add_argument("--output-png", default="build/frf_bode.png")
    parser.add_argument("--fmin", type=float, default=2.0)
    parser.add_argument("--fmax", type=float, default=120.0)
    parser.add_argument("--points", type=int, default=40)
    parser.add_argument("--band", type=float, default=2.0, help="frequency neighborhood half-band (Hz)")
    args = parser.parse_args()

    in_path = Path(args.input)
    if not in_path.exists():
        raise SystemExit(f"input not found: {in_path}")

    t, u, y, f = load_csv(in_path)
    if np.max(np.abs(u)) < 1.0e-6:
        raise SystemExit("iq_inj is near zero; please enable FRF sweep in app/control_main.c")

    f_points = np.logspace(np.log10(args.fmin), np.log10(args.fmax), args.points)
    f_out, mag_db, phase_deg, coh = estimate_frf_lockin(t, u, y, f, f_points, args.band)

    out_csv = Path(args.output_csv)
    out_png = Path(args.output_png)
    out_csv.parent.mkdir(parents=True, exist_ok=True)

    with out_csv.open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["freq_hz", "mag_db", "phase_deg", "coherence_proxy"])
        for row in zip(f_out, mag_db, phase_deg, coh):
            w.writerow([f"{row[0]:.6f}", f"{row[1]:.6f}", f"{row[2]:.6f}", f"{row[3]:.6f}"])

    fig, axs = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    axs[0].semilogx(f_out, mag_db)
    axs[0].set_ylabel("Magnitude (dB)")
    axs[0].set_title("FRF: wm / iq_inj")
    axs[0].grid(True, which="both")

    axs[1].semilogx(f_out, phase_deg)
    axs[1].set_ylabel("Phase (deg)")
    axs[1].grid(True, which="both")

    axs[2].semilogx(f_out, coh)
    axs[2].set_ylabel("Coherence proxy")
    axs[2].set_xlabel("Frequency (Hz)")
    axs[2].grid(True, which="both")

    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    print(f"saved: {out_csv}")
    print(f"saved: {out_png}")


if __name__ == "__main__":
    main()
