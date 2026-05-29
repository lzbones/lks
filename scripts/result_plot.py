#!/usr/bin/env python3
"""可视化 LKS 纯跟踪仿真结果"""

import numpy as np
import matplotlib.pyplot as plt

# ---------- 读取数据 ----------
lane = np.genfromtxt("lane.csv", delimiter=",")
traj = np.genfromtxt("trajectory.csv", delimiter=",", skip_header=1)

lane_x, lane_y = lane[:, 0], lane[:, 1]
step, x, y, v, delta, cross_err = traj[:, 0], traj[:, 1], traj[:, 2], \
                                  traj[:, 3], traj[:, 4], traj[:, 5]

# ---------- 绘图配置 ----------
import matplotlib
matplotlib.use("Agg")   # 非交互后端，适合后台运行

plt.rcParams["font.family"] = ["PingFang TC", "Songti SC", "Arial Unicode MS"]
fig, axes = plt.subplots(2, 1, figsize=(14, 6),
                          gridspec_kw={"height_ratios": [3, 2]})

# --- 1) 轨迹图 ---
ax = axes[0]
scatter_pts = np.linspace(0, len(step) - 1, 300).astype(int)
ax.scatter(lane_x[::2], lane_y[::2], s=3, c="steelblue", alpha=0.5,
           label="车道中心线")
ax.scatter(x[scatter_pts], y[scatter_pts], s=12, c="red",
           label="车辆轨迹")

# 标注起点和终点
ax.plot(x[0], y[0], "go", markersize=10, label="起点")
ax.plot(x[-1], y[-1], "mx", markersize=10, label="终点")

ax.set_title("纯跟踪 (Pure Pursuit) 仿真轨迹", fontsize=14, fontweight="bold")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.legend(fontsize=10)
ax.grid(True, alpha=0.3)
ax.set_aspect("equal")

# --- 2) 横向误差 & 转向角 ---
ax = axes[1]
time_arr = step * 0.05   # dt=0.05s

ax_twin = ax.twinx()
line1, = ax.plot(time_arr, cross_err, "b-", linewidth=1.0, label="横向误差")
line2, = ax_twin.plot(time_arr, np.degrees(delta), "r-", linewidth=1.0,
                      label="转向角")

ax.set_xlabel("时间 (s)")
ax.set_ylabel("横向误差 (m)", color="b")
ax_twin.set_ylabel("转向角 (度)", color="r")

# 共用图例
lines = ax.get_legend_handles_labels() + \
        ax_twin.get_legend_handles_labels()
ax.legend(lines[0], lines[1], loc="upper right", fontsize=9)

# 自动缩放 y 轴，去掉硬编码的 12m 上限
margin = max(abs(cross_err.min()), abs(cross_err.max())) * 0.2 + 0.05
ax.set_ylim(-margin, margin)
ax.grid(True, alpha=0.3)

# --- 保存 & 显示 ----------
fig.tight_layout()
fig.savefig("result.png", dpi=150)
plt.show()
