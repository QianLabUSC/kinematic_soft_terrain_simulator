import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
from pathlib import Path

# --- Matplotlib settings for Illustrator-friendly PDFs (Times New Roman) ---
mpl.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'Times', 'Nimbus Roman No9 L', 'Liberation Serif', 'DejaVu Serif'],
    'pdf.fonttype': 42,   # Store text as TrueType (editable in Illustrator)
    'ps.fonttype': 42,
    'svg.fonttype': 'none',
    # 2x larger fonts
    'font.size': 24,           # base font size
    'axes.titlesize': 28,      # title font size
    'axes.labelsize': 24,      # axis label font size
    'xtick.labelsize': 20,
    'ytick.labelsize': 20,
    'legend.fontsize': 20,
})

# --- Load CSV file ---
# test_name = "exp3_success_safe"
test_name = "exp1_fail_at_yellow"
# test_name = "exp2_fail_at_red"
file_path = Path.home() / "ros2_ws" / "src" / "trusses_chrono_sim" / "scripts" / f"{test_name}_output_data.csv"
df = pd.read_csv(file_path)

# --- Extract values ---
time = df["time_s"].values
vx = df["vel_x"].astype(float).values
vy = df["vel_y"].astype(float).values
px = df["pos_x"].astype(float).values
py = df["pos_y"].astype(float).values

# Waypoint direction vectors
dx = (df["next_wp_x"] - df["prev_wp_x"]).astype(float).values
dy = (df["next_wp_y"] - df["prev_wp_y"]).astype(float).values
norm = np.sqrt(dx**2 + dy**2)

# Normalize direction vectors (force float arrays)
dx_unit = np.divide(dx, norm, out=np.zeros(norm.shape, dtype=float), where=norm != 0)
dy_unit = np.divide(dy, norm, out=np.zeros(norm.shape, dtype=float), where=norm != 0)

# --- Compute projections ---
# Velocity projected along waypoint direction
productive_velocity = vx * dx_unit + vy * dy_unit

# Position projected along waypoint direction
rel_x = px - df["prev_wp_x"].astype(float).values
rel_y = py - df["prev_wp_y"].astype(float).values
productive_position = rel_x * dx_unit + rel_y * dy_unit

# --- Plot: Productive Velocity (with moving average + 95% CI) ---
# Output directory for figures
fig_dir = Path.home() / "ros2_ws" / "src" / "trusses_chrono_sim" / "scripts" / "figures" / test_name
fig_dir.mkdir(parents=True, exist_ok=True)

plt.figure(figsize=(12, 5))

# Base series (light gray)
plt.plot(time, productive_velocity, color="#bbbbbb", linewidth=1.2, label="Velocity")

# Determine a window ~1s based on median dt
if len(time) > 2:
    dt = np.diff(time)
    median_dt = float(np.median(dt)) if np.all(np.isfinite(dt)) and np.median(dt) > 0 else 0.02
else:
    median_dt = 0.02
window_sec = 3.0
window = max(5, int(round(window_sec / median_dt)))

pv = pd.Series(productive_velocity)
ma = pv.rolling(window=window, center=True, min_periods=max(3, window//5)).mean().values
std = pv.rolling(window=window, center=True, min_periods=max(3, window//5)).std().values
count = pv.rolling(window=window, center=True, min_periods=max(3, window//5)).count().values

# Variability band (±1σ)
with np.errstate(invalid='ignore', divide='ignore'):
    ci = std

plt.plot(time, ma, color="#1f77b4", linewidth=2.5, label=f"{window_sec:.1f}s moving average")
plt.fill_between(time, ma - ci, ma + ci, color="#1f77b4", alpha=0.1, label="±1σ")

# Mark segment boundaries with vertical dashed lines
unique_segments_v = list(zip(df["prev_wp_x"], df["prev_wp_y"], df["next_wp_x"], df["next_wp_y"]))
segment_ids_v = np.array([unique_segments_v.index(seg) for seg in unique_segments_v])
if len(segment_ids_v) > 1:
    change_idx_v = np.where(np.diff(segment_ids_v) != 0)[0] + 1
    for idx in change_idx_v:
        if 0 <= idx < len(time):
            plt.axvline(time[idx], color="k", linestyle="--", linewidth=1, alpha=0.4)

plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Productive Velocity Over Time")
plt.grid(True)
plt.tight_layout()
plt.savefig(fig_dir / f"{test_name}_productive_velocity.pdf", format='pdf', bbox_inches='tight')
plt.show()

# --- Plot: Productive Position (segment 2 stacked on segment 1) ---
plt.figure(figsize=(12, 5))

# Build segment ids from waypoint pairs
unique_segments = list(zip(df["prev_wp_x"], df["prev_wp_y"], df["next_wp_x"], df["next_wp_y"]))
segment_ids = [unique_segments.index(seg) for seg in unique_segments]

# Create continuous position by stacking each new segment atop the previous
continuous_position = np.empty_like(productive_position, dtype=float)
offset = 0.0
prev_sid = segment_ids[0] if len(segment_ids) > 0 else 0
for i, (sid, pos) in enumerate(zip(segment_ids, productive_position)):
    if i > 0 and sid != prev_sid:
        # Align start of this segment with previous segment end
        offset = continuous_position[i-1] - pos
    continuous_position[i] = pos + offset
    prev_sid = sid

plt.plot(time, continuous_position, color="#1f77b4", linewidth=2.0, label="Position (stacked)")

# Mark segment boundaries on position plot
if len(segment_ids) > 1:
    change_idx_p = np.where(np.diff(np.array(segment_ids)) != 0)[0] + 1
    for idx in change_idx_p:
        if 0 <= idx < len(time):
            plt.axvline(time[idx], color="k", linestyle="--", linewidth=1, alpha=0.4)

plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.title("Productive Position Over Time")
plt.grid(True)
plt.tight_layout()
plt.savefig(fig_dir / f"{test_name}_productive_position.pdf", format='pdf', bbox_inches='tight')
plt.show()

# --- Composite Figure: 3 rows x 2 cols (exp1, exp2, exp3 | velocity, position) ---
all_tests = [
    ("exp1_fail_at_yellow", "Exp1"),
    ("exp2_fail_at_red", "Exp2"),
    ("exp3_success_safe", "Exp3"),
]

fig, axes = plt.subplots(nrows=3, ncols=2, figsize=(24, 15), sharex=False, sharey=False)

for r, (tname, tlabel) in enumerate(all_tests):
    fpath = Path.home() / "ros2_ws" / "src" / "trusses_chrono_sim" / "scripts" / f"{tname}_output_data.csv"
    if not fpath.exists():
        # leave empty axes if file missing
        for c in range(2):
            axes[r, c].axis('off')
        continue

    dfi = pd.read_csv(fpath)
    time_i = dfi["time_s"].values
    vx_i = dfi["vel_x"].astype(float).values
    vy_i = dfi["vel_y"].astype(float).values
    px_i = dfi["pos_x"].astype(float).values
    py_i = dfi["pos_y"].astype(float).values

    dx_i = (dfi["next_wp_x"] - dfi["prev_wp_x"]).astype(float).values
    dy_i = (dfi["next_wp_y"] - dfi["prev_wp_y"]).astype(float).values
    norm_i = np.sqrt(dx_i**2 + dy_i**2)
    dxu_i = np.divide(dx_i, norm_i, out=np.zeros(norm_i.shape, dtype=float), where=norm_i != 0)
    dyu_i = np.divide(dy_i, norm_i, out=np.zeros(norm_i.shape, dtype=float), where=norm_i != 0)

    prod_vel_i = vx_i * dxu_i + vy_i * dyu_i
    relx_i = px_i - dfi["prev_wp_x"].astype(float).values
    rely_i = py_i - dfi["prev_wp_y"].astype(float).values
    prod_pos_i = relx_i * dxu_i + rely_i * dyu_i

    # Velocity subplot (left col)
    axv = axes[r, 0]
    axv.plot(time_i, prod_vel_i, color="#bbbbbb", linewidth=1.2)
    if len(time_i) > 2:
        dt_i = np.diff(time_i)
        median_dt_i = float(np.median(dt_i)) if np.all(np.isfinite(dt_i)) and np.median(dt_i) > 0 else 0.02
    else:
        median_dt_i = 0.02
    window = max(5, int(round(3.0 / median_dt_i)))
    pv_s = pd.Series(prod_vel_i)
    ma_i = pv_s.rolling(window=window, center=True, min_periods=max(3, window//5)).mean().values
    sd_i = pv_s.rolling(window=window, center=True, min_periods=max(3, window//5)).std().values
    axv.plot(time_i, ma_i, color="#1f77b4", linewidth=2.5)
    axv.fill_between(time_i, ma_i - sd_i, ma_i + sd_i, color="#1f77b4", alpha=0.1)
    # segment boundaries
    uniq_segs_i = list(zip(dfi["prev_wp_x"], dfi["prev_wp_y"], dfi["next_wp_x"], dfi["next_wp_y"]))
    seg_ids_i = np.array([uniq_segs_i.index(seg) for seg in uniq_segs_i])
    if len(seg_ids_i) > 1:
        ch_idx = np.where(np.diff(seg_ids_i) != 0)[0] + 1
        for idx in ch_idx:
            if 0 <= idx < len(time_i):
                axv.axvline(time_i[idx], color="k", linestyle="--", linewidth=1, alpha=0.4)
    if r == 0:
        axv.set_title("Velocity (m/s)")
    axv.set_ylabel(tlabel)
    axv.grid(True)

    # Position subplot (right col) with stacked segments
    axp = axes[r, 1]
    uniq_segs_i = list(zip(dfi["prev_wp_x"], dfi["prev_wp_y"], dfi["next_wp_x"], dfi["next_wp_y"]))
    seg_ids_i = [uniq_segs_i.index(seg) for seg in uniq_segs_i]
    cont_pos = np.empty_like(prod_pos_i, dtype=float)
    offset = 0.0
    prev_sid = seg_ids_i[0] if len(seg_ids_i) > 0 else 0
    for i_idx, (sid, pos) in enumerate(zip(seg_ids_i, prod_pos_i)):
        if i_idx > 0 and sid != prev_sid:
            offset = cont_pos[i_idx-1] - pos
        cont_pos[i_idx] = pos + offset
        prev_sid = sid
    axp.plot(time_i, cont_pos, color="#1f77b4", linewidth=2.0)
    if len(seg_ids_i) > 1:
        ch_idx = np.where(np.diff(np.array(seg_ids_i)) != 0)[0] + 1
        for idx in ch_idx:
            if 0 <= idx < len(time_i):
                axp.axvline(time_i[idx], color="k", linestyle="--", linewidth=1, alpha=0.4)
    if r == 0:
        axp.set_title("Position (m)")
    axp.grid(True)

for ax in axes[-1, :]:
    ax.set_xlabel("Time (s)")

plt.tight_layout()
combo_dir = Path.home() / "ros2_ws" / "src" / "trusses_chrono_sim" / "scripts" / "figures"
combo_dir.mkdir(parents=True, exist_ok=True)
plt.savefig(combo_dir / "combined_velocity_position.pdf", format='pdf', bbox_inches='tight')
plt.show()
