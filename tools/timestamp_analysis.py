import os
import re
import argparse
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

def parse_filename(filename):
    match = re.match(r"(.*?)_(\d+)\.npy", filename)
    if match:
        dtype, timestamp = match.groups()
        return dtype, int(timestamp)
    return None, None

def collect_npy_files(folder_path):
    files = [f for f in os.listdir(folder_path) if f.endswith(".npy")]
    data = defaultdict(dict)  # {timestamp: {dtype: filename}}
    all_dtypes = set()
    timestamps = set()

    for file in files:
        dtype, ts = parse_filename(file)
        if dtype and ts:
            data[ts][dtype] = file
            all_dtypes.add(dtype)
            timestamps.add(ts)

    # Sort: non-raw types first, then raw types (both alphabetically)
    sorted_dtypes = sorted(
        all_dtypes,
        key=lambda x: (x.endswith("raw") or "_raw" in x, x)
    )

    return data, sorted_dtypes, sorted(timestamps)


def compute_avg_left_right_diff(timestamps, data, break_threshold_ms=1000):
    """
    Compute average time difference between left* and right* entries,
    restricted to within temporal groups segmented by timestamp gaps.
    Excludes *_raw types.
    """
    if not timestamps:
        return None

    # 1. Split into temporal groups based on gaps
    sorted_ts = sorted(timestamps)
    groups = []
    current_group = [sorted_ts[0]]

    for i in range(1, len(sorted_ts)):
        if sorted_ts[i] - sorted_ts[i-1] > break_threshold_ms:
            groups.append(current_group)
            current_group = []
        current_group.append(sorted_ts[i])
    groups.append(current_group)  # final group

    # 2. For each group, compute avg diff between closest left/right timestamps
    def is_valid_left(keys):
        return any(k.startswith("left") and "raw" not in k for k in keys)

    def is_valid_right(keys):
        return any(k.startswith("right") and "raw" not in k for k in keys)

    group_diffs = []
    for group in groups:
        left_ts = [ts for ts in group if is_valid_left(data[ts].keys())]
        right_ts = [ts for ts in group if is_valid_right(data[ts].keys())]

        if not left_ts or not right_ts:
            continue

        diffs = []
        for l in left_ts:
            closest_r = min(right_ts, key=lambda r: abs(r - l))
            diffs.append(abs(l - closest_r))

        if diffs:
            group_diffs.append(np.mean(diffs))

    if group_diffs:
        return round(np.mean(group_diffs), 2)

    return None



def assign_colors_to_timestamps(timestamps, time_threshold_ms):
    """
    Assigns a color group to each timestamp based on proximity.
    Timestamps within `time_threshold_ms` milliseconds are grouped.
    """
    color_groups = {}
    current_group = 0
    last_ts = None

    for ts in sorted(timestamps):
        if last_ts is None or ts - last_ts > time_threshold_ms:
            current_group += 1
        color_groups[ts] = current_group
        last_ts = ts

    # Use tab20 (visually distinct, green-friendly)
    cmap = plt.cm.get_cmap('tab20')
    group_list = sorted(set(color_groups.values()))
    n_colors = len(group_list)

    group_colors = {group: cmap(i % 20) for i, group in enumerate(group_list)}
    return {ts: group_colors[group] for ts, group in color_groups.items()}

def assign_temporal_stripes_to_timestamps(timestamps, break_threshold_ms=1000):
    """
    Assigns group colors based on timestamp continuity. When there's a large jump
    (e.g., > break_threshold_ms), we switch to a new group.
    """
    group_map = {}
    current_group = 0
    last_ts = None

    for ts in sorted(timestamps):
        if last_ts is not None and ts - last_ts > break_threshold_ms:
            current_group += 1
        group_map[ts] = current_group
        last_ts = ts

    # Assign colors (cycling through a colormap)
    cmap = plt.cm.get_cmap('tab20')
    group_colors = {group: cmap(group % 20) for group in set(group_map.values())}
    return {ts: group_colors[group_map[ts]] for ts in timestamps}


def plot_matrix(ax, folder_name, data, dtypes, timestamps, colors, avg_diff_ms=None):
    matrix = np.zeros((len(timestamps), len(dtypes)))

    for i, ts in enumerate(timestamps):
        for j, dtype in enumerate(dtypes):
            matrix[i, j] = 1 if dtype in data[ts] else 0

    for i, ts in enumerate(timestamps):
        for j, dtype in enumerate(dtypes):
            if matrix[i, j]:
                ax.add_patch(plt.Rectangle((j, i), 1, 1, color=colors[ts]))

    ax.set_xticks(np.arange(len(dtypes)) + 0.5)
    ax.set_xticklabels(dtypes, rotation=45, ha='right', fontsize=8)
    ax.set_yticks(np.arange(len(timestamps)) + 0.5)
    ax.set_yticklabels([str(ts) for ts in timestamps], fontsize=6)

    ax.set_xlim(0, len(dtypes))
    ax.set_ylim(0, len(timestamps))
    ax.invert_yaxis()

    title = f"{folder_name} ({len(timestamps)} ts"
    if avg_diff_ms is not None:
        title += f", Δ={avg_diff_ms}ms"
    title += ")"
    ax.set_title(title, fontsize=10)

    ax.set_aspect('auto')

def visualize_all_folders(parent_folder, time_threshold_ms, group_colors=False):
    subfolders = sorted(
        [os.path.join(parent_folder, d) for d in os.listdir(parent_folder)
         if os.path.isdir(os.path.join(parent_folder, d))],
        key=lambda x: os.path.basename(x)
    )

    num = len(subfolders)
    if num == 0:
        print("No subfolders found.")
        return

    cols = min(3, num)
    rows = (num + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 4 * rows))
    axes = np.array(axes).reshape(-1)

    plotted = 0
    for i, subfolder in enumerate(subfolders):
        folder_name = os.path.basename(subfolder)
        data, dtypes, timestamps = collect_npy_files(subfolder)

        if not timestamps:
            continue  # skip empty folders

        if group_colors: colors = assign_temporal_stripes_to_timestamps(timestamps, break_threshold_ms=1000)
        else: colors = assign_colors_to_timestamps(timestamps, time_threshold_ms)

        avg_diff = compute_avg_left_right_diff(timestamps, data)
        plot_matrix(axes[plotted], folder_name, data, dtypes, timestamps, colors, avg_diff)

        plotted += 1

    # Remove any unused subplots
    for j in range(plotted, len(axes)):
        fig.delaxes(axes[j])

    plt.suptitle(f"NPY File Structure by Folder\nThreshold: {time_threshold_ms} ms", fontsize=14)
    plt.tight_layout(rect=[0, 0.05, 1, 0.95])
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize .npy file structures from subfolders.")
    parser.add_argument("parent_folder", type=str, help="Parent folder containing subfolders with .npy files")
    parser.add_argument("--threshold", type=int, default=2,
                        help="Timestamp grouping threshold in milliseconds (default: 60000)")
    parser.add_argument("--group_color", action="store_true", help="Whether to color timestamps individually or based on time segment")
    args = parser.parse_args()

    if not os.path.isdir(args.parent_folder):
        print(f"Error: {args.parent_folder} is not a valid directory.")
    else:
        visualize_all_folders(args.parent_folder, args.threshold, args.group_color)
