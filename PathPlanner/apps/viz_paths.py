#!/usr/bin/env python3

#
# Visualize precomputed paths + obstacles
#
# author: siris-Kang
#
# python apps/viz_paths.py --path_file maps/node01/paths/path_A1_to_A2_20260123_034010.json --oxoy_json maps/node01/node01_oxoy.json --nodes maps/node01/node.json --show
#

import argparse
import json
import math
from pathlib import Path
import matplotlib.pyplot as plt

def load_json(p: Path):
    with open(p, "r", encoding="utf-8") as f:
        return json.load(f)

def extract_xy_from_path(d):
    x = d.get("x", None)
    y = d.get("y", None)
    if isinstance(x, list) and isinstance(y, list) and len(x) >= 2 and len(y) >= 2:
        return x, y
    return [], []


def extract_meta(d):
    start_id = d.get("start_id", "")
    goal_id = d.get("goal_id", "")
    frame_id = d.get("frame_id", "")
    cost = d.get("cost", None)
    ok = d.get("ok", None)
    return ok, start_id, goal_id, frame_id, cost


def load_oxoy(oxoy_path: Path):
    if not oxoy_path or not oxoy_path.exists():
        return [], []

    d = load_json(oxoy_path)

    # dict arrays
    for (kx, ky) in (("ox", "oy"), ("x", "y")):
        ox = d.get(kx, None) if isinstance(d, dict) else None
        oy = d.get(ky, None) if isinstance(d, dict) else None
        if isinstance(ox, list) and isinstance(oy, list) and len(ox) == len(oy) and len(ox) > 0:
            return ox, oy

    # dict points
    if isinstance(d, dict) and isinstance(d.get("points"), list) and d["points"]:
        pts = d["points"]
        if isinstance(pts[0], dict) and ("x" in pts[0] and "y" in pts[0]):
            ox = [p["x"] for p in pts]
            oy = [p["y"] for p in pts]
            return ox, oy

    if isinstance(d, list) and d and isinstance(d[0], dict) and ("x" in d[0] and "y" in d[0]):
        ox = [p["x"] for p in d]
        oy = [p["y"] for p in d]
        return ox, oy

    return [], []


def load_nodes(nodes_path: Path):
    if not nodes_path or not nodes_path.exists():
        return {}

    d = load_json(nodes_path)
    out = {}

    if isinstance(d, dict) and isinstance(d.get("nodes"), list):
        arr = d["nodes"]
    elif isinstance(d, list):
        arr = d
    elif isinstance(d, dict):
        for k, v in d.items():
            if isinstance(v, dict) and ("x" in v and "y" in v):
                out[str(k)] = (float(v["x"]), float(v["y"]), float(v.get("yaw", 0.0)))
        return out
    else:
        return out

    for it in arr:
        if not isinstance(it, dict):
            continue
        nid = it.get("id", it.get("name", it.get("key", None)))
        if nid is None:
            continue
        if "x" in it and "y" in it:
            out[str(nid)] = (float(it["x"]), float(it["y"]), float(it.get("yaw", 0.0)))

    return out


def plot_nodes(nodes_dict):
    if not nodes_dict:
        return
    xs = [v[0] for v in nodes_dict.values()]
    ys = [v[1] for v in nodes_dict.values()]
    plt.scatter(xs, ys, s=30, marker="x")

    # label only a few if there are many
    for nid, (x, y, yaw) in nodes_dict.items():
        plt.text(x, y, f" {nid}", fontsize=9)


def main():
    ap = argparse.ArgumentParser(description="Visualize precomputed paths + obstacles")
    ap.add_argument("--paths_dir", default="paths", help="directory that contains path_*.json")
    ap.add_argument("--path_file", type=str, default=None, help="single path json file to plot")
    ap.add_argument("--oxoy_json", default="", help="oxoy.json file (from precompute_obstacles)")
    ap.add_argument("--nodes", default="", help="nodes json (optional)")
    ap.add_argument("--save", default="", help="save figure to this path (optional)")
    ap.add_argument("--show", action="store_true", help="show interactive window")
    ap.add_argument("--max_paths", type=int, default=0, help="0=all, otherwise limit number of paths")
    args = ap.parse_args()

    # Resolve inputs
    paths_dir = Path(args.paths_dir).expanduser().resolve()
    oxoy_path = Path(args.oxoy_json).expanduser().resolve() if args.oxoy_json else None
    nodes_path = Path(args.nodes).expanduser().resolve() if args.nodes else None

    if args.path_file:
        path_files = [Path(args.path_file).expanduser().resolve()]
    else:
        if not paths_dir.exists():
            raise FileNotFoundError(f"--paths_dir not found: {paths_dir}")
        path_files = sorted(paths_dir.glob("path_*.json"))

    if args.max_paths and len(path_files) > args.max_paths:
        path_files = path_files[: args.max_paths]

    if not path_files:
        print(f"[warn] no path files found. paths_dir={paths_dir}")
        print("       try: python viz_paths.py --paths_dir paths --show")
        return

    missing = [str(p) for p in path_files if not p.exists()]
    if missing:
        raise FileNotFoundError(f"path json not found: {missing}")

    # Load obstacle points
    ox, oy = ([], [])
    if oxoy_path:
        ox, oy = load_oxoy(oxoy_path)
        print(f"[oxoy] {oxoy_path}  points={len(ox)}")

    # Load nodes
    nodes = {}
    if nodes_path:
        nodes = load_nodes(nodes_path)
        print(f"[nodes] {nodes_path}  count={len(nodes)}")

    # Plot
    plt.figure(figsize=(10, 7))

    if ox and oy:
        plt.scatter(ox, oy, s=2, marker=".", alpha=0.8)

    plot_nodes(nodes)

    ok_count = 0
    total = 0
    shown = 0

    # Track extents
    all_x = []
    all_y = []

    for pf in path_files:
        total += 1
        d = load_json(pf)

        ok, start_id, goal_id, frame_id, cost = extract_meta(d)
        x, y = extract_xy_from_path(d)

        if ok is False:
            # explicitly failed
            continue
        if len(x) < 2 or len(y) < 2:
            # nothing to draw
            continue

        ok_count += 1
        shown += 1

        all_x.extend(x)
        all_y.extend(y)

        # Draw path line
        plt.plot(x, y, linewidth=2)

        # Mark start/end
        plt.scatter([x[0]], [y[0]], s=40, marker="o")
        plt.scatter([x[-1]], [y[-1]], s=40, marker="s")

        # Optional annotation
        label = pf.name
        if start_id or goal_id:
            label += f"  {start_id}->{goal_id}"
        if cost is not None:
            label += f"  cost={cost}"
        plt.text(x[-1], y[-1], f" {goal_id}" if goal_id else "", fontsize=9)

    if shown == 0:
        print("[warn] paths loaded but nothing plotted.")
        print("       Likely path JSON format mismatch or ok=false filtered.")
        print("       Try printing keys of the JSON to verify.")
        return

    # Axis formatting
    plt.gca().set_aspect("equal", adjustable="box")
    plt.grid(True, linewidth=0.3)

    # set limits from data if available
    if ox and oy:
        all_x.extend(ox)
        all_y.extend(oy)
    if all_x and all_y:
        minx, maxx = min(all_x), max(all_x)
        miny, maxy = min(all_y), max(all_y)
        pad = 0.5
        plt.xlim(minx - pad, maxx + pad)
        plt.ylim(miny - pad, maxy + pad)

    title = f"paths ok={ok_count} / total_files={len(path_files)}"
    plt.title(title)

    if args.save:
        out = Path(args.save).expanduser().resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(out, dpi=150)
        print(f"[save] {out}")

    if args.show or not args.save:
        plt.show()


if __name__ == "__main__":
    main()