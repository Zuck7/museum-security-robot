#!/usr/bin/env python3
"""Score a slam_toolbox map against the ground-truth map.

The project rubric wants a *critical* evaluation of system performance, not a
description. "The map looks good" is a description. "94% of occupied cells lie
within 10 cm of a true wall, with a mean absolute wall error of 4.1 cm, and
the largest error is 31 cm in the north-east corner where the loop did not
close" is an evaluation - and this script produces the second kind.

Usage:
    python3 tools/compare_maps.py maps/museum_map_slam.yaml
    python3 tools/compare_maps.py maps/museum_map_slam.yaml --plot out.png

What it reports:
    coverage      how much of the truly-free floor area SLAM actually explored
    wall accuracy distance from each mapped wall cell to the nearest real wall
    false walls   mapped obstacles with no real wall anywhere near them
    (drift, blur from a bad scan match, and doorways closed off by smear all
     show up clearly in these three numbers)
"""

import argparse
import os
import sys

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

TRUTH_DIR = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "maps")


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img_path = meta["image"]
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)),
                                img_path)
    grid = np.array(Image.open(img_path).convert("L"))
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    occ_t = float(meta.get("occupied_thresh", 0.65))
    free_t = float(meta.get("free_thresh", 0.196))
    p = (255.0 - grid) / 255.0
    if int(meta.get("negate", 0)):
        p = 1.0 - p
    return dict(occ=p > occ_t, free=p < free_t, res=res, ox=ox, oy=oy,
                shape=grid.shape)


def world_points(mask, m):
    """Convert a boolean mask into an (N,2) array of world-frame xy points."""
    rows, cols = np.nonzero(mask)
    h = m["shape"][0]
    x = m["ox"] + (cols + 0.5) * m["res"]
    y = m["oy"] + (h - 1 - rows + 0.5) * m["res"]
    return np.column_stack([x, y])


def to_truth_index(pts, truth):
    h, w = truth["shape"]
    cols = ((pts[:, 0] - truth["ox"]) / truth["res"]).astype(int)
    rows = (h - 1 - ((pts[:, 1] - truth["oy"]) / truth["res"]).astype(int))
    inside = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
    return rows[inside], cols[inside], inside


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("slam_yaml", help="your slam_toolbox map .yaml")
    ap.add_argument("--truth",
                    default=os.path.join(TRUTH_DIR, "museum_map.yaml"))
    ap.add_argument("--plot", metavar="PNG", help="write an overlay figure")
    a = ap.parse_args()

    slam = load_map(a.slam_yaml)
    truth = load_map(a.truth)

    # Distance (in metres) from any cell to the nearest true wall / free cell.
    d_wall = ndimage.distance_transform_edt(~truth["occ"]) * truth["res"]
    d_free = ndimage.distance_transform_edt(~truth["free"]) * truth["res"]

    print("=" * 66)
    print("SLAM MAP QUALITY REPORT")
    print("=" * 66)
    print(f"slam map : {a.slam_yaml}")
    print(f"truth    : {a.truth}")
    print(f"slam grid: {slam['shape'][1]}x{slam['shape'][0]} @ {slam['res']} m")
    print()

    # ---- coverage --------------------------------------------------------
    truth_free_pts = world_points(truth["free"], truth)
    h, w = slam["shape"]
    cols = ((truth_free_pts[:, 0] - slam["ox"]) / slam["res"]).astype(int)
    rows = h - 1 - ((truth_free_pts[:, 1] - slam["oy"]) / slam["res"]).astype(int)
    inside = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
    known = np.zeros(len(truth_free_pts), bool)
    known[inside] = (slam["free"][rows[inside], cols[inside]]
                     | slam["occ"][rows[inside], cols[inside]])
    coverage = 100.0 * known.mean()
    truth_area = truth["free"].sum() * truth["res"] ** 2
    print(f"COVERAGE      {coverage:5.1f}% of the {truth_area:.0f} m2 of real "
          f"floor space was observed")
    if coverage < 85:
        print("              -> drive the unexplored rooms and re-save; "
              "AMCL cannot localise in space that was never mapped")
    print()

    # ---- wall accuracy ---------------------------------------------------
    slam_wall_pts = world_points(slam["occ"], slam)
    r, c, ins = to_truth_index(slam_wall_pts, truth)
    err = d_wall[r, c]
    if len(err) == 0:
        print("no occupied cells in the slam map - nothing to compare")
        return 1
    print(f"WALL ACCURACY over {len(err)} mapped obstacle cells")
    print(f"  mean error        {err.mean() * 100:6.1f} cm")
    print(f"  median error      {np.median(err) * 100:6.1f} cm")
    print(f"  90th percentile   {np.percentile(err, 90) * 100:6.1f} cm")
    print(f"  worst             {err.max() * 100:6.1f} cm")
    for tol in (0.05, 0.10, 0.20):
        print(f"  within {tol*100:4.0f} cm     {100.0*(err <= tol).mean():5.1f}%")
    print()

    # ---- false walls -----------------------------------------------------
    # A mapped wall more than 0.3 m from any real wall is not measurement
    # noise - it is smear from a bad scan match, or a moving artefact.
    false_frac = 100.0 * (err > 0.30).mean()
    print(f"FALSE OBSTACLES   {false_frac:5.1f}% of mapped walls are >30 cm "
          f"from any real wall")
    if false_frac > 5:
        print("              -> usually scan-match drift. Drive slower, and "
              "make sure you close the corridor loop so the graph optimises.")
    print()

    # ---- doorway check ---------------------------------------------------
    # If SLAM smeared a doorway shut, a wall cell will land in the middle of
    # somewhere the truth map says is comfortably open floor.
    blocked = (d_free[r, c] == 0) & (d_wall[r, c] > 0.30)
    print(f"BLOCKED FREE SPACE {blocked.sum()} mapped wall cells sit on top of "
          f"genuinely open floor")
    if blocked.sum() > 200:
        print("              -> check that no doorway got smeared closed; "
              "Nav2 will report 'failed to compute path' if one did.")
    print("=" * 66)

    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(12, 10))
        te = [truth["ox"], truth["ox"] + truth["shape"][1] * truth["res"],
              truth["oy"], truth["oy"] + truth["shape"][0] * truth["res"]]
        base = np.ones((*truth["shape"], 3))
        base[truth["occ"]] = [0.15, 0.15, 0.2]
        base[~truth["occ"] & ~truth["free"]] = [0.75, 0.75, 0.75]
        ax.imshow(base, extent=te, origin="upper")
        ok = err <= 0.10
        pts = slam_wall_pts[ins]
        ax.scatter(pts[ok, 0], pts[ok, 1], s=1, c="tab:green",
                   label="SLAM wall within 10 cm")
        ax.scatter(pts[~ok, 0], pts[~ok, 1], s=2, c="tab:red",
                   label="SLAM wall off by >10 cm")
        ax.legend(loc="upper right")
        ax.set_title(f"SLAM map vs ground truth  -  coverage {coverage:.0f}%, "
                     f"mean wall error {err.mean()*100:.1f} cm")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        plt.tight_layout()
        plt.savefig(a.plot, dpi=130)
        print(f"wrote {a.plot}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
