#!/usr/bin/env python3
"""
Single source of truth for the Centennial Robotics Museum.

Running this script emits THREE artifacts that are guaranteed to agree with
each other:

    worlds/museum.sdf      - the Gazebo world
    maps/museum_map.pgm    - a ground-truth occupancy grid
    maps/museum_map.yaml   - its metadata

Why generate the map instead of running SLAM?
    AMCL is a *local* filter: it refines a pose you hand it by matching the
    live laser scan against a prior map.  If that map disagrees with the
    world - even slightly - the particle filter drifts and every navigation
    goal becomes unreliable.  Because we author the world here in Python, we
    already know where every wall is, so we can rasterise a pixel-perfect
    map instead of hoping a SLAM run came out clean.  Re-run this script
    after ANY change to the world and the map follows automatically.

The map is a horizontal slice of the world taken at LIDAR_Z, the height the
laser actually sweeps.  Anything above or below that plane (the T-rex bones
hanging at 2.5 m, the paintings at 1.8 m) is correctly absent from the map,
exactly as a real scan would see it.

Usage:
    python3 tools/generate_world.py            # writes into the package
    python3 tools/generate_world.py --check    # verify goal clearances only
"""

import argparse
import math
import os
import sys

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ----------------------------------------------------------------------------
# Constants
# ----------------------------------------------------------------------------

WALL_T = 0.20          # wall thickness (m)
WALL_H = 3.00          # wall height (m)
DOOR_W = 1.80          # doorway clear width (m) - robot is 0.50 m wide
CORRIDOR_HALF = 1.50   # corridor half-width (m)

# Height of the laser plane above the floor.
#   base_link sits at wheel_radius (0.10) above ground,
#   lidar_link sits 0.13 above base_link  ->  0.23 m
LIDAR_Z = 0.23

MAP_RES = 0.05
MAP_MIN_X, MAP_MAX_X = -12.5, 12.5
MAP_MIN_Y, MAP_MAX_Y = -10.5, 10.5

ROBOT_SPAWN = (0.0, -8.5, 1.5708)   # x, y, yaw - in the lobby, facing north
ROBOT_RADIUS = 0.26                 # true half-width incl. wheels, + margin

# ----------------------------------------------------------------------------
# Materials
# ----------------------------------------------------------------------------

MAT = {
    "wall":      dict(ambient=(0.88, 0.88, 0.86), specular=(0.1, 0.1, 0.1)),
    "floor":     dict(ambient=(0.16, 0.16, 0.18), specular=(0.45, 0.45, 0.5)),
    "gold":      dict(ambient=(0.72, 0.55, 0.12), specular=(1.0, 0.9, 0.6)),
    "dark":      dict(ambient=(0.10, 0.10, 0.12), specular=(0.3, 0.3, 0.3)),
    "steel":     dict(ambient=(0.45, 0.47, 0.50), specular=(0.7, 0.7, 0.7)),
    "wood":      dict(ambient=(0.38, 0.24, 0.13), specular=(0.2, 0.2, 0.2)),
    "marble":    dict(ambient=(0.90, 0.89, 0.85), specular=(0.8, 0.8, 0.8)),
    "bone":      dict(ambient=(0.72, 0.66, 0.52), specular=(0.15, 0.15, 0.15)),
    "rock":      dict(ambient=(0.32, 0.30, 0.28), specular=(0.1, 0.1, 0.1)),
    "glass":     dict(ambient=(0.55, 0.78, 0.85), specular=(1.0, 1.0, 1.0), alpha=0.35),
    "water":     dict(ambient=(0.05, 0.42, 0.62), emissive=(0.02, 0.16, 0.26), alpha=0.7),
    "neon":      dict(ambient=(0.0, 0.85, 0.55), emissive=(0.0, 0.95, 0.6)),
    "magenta":   dict(ambient=(0.85, 0.15, 0.45), emissive=(0.75, 0.10, 0.38)),
    "exit":      dict(ambient=(0.05, 0.7, 0.2), emissive=(0.05, 0.9, 0.25)),
    "screen":    dict(ambient=(0.05, 0.30, 0.55), emissive=(0.03, 0.25, 0.45)),
    "velvet":    dict(ambient=(0.45, 0.05, 0.10), specular=(0.2, 0.2, 0.2)),
    "plant":     dict(ambient=(0.12, 0.38, 0.14), specular=(0.1, 0.1, 0.1)),
    "canvas":    dict(ambient=(0.55, 0.42, 0.28), specular=(0.1, 0.1, 0.1)),
}

LINKS = []   # every piece of the museum ends up here


def box(name, x, y, z, sx, sy, sz, mat="wall", collision=True, yaw=0.0):
    LINKS.append(dict(kind="box", name=name, x=x, y=y, z=z,
                      sx=sx, sy=sy, sz=sz, mat=mat,
                      collision=collision, yaw=yaw))


def cyl(name, x, y, z, r, h, mat="wall", collision=True, rpy=(0, 0, 0)):
    LINKS.append(dict(kind="cylinder", name=name, x=x, y=y, z=z,
                      r=r, h=h, mat=mat, collision=collision, rpy=rpy))


def sphere(name, x, y, z, r, mat="wall", collision=True):
    LINKS.append(dict(kind="sphere", name=name, x=x, y=y, z=z,
                      r=r, mat=mat, collision=collision))


def wall_run(tag, axis, fixed, start, end, gaps=()):
    """Build a straight wall with doorway gaps punched out of it.

    axis 'x' -> wall runs along x at y = fixed
    axis 'y' -> wall runs along y at x = fixed
    gaps     -> list of (from, to) openings along the running axis
    """
    edges = [start]
    for a, b in sorted(gaps):
        edges += [a, b]
    edges.append(end)
    for i in range(0, len(edges) - 1, 2):
        a, b = edges[i], edges[i + 1]
        if b - a < 1e-6:
            continue
        mid, length = (a + b) / 2.0, b - a
        n = f"{tag}_{i//2}"
        if axis == "x":
            box(n, mid, fixed, WALL_H / 2, length, WALL_T, WALL_H)
        else:
            box(n, fixed, mid, WALL_H / 2, WALL_T, length, WALL_H)


# ============================================================================
# FLOOR PLAN
# ============================================================================
#
#  y=+10  +--------------------+-----+--------------------+
#         |   DINOSAUR HALL    | SEC |     CAFETERIA      |
#         |                    | OFF |                    |
#  y=+1.5 +---[door]-----------+ ICE +---------[door]-----+
#         |     <<<<  EAST-WEST PATROL CORRIDOR  >>>>     |
#  y=-1.5 +--------[door]------+-----+---[door]-----------+
#         | AI INNOVATION WING |     | RENAISSANCE GALLERY|
#  y=-6   +---[door]-----------+-----+---------[door]-----+
#         |               MAIN  LOBBY                     |
#  y=-10  +-----------------------------------------------+
#        x=-12              -1.5  +1.5                   +12

# ---- Floor ---------------------------------------------------------------
box("floor", 0, 0, -0.05, 24.4, 20.4, 0.10, mat="floor")

# ---- Outer shell ---------------------------------------------------------
wall_run("outer_s", "x", -10.0, -12.1, 12.1)
wall_run("outer_n", "x",  10.0, -12.1, 12.1)
wall_run("outer_w", "y", -12.0, -10.1, 10.1)
wall_run("outer_e", "y",  12.0, -10.1, 10.1)

# ---- Lobby / room divider at y = -6 --------------------------------------
wall_run("div_s", "x", -6.0, -12.0, 12.0, gaps=[
    (-8.4, -8.4 + DOOR_W),                    # -> AI wing
    (-CORRIDOR_HALF, CORRIDOR_HALF),          # -> north-south corridor
    (8.4 - DOOR_W, 8.4),                      # -> Renaissance gallery
])

# ---- South rooms / corridor divider at y = -1.5 --------------------------
wall_run("div_m", "x", -1.5, -12.0, 12.0, gaps=[
    (-4.4, -4.4 + DOOR_W),
    (-CORRIDOR_HALF, CORRIDOR_HALF),
    (4.4 - DOOR_W, 4.4),
])

# ---- Corridor / north rooms divider at y = +1.5 --------------------------
wall_run("div_n", "x", 1.5, -12.0, 12.0, gaps=[
    (-8.4, -8.4 + DOOR_W),
    (-CORRIDOR_HALF, CORRIDOR_HALF),
    (8.4 - DOOR_W, 8.4),
])

# ---- North-south corridor side walls -------------------------------------
wall_run("cor_sw", "y", -CORRIDOR_HALF, -6.0, -1.5, gaps=[(-4.9, -4.9 + DOOR_W)])
wall_run("cor_se", "y",  CORRIDOR_HALF, -6.0, -1.5, gaps=[(-4.9, -4.9 + DOOR_W)])
wall_run("cor_nw", "y", -CORRIDOR_HALF,  1.5, 10.0, gaps=[(4.9, 4.9 + DOOR_W)])
wall_run("cor_ne", "y",  CORRIDOR_HALF,  1.5, 10.0, gaps=[(4.9, 4.9 + DOOR_W)])

# ============================================================================
# MAIN LOBBY  (entrance hall, south)
# ============================================================================
box("reception_desk", -6.0, -8.0, 0.55, 4.0, 0.8, 1.10, mat="wood")
box("reception_top",  -6.0, -8.0, 1.13, 4.2, 1.0, 0.06, mat="marble")
box("reception_screen", -6.0, -7.55, 1.45, 1.2, 0.05, 0.7, mat="screen", collision=False)

cyl("fountain_basin", 6.0, -8.0, 0.25, 1.50, 0.50, mat="marble")
cyl("fountain_water", 6.0, -8.0, 0.52, 1.35, 0.08, mat="water", collision=False)
cyl("fountain_spout", 6.0, -8.0, 0.85, 0.15, 0.70, mat="marble", collision=False)
sphere("fountain_droplet", 6.0, -8.0, 1.35, 0.20, mat="water", collision=False)

for sx, tag in ((-3.5, "l"), (3.5, "r")):
    box(f"pillar_{tag}_base", sx, -9.0, 0.15, 1.0, 1.0, 0.30, mat="marble")
    cyl(f"pillar_{tag}", sx, -9.0, 1.65, 0.35, 3.00, mat="gold")
    box(f"pillar_{tag}_cap", sx, -9.0, 3.20, 0.9, 0.9, 0.20, mat="marble")

box("lobby_bench", 0.0, -9.4, 0.22, 2.4, 0.50, 0.45, mat="wood")
box("info_kiosk", 9.6, -9.0, 0.70, 0.9, 0.9, 1.40, mat="dark")
box("kiosk_screen", 9.6, -8.53, 1.10, 0.7, 0.05, 0.5, mat="screen", collision=False)
box("welcome_banner", 0.0, -9.85, 2.30, 6.0, 0.06, 1.20, mat="gold", collision=False)

for px, py in ((-2.6, -6.8), (2.6, -6.8)):
    cyl(f"planter_lobby_{px:+.0f}".replace("+", "p").replace("-", "m"),
        px, py, 0.30, 0.38, 0.60, mat="rock")

# ============================================================================
# AI INNOVATION WING  (south-west)
# ============================================================================
box("mainframe", -11.0, -3.75, 1.25, 1.2, 4.0, 2.50, mat="dark")
for i, gy in enumerate((-5.0, -4.0, -3.0, -2.0)):
    box(f"mainframe_glow_{i}", -10.35, gy, 1.60, 0.06, 0.7, 0.12,
        mat="neon", collision=False)

cyl("cooling_tower_1", -9.3, -5.0, 1.25, 0.60, 2.50, mat="steel")
cyl("cooling_tower_2", -9.3, -2.6, 1.25, 0.60, 2.50, mat="steel")
cyl("cooling_cap_1", -9.3, -5.0, 2.60, 0.65, 0.20, mat="dark", collision=False)
cyl("cooling_cap_2", -9.3, -2.6, 2.60, 0.65, 0.20, mat="dark", collision=False)

box("server_rack_1", -7.0, -2.3, 1.00, 2.0, 0.80, 2.00, mat="dark")
box("server_rack_1_leds", -7.0, -1.88, 1.40, 1.7, 0.04, 0.60, mat="neon", collision=False)
box("server_rack_2", -4.0, -5.2, 0.90, 2.0, 0.80, 1.80, mat="dark")
box("server_rack_2_leds", -4.0, -4.78, 1.30, 1.7, 0.04, 0.50, mat="neon", collision=False)

box("ai_wall_display", -11.85, -3.75, 1.80, 0.06, 3.0, 1.60, mat="screen", collision=False)

# ============================================================================
# RENAISSANCE GALLERY  (south-east)
# ============================================================================
cyl("statue_pedestal", 10.0, -3.75, 0.60, 0.80, 1.20, mat="marble")
box("statue_body", 10.0, -3.75, 1.90, 0.45, 0.45, 1.40, mat="marble")
sphere("statue_head", 10.0, -3.75, 2.75, 0.22, mat="marble")

for i, (rx, ry) in enumerate(((8.6, -2.8), (8.6, -4.7), (11.4, -2.8), (11.4, -4.7))):
    cyl(f"rope_post_{i}", rx, ry, 0.50, 0.08, 1.00, mat="gold")
    sphere(f"rope_knob_{i}", rx, ry, 1.05, 0.09, mat="gold", collision=False)
box("rope_span_a", 10.0, -2.8, 0.85, 2.8, 0.04, 0.04, mat="velvet", collision=False)
box("rope_span_b", 10.0, -4.7, 0.85, 2.8, 0.04, 0.04, mat="velvet", collision=False)

box("display_case_1_base", 6.0, -5.0, 0.20, 1.2, 1.2, 0.40, mat="dark")
box("display_case_1_glass", 6.0, -5.0, 1.05, 1.1, 1.1, 1.30, mat="glass")
box("relic_1", 6.0, -5.0, 0.65, 0.35, 0.35, 0.50, mat="gold", collision=False)
box("display_case_2_base", 6.0, -2.5, 0.20, 1.2, 1.2, 0.40, mat="dark")
box("display_case_2_glass", 6.0, -2.5, 1.05, 1.1, 1.1, 1.30, mat="glass")
box("relic_2", 6.0, -2.5, 0.60, 0.30, 0.30, 0.40, mat="bone", collision=False)

for i, py in enumerate((-2.2, -3.75, -5.3)):
    box(f"painting_{i}", 11.85, py, 1.85, 0.06, 1.20, 0.90, mat="canvas", collision=False)
    box(f"painting_frame_{i}", 11.88, py, 1.85, 0.04, 1.34, 1.04, mat="gold", collision=False)

# ============================================================================
# DINOSAUR HALL  (north-west)
# ============================================================================
box("rock_pedestal", -7.5, 7.0, 0.40, 7.0, 2.50, 0.80, mat="rock")
cyl("dino_spine", -7.5, 7.0, 2.30, 0.16, 5.20, mat="bone",
    collision=False, rpy=(0, 1.5708, 0))
for i, sx in enumerate((-9.6, -8.6, -7.6, -6.6, -5.6)):
    cyl(f"dino_rib_{i}", sx, 7.0, 2.30, 0.07, 2.0 - abs(i - 2) * 0.25,
        mat="bone", collision=False, rpy=(1.5708, 0, 0))
box("dino_skull", -4.6, 7.0, 2.35, 1.10, 0.90, 0.80, mat="bone", collision=False)
cyl("dino_jaw", -4.3, 7.0, 1.95, 0.12, 1.00, mat="bone",
    collision=False, rpy=(1.5708, 0, 0))
for i, sx in enumerate((-10.4, -5.2)):
    cyl(f"dino_leg_{i}", sx, 7.0, 1.20, 0.12, 1.60, mat="bone", collision=False)

box("fossil_case_1", -9.5, 3.0, 0.60, 1.5, 1.5, 1.20, mat="dark")
box("fossil_glass_1", -9.5, 3.0, 1.35, 1.4, 1.4, 0.30, mat="glass", collision=False)
box("fossil_case_2", -4.5, 3.0, 0.60, 1.5, 1.5, 1.20, mat="dark")
box("fossil_glass_2", -4.5, 3.0, 1.35, 1.4, 1.4, 0.30, mat="glass", collision=False)
box("dino_kiosk", -2.8, 8.0, 0.75, 0.8, 0.8, 1.50, mat="dark")
box("dino_placard", -7.5, 5.6, 0.55, 2.0, 0.10, 0.35, mat="steel", collision=False)

# ============================================================================
# CAFETERIA  (north-east)
# ============================================================================
box("cafe_counter", 8.0, 9.2, 0.55, 6.0, 0.90, 1.10, mat="steel")
box("cafe_counter_top", 8.0, 9.2, 1.13, 6.2, 1.05, 0.06, mat="marble")
box("cafe_menu", 8.0, 9.75, 2.00, 3.0, 0.05, 0.80, mat="screen", collision=False)
box("vending_machine", 2.6, 9.2, 0.90, 1.0, 0.70, 1.80, mat="dark")
box("vending_glass", 2.6, 8.83, 1.10, 0.8, 0.05, 1.20, mat="glass", collision=False)

for i, (tx, ty) in enumerate(((4.0, 3.5), (8.0, 3.5), (4.0, 7.5),
                              (10.5, 7.5), (10.5, 3.5))):
    cyl(f"cafe_table_{i}", tx, ty, 0.37, 0.60, 0.74, mat="wood")
    cyl(f"cafe_table_top_{i}", tx, ty, 0.76, 0.75, 0.05, mat="marble", collision=False)
    for j, (ox, oy) in enumerate(((0.95, 0), (-0.95, 0), (0, 0.95), (0, -0.95))):
        cyl(f"cafe_chair_{i}_{j}", tx + ox, ty + oy, 0.22, 0.22, 0.44, mat="wood")

cyl("cafe_plant", 11.3, 2.2, 0.45, 0.40, 0.90, mat="plant")

# ============================================================================
# SECURITY OFFICE  (north corridor dead-end)
# ============================================================================
box("security_desk", 0.0, 9.4, 0.50, 2.4, 0.60, 1.00, mat="steel")
for i, ox in enumerate((-0.7, 0.0, 0.7)):
    box(f"security_monitor_{i}", ox, 9.72, 1.55, 0.60, 0.05, 0.42,
        mat="screen", collision=False)
box("server_closet", -1.15, 8.2, 0.85, 0.45, 1.20, 1.70, mat="dark")

# ============================================================================
# PATROL CORRIDOR dressing
# ============================================================================
for i, (px, py) in enumerate(((-2.6, 1.0), (2.6, 1.0), (-2.6, -1.0), (2.6, -1.0))):
    cyl(f"corridor_plant_{i}", px, py, 0.45, 0.35, 0.90, mat="plant")

box("corridor_bench_w", -6.0, 1.10, 0.22, 1.6, 0.42, 0.45, mat="wood")
box("corridor_bench_e", 6.0, -1.10, 0.22, 1.6, 0.42, 0.45, mat="wood")

box("fire_exit_sign", -11.85, 0.0, 2.20, 0.06, 1.00, 0.35, mat="exit", collision=False)
box("fire_exit_door", -11.88, 0.0, 1.05, 0.04, 1.60, 2.10, mat="exit", collision=False)
box("service_door", 11.88, 0.0, 1.05, 0.04, 1.60, 2.10, mat="steel", collision=False)

# security cameras - visual only, mounted high, purely for flavour
for i, (cx, cy, cyaw) in enumerate(((-1.3, -1.3, 0.0), (1.3, 1.3, 0.0),
                                    (-1.3, 1.3, 0.0), (1.3, -1.3, 0.0),
                                    (-11.6, -3.75, 0.0), (11.6, -3.75, 0.0),
                                    (-7.5, 9.6, 0.0), (7.5, 1.9, 0.0))):
    box(f"camera_{i}", cx, cy, 2.70, 0.22, 0.14, 0.14, mat="dark", collision=False)
    sphere(f"camera_lens_{i}", cx, cy, 2.62, 0.07, mat="magenta", collision=False)


# ============================================================================
# NAMED PATROL LOCATIONS
# ============================================================================
LOCATIONS = {
    "main_lobby":         dict(x=0.0,   y=-8.0,  yaw=1.5708,
                               desc="Visitor entrance, reception desk, fountain"),
    "ai_innovation_wing": dict(x=-6.0,  y=-3.8,  yaw=3.1416,
                               desc="Mainframe, server racks, cooling towers"),
    "renaissance_gallery": dict(x=8.0,  y=-3.75, yaw=0.0,
                               desc="Marble statue, roped-off area, paintings"),
    "dinosaur_hall":      dict(x=-7.0,  y=4.5,   yaw=1.5708,
                               desc="T-rex skeleton on rock pedestal, fossil cases"),
    "cafeteria":          dict(x=7.0,   y=6.0,   yaw=1.5708,
                               desc="Tables, chairs, food counter, vending machine"),
    "central_atrium":     dict(x=0.0,   y=0.0,   yaw=0.0,
                               desc="Corridor crossroads, default patrol standby"),
    "west_fire_exit":     dict(x=-10.5, y=0.0,   yaw=3.1416,
                               desc="Emergency exit door at the west end"),
    "security_office":    dict(x=0.0,   y=8.0,   yaw=1.5708,
                               desc="Monitoring station and server closet"),
}


# ============================================================================
# SDF EMITTER
# ============================================================================

def mat_xml(key, indent):
    m = MAT[key]
    a = m.get("alpha", 1.0)
    amb = m["ambient"]
    dif = m.get("diffuse", tuple(min(1.0, c * 1.15) for c in amb))
    spec = m.get("specular", (0.1, 0.1, 0.1))
    emi = m.get("emissive", (0.0, 0.0, 0.0))
    p = " " * indent
    out = [f"{p}<material>"]
    out.append(f"{p}  <ambient>{amb[0]} {amb[1]} {amb[2]} {a}</ambient>")
    out.append(f"{p}  <diffuse>{dif[0]} {dif[1]} {dif[2]} {a}</diffuse>")
    out.append(f"{p}  <specular>{spec[0]} {spec[1]} {spec[2]} 1</specular>")
    out.append(f"{p}  <emissive>{emi[0]} {emi[1]} {emi[2]} 1</emissive>")
    out.append(f"{p}</material>")
    return "\n".join(out)


def geom_xml(link, indent):
    p = " " * indent
    if link["kind"] == "box":
        return f"{p}<box><size>{link['sx']} {link['sy']} {link['sz']}</size></box>"
    if link["kind"] == "cylinder":
        return (f"{p}<cylinder><radius>{link['r']}</radius>"
                f"<length>{link['h']}</length></cylinder>")
    return f"{p}<sphere><radius>{link['r']}</radius></sphere>"


def build_sdf():
    L = []
    L.append('<?xml version="1.0" ?>')
    L.append('<sdf version="1.9">')
    L.append('  <world name="museum_world">')
    L.append('')
    L.append('    <!-- Generated by tools/generate_world.py - DO NOT EDIT BY HAND -->')
    L.append('')
    L.append('    <plugin filename="gz-sim-physics-system" '
             'name="gz::sim::systems::Physics"/>')
    L.append('    <plugin filename="gz-sim-user-commands-system" '
             'name="gz::sim::systems::UserCommands"/>')
    L.append('    <plugin filename="gz-sim-scene-broadcaster-system" '
             'name="gz::sim::systems::SceneBroadcaster"/>')
    L.append('    <plugin filename="gz-sim-sensors-system" '
             'name="gz::sim::systems::Sensors">')
    L.append('      <!-- ogre2 needs a real GPU.  On a VM with software rendering,')
    L.append('           launch with:  render_engine:=ogre -->')
    L.append('      <render_engine>ogre2</render_engine>')
    L.append('    </plugin>')
    L.append('')
    L.append('    <!-- real_time_factor 0 = run as fast as the machine allows.')
    L.append('         Set it to 1.0 if you want wall-clock pacing for a demo video. -->')
    L.append('    <physics name="2ms" type="ignored">')
    L.append('      <max_step_size>0.002</max_step_size>')
    L.append('      <real_time_factor>0</real_time_factor>')
    L.append('    </physics>')
    L.append('')
    L.append('    <scene>')
    L.append('      <ambient>0.55 0.55 0.58 1</ambient>')
    L.append('      <background>0.05 0.06 0.09 1</background>')
    L.append('      <shadows>true</shadows>')
    L.append('    </scene>')
    L.append('')
    L.append('    <light type="directional" name="sun">')
    L.append('      <cast_shadows>true</cast_shadows>')
    L.append('      <pose>0 0 12 0 0 0</pose>')
    L.append('      <diffuse>0.75 0.75 0.72 1</diffuse>')
    L.append('      <specular>0.25 0.25 0.25 1</specular>')
    L.append('      <direction>-0.4 0.3 -0.9</direction>')
    L.append('    </light>')
    for i, (lx, ly, col) in enumerate((
            (0, -8, "0.9 0.85 0.7"), (-7, -3.75, "0.5 0.8 0.9"),
            (8, -3.75, "0.95 0.9 0.8"), (-7, 6, "0.9 0.8 0.6"),
            (7, 6, "0.85 0.9 0.85"), (0, 0, "0.8 0.8 0.85"),
            (0, 8, "0.6 0.75 0.9"))):
        L.append(f'    <light type="point" name="ceiling_{i}">')
        L.append(f'      <pose>{lx} {ly} 2.8 0 0 0</pose>')
        L.append(f'      <diffuse>{col} 1</diffuse>')
        L.append('      <specular>0.2 0.2 0.2 1</specular>')
        L.append('      <attenuation><range>14</range><linear>0.10</linear>'
                 '<constant>0.4</constant><quadratic>0.005</quadratic></attenuation>')
        L.append('      <cast_shadows>false</cast_shadows>')
        L.append('    </light>')
    L.append('')
    L.append('    <model name="centennial_robotics_museum">')
    L.append('      <static>true</static>')

    for lk in LINKS:
        rpy = lk.get("rpy", (0, 0, 0)) if lk["kind"] == "cylinder" else (0, 0, lk.get("yaw", 0.0))
        pose = f"{lk['x']} {lk['y']} {lk['z']} {rpy[0]} {rpy[1]} {rpy[2]}"
        L.append(f'      <link name="{lk["name"]}">')
        L.append(f'        <pose>{pose}</pose>')
        L.append('        <visual name="visual">')
        L.append('          <geometry>')
        L.append(geom_xml(lk, 12))
        L.append('          </geometry>')
        L.append(mat_xml(lk["mat"], 10))
        if MAT[lk["mat"]].get("alpha", 1.0) < 1.0:
            L.append(f'          <transparency>{1 - MAT[lk["mat"]]["alpha"]:.2f}</transparency>')
        L.append('        </visual>')
        if lk["collision"]:
            L.append('        <collision name="collision">')
            L.append('          <geometry>')
            L.append(geom_xml(lk, 12))
            L.append('          </geometry>')
            L.append('        </collision>')
        L.append('      </link>')

    L.append('    </model>')
    L.append('  </world>')
    L.append('</sdf>')
    return "\n".join(L) + "\n"


# ============================================================================
# MAP RASTERISER
# ============================================================================

def footprints_at(z):
    """Return the 2-D shapes that a laser sweeping at height z would hit."""
    shapes = []
    for lk in LINKS:
        if not lk["collision"] or lk["name"] == "floor":
            continue
        if lk["kind"] == "box":
            lo, hi = lk["z"] - lk["sz"] / 2, lk["z"] + lk["sz"] / 2
            if lo <= z <= hi:
                shapes.append(("rect", lk["x"] - lk["sx"] / 2, lk["y"] - lk["sy"] / 2,
                               lk["x"] + lk["sx"] / 2, lk["y"] + lk["sy"] / 2))
        elif lk["kind"] == "cylinder":
            rpy = lk.get("rpy", (0, 0, 0))
            if abs(rpy[0]) > 1e-3 or abs(rpy[1]) > 1e-3:
                continue                      # lying-down cylinders: skip
            lo, hi = lk["z"] - lk["h"] / 2, lk["z"] + lk["h"] / 2
            if lo <= z <= hi:
                shapes.append(("circ", lk["x"], lk["y"], lk["r"]))
        else:
            if abs(lk["z"] - z) < lk["r"]:
                rr = math.sqrt(lk["r"] ** 2 - (lk["z"] - z) ** 2)
                shapes.append(("circ", lk["x"], lk["y"], rr))
    return shapes


def rasterise():
    import numpy as np
    w = int(round((MAP_MAX_X - MAP_MIN_X) / MAP_RES))
    h = int(round((MAP_MAX_Y - MAP_MIN_Y) / MAP_RES))
    grid = np.full((h, w), 205, dtype=np.uint8)          # 205 = unknown

    xs = MAP_MIN_X + (np.arange(w) + 0.5) * MAP_RES
    ys = MAP_MIN_Y + (np.arange(h) + 0.5) * MAP_RES
    X, Y = np.meshgrid(xs, ys)

    # everything inside the outer shell starts as free space
    inside = (np.abs(X) <= 12.1) & (np.abs(Y) <= 10.1)
    grid[inside] = 254

    occ = np.zeros_like(inside)
    for s in footprints_at(LIDAR_Z):
        if s[0] == "rect":
            _, x0, y0, x1, y1 = s
            occ |= (X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)
        else:
            _, cx, cy, r = s
            occ |= ((X - cx) ** 2 + (Y - cy) ** 2) <= r ** 2
    grid[occ] = 0                                        # 0 = occupied

    return np.flipud(grid), w, h                         # PGM rows go top-down


def check_clearances(grid, w, h):
    import numpy as np
    from scipy import ndimage
    occ = grid == 0
    dist = ndimage.distance_transform_edt(~occ) * MAP_RES
    print(f"{'location':<22}{'x':>8}{'y':>8}{'clearance':>12}  status")
    print("-" * 62)
    ok = True
    for name, loc in LOCATIONS.items():
        col = int((loc["x"] - MAP_MIN_X) / MAP_RES)
        row = h - 1 - int((loc["y"] - MAP_MIN_Y) / MAP_RES)
        d = dist[row, col]
        state = grid[row, col]
        if state != 254:
            tag, ok = "NOT FREE SPACE", False
        elif d < ROBOT_RADIUS:
            tag, ok = "TOO TIGHT", False
        elif d < ROBOT_RADIUS + 0.25:
            tag = "tight but drivable"
        else:
            tag = "ok"
        print(f"{name:<22}{loc['x']:>8.2f}{loc['y']:>8.2f}{d:>10.2f} m  {tag}")
    sx, sy, _ = ROBOT_SPAWN
    col = int((sx - MAP_MIN_X) / MAP_RES)
    row = h - 1 - int((sy - MAP_MIN_Y) / MAP_RES)
    print(f"{'ROBOT SPAWN':<22}{sx:>8.2f}{sy:>8.2f}{dist[row, col]:>10.2f} m")
    free = int((grid == 254).sum())
    print(f"\nmap {w}x{h} @ {MAP_RES} m   free {100*free/grid.size:.1f}%   "
          f"occupied {100*(grid == 0).sum()/grid.size:.1f}%")

    # ---- reachability -----------------------------------------------------
    # Erode the free space by the robot radius, then flood-fill from the spawn
    # pose.  Any waypoint outside that connected component can never be
    # reached no matter how good the planner is - usually it means a doorway
    # is too narrow or a prop is parked in front of one.
    drivable = (grid == 254) & (dist >= ROBOT_RADIUS)
    labels, _ = ndimage.label(drivable)
    home = labels[row, col]
    if home == 0:
        print("SPAWN POSE IS NOT DRIVABLE")
        return False
    print(f"\nreachability from spawn (component {home}, "
          f"{int((labels == home).sum()) * MAP_RES**2:.0f} m2 drivable):")
    for name, loc in LOCATIONS.items():
        c = int((loc["x"] - MAP_MIN_X) / MAP_RES)
        r = h - 1 - int((loc["y"] - MAP_MIN_Y) / MAP_RES)
        reach = labels[r, c] == home
        print(f"  {name:<22}{'reachable' if reach else 'UNREACHABLE'}")
        ok = ok and reach
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="only verify goal clearances, write nothing")
    args = ap.parse_args()

    grid, w, h = rasterise()
    ok = check_clearances(grid, w, h)
    if args.check:
        return 0 if ok else 1
    if not ok:
        print("\nRefusing to write: at least one location is unreachable.")
        return 1

    os.makedirs(os.path.join(PKG_DIR, "worlds"), exist_ok=True)
    os.makedirs(os.path.join(PKG_DIR, "maps"), exist_ok=True)

    sdf_path = os.path.join(PKG_DIR, "worlds", "museum.sdf")
    with open(sdf_path, "w") as f:
        f.write(build_sdf())

    pgm_path = os.path.join(PKG_DIR, "maps", "museum_map.pgm")
    with open(pgm_path, "wb") as f:
        f.write(b"P5\n")
        f.write(b"# Ground-truth map generated by tools/generate_world.py\n")
        f.write(f"{w} {h}\n255\n".encode())
        f.write(grid.tobytes())

    yaml_path = os.path.join(PKG_DIR, "maps", "museum_map.yaml")
    with open(yaml_path, "w") as f:
        f.write("image: museum_map.pgm\n")
        f.write("mode: trinary\n")
        f.write(f"resolution: {MAP_RES}\n")
        f.write(f"origin: [{MAP_MIN_X}, {MAP_MIN_Y}, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")

    # The semantic-to-coordinate mapping the LLM layer consumes.  Exporting it
    # from here rather than hand-typing it in the navigator is the whole point:
    # move a wall in this file and the waypoints can never silently go stale.
    import json
    loc_path = os.path.join(PKG_DIR, "config", "locations.json")
    os.makedirs(os.path.dirname(loc_path), exist_ok=True)
    with open(loc_path, "w") as f:
        json.dump({
            "_comment": "Generated by tools/generate_world.py - do not edit by hand.",
            "frame_id": "map",
            "spawn_pose": {"x": ROBOT_SPAWN[0], "y": ROBOT_SPAWN[1],
                           "yaw": ROBOT_SPAWN[2]},
            "locations": {
                k: {"x": v["x"], "y": v["y"], "yaw": v["yaw"],
                    "description": v["desc"]}
                for k, v in LOCATIONS.items()
            },
        }, f, indent=2)
    print(f"wrote {loc_path}")

    print(f"\nwrote {sdf_path}")
    print(f"wrote {pgm_path}")
    print(f"wrote {yaml_path}")
    print(f"{len(LINKS)} links, {len(LOCATIONS)} named locations")
    return 0


if __name__ == "__main__":
    sys.exit(main())
