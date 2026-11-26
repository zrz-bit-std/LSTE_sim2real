#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Build and validate the enriched real_lab.world from real_lab_walls_only.world."""

from __future__ import annotations

import math
import os
import sys
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np

WS = "/home/zrz/work/sim2real/LSTE"
WALLS_ONLY = os.path.join(WS, "worlds/sim2real/real_lab_walls_only.world")
OUT_WORLD = os.path.join(WS, "worlds/sim2real/real_lab.world")
OUT_PLAN = os.path.join(WS, "docs/assets/real_lab/real_lab_enriched_plan.png")

WALL_H = 2.50
DOOR_WIDTH = 1.80
INWARD_DEPTH = 0.35  # 0.15 m base wall + 0.35 m inward = 0.50 m total doorway depth
SPAWN_XY = (5.20, 0.00)

WALL_OVERRIDES = {
    "wall_00_corr_south_s0": (-1.57, -1.095,  1.000, -0.945),
    "wall_01_corr_south_s1": ( 2.80, -1.095, 13.475, -0.945),
    "wall_02_corr_south_s2": (15.275,-1.095, 21.900, -0.945),
    "wall_03_corr_south_s3": (23.70, -1.095, 27.660, -0.945),
    "wall_04_corr_north_s0": (-1.57, 12.955,  3.100, 13.105),
    "wall_05_corr_north_s1": ( 4.90, 12.955, 16.050, 13.105),
    "wall_06_corr_north_s2": (17.85, 12.955, 27.660, 13.105),
    "wall_07_corr_west_s0":  (-1.575, -0.94, -1.425,  8.940),
    "wall_09_corr_east":     (27.50,  0.850, 27.650, 11.160),
}

INWARD_DOOR_WALLS = [
    ("wall_41_inward_N1_w",  0.670, 13.105,  3.100, 13.105 + INWARD_DEPTH),
    ("wall_42_inward_N1_e",  4.900, 13.105,  7.380, 13.105 + INWARD_DEPTH),
    ("wall_43_inward_N23_w", 13.520, 13.105, 16.050, 13.105 + INWARD_DEPTH),
    ("wall_44_inward_N23_e", 17.850, 13.105, 24.070, 13.105 + INWARD_DEPTH),
    ("wall_45_inward_S1_w",  0.330, -1.095 - INWARD_DEPTH,  1.000, -1.095),
    ("wall_46_inward_S1_e",  2.800, -1.095 - INWARD_DEPTH,  3.920, -1.095),
    ("wall_47_inward_S23_w", 10.020, -1.095 - INWARD_DEPTH, 13.475, -1.095),
    ("wall_48_inward_S23_e", 15.275, -1.095 - INWARD_DEPTH, 18.480, -1.095),
    ("wall_49_inward_S4_w",  20.970, -1.095 - INWARD_DEPTH, 21.900, -1.095),
    ("wall_50_inward_S4_e",  23.700, -1.095 - INWARD_DEPTH, 25.570, -1.095),
    ("wall_51_inward_W_s",  -1.575 - INWARD_DEPTH, -0.950, -1.575,  8.940),
    ("wall_52_inward_E12_n", 27.650,  0.850, 27.650 + INWARD_DEPTH,  9.540),
    ("wall_53_inward_E3_s",  27.650,  9.680, 27.650 + INWARD_DEPTH, 11.160),
    ("wall_54_inward_E3_n",  27.650, 12.960, 27.650 + INWARD_DEPTH, 15.730),
]

DOORS_SPEC = [
    ("D_N1",  "N1",  "H", 13.030,  4.000, 1.80),
    ("D_N23", "N23", "H", 13.030, 16.950, 1.80),
    ("D_S1",  "S1",  "H", -1.020,  1.900, 1.80),
    ("D_S23", "S23", "H", -1.020, 14.375, 1.80),
    ("D_S4",  "S4",  "H", -1.020, 22.800, 1.80),
    ("D_W",   "W",   "V", -1.500,  9.840, 1.80),
    ("D_E12", "E12", "V", 27.575, -0.050, 1.80),
    ("D_E3",  "E3",  "V", 27.575, 12.060, 1.80),
]

MODEL_BOUNDS = {
    "table_conference_2":       (-1.36, 1.34, -0.61, 0.60, 0.00, 0.71),
    "desk_brown":               (-0.94, 0.83, -0.38, 0.42, 0.00, 0.72),
    "office_chair":             (-0.38, 0.38, -0.38, 0.38, 0.00, 0.99),
    "fuel_office_chair_black":  (-0.37, 0.37, -0.39, 0.36, 0.00, 1.25),
    "fuel_office_chair_grey":   (-0.35, 0.36, -0.25, 0.45, 0.00, 1.14),
    "bookshelf_large":          (-0.80, 0.86, -0.21, 0.21, 0.00, 1.84),
    "file_cabinet_small":       (-0.26, 0.35, -0.36, 0.39, 0.00, 0.65),
    "fuel_white_cabinet":       (-0.45, 0.45, -0.23, 0.23, 0.00, 1.08),
    "fuel_metal_cabinet":       (-0.40, 0.40, -0.22, 0.22, 0.00, 1.68),
    "fuel_square_shelf":        (-0.38, 0.38, -0.20, 0.19, 0.00, 0.77),
    "fuel_drawer":              (-0.20, 0.20, -0.26, 0.24, 0.00, 0.63),
    "fuel_refrigerator":        (-0.37, 0.42, -0.36, 0.36, 0.00, 1.73),
    "fuel_mini_sofa":           (-0.36, 0.36, -0.36, 0.33, 0.00, 0.61),
    "fuel_storage_rack":        (-0.63, 0.63, -0.36, 0.36, 0.00, 1.74),
    "fuel_storage_rack_covered":(-0.64, 0.64, -0.39, 0.37, 0.00, 1.76),
    "fuel_large_dry_box":       (-0.22, 0.23, -0.13, 0.13, 0.00, 0.32),
    "fuel_large_grey_box":      (-0.14, 0.14, -0.06, 0.06, 0.00, 0.23),
    "fuel_small_blue_box":      (-0.13, 0.13, -0.05, 0.05, 0.00, 0.23),
    "fuel_mop_cart":            (-0.53, 0.57, -0.26, 0.28, 0.00, 0.92),
    "fuel_trash_bin":           (-0.14, 0.14, -0.19, 0.19, 0.00, 0.45),
    "fuel_fire_extinguisher":   (-0.11, 0.11, -0.06, 0.06, 0.00, 0.43),
    "monitor_1":                (-0.31, 0.29, -0.06, 0.04, 0.00, 0.44),
    "monitor_2":                (-0.31, 0.29, -0.06, 0.04, 0.00, 0.44),
    "monitor_3":                (-0.31, 0.29, -0.06, 0.04, 0.00, 0.44),
    "keyboard":                 (-0.21, 0.23, -0.08, 0.07, 0.00, 0.02),
    "mouse":                    (-0.04, 0.17, -0.02, 0.03, 0.00, 0.02),
    "computer_case":            ( 0.00, 0.20,  0.00, 0.48, 0.00, 0.44),
    "file_1":                   (-0.12, 0.11, -0.04, 0.04, 0.00, 0.31),
    "file_2":                   (-0.04, 0.04, -0.14, 0.14, 0.00, 0.35),
    "file_box_1":               (-0.23, 0.20, -0.18, 0.18, 0.00, 0.28),
    "cup_yellow":               (-0.06, 0.06, -0.04, 0.04, 0.00, 0.08),
    "lamp_table_metal":         (-0.15, 0.15, -0.17, 0.13, 0.00, 0.40),
    "plant_2":                  (-0.35, 0.35, -0.35, 0.35, 0.00, 0.62),
    "ladder_75deg_0rails":      (-0.48, 0.48, -0.07, 1.47, 0.00, 2.52),
    "cardboard_box":            (-0.25, 0.25, -0.20, 0.20, 0.00, 0.30),
    "fire_hydrant":             (-0.21, 0.21, -0.21, 0.21, 0.00, 0.93),
    "fire_hydrant_35":          (-0.08, 0.08, -0.08, 0.08, 0.00, 0.33),
    "construction_cone":        (-0.25, 0.25, -0.25, 0.25, 0.00, 1.09),
    "construction_cone_35":     (-0.09, 0.09, -0.09, 0.09, 0.00, 0.38),
}


def world_aabb(model: str, x: float, y: float, z: float, yaw_deg: float):
    mnx, mxx, mny, mxy, mnz, mxz = MODEL_BOUNDS[model]
    rad = math.radians(yaw_deg)
    c, s = math.cos(rad), math.sin(rad)
    corners = [(px, py) for px in (mnx, mxx) for py in (mny, mxy)]
    xs = [x + px * c - py * s for px, py in corners]
    ys = [y + px * s + py * c for px, py in corners]
    return (min(xs), min(ys), max(xs), max(ys), z + mnz, z + mxz)


def build_objects():
    objs = []
    add = lambda name, model, x, y, z, yaw, role, room: objs.append(
        (name, model, x, y, z, yaw, role, room)
    )

    # =========================================================================
    # 1. CORRIDORS — Negative clues (clear of SPAWN=(5.2,0.0) & all doorways)
    # =========================================================================
    add("corr_s_hyd_1",  "fire_hydrant_35",        8.00,  0.73, 0.0,   0, "neg_corr", "CORR")
    add("corr_s_cone_1", "construction_cone_35",  10.60,  0.72, 0.0,   0, "neg_corr", "CORR")
    add("corr_s_hyd_2",  "fire_hydrant_35",       17.50,  0.73, 0.0,   0, "neg_corr", "CORR")
    add("corr_s_ext_1",  "fuel_fire_extinguisher",20.20,  0.74, 0.0, 180, "neg_corr", "CORR")

    add("corr_n_cone_1", "construction_cone_35",   1.50, 11.26, 0.0,   0, "neg_corr", "CORR")
    add("corr_n_hyd_1",  "fire_hydrant_35",        7.80, 11.25, 0.0,   0, "neg_corr", "CORR")
    add("corr_n_cone_2", "construction_cone_35",  10.50, 11.26, 0.0,   0, "neg_corr", "CORR")
    add("corr_n_hyd_2",  "fire_hydrant_35",       20.80, 11.25, 0.0,   0, "neg_corr", "CORR")
    add("corr_n_ext_1",  "fuel_fire_extinguisher",24.20, 11.24, 0.0,   0, "neg_corr", "CORR")

    add("corr_w_hyd_1",  "fire_hydrant_35",       -1.33,  3.20, 0.0,   0, "neg_corr", "CORR")
    add("corr_w_cone_1", "construction_cone_35",  -1.32,  6.80, 0.0,   0, "neg_corr", "CORR")

    add("corr_e_hyd_1",  "fire_hydrant_35",       25.81,  3.50, 0.0,   0, "neg_corr", "CORR")
    add("corr_e_cone_1", "construction_cone_35",  25.82,  7.20, 0.0,   0, "neg_corr", "CORR")

    add("corr_c_cone_1", "construction_cone_35",  11.60,  4.20, 0.0,   0, "neg_corr", "CORR")
    add("corr_c_hyd_1",  "fire_hydrant_35",       14.48,  7.80, 0.0,   0, "neg_corr", "CORR")

    # =========================================================================
    # 2. CORE TARGET ROOM E3: inner x in [28.00, 37.59], y in [9.68, 15.73]
    #    - Conference tables FLUSH AGAINST NORTH WALL (y=15.73 -> cy=15.11)
    #    - Chairs along South edge of Table 1 & west of Table 2 (keep target clear!)
    #    - Cabinets & new Fuel models FLUSH AGAINST SOUTH WALL (y=9.68) & EAST WALL
    #    - Huge unobstructed central exploration hall (y in [10.6, 13.7])
    # =========================================================================
    # Two conference tables abutted seamlessly flush against North wall (y_max=15.71, gap=0.02m)
    add("e3_conf_1", "table_conference_2", 31.05, 15.11, 0.0, 0, "env", "E3")
    add("e3_conf_2", "table_conference_2", 33.76, 15.11, 0.0, 0, "env", "E3")
    # Small drawer cabinet flush against North wall right next to Table 2
    add("e3_drawer_n1", "fuel_drawer", 35.34, 15.46, 0.0, 180, "clutter", "E3")

    # Chairs tucked cleanly on the South side of Table 1 & Table 2 West end;
    # KEEP x in [33.1, 35.0] south of cup_yellow completely OPEN!
    add("e3_chair_1", "fuel_office_chair_black", 30.30, 14.08, 0.0, 180, "env", "E3")
    add("e3_chair_2", "office_chair",            31.70, 14.08, 0.0, 0, "env", "E3")
    add("e3_chair_3", "fuel_office_chair_grey",  32.65, 14.00, 0.0, 180, "env", "E3")

    # Desktop items on Table 1 (z=0.71)
    add("e3_mon_1",   "monitor_1",     30.30, 15.20, 0.71, 0, "env",     "E3")
    add("e3_kb_1",    "keyboard",      30.30, 14.85, 0.71, 0, "clutter", "E3")
    add("e3_mouse_1", "mouse",         30.62, 14.85, 0.71, 0, "clutter", "E3")
    add("e3_mon_2",   "monitor_2",     31.70, 15.20, 0.71, 0, "env",     "E3")
    add("e3_kb_2",    "keyboard",      31.70, 14.85, 0.71, 0, "clutter", "E3")
    add("e3_file_1",  "file_box_1",    31.02, 15.28, 0.71, 0, "env",     "E3")
    add("e3_case_1",  "computer_case", 29.78, 15.05, 0.71, 0, "clutter", "E3")

    # Desktop items on Table 2 (z=0.71): TARGET + TARGET_CTX (table lamp + plant)
    add("e3_mon_3",     "monitor_3",        32.68, 15.25, 0.71, 0, "env",     "E3")
    add("e3_kb_3",      "keyboard",         32.68, 14.88, 0.71, 0, "clutter", "E3")
    add("e3_ctx_lamp",  "lamp_table_metal", 33.42, 14.74, 0.71, 0, "ctx",     "E3")
    add("cup_yellow",   "cup_yellow",       33.96, 14.66, 0.71, 0, "target",  "E3")
    add("e3_ctx_plant", "plant_2",          34.62, 14.94, 0.71, 0, "ctx",     "E3")
    add("e3_file_2",    "file_1",           34.92, 15.42, 0.71, 0, "env",     "E3")

    # South wall (y=9.68): flush row of new Fuel cabinets, shelves, trash bin & refrigerator
    add("e3_mcab_s1",  "fuel_metal_cabinet", 29.45,  9.92, 0.0, 0, "clutter", "E3")
    add("e3_mcab_s2",  "fuel_metal_cabinet", 30.27,  9.92, 0.0, 0, "clutter", "E3")
    add("e3_wcab_s1",  "fuel_white_cabinet", 31.14,  9.93, 0.0, 0, "clutter", "E3")
    add("e3_wcab_s2",  "fuel_white_cabinet", 32.06,  9.93, 0.0, 0, "clutter", "E3")
    add("e3_shelf_s1", "fuel_square_shelf",  32.92,  9.90, 0.0, 0, "env",     "E3")
    add("e3_shelf_s2", "fuel_square_shelf",  33.70,  9.90, 0.0, 0, "env",     "E3")
    add("e3_trash_s1", "fuel_trash_bin",     34.26,  9.89, 0.0, 0, "clutter", "E3")
    add("e3_fridge_s1","fuel_refrigerator",  34.95, 10.06, 0.0, 180, "clutter", "E3")

    # East wall (x=37.59): flush bookshelves + mini lounge sofa
    add("e3_shelf_e1", "bookshelf_large", 37.36, 11.20, 0.0, -90, "env",     "E3")
    add("e3_shelf_e2", "bookshelf_large", 37.36, 12.88, 0.0, -90, "env",     "E3")
    add("e3_sofa_e1",  "fuel_mini_sofa",  37.21, 14.55, 0.0,  90, "clutter", "E3")

    # =========================================================================
    # 3. DECOY ROOM S23 (unchanged — approved by user)
    # =========================================================================
    for idx, dx in enumerate((12.50, 14.28, 16.06), start=1):
        add(f"s23_desk_{idx}",  "desk_brown",   dx,        -8.51, 0.0,  0, "env",     "S23")
        add(f"s23_mon_{idx}",   f"monitor_{idx}", dx,      -8.58, 0.72, 0, "env",     "S23")
        add(f"s23_kb_{idx}",    "keyboard",     dx,        -8.32, 0.72, 0, "clutter", "S23")
        add(f"s23_mouse_{idx}", "mouse",        dx + 0.30, -8.32, 0.72, 0, "clutter", "S23")
        add(f"s23_chair_{idx}", "office_chair", dx,        -7.66, 0.0,  0, "env",     "S23")
    add("s23_file_1",   "file_box_1",         13.38, -8.52, 0.72,  0, "env",     "S23")
    add("s23_file_2",   "file_1",             15.16, -8.56, 0.72,  0, "env",     "S23")
    add("s23_shelf_w1", "bookshelf_large",    10.25, -4.60, 0.0,  90, "env",     "S23")
    add("s23_shelf_w2", "bookshelf_large",    10.25, -6.28, 0.0,  90, "env",     "S23")
    add("s23_cab_e1",   "fuel_white_cabinet", 18.23, -5.00, 0.0,  90, "clutter", "S23")
    add("s23_cab_e2",   "fuel_white_cabinet", 18.23, -5.92, 0.0,  90, "clutter", "S23")
    add("s23_trash_1",  "fuel_trash_bin",     18.26, -6.62, 0.0,  90, "clutter", "S23")

    # =========================================================================
    # 4. DECOY ROOM N23 (unchanged)
    # =========================================================================
    for idx, dx in enumerate((15.40, 17.18, 18.96, 20.74), start=1):
        mon = f"monitor_{((idx - 1) % 3) + 1}"
        add(f"n23_desk_{idx}",  "desk_brown",   dx,        20.45, 0.0,  180, "env",     "N23")
        add(f"n23_mon_{idx}",   mon,            dx,        20.52, 0.72, 180, "env",     "N23")
        add(f"n23_kb_{idx}",    "keyboard",     dx,        20.26, 0.72, 180, "clutter", "N23")
        add(f"n23_chair_{idx}", "office_chair", dx,        19.60, 0.0,  180, "env",     "N23")
    add("n23_file_1",   "file_box_1",         16.25, 20.48, 0.72, 180, "env",     "N23")
    add("n23_file_2",   "file_2",             18.08, 20.48, 0.72, 180, "env",     "N23")
    add("n23_shelf_e1", "bookshelf_large",    23.84, 16.00, 0.0,  -90, "env",     "N23")
    add("n23_shelf_e2", "bookshelf_large",    23.84, 17.68, 0.0,  -90, "env",     "N23")
    add("n23_cab_w1",   "fuel_white_cabinet", 13.77, 16.40, 0.0,  -90, "clutter", "N23")
    add("n23_cab_w2",   "fuel_white_cabinet", 13.77, 17.32, 0.0,  -90, "clutter", "N23")

    # =========================================================================
    # 5. NEGATIVE SMALL WAREHOUSE S1 (unchanged — approved by user)
    # =========================================================================
    add("s1_ladder_1", "ladder_75deg_0rails", 1.95, -3.76, 0.0, 180, "neg_wh", "S1")
    for idx, by in enumerate((-3.50, -3.92, -4.34, -4.76)):
        add(f"s1_box_w{idx}a", "cardboard_box", 0.59, by, 0.00, 0, "neg_wh", "S1")
        add(f"s1_box_w{idx}b", "cardboard_box", 0.59, by, 0.30, 0, "neg_wh", "S1")
        add(f"s1_box_e{idx}a", "cardboard_box", 3.66, by, 0.00, 0, "neg_wh", "S1")
        add(f"s1_box_e{idx}b", "cardboard_box", 3.66, by, 0.30, 0, "neg_wh", "S1")

    # =========================================================================
    # 6. NEGATIVE WAREHOUSE E12: inner x in [28.00, 37.59], y in [-0.95, 9.54]
    #    - Doorway sightline from South corridor (y < 2.00) COMPLETELY EMPTY!
    #      Nothing is visible down the South corridor; warehouse clutter begins at y >= 2.25.
    # =========================================================================
    add("e12_ladder_1", "ladder_75deg_0rails", 32.80, 3.60, 0.0, 90, "neg_wh", "E12")

    # Mid-warehouse aisle stack (y=2.45, clear of South-corridor line of sight)
    for idx, bx in enumerate((31.80, 32.32, 32.84, 33.36)):
        add(f"e12_box_m{idx}a", "cardboard_box", bx, 2.45, 0.00, 0, "neg_wh", "E12")
        add(f"e12_box_m{idx}b", "cardboard_box", bx, 2.45, 0.30, 0, "neg_wh", "E12")
    add("e12_dry_m1",  "fuel_large_dry_box",  34.00, 2.45, 0.00, 0, "neg_wh", "E12")
    add("e12_dry_m2",  "fuel_large_dry_box",  34.00, 2.45, 0.32, 0, "neg_wh", "E12")
    add("e12_grey_m1", "fuel_large_grey_box", 34.48, 2.45, 0.00, 0, "neg_wh", "E12")
    add("e12_blue_m1", "fuel_small_blue_box", 34.82, 2.45, 0.00, 0, "neg_wh", "E12")

    # East wall (x=37.59): warehouse racking bay starting at y=2.70 (hidden from South corridor)
    add("e12_rack_e2", "fuel_storage_rack",         37.21, 2.70, 0.0, 90, "neg_wh", "E12")
    add("e12_rack_e3", "fuel_storage_rack_covered", 37.18, 4.02, 0.0, 90, "neg_wh", "E12")
    add("e12_rack_e4", "fuel_storage_rack",         37.21, 5.34, 0.0, 90, "neg_wh", "E12")
    add("e12_rack_e5", "fuel_storage_rack_covered", 37.18, 6.66, 0.0, 90, "neg_wh", "E12")
    add("e12_rack_e6", "fuel_storage_rack",         37.21, 7.98, 0.0, 90, "neg_wh", "E12")

    # North wall (y=9.54): flush cabinets + racks + cardboard boxes
    add("e12_mcab_n1", "fuel_metal_cabinet",        29.30, 9.30, 0.0, 180, "neg_wh", "E12")
    add("e12_mcab_n2", "fuel_metal_cabinet",        30.12, 9.30, 0.0, 180, "neg_wh", "E12")
    add("e12_rack_n1", "fuel_storage_rack_covered", 31.40, 9.15, 0.0, 180, "neg_wh", "E12")
    add("e12_rack_n2", "fuel_storage_rack",         32.72, 9.16, 0.0, 180, "neg_wh", "E12")
    for idx, bx in enumerate((33.80, 34.32, 34.84, 35.36)):
        add(f"e12_box_n{idx}a", "cardboard_box", bx, 9.32, 0.00, 0, "neg_wh", "E12")
        add(f"e12_box_n{idx}b", "cardboard_box", bx, 9.32, 0.30, 0, "neg_wh", "E12")

    # Mid-north warehouse zone: 2nd ladder + mop cart + West wall inner boxes
    add("e12_ladder_2", "ladder_75deg_0rails", 35.80, 6.20, 0.0,  90, "neg_wh", "E12")
    add("e12_mop_1",    "fuel_mop_cart",       31.50, 5.80, 0.0,  90, "neg_wh", "E12")
    for idx, by in enumerate((4.50, 4.92, 5.34, 5.76)):
        add(f"e12_box_w{idx}a", "cardboard_box", 28.28, by, 0.00, 0, "neg_wh", "E12")
        add(f"e12_box_w{idx}b", "cardboard_box", 28.28, by, 0.30, 0, "neg_wh", "E12")

    # =========================================================================
    # 7. NEGATIVE LARGE WAREHOUSE W: inner x in [-11.51, -1.925], y in [-0.95, 10.74]
    #    - Doorway entry zone x in [-5.40, -1.925] COMPLETELY CLEAR (no blocking!)
    #    - Rich warehouse interior with ladders, stacked cardboard boxes,
    #      storage racks, covered racks, dry/grey/blue crates, metal cabinets, mop cart
    # =========================================================================
    # Primary doorway sightline (deeper inside at x=-7.60, y=9.70) — clearly framed from doorway!
    add("w_ladder_1", "ladder_75deg_0rails", -7.60, 9.70, 0.0, -90, "neg_wh", "W")

    # North wall (y=10.74): flush stacked cardboard boxes + crates starting at x=-5.60
    for idx, bx in enumerate((-5.60, -6.12, -6.64, -8.30, -8.82, -9.34)):
        add(f"w_box_n{idx}a", "cardboard_box", bx, 10.52, 0.00, 0, "neg_wh", "W")
        add(f"w_box_n{idx}b", "cardboard_box", bx, 10.52, 0.30, 0, "neg_wh", "W")
    add("w_dry_n1",  "fuel_large_dry_box",  -9.95, 10.58, 0.00, 0, "neg_wh", "W")
    add("w_dry_n2",  "fuel_large_dry_box",  -9.95, 10.58, 0.32, 0, "neg_wh", "W")
    add("w_blue_n1", "fuel_small_blue_box", -10.45, 10.62, 0.00, 0, "neg_wh", "W")

    # Mid-north aisle stack (visible from doorway without blocking entrance!)
    for idx, bx in enumerate((-6.20, -6.72, -7.24)):
        add(f"w_box_m{idx}a", "cardboard_box", bx, 7.60, 0.00, 0, "neg_wh", "W")
        add(f"w_box_m{idx}b", "cardboard_box", bx, 7.60, 0.30, 0, "neg_wh", "W")
    add("w_dry_m1",  "fuel_large_dry_box",  -7.85, 7.60, 0.00, 0, "neg_wh", "W")
    add("w_grey_m1", "fuel_large_grey_box", -8.30, 7.60, 0.00, 0, "neg_wh", "W")

    # West wall (x=-11.51): full warehouse racking bay
    add("w_rack_w1", "fuel_storage_rack",         -11.13, 1.20, 0.0, 90, "neg_wh", "W")
    add("w_rack_w2", "fuel_storage_rack_covered", -11.10, 2.52, 0.0, 90, "neg_wh", "W")
    add("w_rack_w3", "fuel_storage_rack",         -11.13, 3.84, 0.0, 90, "neg_wh", "W")
    add("w_rack_w4", "fuel_storage_rack_covered", -11.10, 5.16, 0.0, 90, "neg_wh", "W")
    add("w_rack_w5", "fuel_storage_rack",         -11.13, 6.48, 0.0, 90, "neg_wh", "W")
    add("w_rack_w6", "fuel_storage_rack",         -11.13, 7.80, 0.0, 90, "neg_wh", "W")

    # South wall (y=-0.95): flush racks + metal cabinets + cardboard boxes
    add("w_rack_s1", "fuel_storage_rack_covered", -9.30, -0.55, 0.0, 0, "neg_wh", "W")
    add("w_rack_s2", "fuel_storage_rack",         -7.98, -0.57, 0.0, 0, "neg_wh", "W")
    add("w_mcab_s1", "fuel_metal_cabinet",        -6.60, -0.71, 0.0, 0, "neg_wh", "W")
    add("w_mcab_s2", "fuel_metal_cabinet",        -5.78, -0.71, 0.0, 0, "neg_wh", "W")
    for idx, bx in enumerate((-4.80, -4.28, -3.76, -3.24)):
        add(f"w_box_s{idx}a", "cardboard_box", bx, -0.73, 0.00, 0, "neg_wh", "W")
        add(f"w_box_s{idx}b", "cardboard_box", bx, -0.73, 0.30, 0, "neg_wh", "W")

    # Mid-south warehouse zone: 2nd ladder + mop cart + East inner wall boxes
    add("w_ladder_2", "ladder_75deg_0rails", -9.80, 4.20, 0.0, -90, "neg_wh", "W")
    add("w_mop_1",    "fuel_mop_cart",       -5.20, 4.20, 0.0, 90, "neg_wh", "W")
    for idx, by in enumerate((2.60, 3.02, 3.44, 3.86)):
        add(f"w_box_e{idx}a", "cardboard_box", -2.20, by, 0.00, 0, "neg_wh", "W")
        add(f"w_box_e{idx}b", "cardboard_box", -2.20, by, 0.30, 0, "neg_wh", "W")

    return objs


def emit_and_load_walls(objs):
    tree = ET.parse(WALLS_ONLY)
    root = tree.getroot()
    # Update ground_plane to dark grey (less harsh than pure black)
    gp_mat = root.find(".//model[@name='ground_plane']//visual/material")
    if gp_mat is not None:
        sc_el = gp_mat.find("script")
        if sc_el is not None:
            gp_mat.remove(sc_el)
        amb = gp_mat.find("ambient")
        if amb is not None:
            amb.text = "0.56 0.54 0.49 1"
        dif = gp_mat.find("diffuse")
        if dif is not None:
            dif.text = "0.62 0.60 0.54 1"
        spec = gp_mat.find("specular")
        if spec is not None:
            spec.text = "0.02 0.02 0.02 1"

    model = root.find(".//model[@name='real_lab']")
    assert model is not None

    for link in model.findall("link"):
        name = link.attrib["name"]
        if name in WALL_OVERRIDES:
            x0, y0, x1, y1 = WALL_OVERRIDES[name]
            cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
            sx, sy = x1 - x0, y1 - y0
            link.find("pose").text = f"{cx:.4f} {cy:.4f} {WALL_H/2:.3f} 0 -0 0"
            for box_sz in link.findall(".//geometry/box/size"):
                box_sz.text = f"{sx:.4f} {sy:.4f} {WALL_H:.2f}"

    for lname, x0, y0, x1, y1 in INWARD_DOOR_WALLS:
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        sx, sy = x1 - x0, y1 - y0
        link_xml = f"""
      <link name='{lname}'>
        <pose>{cx:.4f} {cy:.4f} {WALL_H/2:.3f} 0 -0 0</pose>
        <collision name='collision'>
          <geometry><box><size>{sx:.4f} {sy:.4f} {WALL_H:.2f}</size></box></geometry>
        </collision>
        <visual name='visual'>
          <geometry><box><size>{sx:.4f} {sy:.4f} {WALL_H:.2f}</size></box></geometry>
          <material>
            <script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Wood</name></script>
          </material>
        </visual>
      </link>"""
        model.append(ET.fromstring(link_xml))

    walls = []
    for link in model.findall("link"):
        name = link.attrib["name"]
        p = [float(v) for v in link.findtext("pose").split()]
        sz = [float(v) for v in link.findtext("collision/geometry/box/size").split()]
        walls.append((name, p[0] - sz[0]/2, p[1] - sz[1]/2, p[0] + sz[0]/2, p[1] + sz[1]/2))
        # Visual-only black baseboard line where wall meets ground (zero collision impact)
        trim_xml = f"""
        <visual name='baseboard'>
          <pose>0 0 {-WALL_H/2 + 0.02:.3f} 0 0 0</pose>
          <geometry><box><size>{sz[0] + 0.014:.4f} {sz[1] + 0.014:.4f} 0.040</size></box></geometry>
          <material>
            <script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Black</name></script>
            <ambient>0.03 0.03 0.03 1</ambient>
            <diffuse>0.04 0.04 0.04 1</diffuse>
          </material>
        </visual>"""
        link.append(ET.fromstring(trim_xml))

    world = root.find("world")
    gui_el = world.find("gui")
    gui_idx = list(world).index(gui_el) if gui_el is not None else len(world)
    for name, model_uri, x, y, z, yaw, role, room in objs:
        inc_xml = f"""
    <include>
      <uri>model://{model_uri}</uri>
      <name>{name}</name>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 -0 {math.radians(yaw):.6f}</pose>
      <static>1</static>
    </include>"""
        world.insert(gui_idx, ET.fromstring(inc_xml))
        gui_idx += 1

    ET.indent(tree, space="  ") if hasattr(ET, "indent") else None
    tree.write(OUT_WORLD, encoding="utf-8", xml_declaration=True)
    ET.parse(OUT_WORLD)
    print(f"Wrote {OUT_WORLD} ({os.path.getsize(OUT_WORLD)} bytes, {len(walls)} wall links, {len(objs)} objects)")
    return walls


def validate(walls, objs):
    print(f"=== Validating {len(objs)} objects against {len(walls)} walls ===")
    errors = 0

    for did, room, axis, at, center, w in DOORS_SPEC:
        if axis == "H":
            sx0, sy0, sx1, sy1 = center - w/2 + 0.01, at - 0.50, center + w/2 - 0.01, at + 0.50
        else:
            sx0, sy0, sx1, sy1 = at - 0.50, center - w/2 + 0.01, at + 0.50, center + w/2 - 0.01
        bad = [wn for wn, wx0, wy0, wx1, wy1 in walls
               if min(sx1, wx1) - max(sx0, wx0) > 0.01 and min(sy1, wy1) - max(sy0, wy0) > 0.01]
        if bad:
            print(f"  [DOOR-BLOCK] {did}: {bad}")
            errors += 1

    for name, model, x, y, z, yaw, role, room in objs:
        a = world_aabb(model, x, y, z, yaw)
        for wname, wx0, wy0, wx1, wy1 in walls:
            dx = min(a[2], wx1) - max(a[0], wx0)
            dy = min(a[3], wy1) - max(a[1], wy0)
            if dx > 0.015 and dy > 0.015:
                print(f"  [OVERLAP-WALL] {name} ({model}) <-> {wname}: dx={dx:.3f} dy={dy:.3f}")
                errors += 1

    aabbs = [world_aabb(m, x, y, z, yaw) for _, m, x, y, z, yaw, _, _ in objs]
    for i in range(len(objs)):
        for j in range(i + 1, len(objs)):
            a, b = aabbs[i], aabbs[j]
            dx = min(a[2], b[2]) - max(a[0], b[0])
            dy = min(a[3], b[3]) - max(a[1], b[1])
            dz = min(a[5], b[5]) - max(a[4], b[4])
            if dx > 0.02 and dy > 0.02 and dz > 0.02:
                print(f"  [OVERLAP-OBJ] {objs[i][0]} <-> {objs[j][0]}: dx={dx:.3f} dy={dy:.3f} dz={dz:.3f}")
                errors += 1

    res = 0.05
    x0, y0, x1, y1 = -12.5, -10.0, 38.5, 22.0
    W, H = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
    occ_all = np.zeros((H, W), dtype=bool)
    for _, wx0, wy0, wx1, wy1 in walls:
        occ_all[max(0,int((wy0-y0)/res)):min(H,int(math.ceil((wy1-y0)/res))),
                max(0,int((wx0-x0)/res)):min(W,int(math.ceil((wx1-x0)/res)))] = True
    for idx, (name, model, x, y, z, yaw, role, room) in enumerate(objs):
        a = aabbs[idx]
        if a[4] >= 0.65:
            continue
        occ_all[max(0,int((a[1]-y0)/res)):min(H,int(math.ceil((a[3]-y0)/res))),
                max(0,int((a[0]-x0)/res)):min(W,int(math.ceil((a[2]-x0)/res)))] = True

    rad_cells = int(math.ceil(0.35 / res))
    inflated = occ_all.copy()
    for dr in range(-rad_cells, rad_cells + 1):
        for dc in range(-rad_cells, rad_cells + 1):
            if dr*dr + dc*dc <= rad_cells*rad_cells:
                r0s, r1s = max(0, -dr), min(H, H - dr)
                c0s, c1s = max(0, -dc), min(W, W - dc)
                inflated[r0s+dr:r1s+dr, c0s+dc:c1s+dc] |= occ_all[r0s:r1s, c0s:c1s]

    free = ~inflated
    sr, sc = int((SPAWN_XY[1] - y0)/res), int((SPAWN_XY[0] - x0)/res)
    seen = np.zeros_like(free)
    if free[sr, sc]:
        q = deque([(sr, sc)]); seen[sr, sc] = True
        while q:
            r, c = q.popleft()
            for dr, dc in ((1,0),(-1,0),(0,1),(0,-1)):
                nr, nc = r+dr, c+dc
                if 0 <= nr < H and 0 <= nc < W and free[nr, nc] and not seen[nr, nc]:
                    seen[nr, nc] = True; q.append((nr, nc))

    checkpoints = [
        ("South corridor mid",    12.5,  0.0),
        ("North corridor mid",    12.5, 12.0),
        ("West corridor mid",     -0.5,  5.0),
        ("East corridor mid",     26.6,  5.0),
        ("Central corridor",      13.0,  6.0),
        ("N1 inside",              4.0, 16.5),
        ("N23 inside",            17.0, 16.5),
        ("S1 doorway buffer",      1.9, -2.2),
        ("S23 inside",            14.3, -4.5),
        ("S4 inside",             23.2, -4.5),
        ("W entry hall clear",    -3.5,  9.8),
        ("W interior aisle",      -4.5,  6.0),
        ("E12 entry hall clear",  29.8,  0.0),
        ("E12 interior aisle",    30.5,  3.5),
        ("E3 entry hall",         29.5, 12.1),
        ("E3 center hall",        32.5, 12.2),
        ("E3 right in front cup", 33.96,13.6),
    ]
    for label, cx, cy in checkpoints:
        r, c = int((cy - y0)/res), int((cx - x0)/res)
        ok = bool(seen[r, c])
        print(f"  BFS [{ 'OK ' if ok else 'FAIL' }] {label:24s} ({cx:6.2f}, {cy:6.2f})")
        if not ok:
            errors += 1

    return errors, seen, (x0, y0, x1, y1, res)


def render_plan(walls, objs, seen, grid_meta):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Patch

    x0, y0, x1, y1, res = grid_meta
    fig, ax = plt.subplots(figsize=(16, 10), dpi=150)
    ax.set_facecolor("#1e1e1e")

    rgba = np.zeros((*seen.shape, 4), dtype=float)
    rgba[seen] = [0.22, 0.55, 0.35, 0.32]
    ax.imshow(rgba, origin="lower", extent=[x0, x1, y0, y1], interpolation="nearest")

    for wname, wx0, wy0, wx1, wy1 in walls:
        fc = "#c8a97e" if "inward_" in wname else "#d0d0d0"
        ax.add_patch(Rectangle((wx0, wy0), wx1 - wx0, wy1 - wy0, fc=fc, ec="#ffffff", lw=0.4, zorder=3))

    role_colors = {
        "target":   ("#ffd700", "#ff0000", 6),
        "ctx":      ("#00e5ff", "#006064", 5),
        "env":      ("#42a5f5", "#0d47a1", 4),
        "clutter":  ("#90a4ae", "#455a64", 4),
        "neg_wh":   ("#ff7043", "#bf360c", 4),
        "neg_corr": ("#ef5350", "#b71c1c", 5),
    }
    for name, model, x, y, z, yaw, role, room in objs:
        a = world_aabb(model, x, y, z, yaw)
        fc, ec, zo = role_colors.get(role, ("#cccccc", "#333333", 4))
        ax.add_patch(Rectangle((a[0], a[1]), a[2] - a[0], a[3] - a[1], fc=fc, ec=ec, lw=0.8, alpha=0.9, zorder=zo))

    ax.plot(SPAWN_XY[0], SPAWN_XY[1], marker=">", markersize=11, color="#00ff88", markeredgecolor="black", zorder=7)
    ax.text(SPAWN_XY[0], SPAWN_XY[1] - 0.58, f"SPAWN ({SPAWN_XY[0]:.1f},{SPAWN_XY[1]:.1f})",
            color="#00ff88", fontsize=8, ha="center", weight="bold", zorder=8)
    ax.plot(33.96, 14.66, marker="*", markersize=13, color="#ffd700", markeredgecolor="#d50000", zorder=8)

    labels = [
        ("N1\n(Empty, 1.8m door)",         4.0, 17.0, "#bbbbbb"),
        ("N23\n(Decoy, 1.8m door)",       18.8, 16.5, "#64b5f6"),
        ("S1\n(Neg Warehouse, 1.8m)",      2.1, -2.5, "#ffab91"),
        ("S23\n(Decoy, 1.8m door)",       14.2, -4.5, "#64b5f6"),
        ("S4\n(Empty, 1.8m door)",        23.2, -5.0, "#bbbbbb"),
        ("W\n(Rich Neg Warehouse)",       -6.5,  5.8, "#ffab91"),
        ("E12\n(Rich Neg Warehouse)",     31.2,  3.8, "#ffab91"),
        ("E3\n(TARGET: N-wall tables)",   32.4, 12.1, "#ffd700"),
    ]
    for txt, lx, ly, col in labels:
        ax.text(lx, ly, txt, color=col, fontsize=9.2, ha="center", va="center", weight="bold",
                bbox=dict(boxstyle="round,pad=0.25", fc="#111111", ec=col, alpha=0.75), zorder=9)

    legend_items = [
        Patch(fc="#c8a97e", ec="#ffffff", label="Inward doorway thickening (+0.35m, total depth 0.50m)"),
        Patch(fc="#ffd700", ec="#ff0000", label="Target: cup_yellow (E3 North-wall table)"),
        Patch(fc="#00e5ff", ec="#006064", label="Target Ctx: lamp_table_metal + plant_2 (E3 only)"),
        Patch(fc="#42a5f5", ec="#0d47a1", label="Env positive: desks / monitors / chairs / shelves"),
        Patch(fc="#90a4ae", ec="#455a64", label="Clutter & new Fuel props: cabinets / fridge / sofa / kb"),
        Patch(fc="#ff7043", ec="#bf360c", label="Warehouse negative: ladders / boxes / racks / crates / cart"),
        Patch(fc="#ef5350", ec="#b71c1c", label="Corridor negative: cones / hydrants / extinguishers"),
        Patch(fc="#2e6b44", ec="#2e6b44", alpha=0.5, label="Reachable free space (BFS, clearance=0.35m)"),
    ]
    leg = ax.legend(handles=legend_items, loc="upper left", fontsize=8.1, facecolor="#222222", edgecolor="#666666")
    for txt in leg.get_texts():
        txt.set_color("white")
    ax.set_xlim(-12.2, 38.3)
    ax.set_ylim(-9.6, 21.6)
    ax.set_aspect("equal")
    ax.set_title("real_lab.world v2 — N-wall E3, Rich Unblocked Warehouses (E12/W), Spawn=(5.2,0)", color="white", fontsize=13)
    ax.tick_params(colors="#cccccc")
    plt.tight_layout()
    plt.savefig(OUT_PLAN, facecolor=fig.get_facecolor())
    print(f"Saved annotated floor plan: {OUT_PLAN}")


if __name__ == "__main__":
    objs = build_objects()
    walls = emit_and_load_walls(objs)
    errors, seen, grid_meta = validate(walls, objs)
    render_plan(walls, objs, seen, grid_meta)
    if errors:
        print(f"VALIDATION_FAILED with {errors} error(s)")
        sys.exit(1)
    print("VALIDATION_PASSED: 0 errors")
