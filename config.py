"""config.py

Central configuration for the GLOSA simulation suite.

HOW TO SELECT A STUDY / VARIANT
--------------------------------
Set the four variables directly below:

    ACTIVE_STUDY   = "A"               # "A" → 12-hour study  |  "B" → 1-hour study
    ACTIVE_VARIANT = "Conv_above_60"   # Study B only — ignored when ACTIVE_STUDY = "A"
    SIM_BEGIN      = 39600             # simulation start time in seconds
    SIM_END        = 43200             # simulation end time in seconds

Simulation time window:
    Study A (12H):  SIM_BEGIN = 0      SIM_END = 43200   (00:00 to 12:00)
    Study B (1H):   SIM_BEGIN = 39600  SIM_END = 43200   (11:00 to 12:00)

Valid ACTIVE_VARIANT values for Study B:
    "Conv_above_60"               Conventional bicycle, rider age > 60
    "Conv_below_40"               Conventional bicycle, rider age < 40
    "Conv_between_40_and_60"      Conventional bicycle, rider age between 40-60
    "Ebike_above_60"              E-bike, rider age > 60
    "Ebike_below_40"              E-bike, rider age < 40
    "Ebike_between_40_and_60"     E-bike, rider age between 40-60
    "SPedelec_above_60"           S-Pedelec, rider age > 60
    "SPedelec_below_40"           S-Pedelec, rider age < 40
    "SPedelec_between_40_and_60"  S-Pedelec, rider age between 40-60

After changing these four values, run any simulation script — no other file needs editing.
"""

from __future__ import annotations

import os

# =========================================================
# >>>  EDIT THESE FOUR LINES BEFORE EACH RUN  <<<
# =========================================================
ACTIVE_STUDY   = "B"               # "A" or "B"
ACTIVE_VARIANT = "Conv_above_60"   # Study B only (ignored for Study A)

SIM_BEGIN = 39600   # seconds  —  Study A: 0 (00:00)     |  Study B: 39600 (11:00)
SIM_END   = 43200   # seconds  —  Study A: 43200 (12:00) |  Study B: 43200 (12:00)
# =========================================================


# =========================================================
# Internal path resolution — do not edit below this line
# =========================================================

_ROOT          = os.path.dirname(os.path.abspath(__file__))
SUMO_FILES_DIR = os.path.join(_ROOT, "sumo_files")
NET_DIR        = os.path.join(SUMO_FILES_DIR, "net")
ROUTES_DIR     = os.path.join(SUMO_FILES_DIR, "routes")

SUMO_CONFIG_PATH = os.path.join(SUMO_FILES_DIR, "ingolstadt_tiny.sumocfg")

# Network file — shared by both studies
NET_FILE = os.path.join(NET_DIR, "ingolstadt_tiny.net.xml")

# Vehicle route files per study
_VEHICLE_ROUTES = {
    "A": "ingolstadt_tiny_vehicles_12H.rou.xml",
    "B": "ingolstadt_tiny_vehicles_12H.rou.xml",  # same vehicle routes for all Study B variants but only the 11:00-12:00 portion is used
}

# Bicycle route files per study / variant
_BIKE_ROUTES = {
    "A": "ingolstadt_tiny_bikes_12H.rou.xml",
    "B": {
        "Conv_above_60":              "ingolstadt_tiny_bikes_1H_Conv_above_60.rou.xml",
        "Conv_below_40":              "ingolstadt_tiny_bikes_1H_Conv_below_40.rou.xml",
        "Conv_between_40_and_60":     "ingolstadt_tiny_bikes_1H_Conv_between_40_and_60.rou.xml",
        "Ebike_above_60":             "ingolstadt_tiny_bikes_1H_Ebike_above_60.rou.xml",
        "Ebike_below_40":             "ingolstadt_tiny_bikes_1H_Ebike_below_40.rou.xml",
        "Ebike_between_40_and_60":    "ingolstadt_tiny_bikes_1H_Ebike_between_40_and_60.rou.xml",
        "SPedelec_above_60":          "ingolstadt_tiny_bikes_1H_SPedelec_above_60.rou.xml",
        "SPedelec_below_40":          "ingolstadt_tiny_bikes_1H_SPedelec_below_40.rou.xml",
        "SPedelec_between_40_and_60": "ingolstadt_tiny_bikes_1H_SPedelec_between_40_and_60.rou.xml",
    },
}

# Validate study selection
if ACTIVE_STUDY not in ("A", "B"):
    raise ValueError(f"ACTIVE_STUDY must be 'A' or 'B', got '{ACTIVE_STUDY}'.")

VEHICLE_ROUTE_FILE = os.path.join(ROUTES_DIR, _VEHICLE_ROUTES[ACTIVE_STUDY])

if ACTIVE_STUDY == "A":
    BIKE_ROUTE_FILE = os.path.join(ROUTES_DIR, _BIKE_ROUTES["A"])
else:
    _b = _BIKE_ROUTES["B"]
    if ACTIVE_VARIANT not in _b:
        raise ValueError(
            f"Unknown ACTIVE_VARIANT '{ACTIVE_VARIANT}' for Study B.\n"
            f"Valid options: {list(_b.keys())}"
        )
    BIKE_ROUTE_FILE = os.path.join(ROUTES_DIR, _b[ACTIVE_VARIANT])


# =========================================================
# Output directory — auto-resolved from study / variant
# =========================================================

_OUTPUTS_ROOT = os.path.join(_ROOT, "outputs")

if ACTIVE_STUDY == "A":
    _STUDY_DIR = os.path.join(_OUTPUTS_ROOT, "study_A")
else:
    _STUDY_DIR = os.path.join(_OUTPUTS_ROOT, "study_B", ACTIVE_VARIANT)

OUTPUT_DIRS = {
    "baseline":        os.path.join(_STUDY_DIR, "baseline"),
    "glosa":           os.path.join(_STUDY_DIR, "glosa"),
    "vru_aware_glosa": os.path.join(_STUDY_DIR, "vru_aware_glosa"),
}

for _d in OUTPUT_DIRS.values():
    os.makedirs(_d, exist_ok=True)


def output_path(scenario: str, filename: str) -> str:
    """Return the absolute path for an output file in the correct scenario folder.

    Args:
        scenario — ``"baseline"``, ``"glosa"``, or ``"vru_aware_glosa"``
        filename — file name only, e.g. ``"right_turn_metrics.csv"``

    Example::

        output_path("baseline", "right_turn_metrics.csv")
        # Study A → <root>/outputs/study_A/baseline/right_turn_metrics.csv
        # Study B → <root>/outputs/study_B/Conv_above_60/baseline/right_turn_metrics.csv
    """
    if scenario not in OUTPUT_DIRS:
        raise ValueError(f"Unknown scenario '{scenario}'. Valid: {list(OUTPUT_DIRS.keys())}")
    return os.path.join(OUTPUT_DIRS[scenario], filename)


# =========================================================
# SUMO command flags — shared by all three scenarios
#
# Net, route, timing, seed, and collision settings are all
# passed as CLI flags here so ingolstadt_tiny.sumocfg only
# needs to hold step-length (not overridable via CLI flag).
# =========================================================

SUMO_BASE_FLAGS = [
    "--net-file",    NET_FILE,
    "--route-files", f"{VEHICLE_ROUTE_FILE},{BIKE_ROUTE_FILE}",
    "--seed",        "42",
    "--begin",       str(SIM_BEGIN),
    "--end",         str(SIM_END),
    "--step-length", "0.10",
    "--threads",     "1",
    "--collision.check-junctions", "true",
    "--collision.action",          "warn",
    "--collision-output",          os.path.join(SUMO_FILES_DIR, "collision.xml"),
]

# =========================================================
# Network / junction topology
# =========================================================

TLS_ID = "3040"

# South approach edge sequence (entry → stop line)
APPROACH_EDGES = [
    "-137463475#2",
    "-137463475#1",
    "-137463475#0",
    "137463477#0",
    "137463477#1",
]
ENTRY_EDGE         = "-137463475#2"   # first edge of the approach
LAST_APPROACH_EDGE = "137463477#1"    # last edge before the stop line

# Edges immediately after the junction, one per movement
STRAIGHT_EDGE = "25902677#1"
RIGHT_EDGE    = "305495773#0"
LEFT_EDGE     = "305390325#0"

# TLS signal indices for the south approach (Schrillerstr N-S)
#   20 → right (right lane)
#   21 → straight (right lane)
#   22 → straight (middle lane)
#   23 → left (left lane)
THROUGH_REP_INDEX = 20   # representative index for straight + right movements
LEFT_REP_INDEX    = 23   # representative index for left movement
BIKE_TLS_INDEX    = 18   # 18 / 19 share the same timing

# =========================================================
# Conflict zone (right-turn vehicles vs straight bicycles)
# =========================================================

CONFLICT_CENTER_XY = (6740.38, 5306.24)
CONFLICT_RADIUS_M  = 1.3

# =========================================================
# GLOSA algorithm parameters
# =========================================================

ACTIVATION_DISTANCE_M = 200.0   # begin advising vehicles within this distance of stop line
PHASE_HORIZON_S       = 300     # seconds of future signal states to pre-compute
QUEUE_DELAY_S         = 1.0     # queue-dissipation factor used in the WAITING algorithm
QUEUE_DEPTH           = 0       # kept for backward compatibility

# =========================================================
# Bicycle / VRU tracking parameters
# =========================================================

BIKE_TRACK_DISTANCE_M = 70.0   # track bikes from this distance to the stop line
BIKE_VRU_DISTANCE_M   = 50.0   # apply VRU conflict logic within this distance

# =========================================================
# VRU-aware GLOSA overlay parameters
# =========================================================

VRU_TIME_WINDOW_S      = 2.0          # ETA overlap window that triggers a conflict
QUEUE_SPEED_TH_MPS     = 10.0 / 3.6  # speed threshold for treating a vehicle as queued
BIKE_CLEAR_BUFFER_M    = 5.0          # metres past stop line before a bike is "cleared"
STOPLINE_STOP_BUFFER_M = 1.0          # stop this far before the stop line when yielding

MAX_HARD_DECEL = 7.0   # m/s² emergency deceleration limit
NORMAL_DECEL   = 4.5   # m/s² SUMO default deceleration

# =========================================================
# I/O parameters
# =========================================================

FLUSH_INTERVAL_S = 300   # flush finished vehicles to CSV every N simulation seconds