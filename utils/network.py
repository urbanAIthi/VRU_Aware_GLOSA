"""utils/network.py

Geometry and route helpers that depend on the Ingolstadt network topology.
All functions talk to SUMO via TraCI and use the constants from config.py.
"""

from __future__ import annotations

import math
import traci

from config import (
    APPROACH_EDGES,
    LAST_APPROACH_EDGE,
    STRAIGHT_EDGE,
    RIGHT_EDGE,
    LEFT_EDGE,
)


# =========================================================
# Lane / edge length helpers
# =========================================================

def lane_length(lane_id: str) -> float:
    """Return the length of a lane by its ID."""
    return traci.lane.getLength(lane_id)


def edge_vehicle_lane_length(edge_id: str) -> float:
    """Return the length of a vehicle lane on the given edge.

    Vehicle lanes use suffixes _2 / _3 / _4 (pedestrian/bike lanes are _0 / _1).
    """
    for suf in (2, 3, 4):
        lane_id = f"{edge_id}_{suf}"
        try:
            return traci.lane.getLength(lane_id)
        except traci.TraCIException:
            continue
    raise RuntimeError(
        f"No vehicle lane (_2/_3/_4) found for edge '{edge_id}'. "
        "Check lane numbering or update the suffix list in edge_vehicle_lane_length()."
    )


def edge_bike_lane_length(edge_id: str) -> float:
    """Return the length of the bike lane on the given edge.

    Bike lanes use suffix _1 in this network.
    """
    lane_id = f"{edge_id}_1"
    try:
        return traci.lane.getLength(lane_id)
    except traci.TraCIException:
        raise RuntimeError(
            f"No bike lane (_1) found for edge '{edge_id}'. "
            "Check lane numbering or update the suffix list in edge_bike_lane_length()."
        )


# =========================================================
# Distance-to-stop-line helpers
# =========================================================

def distance_to_stopline_on_approach(veh_id: str) -> float | None:
    """Remaining driving distance from the vehicle's current position to the end of
    LAST_APPROACH_EDGE, summed along APPROACH_EDGES.

    Returns None when the vehicle is not currently on the approach.
    """
    edge = traci.vehicle.getRoadID(veh_id)
    if edge not in APPROACH_EDGES:
        return None

    lane_id = traci.vehicle.getLaneID(veh_id)
    pos_on_lane = traci.vehicle.getLanePosition(veh_id)
    remaining = max(lane_length(lane_id) - pos_on_lane, 0.0)

    idx = APPROACH_EDGES.index(edge)
    for e in APPROACH_EDGES[idx + 1:]:
        remaining += edge_vehicle_lane_length(e)

    return remaining


def distance_to_stopline_bike(veh_id: str) -> float | None:
    """Same as distance_to_stopline_on_approach but uses bike-lane lengths for
    the downstream edges (bike lanes may differ in length from vehicle lanes).

    Returns None when the bicycle is not on the approach.
    """
    try:
        edge = traci.vehicle.getRoadID(veh_id)
        lane_id = traci.vehicle.getLaneID(veh_id)
        pos_on_lane = traci.vehicle.getLanePosition(veh_id)
    except traci.TraCIException:
        return None

    if edge not in APPROACH_EDGES:
        return None

    remaining = max(lane_length(lane_id) - pos_on_lane, 0.0)

    idx = APPROACH_EDGES.index(edge)
    for e in APPROACH_EDGES[idx + 1:]:
        remaining += edge_bike_lane_length(e)

    return round(remaining, 2)


# =========================================================
# Route → movement classification
# =========================================================

def get_next_edge_after_last_approach(route: tuple[str, ...] | list[str]) -> str | None:
    """Return the first edge after LAST_APPROACH_EDGE in *route*, or None."""
    if LAST_APPROACH_EDGE not in route:
        return None
    i = route.index(LAST_APPROACH_EDGE)
    if i + 1 >= len(route):
        return None
    return route[i + 1]


def movement_from_route(route: tuple[str, ...] | list[str]) -> str | None:
    """Classify vehicle movement from its route.

    Returns ``"left"``, ``"straight"``, ``"right"``, or ``None`` when the route
    does not pass through the monitored junction.
    """
    nxt = get_next_edge_after_last_approach(route)
    if nxt is None:
        return None
    if nxt == LEFT_EDGE:
        return "left"
    if nxt == STRAIGHT_EDGE:
        return "straight"
    if nxt == RIGHT_EDGE:
        return "right"
    return None


# =========================================================
# TLS index helpers
# =========================================================

def rep_index_from_movement(mov: str) -> int | None:
    """Map a movement string to its representative TLS signal index."""
    from config import THROUGH_REP_INDEX, LEFT_REP_INDEX

    if mov in ("straight", "right"):
        return THROUGH_REP_INDEX
    if mov == "left":
        return LEFT_REP_INDEX
    return None


def tls_index_from_movement_and_lane_near_stopline(mov: str, lane_id: str) -> int | None:
    """Return the exact TLS index when the vehicle is on LAST_APPROACH_EDGE.

    Lane suffixes on ``137463477#1``:
        _2 → rightmost lane
        _3 → middle lane
        _4 → leftmost lane
    """
    if not lane_id.startswith("137463477#1_"):
        return None
    try:
        lane_suffix = int(lane_id.rsplit("_", 1)[1])
    except Exception:
        return None

    if mov == "left":
        return 23
    if mov == "right":
        return 20
    if mov == "straight":
        if lane_suffix == 2:
            return 21
        if lane_suffix == 3:
            return 22
    return None


# =========================================================
# Conflict-zone visualisation
# =========================================================

def circle_as_polygon(cx: float, cy: float, r: float, n_points: int = 40) -> list[tuple[float, float]]:
    """Return a closed polygon approximating a circle (for TraCI polygon display)."""
    pts = [
        (cx + r * math.cos(2.0 * math.pi * i / n_points),
         cy + r * math.sin(2.0 * math.pi * i / n_points))
        for i in range(n_points)
    ]
    pts.append(pts[0])
    return pts


def add_conflict_visuals(center_xy: tuple[float, float], radius_m: float) -> None:
    """Add a filled polygon and a centre POI to the SUMO-GUI for the conflict zone."""
    cx, cy = center_xy

    shape = circle_as_polygon(cx, cy, radius_m, n_points=48)
    traci.polygon.add(
        "conflict_zone",
        shape,
        color=(255, 0, 0, 80),
        fill=True,
        layer=100,
        lineWidth=2,
    )
    traci.poi.add(
        "conflict_center",
        cx, cy,
        color=(0, 0, 255, 255),
        layer=110,
    )
