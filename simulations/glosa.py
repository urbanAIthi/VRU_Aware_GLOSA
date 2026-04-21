"""simulations/glosa.py

GLOSA (Green Light Optimal Speed Advisory) simulation.

Speed advisories are issued to all motor vehicles on the south approach for
both signal groups (through / straight+right, and left).  No VRU awareness.

Metrics collected:
  1. Traffic-efficiency metrics for RIGHT-turning motor vehicles.
  2. Safety metrics (TTC & PET) for the right-turn / straight-bicycle conflict zone.

Run with:
    python -m simulations.glosa
    (from the project root directory)

Output CSVs and the SUMO log are written to:
    outputs/study_<A|B>/[<variant>/]glosa/
"""

from __future__ import annotations

from collections import defaultdict

import traci

from config import (
    SUMO_BASE_FLAGS,
    SUMO_CONFIG_PATH,
    SIM_END,
    TLS_ID,
    APPROACH_EDGES,
    ENTRY_EDGE,
    LAST_APPROACH_EDGE,
    STRAIGHT_EDGE,
    RIGHT_EDGE,
    CONFLICT_CENTER_XY,
    CONFLICT_RADIUS_M,
    THROUGH_REP_INDEX,
    LEFT_REP_INDEX,
    PHASE_HORIZON_S,
    FLUSH_INTERVAL_S,
    output_path,
)
from evaluators.traffic_efficiency import RightTurnEfficiencyEvaluator
from evaluators.safety_metrics import JunctionSafetyEvaluator
from utils.network import (
    movement_from_route,
    rep_index_from_movement,
    tls_index_from_movement_and_lane_near_stopline,
)
from utils.tls import build_tls_program_cache, build_future_index_states
from utils.glosa_core import glosa_speed_for_vehicle


# =========================================================
# SUMO command
# =========================================================

SUMO_CMD = [
    "sumo",
    "-c", SUMO_CONFIG_PATH,
    "--log", output_path("glosa", "sumo_log_glosa.txt"),
] + SUMO_BASE_FLAGS


# =========================================================
# Output paths
# =========================================================

EFFICIENCY_CSV = output_path("glosa", "right_turn_metrics.csv")
SAFETY_CSV     = output_path("glosa", "safety_conflicts_ttc5_pet3.csv")


# =========================================================
# Module-level vehicle tracking state
# =========================================================

tracked: set[str] = set()
veh_movement: dict[str, str] = {}
veh_rep_index: dict[str, int] = {}
veh_exact_index: dict[str, int] = {}

vehicles_by_rep_index:   defaultdict[int, set[str]] = defaultdict(set)
vehicles_by_exact_index: defaultdict[int, set[str]] = defaultdict(set)
vehicles_by_movement:    defaultdict[str, set[str]] = defaultdict(set)

reported_exact_once: set[str] = set()


# =========================================================
# Per-step tracking helpers
# =========================================================

def add_if_enters_approach() -> None:
    """Register motor vehicles that depart on the entry edge."""
    for vid in traci.simulation.getDepartedIDList():
        try:
            vclass = traci.vehicle.getVehicleClass(vid)
            if vclass in ("bicycle", "pedestrian"):
                continue
            edge_now = traci.vehicle.getRoadID(vid)
            route = traci.vehicle.getRoute(vid)
        except traci.TraCIException:
            continue

        if edge_now != ENTRY_EDGE:
            continue

        mov = movement_from_route(route)
        rep = rep_index_from_movement(mov) if mov else None
        if mov is None or rep is None:
            continue

        tracked.add(vid)
        veh_movement[vid] = mov
        veh_rep_index[vid] = rep


def cleanup_arrived() -> None:
    """Remove arrived vehicles from tracking structures."""
    arrived = set(traci.simulation.getArrivedIDList())
    if not arrived:
        return
    for vid in list(tracked):
        if vid in arrived:
            tracked.discard(vid)
            veh_movement.pop(vid, None)
            veh_rep_index.pop(vid, None)
            veh_exact_index.pop(vid, None)
            reported_exact_once.discard(vid)


def update_step_views() -> None:
    """Rebuild per-step lookup dictionaries (movement / rep-index / exact-index)."""
    vehicles_by_rep_index.clear()
    vehicles_by_exact_index.clear()
    vehicles_by_movement.clear()

    for vid in list(tracked):
        try:
            edge = traci.vehicle.getRoadID(vid)
        except traci.TraCIException:
            continue

        if edge not in APPROACH_EDGES:
            continue

        mov = veh_movement.get(vid)
        rep = veh_rep_index.get(vid)
        if mov:
            vehicles_by_movement[mov].add(vid)
        if rep is not None:
            vehicles_by_rep_index[rep].add(vid)

        if edge == LAST_APPROACH_EDGE and mov:
            try:
                lane_id = traci.vehicle.getLaneID(vid)
            except traci.TraCIException:
                continue
            ex = tls_index_from_movement_and_lane_near_stopline(mov, lane_id)
            if ex is not None:
                veh_exact_index[vid] = ex
                vehicles_by_exact_index[ex].add(vid)
                reported_exact_once.add(vid)


# =========================================================
# Main
# =========================================================

def main() -> None:
    traci.start(SUMO_CMD)

    evaluator = RightTurnEfficiencyEvaluator(
        last_approach_edge=LAST_APPROACH_EDGE,
        right_edge=RIGHT_EDGE,
        output_csv_path=EFFICIENCY_CSV,
        stop_speed_mps=0.1,
    )

    safety = JunctionSafetyEvaluator(
        conflict_center_xy=CONFLICT_CENTER_XY,
        conflict_radius_m=CONFLICT_RADIUS_M,
        last_approach_edge=LAST_APPROACH_EDGE,
        straight_edge=STRAIGHT_EDGE,
        right_edge=RIGHT_EDGE,
        output_csv_path=SAFETY_CSV,
        output_only_conflicts=True,
        ttc_keep_threshold_s=5.0,
        pet_keep_threshold_s=3.0,
        pairing_distance_m=80.0,
        ttc_time_diff_threshold_s=1.0,
    )

    open(EFFICIENCY_CSV, "w", encoding="utf-8").close()
    open(SAFETY_CSV,     "w", encoding="utf-8").close()

    traci.simulationStep()
    now = float(traci.simulation.getTime())
    evaluator.step(now)
    safety.step(now)

    phase_states, phase_durations, _ = build_tls_program_cache(TLS_ID)
    last_flush_t = now

    try:
        while (
            traci.simulation.getMinExpectedNumber() > 0
            and traci.simulation.getTime() < SIM_END
        ):
            traci.simulationStep()
            sim_t = float(traci.simulation.getTime())

            add_if_enters_approach()
            cleanup_arrived()
            update_step_views()

            # Build future signal sequences once per step for both representative indices
            up20 = build_future_index_states(TLS_ID, THROUGH_REP_INDEX, PHASE_HORIZON_S, phase_states, phase_durations)
            up23 = build_future_index_states(TLS_ID, LEFT_REP_INDEX,    PHASE_HORIZON_S, phase_states, phase_durations)

            cur_phase = traci.trafficlight.getPhase(TLS_ID)
            cur20 = phase_states[cur_phase][THROUGH_REP_INDEX].lower()
            cur23 = phase_states[cur_phase][LEFT_REP_INDEX].lower()

            active_on_approach = set().union(*vehicles_by_rep_index.values()) if vehicles_by_rep_index else set()

            # Through (straight + right) vehicles
            for vid in list(vehicles_by_rep_index.get(THROUGH_REP_INDEX, set())):
                adv, _ = glosa_speed_for_vehicle(
                    vid=vid,
                    rep_index=THROUGH_REP_INDEX,
                    up_phases_index=up20,
                    current_state_char=cur20,
                    veh_rep_index=veh_rep_index,
                    active_vehicle_ids=active_on_approach,
                    use_waiting=True,
                )
                if adv is None:
                    continue
                try:
                    traci.vehicle.slowDown(vid, adv, 1.0)
                except traci.TraCIException:
                    pass

            # Left-turn vehicles
            for vid in list(vehicles_by_rep_index.get(LEFT_REP_INDEX, set())):
                adv, _ = glosa_speed_for_vehicle(
                    vid=vid,
                    rep_index=LEFT_REP_INDEX,
                    up_phases_index=up23,
                    current_state_char=cur23,
                    veh_rep_index=veh_rep_index,
                    active_vehicle_ids=active_on_approach,
                    use_waiting=True,
                )
                if adv is None:
                    continue
                try:
                    traci.vehicle.slowDown(vid, adv, 1.0)
                except traci.TraCIException:
                    pass

            evaluator.step(sim_t)
            safety.step(sim_t)

            if sim_t - last_flush_t >= FLUSH_INTERVAL_S:
                evaluator.flush_finished_to_csv(append=True)
                safety.flush_finished_to_csv(append=True)
                last_flush_t = sim_t

        evaluator.flush_finished_to_csv(append=True)
        safety.flush_finished_to_csv(append=True)

    finally:
        if traci.isLoaded():
            traci.close()


if __name__ == "__main__":
    main()
