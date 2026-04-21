"""simulations/vru_aware_glosa.py

VRU-aware GLOSA simulation.

Extends the standard GLOSA scenario with an overlay that monitors straight-going
bicycles and yields the lead right-turn vehicle when a collision risk is detected.

Control logic:
  1. Standard GLOSA advisories are applied to all vehicles on the approach
     (same as simulations/glosa.py).
  2. For the lead right-turn vehicle only, a VRU conflict check is run every step:
       - Bikes within BIKE_VRU_DISTANCE_M whose ETA overlaps the vehicle ETA by
         less than VRU_TIME_WINDOW_S are treated as a conflict.
       - The vehicle is slowed using a dynamic braking envelope that ensures it can
         always stop before the stop line if the conflict persists.
       - Once all blocking bikes have cleared, normal driving is restored.

Metrics collected:
  1. Traffic-efficiency metrics for RIGHT-turning motor vehicles.
  2. Safety metrics (TTC & PET) for the right-turn / straight-bicycle conflict zone.

Run with:
    python -m simulations.vru_aware_glosa
    (from the project root directory)

Output CSVs and the SUMO log are written to:
    outputs/study_<A|B>/[<variant>/]vru_aware_glosa/
"""

from __future__ import annotations

import math
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
    BIKE_TLS_INDEX,
    PHASE_HORIZON_S,
    FLUSH_INTERVAL_S,
    BIKE_TRACK_DISTANCE_M,
    BIKE_VRU_DISTANCE_M,
    VRU_TIME_WINDOW_S,
    QUEUE_SPEED_TH_MPS,
    BIKE_CLEAR_BUFFER_M,
    STOPLINE_STOP_BUFFER_M,
    MAX_HARD_DECEL,
    NORMAL_DECEL,
    output_path,
)
from evaluators.traffic_efficiency import RightTurnEfficiencyEvaluator
from evaluators.safety_metrics import JunctionSafetyEvaluator
from utils.network import (
    distance_to_stopline_on_approach,
    distance_to_stopline_bike,
    lane_length,
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
    "--log", output_path("vru_aware_glosa", "sumo_log_vru_aware_glosa.txt"),
] + SUMO_BASE_FLAGS


# =========================================================
# Output paths
# =========================================================

EFFICIENCY_CSV = output_path("vru_aware_glosa", "right_turn_metrics.csv")
SAFETY_CSV     = output_path("vru_aware_glosa", "safety_conflicts_ttc5_pet3.csv")


# =========================================================
# Module-level vehicle / bicycle tracking state
# =========================================================

tracked: set[str] = set()
veh_movement:    dict[str, str] = {}
veh_rep_index:   dict[str, int] = {}
veh_exact_index: dict[str, int] = {}

bike_tracked: set[str] = set()

vru_active:        dict[str, bool]      = {}   # vehID → True when VRU override is active
vru_blocking_bikes: dict[str, set[str]] = {}   # vehID → set of blocking bike IDs

vehicles_by_rep_index:   defaultdict[int, set[str]] = defaultdict(set)
vehicles_by_exact_index: defaultdict[int, set[str]] = defaultdict(set)
vehicles_by_movement:    defaultdict[str, set[str]] = defaultdict(set)

reported_exact_once: set[str] = set()


# =========================================================
# Vehicle tracking helpers
# =========================================================

def add_if_enters_approach() -> None:
    """Register motor vehicles that depart on the entry edge."""
    for vid in traci.simulation.getDepartedIDList():
        try:
            vclass = traci.vehicle.getVehicleClass(vid)
            if vclass in ("bicycle", "pedestrian"):
                continue
            edge_now = traci.vehicle.getRoadID(vid)
            route    = traci.vehicle.getRoute(vid)
        except traci.TraCIException:
            continue

        if edge_now != ENTRY_EDGE:
            continue

        mov = movement_from_route(route)
        rep = rep_index_from_movement(mov) if mov else None
        if mov is None or rep is None:
            continue

        tracked.add(vid)
        veh_movement[vid]  = mov
        veh_rep_index[vid] = rep


def cleanup_arrived() -> None:
    """Remove arrived vehicles from all tracking structures."""
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
# Bicycle tracking helpers
# =========================================================

def add_bikes_if_enter() -> None:
    """Register bicycles that depart on the entry edge."""
    for vid in traci.simulation.getDepartedIDList():
        try:
            if traci.vehicle.getVehicleClass(vid) != "bicycle":
                continue
            if traci.vehicle.getRoadID(vid) == ENTRY_EDGE:
                bike_tracked.add(vid)
        except traci.TraCIException:
            continue


def cleanup_bikes() -> None:
    """Remove arrived bicycles from the tracking set."""
    arrived = set(traci.simulation.getArrivedIDList())
    for vid in list(bike_tracked):
        if vid in arrived:
            bike_tracked.discard(vid)


# =========================================================
# VRU helper functions
# =========================================================

def _has_leader_close(veh_id: str, max_gap: float = 10.0) -> bool:
    try:
        lead = traci.vehicle.getLeader(veh_id, max_gap)
    except traci.TraCIException:
        return False
    return lead is not None


def _is_queue_like(veh_id: str, speed_mps: float) -> bool:
    """Return True when the vehicle is moving slowly or has a close leader."""
    return (speed_mps < QUEUE_SPEED_TH_MPS) or _has_leader_close(veh_id, 10.0)


def _count_run(states: list[str], start_idx: int, ch: str) -> int:
    """Count consecutive occurrences of *ch* starting at *start_idx*."""
    count = 0
    for s in states[start_idx:]:
        if s == ch:
            count += 1
        else:
            break
    return count


def bike_is_cleared_xy(veh_id: str, bike_id: str) -> bool:
    """Return True when the bike's rear is at least BIKE_CLEAR_BUFFER_M ahead of the vehicle."""
    try:
        vx, vy = traci.vehicle.getPosition(veh_id)
        bx, by = traci.vehicle.getPosition(bike_id)
        bike_len = traci.vehicle.getLength(bike_id)
    except traci.TraCIException:
        return True  # treat disappeared entities as cleared

    dist_front_to_front = math.hypot(bx - vx, by - vy)
    dist_front_to_rear  = dist_front_to_front - bike_len
    return dist_front_to_rear >= BIKE_CLEAR_BUFFER_M


def get_next_green_info(up_states: list[str]) -> tuple[int | None, int]:
    """Return (seconds_until_next_green, green_duration_s) from a future-states list."""
    time_to_green  = None
    green_duration = 0

    for i, s in enumerate(up_states):
        if s == "g":
            time_to_green = i + 1
            for s2 in up_states[i:]:
                if s2 == "g":
                    green_duration += 1
                else:
                    break
            break

    return time_to_green, green_duration


def estimate_eta_to_stopline(veh_id: str, dist_m: float, use_accel: bool = True) -> int | None:
    """Estimate seconds until the vehicle reaches the stop line, with queue-aware fallback."""
    try:
        v = float(traci.vehicle.getSpeed(veh_id))
        a = float(traci.vehicle.getAcceleration(veh_id)) if use_accel else 0.0
    except traci.TraCIException:
        return None

    if dist_m is None:
        return None

    if _is_queue_like(veh_id, v):
        return int(dist_m / max(v, 0.5))

    if v < 0.1:
        return int(dist_m / 0.5)

    if use_accel and a > 0.01:
        try:
            t = (-v + math.sqrt(v * v + 2.0 * a * dist_m)) / a
            return int(max(t, 0.1))
        except Exception:
            return int(dist_m / max(v, 0.1))

    return int(dist_m / max(v, 0.1))


# =========================================================
# Bicycle crossing evaluation
# =========================================================

def evaluate_bike_crossing(
    vid: str,
    phase_states: list[str],
    phase_durations: list[float],
) -> dict | None:
    """Assess a bicycle's TLS state, ETA, and whether it will stop at the line.

    Returns a dict with keys: id, dist, v, a, eta, reach_state, cur_state,
    time_to_green, green_duration, queued, will_stop.
    Returns None if the bike cannot be evaluated.
    """
    try:
        d     = distance_to_stopline_bike(vid)
        speed = float(traci.vehicle.getSpeed(vid))
        accel = float(traci.vehicle.getAcceleration(vid))
    except traci.TraCIException:
        return None

    if d is None:
        return None

    up_bike = build_future_index_states(
        TLS_ID, BIKE_TLS_INDEX, PHASE_HORIZON_S, phase_states, phase_durations
    )

    cur_phase     = traci.trafficlight.getPhase(TLS_ID)
    current_state = phase_states[cur_phase][BIKE_TLS_INDEX].lower()
    queued        = _is_queue_like(vid, speed)

    if queued:
        time_to_tl = int(d / max(speed, 0.5))
    elif speed < 0.1:
        time_to_tl = int(d / 0.5)
    elif accel > 0.01:
        time_to_tl = int(max((-speed + math.sqrt(speed ** 2 + 2 * accel * d)) / accel, 0.1))
    else:
        time_to_tl = int(d / max(speed, 0.1))

    if time_to_tl <= 0 or time_to_tl > len(up_bike):
        reaching_state = "r"
    else:
        reaching_state = up_bike[time_to_tl - 1]

    time_to_green, green_duration = get_next_green_info(up_bike)

    # Determine whether the bike is expected to stop
    will_stop = False
    y_remaining_now      = _count_run(up_bike, 0, "y")              if current_state  == "y" else 0
    y_remaining_at_arr   = _count_run(up_bike, max(time_to_tl - 1, 0), "y") if reaching_state == "y" else 0

    if reaching_state == "y":
        if (current_state == "y" and time_to_tl > y_remaining_now) \
                or queued or (speed < QUEUE_SPEED_TH_MPS) or (y_remaining_at_arr < 2):
            will_stop = True

    if reaching_state == "r":
        will_stop = queued or (speed < QUEUE_SPEED_TH_MPS)

    return {
        "id":             vid,
        "dist":           d,
        "v":              speed,
        "a":              accel,
        "eta":            time_to_tl,
        "reach_state":    reaching_state,
        "cur_state":      current_state,
        "time_to_green":  time_to_green,
        "green_duration": green_duration,
        "queued":         queued,
        "will_stop":      will_stop,
    }


# =========================================================
# VRU override control
# =========================================================

def apply_vru_control(
    veh_id: str,
    dveh: float,
    blocking: set[str],
    bike_eval_results: dict[str, dict],
) -> None:
    """Slow or stop the vehicle to yield to conflicting bicycles.

    The function applies the tightest of two constraints:
      - A yield speed derived from the earliest conflicting bike's ETA.
      - A braking envelope ensuring the vehicle can stop before the stop line.
    """
    d_stop = max(dveh - STOPLINE_STOP_BUFFER_M, 0.0)

    if d_stop <= 0.05:
        # Already at the stop buffer — hold position
        try:
            traci.vehicle.setDecel(veh_id, MAX_HARD_DECEL)
        except traci.TraCIException:
            pass
        try:
            traci.vehicle.slowDown(veh_id, 0.0, 1.0)
        except traci.TraCIException:
            pass
        return

    try:
        v     = float(traci.vehicle.getSpeed(veh_id))
        lane_id = traci.vehicle.getLaneID(veh_id)
        vmax  = float(traci.lane.getMaxSpeed(lane_id))
    except traci.TraCIException:
        return

    # Earliest bike ETA among blocking bikes
    earliest_bike_eta = None
    for bid in blocking:
        binfo = bike_eval_results.get(bid)
        if binfo is None:
            continue
        be = binfo.get("eta")
        if be is None:
            continue
        if earliest_bike_eta is None or be < earliest_bike_eta:
            earliest_bike_eta = be

    # Yield speed: arrive after the bike + time window
    v_yield = None
    if earliest_bike_eta is not None:
        target_t = max(float(earliest_bike_eta) + float(VRU_TIME_WINDOW_S), 0.5)
        v_yield  = dveh / target_t

    # Braking envelope: max speed that still allows a stop within d_stop
    v_stop_env = math.sqrt(max(0.0, 2.0 * MAX_HARD_DECEL * d_stop))

    v_target = min(v_stop_env, vmax)
    if v_yield is not None:
        v_target = min(v_target, max(v_yield, 0.0))

    if v_target < 0.2:
        v_target = 0.0

    a_step = max((v - v_target) / 1.0, 0.0)
    a_step = min(max(a_step, 0.1), MAX_HARD_DECEL)

    try:
        traci.vehicle.setDecel(veh_id, a_step)
    except traci.TraCIException:
        pass
    try:
        traci.vehicle.slowDown(veh_id, float(v_target), 1.0)
    except traci.TraCIException:
        pass


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

            # ---- Vehicle / bike bookkeeping ----
            add_if_enters_approach()
            cleanup_arrived()
            update_step_views()
            add_bikes_if_enter()
            cleanup_bikes()

            # ---- Bicycle evaluation (within tracking distance) ----
            bike_eval_results: dict[str, dict] = {}
            for bid in list(bike_tracked):
                d = distance_to_stopline_bike(bid)
                if d is None or d > BIKE_TRACK_DISTANCE_M:
                    continue
                res = evaluate_bike_crossing(bid, phase_states, phase_durations)
                if res is not None:
                    bike_eval_results[bid] = res

            # ---- GLOSA advisories (through + left groups) ----
            up20 = build_future_index_states(TLS_ID, THROUGH_REP_INDEX, PHASE_HORIZON_S, phase_states, phase_durations)
            up23 = build_future_index_states(TLS_ID, LEFT_REP_INDEX,    PHASE_HORIZON_S, phase_states, phase_durations)

            cur_phase = traci.trafficlight.getPhase(TLS_ID)
            cur20 = phase_states[cur_phase][THROUGH_REP_INDEX].lower()
            cur23 = phase_states[cur_phase][LEFT_REP_INDEX].lower()

            active_on_approach = set().union(*vehicles_by_rep_index.values()) if vehicles_by_rep_index else set()

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

            # ---- VRU-aware overlay (right-turn vehicles only) ----

            # Identify the lead right-turn vehicle (closest to stop line)
            right_turners = list(vehicles_by_movement.get("right", set()))
            lead_right    = None
            lead_right_d  = None

            if right_turners:
                candidates = [
                    (distance_to_stopline_on_approach(rv), rv)
                    for rv in right_turners
                ]
                candidates = [(d, v) for d, v in candidates if d is not None]
                if candidates:
                    candidates.sort(key=lambda x: x[0])
                    lead_right_d, lead_right = candidates[0]

            # Clear VRU state for bikes that have moved past the vehicle
            for veh_id in list(vru_blocking_bikes.keys()):
                bs = {b for b in vru_blocking_bikes.get(veh_id, set())
                      if not bike_is_cleared_xy(veh_id, b)}
                if bs:
                    vru_blocking_bikes[veh_id] = bs
                    vru_active[veh_id] = True
                else:
                    vru_blocking_bikes.pop(veh_id, None)
                    vru_active.pop(veh_id, None)
                    try:
                        traci.vehicle.setDecel(veh_id, NORMAL_DECEL)
                    except traci.TraCIException:
                        pass

            # Conflict detection for the lead right-turn vehicle
            if lead_right is not None and lead_right_d is not None and lead_right_d <= BIKE_VRU_DISTANCE_M:
                veh_eta = estimate_eta_to_stopline(lead_right, lead_right_d, use_accel=True)
                if veh_eta is None:
                    veh_eta = 9999

                conflict_bikes: set[str] = set()
                for b in bike_eval_results.values():
                    if b["dist"] > BIKE_VRU_DISTANCE_M:
                        continue
                    if b.get("will_stop", False):
                        continue
                    bike_eta = int(b.get("eta", 9999))
                    if abs(veh_eta - bike_eta) <= VRU_TIME_WINDOW_S:
                        conflict_bikes.add(b["id"])

                if conflict_bikes:
                    vru_active[lead_right] = True
                    prev = vru_blocking_bikes.get(lead_right, set())
                    vru_blocking_bikes[lead_right] = prev | conflict_bikes

            # Apply VRU braking to all active vehicles with blocking bikes
            for veh_id, active in list(vru_active.items()):
                if not active:
                    continue
                dveh = distance_to_stopline_on_approach(veh_id)
                if dveh is None:
                    continue
                blocking = vru_blocking_bikes.get(veh_id, set())
                if not blocking:
                    continue
                apply_vru_control(veh_id, dveh, blocking, bike_eval_results)

            # ---- Evaluators ----
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
