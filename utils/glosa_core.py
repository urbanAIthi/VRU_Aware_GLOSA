"""utils/glosa_core.py

Core GLOSA speed-advisory algorithm.  These pure functions are shared by both
the standard GLOSA simulation and the VRU-aware GLOSA simulation.

Public API
----------
calculate_relevantSwitchTime_from_index
calculate_speedAdvisory
get_preceding_vehicles_same_group
calculate_waitingDistance_general
glosa_speed_for_vehicle
"""

from __future__ import annotations

import numpy as np
import traci

from config import ACTIVATION_DISTANCE_M, QUEUE_DELAY_S
from utils.network import distance_to_stopline_on_approach


# =========================================================
# Switch-time and state helpers
# =========================================================

def calculate_relevantSwitchTime_from_index(
    up_phases_index: list[str],
    current_state: str,
    time_to_tl: int,
) -> tuple[str, str, int, int, int]:
    """Determine the relevant switch time for a vehicle approaching the stop line.

    Args:
        up_phases_index — second-by-second future signal characters for the vehicle's TLS index
        current_state   — current signal character (``'g'``, ``'y'``, or ``'r'``)
        time_to_tl      — estimated travel time (seconds) to the stop line

    Returns:
        current_state     — normalised to lowercase
        reaching_state    — predicted signal character when the vehicle arrives
        next_switch       — seconds until the current state changes
        relevant_switchtime — seconds until the next switch *after* arrival
        red_duration      — length of the upcoming red phase (seconds)
    """
    current_state = current_state.lower()
    reaching_state = up_phases_index[time_to_tl - 1]

    switch_state = {"g": "y", "r": "g"}.get(current_state, "r")
    next_switch = up_phases_index.index(switch_state) + 1

    next_state = {"g": "y", "r": "g"}.get(reaching_state, "r")
    new_list = up_phases_index[time_to_tl:]
    idx = new_list.index(next_state)
    relevant_switchtime = idx + time_to_tl + 1

    red_list = up_phases_index[next_switch - 1:]
    ind_r1 = red_list.index("r")
    red_list_2 = red_list[ind_r1:]
    ind_r2 = red_list_2.index("g")
    red_duration = len(red_list_2[:ind_r2])

    return current_state, reaching_state, next_switch, relevant_switchtime, red_duration


def calculate_speedAdvisory(
    up_phases_index: list[str],
    reaching_state: str,
    current_speed: float,
    tl_distance: float,
    relevant_switchtime: int,
    red_duration: int,
    max_speed: float,
    min_speed: float,
) -> float:
    """Compute a speed advisory that keeps the vehicle on green (or targets the next green).

    The function applies a greedy strategy: it searches for the highest speed (up to
    *max_speed*) at which the vehicle still arrives during a green phase.

    Args:
        up_phases_index   — second-by-second future signal characters
        reaching_state    — predicted signal at arrival (from calculate_relevantSwitchTime_from_index)
        current_speed     — current vehicle speed in m/s
        tl_distance       — effective distance to stop line in metres
        relevant_switchtime — seconds to the next switch after arrival
        red_duration      — length of the upcoming red phase in seconds
        max_speed         — lane maximum speed in m/s
        min_speed         — minimum acceptable advisory speed in m/s

    Returns:
        Advisory speed in m/s.
    """
    advisory_speed = max_speed
    reaching_state = reaching_state.lower()

    if reaching_state == "g":
        if current_speed < max_speed:
            for i in np.arange(max(current_speed, 0.1), max_speed, 0.01):
                tt = int(tl_distance / i)
                if tt <= 0 or tt > len(up_phases_index):
                    continue
                if up_phases_index[tt - 1] == "g":
                    advisory_speed = round(max(min_speed, float(i)), 2)
        else:
            advisory_speed = max_speed

    elif reaching_state == "r":
        x = relevant_switchtime - red_duration - 5
        if x > 0:
            prevgreen_speed = ((2 * tl_distance) / x) - current_speed
            if 0 < prevgreen_speed <= max_speed:
                for i in np.arange(prevgreen_speed, max_speed, 0.01):
                    tt = int(tl_distance / i)
                    if tt <= 0 or tt > len(up_phases_index):
                        continue
                    if up_phases_index[tt - 1] == "g":
                        advisory_speed = round(max(min_speed, float(i)), 2)
            else:
                ttg_speed = round(((2 * tl_distance) / relevant_switchtime) - current_speed, 2)
                advisory_speed = max(ttg_speed, min_speed) if ttg_speed <= max_speed else min_speed
        else:
            ttg_speed = round(((2 * tl_distance) / relevant_switchtime) - current_speed, 2)
            advisory_speed = max(ttg_speed, min_speed) if ttg_speed <= max_speed else min_speed

    else:  # reaching_state == "y"
        try:
            prevgreen_speed = ((2 * tl_distance) / (relevant_switchtime - 5)) - current_speed
            if 0 < prevgreen_speed <= max_speed:
                for i in np.arange(prevgreen_speed, max_speed, 0.01):
                    tt = int(tl_distance / i)
                    if tt <= 0 or tt > len(up_phases_index):
                        continue
                    if up_phases_index[tt - 1] == "g":
                        advisory_speed = round(max(min_speed, float(i)), 2)
            elif prevgreen_speed > max_speed:
                advisory_speed = min_speed
        except Exception:
            advisory_speed = min_speed

    return float(advisory_speed)


# =========================================================
# Queue / waiting distance helpers
# =========================================================

def get_preceding_vehicles_same_group(
    ego_id: str,
    ego_dist: float,
    rep_index: int,
    veh_rep_index: dict[str, int],
    active_vehicle_ids: set[str],
    max_count: int = 50,
) -> list[tuple[float, str]]:
    """Return vehicles in the same TLS signal group that are closer to the stop line.

    Args:
        ego_id           — the vehicle for which we are computing GLOSA
        ego_dist         — ego vehicle's distance to the stop line
        rep_index        — representative TLS index of the signal group
        veh_rep_index    — mapping of vehicle ID → assigned rep index
        active_vehicle_ids — set of vehicle IDs currently on the approach
        max_count        — maximum number of preceding vehicles to return

    Returns:
        List of ``(distance, vehicle_id)`` tuples, sorted by distance (closest first).
    """
    out = []
    for vid in active_vehicle_ids:
        if vid == ego_id:
            continue
        if veh_rep_index.get(vid) != rep_index:
            continue
        d = distance_to_stopline_on_approach(vid)
        if d is None or d > ego_dist:
            continue
        out.append((d, vid))
    out.sort(key=lambda x: x[0])
    return out[:max_count]


def calculate_waitingDistance_general(
    ego_id: str,
    tl_distance_init: float,
    up_phases_index: list[str],
    delay: float,
    rep_index: int,
    veh_rep_index: dict[str, int],
    active_vehicle_ids: set[str],
) -> float:
    """Adjust the effective distance to the stop line by subtracting the estimated queue length.

    Args:
        ego_id           — the vehicle being evaluated
        tl_distance_init — raw distance from ego to stop line
        up_phases_index  — future signal states for the relevant TLS index
        delay            — queue dissipation factor (seconds per vehicle)
        rep_index        — TLS representative index for this movement group
        veh_rep_index    — mapping of vehicle ID → assigned rep index
        active_vehicle_ids — vehicles currently on the approach

    Returns:
        Adjusted effective distance (always > 0).
    """
    state_list: list[str] = []
    waiting_distance = 0.0

    preceding = get_preceding_vehicles_same_group(
        ego_id=ego_id,
        ego_dist=tl_distance_init,
        rep_index=rep_index,
        veh_rep_index=veh_rep_index,
        active_vehicle_ids=active_vehicle_ids,
    )
    number_vehicles = len(preceding)

    if number_vehicles > 0:
        for _, v in preceding:
            try:
                tl_distance_v = float(distance_to_stopline_on_approach(v) or 0.0)
                current_speed_v = float(traci.vehicle.getSpeed(v))
            except traci.TraCIException:
                continue

            if current_speed_v > 0.1:
                time_to_tl_v = int(tl_distance_v / current_speed_v)
                if time_to_tl_v <= 0 or time_to_tl_v > len(up_phases_index):
                    reaching_state_v = "r"
                else:
                    reaching_state_v = up_phases_index[time_to_tl_v - 1]
                    if reaching_state_v == "y":
                        reaching_state_v = "r"
                state_list.append(reaching_state_v)
            else:
                state_list.append("r")

        if "r" in state_list:
            ind_state = state_list.index("r")
            try:
                veh_len = float(traci.vehicle.getLength(ego_id))
                min_gap = float(traci.vehicle.getMinGap(ego_id))
            except traci.TraCIException:
                veh_len, min_gap = 5.0, 2.5

            waiting_distance = 1.0 + (number_vehicles - ind_state) * (delay + veh_len + min_gap)

    if tl_distance_init > waiting_distance:
        return round(tl_distance_init - waiting_distance, 2)
    return 0.1


# =========================================================
# Top-level per-vehicle GLOSA evaluation
# =========================================================

def glosa_speed_for_vehicle(
    vid: str,
    rep_index: int,
    up_phases_index: list[str],
    current_state_char: str,
    veh_rep_index: dict[str, int],
    active_vehicle_ids: set[str],
    use_waiting: bool = True,
) -> tuple[float | None, dict]:
    """Compute the GLOSA speed advisory for a single vehicle.

    Args:
        vid               — vehicle ID
        rep_index         — TLS representative index for this vehicle's movement group
        up_phases_index   — future signal states for *rep_index*
        current_state_char — current signal character (``'g'``, ``'y'``, or ``'r'``)
        veh_rep_index     — global mapping of vehicle ID → assigned rep index
        active_vehicle_ids — vehicles currently on the approach
        use_waiting       — whether to apply queue-distance adjustment

    Returns:
        ``(advisory_speed, info_dict)`` where *advisory_speed* is ``None`` when
        the vehicle is outside the activation zone or cannot be evaluated.
    """
    d0 = distance_to_stopline_on_approach(vid)
    if d0 is None:
        return None, {}

    if d0 > ACTIVATION_DISTANCE_M:
        return None, {"reason": "outside_activation", "dist": d0}

    try:
        current_speed = float(traci.vehicle.getSpeed(vid))
        lane_id = traci.vehicle.getLaneID(vid)
        max_speed = float(traci.lane.getMaxSpeed(lane_id))
    except traci.TraCIException:
        return None, {"reason": "traci_error"}

    min_speed = max_speed / 2

    if current_speed < 1.0:
        return None, {"reason": "too_slow", "dist": d0, "v": current_speed}

    if use_waiting:
        d = calculate_waitingDistance_general(
            ego_id=vid,
            tl_distance_init=round(d0, 2),
            up_phases_index=up_phases_index,
            delay=QUEUE_DELAY_S,
            rep_index=rep_index,
            veh_rep_index=veh_rep_index,
            active_vehicle_ids=active_vehicle_ids,
        )
    else:
        d = round(d0, 2)

    time_to_tl = int(d / current_speed)
    if time_to_tl <= 0 or time_to_tl > len(up_phases_index):
        return None, {"reason": "time_out_of_horizon", "dist": d, "ttl": time_to_tl}

    current_state, reaching_state, next_switch, relevant_switchtime, red_duration = (
        calculate_relevantSwitchTime_from_index(
            up_phases_index=up_phases_index,
            current_state=current_state_char,
            time_to_tl=time_to_tl,
        )
    )

    if time_to_tl == next_switch:
        time_to_tl += 1

    advisory = calculate_speedAdvisory(
        up_phases_index=up_phases_index,
        reaching_state=reaching_state,
        current_speed=round(current_speed, 2),
        tl_distance=d,
        relevant_switchtime=relevant_switchtime,
        red_duration=red_duration,
        max_speed=max_speed,
        min_speed=min_speed,
    )

    info = {
        "dist_raw": d0,
        "dist_eff": d,
        "v": current_speed,
        "ttl": time_to_tl,
        "cur_state": current_state,
        "reach_state": reaching_state,
        "rel_sw": relevant_switchtime,
        "red_dur": red_duration,
        "advisory": advisory,
        "max_v": max_speed,
        "min_v": min_speed,
    }
    return advisory, info
