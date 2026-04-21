"""utils/tls.py

Traffic-light system (TLS) helpers:
  - build_tls_program_cache  — loads phase states and durations from SUMO
  - build_future_index_states — projects future per-second signal characters for one TLS index
"""

from __future__ import annotations

import traci


def build_tls_program_cache(tls_id: str) -> tuple[list[str], list[float], float]:
    """Read the first program logic of *tls_id* and return its phase data.

    Returns:
        states    — list of phase state strings (one per phase)
        durations — list of phase durations in seconds (one per phase)
        cycle     — total cycle length in seconds
    """
    logic = traci.trafficlight.getAllProgramLogics(tls_id)[0]
    phases = logic.getPhases()
    states = [p.state for p in phases]
    durations = [p.duration for p in phases]
    cycle = sum(durations)
    return states, durations, cycle


def build_future_index_states(
    tls_id: str,
    tls_index: int,
    horizon_s: int,
    phase_states: list[str],
    phase_durations: list[float],
) -> list[str]:
    """Build a second-by-second list of signal characters for a single TLS signal index.

    Element ``k`` of the returned list is the signal character
    ``(k + 1)`` seconds from the current simulation time.

    Args:
        tls_id        — TraCI traffic-light ID
        tls_index     — index of the signal within the phase state string
        horizon_s     — number of seconds to project into the future
        phase_states  — cached phase state strings (from build_tls_program_cache)
        phase_durations — cached phase durations (from build_tls_program_cache)

    Returns:
        A list of length *horizon_s* containing lowercase signal characters
        (``'g'``, ``'y'``, ``'r'``, …).
    """
    sim_t = traci.simulation.getTime()
    curr_phase = traci.trafficlight.getPhase(tls_id)
    next_switch = traci.trafficlight.getNextSwitch(tls_id)
    t_to_switch = max(next_switch - sim_t, 0.0)

    out: list[str] = []

    # Fill the remaining time in the current phase
    for _ in range(int(t_to_switch)):
        out.append(phase_states[curr_phase][tls_index].lower())

    # Walk future phases cyclically until the horizon is filled
    i = (curr_phase + 1) % len(phase_states)
    while len(out) < horizon_s:
        dur = int(phase_durations[i])
        ch = phase_states[i][tls_index].lower()
        for _ in range(dur):
            out.append(ch)
            if len(out) >= horizon_s:
                break
        i = (i + 1) % len(phase_states)

    return out
