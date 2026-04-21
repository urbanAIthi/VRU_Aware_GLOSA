"""simulations/baseline.py

Baseline SUMO run — no GLOSA control, no VRU intervention.

The simulation drives the Ingolstadt scenario unchanged and collects:
  1. Traffic-efficiency metrics for RIGHT-turning motor vehicles.
  2. Safety metrics (TTC & PET) between RIGHT-turning vehicles and STRAIGHT bicycles
     in the defined conflict zone.

Run with:
    python -m simulations.baseline
    (from the project root directory)

Output CSVs and the SUMO log are written to:
    outputs/study_<A|B>/[<variant>/]baseline/
"""

import traci

from config import (
    SUMO_BASE_FLAGS,
    SUMO_CONFIG_PATH,
    SIM_END,
    LAST_APPROACH_EDGE,
    STRAIGHT_EDGE,
    RIGHT_EDGE,
    CONFLICT_CENTER_XY,
    CONFLICT_RADIUS_M,
    FLUSH_INTERVAL_S,
    output_path,
)
from evaluators.traffic_efficiency import RightTurnEfficiencyEvaluator
from evaluators.safety_metrics import JunctionSafetyEvaluator
from utils.network import add_conflict_visuals


# =========================================================
# SUMO command
# =========================================================

SUMO_CMD = [
    "sumo",
    "-c", SUMO_CONFIG_PATH,
    "--log", output_path("baseline", "sumo_log_baseline.txt"),
] + SUMO_BASE_FLAGS


# =========================================================
# Output paths
# =========================================================

EFFICIENCY_CSV = output_path("baseline", "right_turn_metrics.csv")
SAFETY_CSV     = output_path("baseline", "safety_conflicts_ttc5_pet3.csv")


# =========================================================
# Main
# =========================================================

def main() -> None:
    traci.start(SUMO_CMD)
    add_conflict_visuals(CONFLICT_CENTER_XY, CONFLICT_RADIUS_M)

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

    # Prime the first step so evaluator dt logic is consistent
    traci.simulationStep()
    now = float(traci.simulation.getTime())
    evaluator.step(now)
    safety.step(now)

    last_flush_t = now

    try:
        while (
            traci.simulation.getMinExpectedNumber() > 0
            and traci.simulation.getTime() < SIM_END
        ):
            traci.simulationStep()
            sim_t = float(traci.simulation.getTime())

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
