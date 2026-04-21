"""evaluators/traffic_efficiency.py

Reusable traffic-efficiency evaluation for SUMO/TraCI runs.

Focus: RIGHT-turning motor vehicles (identified from route: LAST_APPROACH_EDGE → RIGHT_EDGE).
Each vehicle is tracked from departure to arrival and the following metrics are recorded:

  - Travel time
  - Waiting time (integrated time below stop_speed threshold)
  - Number of stops (moving → stopped transitions)
  - CO₂ and CO emissions (mg), combined as COx
  - Fuel consumption (mg) and electricity consumption (Wh)
  - Red-stop specific metrics (waiting time at red, stop count, first stop/restart timestamps)

TraCI unit notes:
  - CO₂, CO emissions : mg/s  → integrated by step length to mg
  - Fuel consumption  : mg/s  → integrated by step length to mg
  - Electricity       : Wh/s  → integrated by step length to Wh
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple, Iterable

import traci


# =========================================================
# Data model
# =========================================================

@dataclass
class VehMetrics:
    veh_id: str
    depart_time_s: float

    arrive_time_s: Optional[float] = None
    travel_time_s: Optional[float] = None

    waiting_time_s: float = 0.0
    stops_count: int = 0

    co2_mg: float = 0.0
    co_mg: float = 0.0
    cox_mg: float = 0.0

    fuel_mg: float = 0.0
    electricity_Wh: float = 0.0

    red_waiting_time_s: float = 0.0
    red_stops_count: int = 0
    first_red_stop_start_s: Optional[float] = None
    first_red_restart_s: Optional[float] = None
    restart_edge_id: Optional[str] = None

    _prev_speed_mps: float = 0.0
    _was_stopped: bool = False
    _was_red_stopped: bool = False


# =========================================================
# Internal helpers
# =========================================================

def _is_waiting_at_red(veh_id: str, dist_thresh_m: float = 15.0) -> bool:
    """Return True when the nearest upcoming TLS is red and within *dist_thresh_m*."""
    try:
        nxt = traci.vehicle.getNextTLS(veh_id)
        if not nxt:
            return False
        _tls_id, _tls_idx, dist, state = nxt[0]
        return (float(dist) <= dist_thresh_m) and (str(state).lower() == "r")
    except Exception:
        return False


def _safe_rate(getter, veh_id: str) -> float:
    """Return a TraCI emission/consumption rate, coercing invalid values to 0."""
    try:
        v = float(getter(veh_id))
        if v <= -(2 ** 29):
            return 0.0
        return v
    except Exception:
        return 0.0


def _movement_from_route(
    route: Tuple[str, ...] | Iterable[str],
    last_approach_edge: str,
    right_edge: str,
) -> Optional[str]:
    """Return ``'right'`` when the route turns right after *last_approach_edge*, else ``None``."""
    route_list = list(route)
    if last_approach_edge not in route_list:
        return None
    i = route_list.index(last_approach_edge)
    if i + 1 >= len(route_list):
        return None
    return "right" if route_list[i + 1] == right_edge else "other"


# =========================================================
# CSV field list (single source of truth)
# =========================================================

_CSV_FIELDS = [
    "veh_id", "depart_time_s", "arrive_time_s", "travel_time_s",
    "waiting_time_s", "stops_count",
    "co2_mg", "co_mg", "cox_mg", "fuel_mg", "electricity_Wh",
    "red_waiting_time_s", "red_stops_count",
    "first_red_stop_start_s", "first_red_restart_s", "restart_edge_id",
]


# =========================================================
# Evaluator class
# =========================================================

class RightTurnEfficiencyEvaluator:
    """Collects per-vehicle traffic-efficiency metrics for right-turning motor vehicles.

    Usage::

        evaluator = RightTurnEfficiencyEvaluator(
            last_approach_edge=LAST_APPROACH_EDGE,
            right_edge=RIGHT_EDGE,
            output_csv_path="metrics.csv",
        )

        # inside the simulation loop, after each traci.simulationStep():
        evaluator.step(sim_t)

        # flush finished vehicles to CSV periodically and at the end of the simulation
        evaluator.flush_finished_to_csv(append=True)
    """

    def __init__(
        self,
        *,
        last_approach_edge: str,
        right_edge: str,
        output_csv_path: str = "right_turn_metrics.csv",
        stop_speed_mps: float = 0.1,
        ignore_vclasses: Tuple[str, ...] = ("bicycle", "pedestrian"),
    ) -> None:
        self.last_approach_edge = last_approach_edge
        self.right_edge = right_edge
        self.output_csv_path = output_csv_path
        self.stop_speed_mps = stop_speed_mps
        self.ignore_vclasses = ignore_vclasses

        self._active: set[str] = set()
        self._metrics: Dict[str, VehMetrics] = {}
        self._last_sim_t: Optional[float] = None

    @property
    def metrics(self) -> Dict[str, VehMetrics]:
        return self._metrics

    # ------------------------------------------------------------------
    # Time-step integration
    # ------------------------------------------------------------------

    def _get_dt(self, sim_t: float) -> float:
        if self._last_sim_t is None:
            self._last_sim_t = sim_t
            try:
                return float(traci.simulation.getDeltaT()) / 1000.0
            except Exception:
                return 1.0
        dt = max(sim_t - self._last_sim_t, 0.0)
        self._last_sim_t = sim_t
        if dt == 0.0:
            try:
                dt = float(traci.simulation.getDeltaT()) / 1000.0
            except Exception:
                dt = 1.0
        return dt

    # ------------------------------------------------------------------
    # Vehicle lifecycle
    # ------------------------------------------------------------------

    def _maybe_register_departed(self, sim_t: float) -> None:
        for vid in traci.simulation.getDepartedIDList():
            try:
                vclass = traci.vehicle.getVehicleClass(vid)
                if vclass in self.ignore_vclasses:
                    continue
                route = traci.vehicle.getRoute(vid)
            except Exception:
                continue

            if _movement_from_route(route, self.last_approach_edge, self.right_edge) != "right":
                continue

            try:
                v0 = float(traci.vehicle.getSpeed(vid))
            except Exception:
                v0 = 0.0

            m = VehMetrics(veh_id=vid, depart_time_s=float(sim_t))
            m._prev_speed_mps = v0
            m._was_stopped = v0 < self.stop_speed_mps

            self._metrics[vid] = m
            self._active.add(vid)

    def _finalize_arrived(self, sim_t: float) -> None:
        for vid in set(traci.simulation.getArrivedIDList()):
            if vid not in self._metrics:
                continue
            m = self._metrics[vid]
            m.arrive_time_s = float(sim_t)
            m.travel_time_s = float(sim_t) - m.depart_time_s
            self._active.discard(vid)

    def _finalize_missing(self, vid: str, sim_t: float) -> None:
        """Handle vehicles that disappeared unexpectedly (teleport, error, etc.)."""
        m = self._metrics.get(vid)
        if m is None:
            return
        if m.arrive_time_s is None:
            m.arrive_time_s = float(sim_t)
            m.travel_time_s = float(sim_t) - m.depart_time_s
        self._active.discard(vid)

    # ------------------------------------------------------------------
    # Per-step metric update
    # ------------------------------------------------------------------

    def step(self, sim_t: Optional[float] = None) -> None:
        """Call exactly once after each ``traci.simulationStep()``."""
        if sim_t is None:
            sim_t = float(traci.simulation.getTime())

        dt = self._get_dt(float(sim_t))

        self._maybe_register_departed(float(sim_t))

        for vid in list(self._active):
            if vid not in traci.vehicle.getIDList():
                self._finalize_missing(vid, float(sim_t))
                continue

            m = self._metrics[vid]

            try:
                v = float(traci.vehicle.getSpeed(vid))
            except Exception:
                v = 0.0

            is_stopped = v < self.stop_speed_mps

            if is_stopped:
                m.waiting_time_s += dt

            # Red-stop detection
            red_ahead = _is_waiting_at_red(vid, dist_thresh_m=15.0)
            red_stopped = is_stopped and red_ahead

            if red_stopped:
                m.red_waiting_time_s += dt

            if (not m._was_red_stopped) and red_stopped:
                m.red_stops_count += 1
                if m.first_red_stop_start_s is None:
                    m.first_red_stop_start_s = float(sim_t)

            if m._was_red_stopped and (not is_stopped):
                if m.first_red_restart_s is None:
                    m.first_red_restart_s = float(sim_t)
                    try:
                        m.restart_edge_id = traci.vehicle.getRoadID(vid)
                    except Exception:
                        m.restart_edge_id = None

            m._was_red_stopped = red_stopped

            if (not m._was_stopped) and is_stopped:
                m.stops_count += 1
            m._was_stopped = is_stopped
            m._prev_speed_mps = v

            # Emission / consumption rates (per second) → integrate by dt
            co2_rate  = _safe_rate(traci.vehicle.getCO2Emission, vid)
            co_rate   = _safe_rate(traci.vehicle.getCOEmission, vid)
            fuel_rate = _safe_rate(traci.vehicle.getFuelConsumption, vid)
            elec_rate = _safe_rate(traci.vehicle.getElectricityConsumption, vid)

            m.co2_mg        += co2_rate * dt
            m.co_mg         += co_rate  * dt
            m.cox_mg        += (co2_rate + co_rate) * dt
            m.fuel_mg       += fuel_rate * dt
            m.electricity_Wh += elec_rate * dt

        self._finalize_arrived(float(sim_t))

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------

    def flush_finished_to_csv(self, append: bool = True) -> None:
        """Write all finished vehicles to CSV and free their memory.

        Args:
            append — when True, rows are appended to an existing file;
                     when False, a new file is created with a header row.
        """
        finished = [m for m in self._metrics.values() if m.arrive_time_s is not None]
        if not finished:
            return

        mode = "a" if append else "w"
        write_header = not append

        with open(self.output_csv_path, mode, newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=_CSV_FIELDS)
            if write_header:
                w.writeheader()
            for m in finished:
                w.writerow({k: getattr(m, k) for k in _CSV_FIELDS})

        for m in finished:
            self._metrics.pop(m.veh_id, None)

    def write_csv(self, path: Optional[str] = None) -> str:
        """Write all tracked vehicles (finished or not) to a CSV file.

        Returns the output path.
        """
        out_path = path or self.output_csv_path
        with open(out_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=_CSV_FIELDS)
            w.writeheader()
            for m in self._metrics.values():
                w.writerow({k: getattr(m, k) for k in _CSV_FIELDS})
        return out_path

    def summary(self) -> Dict[str, Any]:
        """Return run-level averages for vehicles that have completed their trip."""
        finished = [m for m in self._metrics.values() if m.arrive_time_s is not None]
        n = len(finished)
        if n == 0:
            return {"n_finished": 0}

        def avg(attr: str) -> float:
            return float(sum(getattr(m, attr) for m in finished) / n)

        return {
            "n_finished": n,
            "avg_travel_time_s":  avg("travel_time_s"),
            "avg_waiting_time_s": avg("waiting_time_s"),
            "avg_stops_count":    avg("stops_count"),
            "avg_co2_mg":         avg("co2_mg"),
            "avg_co_mg":          avg("co_mg"),
            "avg_cox_mg":         avg("cox_mg"),
            "avg_fuel_mg":        avg("fuel_mg"),
            "avg_electricity_Wh": avg("electricity_Wh"),
        }