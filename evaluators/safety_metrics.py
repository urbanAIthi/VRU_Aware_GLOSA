"""evaluators/safety_metrics.py

TTC (Time-To-Collision) and PET (Post-Encroachment Time) evaluation for a single
conflict zone between right-turning motor vehicles and straight-going bicycles.

Filter rule for CSV output:
    keep row if min_ttc_s <= ttc_keep_threshold_s
               AND min_pet_s <= pet_keep_threshold_s

Default thresholds: TTC ≤ 5 s, PET ≤ 3 s.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, Optional, Tuple

import traci


# =========================================================
# Constants
# =========================================================

_INVALID_CUTOFF = -(2 ** 29)   # guard for TraCI INVALID_DOUBLE_VALUE (~−2^30)


# =========================================================
# Internal helpers
# =========================================================

def _safe_float(getter, *args) -> float:
    try:
        v = float(getter(*args))
        return 0.0 if v <= _INVALID_CUTOFF else v
    except Exception:
        return 0.0


def _safe_driving_distance_2d(veh_id: str, x: float, y: float) -> Optional[float]:
    """Remaining driving distance to *(x, y)* along the vehicle route, or ``None``."""
    try:
        d = float(traci.vehicle.getDrivingDistance2D(veh_id, float(x), float(y)))
        if d <= _INVALID_CUTOFF or math.isinf(d) or math.isnan(d) or d > 1e12:
            return None
        return d
    except Exception:
        return None


def _euclid_distance_to_center(veh_id: str, cx: float, cy: float) -> float:
    try:
        x, y = traci.vehicle.getPosition(veh_id)
        return math.hypot(float(x) - cx, float(y) - cy)
    except Exception:
        return float("inf")


def _movement_from_route(
    route: Iterable[str],
    last_approach_edge: str,
    straight_edge: str,
    right_edge: str,
) -> Optional[str]:
    r = list(route)
    if last_approach_edge not in r:
        return None
    i = r.index(last_approach_edge)
    if i + 1 >= len(r):
        return None
    nxt = r[i + 1]
    if nxt == right_edge:
        return "right"
    if nxt == straight_edge:
        return "straight"
    return "other"


# =========================================================
# Data models
# =========================================================

@dataclass
class ConflictPassState:
    """Tracks first entry and exit of a road user through the conflict zone."""
    entered_t: Optional[float] = None
    exited_t: Optional[float] = None
    is_inside: bool = False


@dataclass
class RoadUserSafety:
    user_id: str
    depart_time_s: float

    arrive_time_s: Optional[float] = None
    travel_time_s: Optional[float] = None

    min_ttc_s: Optional[float] = None
    min_ttc_with: Optional[str] = None

    min_pet_s: Optional[float] = None
    min_pet_with: Optional[str] = None

    zone: ConflictPassState = field(default_factory=ConflictPassState)


# =========================================================
# CSV field list (single source of truth)
# =========================================================

_CSV_FIELDS = [
    "user_id", "type", "depart_time_s", "arrive_time_s",
    "travel_time_s", "zone_entered_t", "zone_exited_t",
    "min_ttc_s", "min_ttc_with", "min_pet_s", "min_pet_with",
]


# =========================================================
# Evaluator class
# =========================================================

class JunctionSafetyEvaluator:
    """Evaluates TTC and PET between right-turn motor vehicles and straight bicycles.

    Usage::

        safety = JunctionSafetyEvaluator(
            conflict_center_xy=CONFLICT_CENTER_XY,
            conflict_radius_m=CONFLICT_RADIUS_M,
            last_approach_edge=LAST_APPROACH_EDGE,
            straight_edge=STRAIGHT_EDGE,
            right_edge=RIGHT_EDGE,
            output_csv_path="safety_conflicts.csv",
        )

        # inside the simulation loop, after each traci.simulationStep():
        safety.step(sim_t)

        # flush periodically and at end of simulation
        safety.flush_finished_to_csv(append=True)
    """

    def __init__(
        self,
        *,
        conflict_center_xy: Tuple[float, float] = (6740.38, 5306.24),
        conflict_radius_m: float = 2.0,
        last_approach_edge: str,
        straight_edge: str,
        right_edge: str,
        output_csv_path: str = "safety_conflicts.csv",
        speed_eps_mps: float = 0.2,
        pairing_distance_m: float = 80.0,
        ttc_time_diff_threshold_s: float = 1.0,
        ignore_vclasses: Tuple[str, ...] = ("pedestrian",),
        output_only_conflicts: bool = True,
        ttc_keep_threshold_s: float = 5.0,
        pet_keep_threshold_s: float = 3.0,
    ) -> None:
        self.cx, self.cy = float(conflict_center_xy[0]), float(conflict_center_xy[1])
        self.radius = float(conflict_radius_m)

        self.last_approach_edge = last_approach_edge
        self.straight_edge = straight_edge
        self.right_edge = right_edge

        self.output_csv_path = output_csv_path
        self.speed_eps_mps = float(speed_eps_mps)
        self.pairing_distance_m = float(pairing_distance_m)
        self.ttc_time_diff_threshold_s = float(ttc_time_diff_threshold_s)
        self.ignore_vclasses = tuple(ignore_vclasses)

        self.output_only_conflicts = bool(output_only_conflicts)
        self.ttc_keep_threshold_s = float(ttc_keep_threshold_s)
        self.pet_keep_threshold_s = float(pet_keep_threshold_s)

        self._veh_active: set[str] = set()
        self._bike_active: set[str] = set()
        self._veh: Dict[str, RoadUserSafety] = {}
        self._bike: Dict[str, RoadUserSafety] = {}
        self._pet_done: set[Tuple[str, str]] = set()
        self._last_sim_t: Optional[float] = None

    @property
    def vehicles(self) -> Dict[str, RoadUserSafety]:
        return self._veh

    @property
    def bikes(self) -> Dict[str, RoadUserSafety]:
        return self._bike

    # ------------------------------------------------------------------
    # Time-step bookkeeping
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

    def _register_departed(self, sim_t: float) -> None:
        for uid in traci.simulation.getDepartedIDList():
            try:
                vclass = traci.vehicle.getVehicleClass(uid)
                if vclass in self.ignore_vclasses:
                    continue
                route = traci.vehicle.getRoute(uid)
            except Exception:
                continue

            mov = _movement_from_route(
                route, self.last_approach_edge, self.straight_edge, self.right_edge
            )

            if vclass != "bicycle" and mov == "right":
                if uid not in self._veh:
                    self._veh[uid] = RoadUserSafety(user_id=uid, depart_time_s=float(sim_t))
                self._veh_active.add(uid)

            elif vclass == "bicycle" and mov == "straight":
                if uid not in self._bike:
                    self._bike[uid] = RoadUserSafety(user_id=uid, depart_time_s=float(sim_t))
                self._bike_active.add(uid)

    def _finalize_arrived(self, sim_t: float) -> None:
        arrived = set(traci.simulation.getArrivedIDList())
        if not arrived:
            return
        for uid in arrived:
            for store, active in ((self._veh, self._veh_active), (self._bike, self._bike_active)):
                if uid in store:
                    m = store[uid]
                    if m.arrive_time_s is None:
                        m.arrive_time_s = float(sim_t)
                        m.travel_time_s = float(sim_t) - m.depart_time_s
                    active.discard(uid)

    def _finalize_missing(self, uid: str, sim_t: float) -> None:
        for store, active in ((self._veh, self._veh_active), (self._bike, self._bike_active)):
            if uid in store:
                m = store[uid]
                if m.arrive_time_s is None:
                    m.arrive_time_s = float(sim_t)
                    m.travel_time_s = float(sim_t) - m.depart_time_s
                active.discard(uid)

    # ------------------------------------------------------------------
    # Conflict-zone state
    # ------------------------------------------------------------------

    def _distance_to_zone_boundary(self, uid: str) -> float:
        d_route = _safe_driving_distance_2d(uid, self.cx, self.cy)
        if d_route is not None:
            return max(0.0, d_route - self.radius)
        d_euc = _euclid_distance_to_center(uid, self.cx, self.cy)
        if math.isinf(d_euc) or math.isnan(d_euc):
            return float("inf")
        return max(0.0, d_euc - self.radius)

    def _update_zone_state(self, uid: str, state: ConflictPassState, sim_t: float) -> None:
        try:
            x, y = traci.vehicle.getPosition(uid)
            inside = math.hypot(float(x) - self.cx, float(y) - self.cy) <= self.radius
        except Exception:
            inside = False

        if (not state.is_inside) and inside:
            state.is_inside = True
            if state.entered_t is None:
                state.entered_t = float(sim_t)
        elif state.is_inside and (not inside):
            state.is_inside = False
            if state.entered_t is not None and state.exited_t is None:
                state.exited_t = float(sim_t)

    # ------------------------------------------------------------------
    # TTC / PET computation
    # ------------------------------------------------------------------

    def _update_ttc(self, sim_t: float) -> None:
        if not self._veh_active or not self._bike_active:
            return

        veh_candidates = [
            vid for vid in self._veh_active
            if vid in traci.vehicle.getIDList()
            and _euclid_distance_to_center(vid, self.cx, self.cy) <= self.pairing_distance_m
        ]
        bike_candidates = [
            bid for bid in self._bike_active
            if bid in traci.vehicle.getIDList()
            and _euclid_distance_to_center(bid, self.cx, self.cy) <= self.pairing_distance_m
        ]

        if not veh_candidates or not bike_candidates:
            return

        for vid in veh_candidates:
            v_speed = _safe_float(traci.vehicle.getSpeed, vid)
            v_d = self._distance_to_zone_boundary(vid)
            if not math.isfinite(v_d):
                continue
            t_v = 0.0 if v_d <= 0.0 else (v_d / max(v_speed, self.speed_eps_mps))

            for bid in bike_candidates:
                b_speed = _safe_float(traci.vehicle.getSpeed, bid)
                b_d = self._distance_to_zone_boundary(bid)
                if not math.isfinite(b_d):
                    continue
                t_b = 0.0 if b_d <= 0.0 else (b_d / max(b_speed, self.speed_eps_mps))

                if abs(t_v - t_b) > self.ttc_time_diff_threshold_s:
                    continue

                ttc_pair = max(t_v, t_b)

                vm = self._veh.get(vid)
                if vm is not None and (vm.min_ttc_s is None or ttc_pair < vm.min_ttc_s):
                    vm.min_ttc_s = float(ttc_pair)
                    vm.min_ttc_with = bid

                bm = self._bike.get(bid)
                if bm is not None and (bm.min_ttc_s is None or ttc_pair < bm.min_ttc_s):
                    bm.min_ttc_s = float(ttc_pair)
                    bm.min_ttc_with = vid

    @staticmethod
    def _pet_from_intervals(a_in: float, a_out: float, b_in: float, b_out: float) -> float:
        if a_out <= b_in:
            return b_in - a_out
        if b_out <= a_in:
            return a_in - b_out
        return 0.0

    def _update_pet(self) -> None:
        finished_veh  = [vid for vid, m in self._veh.items()  if m.zone.exited_t is not None]
        finished_bike = [bid for bid, m in self._bike.items() if m.zone.exited_t is not None]
        if not finished_veh or not finished_bike:
            return

        for vid in finished_veh:
            v = self._veh[vid]
            if v.zone.entered_t is None or v.zone.exited_t is None:
                continue
            for bid in finished_bike:
                key = (vid, bid)
                if key in self._pet_done:
                    continue
                b = self._bike[bid]
                if b.zone.entered_t is None or b.zone.exited_t is None:
                    continue

                pet = self._pet_from_intervals(
                    v.zone.entered_t, v.zone.exited_t,
                    b.zone.entered_t, b.zone.exited_t,
                )
                self._pet_done.add(key)

                if v.min_pet_s is None or pet < v.min_pet_s:
                    v.min_pet_s = float(pet)
                    v.min_pet_with = bid

                if b.min_pet_s is None or pet < b.min_pet_s:
                    b.min_pet_s = float(pet)
                    b.min_pet_with = vid

    # ------------------------------------------------------------------
    # Main step method
    # ------------------------------------------------------------------

    def step(self, sim_t: Optional[float] = None) -> None:
        """Call exactly once after each ``traci.simulationStep()``."""
        if sim_t is None:
            sim_t = float(traci.simulation.getTime())
        sim_t = float(sim_t)
        self._get_dt(sim_t)

        self._register_departed(sim_t)

        for vid in list(self._veh_active):
            if vid not in traci.vehicle.getIDList():
                self._finalize_missing(vid, sim_t)
                continue
            self._update_zone_state(vid, self._veh[vid].zone, sim_t)

        for bid in list(self._bike_active):
            if bid not in traci.vehicle.getIDList():
                self._finalize_missing(bid, sim_t)
                continue
            self._update_zone_state(bid, self._bike[bid].zone, sim_t)

        self._update_ttc(sim_t)
        self._update_pet()
        self._finalize_arrived(sim_t)

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------

    def _passes_output_filter(self, m: RoadUserSafety) -> bool:
        if m.min_ttc_s is None or m.min_pet_s is None:
            return False
        return (m.min_ttc_s <= self.ttc_keep_threshold_s) and (m.min_pet_s <= self.pet_keep_threshold_s)

    def _make_row(self, uid: str, typ: str, m: RoadUserSafety) -> dict:
        return {
            "user_id": uid,
            "type": typ,
            "depart_time_s": m.depart_time_s,
            "arrive_time_s": m.arrive_time_s,
            "travel_time_s": m.travel_time_s,
            "zone_entered_t": m.zone.entered_t,
            "zone_exited_t": m.zone.exited_t,
            "min_ttc_s": m.min_ttc_s,
            "min_ttc_with": m.min_ttc_with,
            "min_pet_s": m.min_pet_s,
            "min_pet_with": m.min_pet_with,
        }

    def flush_finished_to_csv(self, append: bool = True) -> None:
        """Write all finished road users to CSV and free their memory.

        Args:
            append — when True, rows are appended to an existing file;
                     when False, a new file is created with a header row.
        """
        def _eligible_rows():
            for uid, m in self._veh.items():
                if m.arrive_time_s is None:
                    continue
                if self.output_only_conflicts and not self._passes_output_filter(m):
                    continue
                yield "vehicle_right", uid, m
            for uid, m in self._bike.items():
                if m.arrive_time_s is None:
                    continue
                if self.output_only_conflicts and not self._passes_output_filter(m):
                    continue
                yield "bicycle_straight", uid, m

        rows_out = list(_eligible_rows())
        if not rows_out:
            return

        mode = "a" if append else "w"
        with open(self.output_csv_path, mode, newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=_CSV_FIELDS)
            if not append:
                w.writeheader()
            for typ, uid, m in rows_out:
                w.writerow(self._make_row(uid, typ, m))

        for _, uid, _ in rows_out:
            self._veh.pop(uid, None)
            self._bike.pop(uid, None)

    def write_csv(self, path: Optional[str] = None) -> str:
        """Write all tracked road users to a CSV file. Returns the output path."""
        out_path = path or self.output_csv_path
        with open(out_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=_CSV_FIELDS)
            w.writeheader()
            for uid, m in self._veh.items():
                if self.output_only_conflicts and not self._passes_output_filter(m):
                    continue
                w.writerow(self._make_row(uid, "vehicle_right", m))
            for uid, m in self._bike.items():
                if self.output_only_conflicts and not self._passes_output_filter(m):
                    continue
                w.writerow(self._make_row(uid, "bicycle_straight", m))
        return out_path

    def summary(self) -> Dict[str, Any]:
        """Return summary statistics for right-turn vehicles."""
        finished_veh = [m for m in self._veh.values() if m.arrive_time_s is not None]
        n = len(finished_veh)
        if n == 0:
            return {"n_vehicle_finished": 0}

        ttc_vals = [m.min_ttc_s for m in finished_veh if m.min_ttc_s is not None]
        pet_vals = [m.min_pet_s for m in finished_veh if m.min_pet_s is not None]
        passing  = [m for m in finished_veh if self._passes_output_filter(m)]

        def avg(vals):
            return float(sum(vals) / len(vals)) if vals else None

        return {
            "n_vehicle_finished":        n,
            "n_vehicle_with_ttc":        len(ttc_vals),
            "n_vehicle_with_pet":        len(pet_vals),
            "n_vehicle_passing_filter":  len(passing),
            "filter_ttc_s":              self.ttc_keep_threshold_s,
            "filter_pet_s":              self.pet_keep_threshold_s,
            "avg_min_ttc_s":             avg([m.min_ttc_s for m in passing if m.min_ttc_s is not None]),
            "min_ttc_s_overall":         float(min(ttc_vals)) if ttc_vals else None,
            "avg_min_pet_s":             avg([m.min_pet_s for m in passing if m.min_pet_s is not None]),
            "min_pet_s_overall":         float(min(pet_vals)) if pet_vals else None,
        }
