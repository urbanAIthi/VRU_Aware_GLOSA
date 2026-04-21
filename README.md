# VRU-aware GLOSA

This repository contains the simulation implementation for the paper:
**"VRU-Aware GLOSA: Integrating VRUs into Green Light Optimized Speed Advisory for Right-Turn Conflicts at Signalized Intersections"**, to be presented at SUMO User Conference 2026

This work evaluates the effect of GLOSA and VRU-aware GLOSA on traffic efficiency and safety at a
signalised junction in Ingolstadt, Germany, using the SUMO traffic simulator. Three scenarios are
compared: a baseline (no control), standard GLOSA, and a VRU-aware GLOSA extension that detects
approaching cyclists and adjusts the speed advisory for right-turning vehicles to prevent conflicts.

---

## Installation

### Prerequisites

- [Anaconda or Miniconda](https://www.anaconda.com/download)
- [SUMO](https://sumo.dlr.de/docs/Installing/index.html) traffic simulator **version 1.23.0**
  (results may vary with other versions)

### Cloning the repository

```bash
git clone https://github.com/mahajanpushkar/VRU_Aware_GLOSA.git
cd VRU_Aware_GLOSA
```

### Setting up the environment

Create and activate a Conda environment using the provided `environment.yml`:

```bash
conda env create -f environment.yml
conda activate vru_aware_glosa
```

### SUMO Python path

Ensure the SUMO tools directory is on your `PYTHONPATH` so that `traci` can be imported:

```bash
# Linux / macOS
export PYTHONPATH="$SUMO_HOME/tools:$PYTHONPATH"

# Windows (PowerShell)
$env:PYTHONPATH = "$env:SUMO_HOME\tools;$env:PYTHONPATH"
```

---

## Input Files

The SUMO network and route files are included in the repository under `sumo_files/`.

```
sumo_files/
├── ingolstadt_tiny.sumocfg
├── net/
│   └── ingolstadt_tiny.net.xml
└── routes/
    ├── ingolstadt_tiny_vehicles_12H.rou.xml
    ├── ingolstadt_tiny_bikes_12H.rou.xml
    ├── ingolstadt_tiny_vehicles_1H.rou.xml
    ├── ingolstadt_tiny_bikes_1H_Conv_above_60.rou.xml
    ├── ingolstadt_tiny_bikes_1H_Conv_below_40.rou.xml
    ├── ingolstadt_tiny_bikes_1H_Conv_between_40_and_60.rou.xml
    ├── ingolstadt_tiny_bikes_1H_Ebike_above_60.rou.xml
    ├── ingolstadt_tiny_bikes_1H_Ebike_below_40.rou.xml
    ├── ingolstadt_tiny_bikes_1H_Ebike_between_40_and_60.rou.xml
    ├── ingolstadt_tiny_bikes_1H_SPedelec_above_60.rou.xml
    ├── ingolstadt_tiny_bikes_1H_SPedelec_below_40.rou.xml
    └── ingolstadt_tiny_bikes_1H_SPedelec_between_40_and_60.rou.xml
```
---

## Configuration

All parameters are centralised in **`config.py`**. Before each run, set the four
variables at the top of the file to select the study, bicycle demand variant, and simulation time window:

```python
ACTIVE_STUDY   = "A"             # "A" = 12-hour study  |  "B" = 1-hour study (11:00–12:00)
ACTIVE_VARIANT = "Conv_above_60" # Study B only — ignored when ACTIVE_STUDY = "A"
SIM_BEGIN      = 0               # Study A: 0  |  Study B: 39600
SIM_END        = 43200           # Study A: 43200  |  Study B: 43200
```

Study B supports nine bicycle variants across three bike types
(Conventional, E-bike, S-Pedelec) and  three rider age groups
(below 40, between 40–60, above 60). No other file needs to be edited between runs —
the correct network, route files, and output paths are resolved automatically.

---

## Running the Simulations

Run all commands from the **project root directory** (the folder containing `config.py`).

```bash
# Baseline — no speed control
python -m simulations.baseline

# GLOSA — speed advisory for all vehicles
python -m simulations.glosa

# VRU-aware GLOSA — GLOSA with bicycle conflict detection
python -m simulations.vru_aware_glosa
```

To use the SUMO GUI, replace `"sumo"` with `"sumo-gui"` in the `SUMO_CMD` list at
the top of the relevant simulation script.

---

## Output Files

Each run writes three files to the corresponding scenario subfolder under `outputs/`:

```
outputs/
├── study_A/
│   ├── baseline/
│   │   ├── right_turn_metrics.csv
│   │   ├── safety_conflicts_ttc5_pet3.csv
│   │   └── sumo_log_baseline.txt
│   ├── glosa/
│   └── vru_aware_glosa/
└── study_B/
    └── <variant>/
        ├── baseline/
        ├── glosa/
        └── vru_aware_glosa/
```

### Traffic-efficiency CSV — `right_turn_metrics.csv`

One row per right-turning motor vehicle that completed its trip.

| Column | Description |
|---|---|
| `veh_id` | Vehicle identifier |
| `depart_time_s` | Departure time (s) |
| `arrive_time_s` | Arrival time (s) |
| `travel_time_s` | Total travel time (s) |
| `waiting_time_s` | Total time spent below the stop-speed threshold (s) |
| `stops_count` | Number of moving → stopped transitions |
| `co2_mg` | Total CO₂ emissions (mg) |
| `co_mg` | Total CO emissions (mg) |
| `cox_mg` | Combined COx = CO₂ + CO (mg) |
| `fuel_mg` | Total fuel consumption (mg) |
| `electricity_Wh` | Total electricity consumption (Wh) |
| `red_waiting_time_s` | Waiting time at red signals specifically (s) |
| `red_stops_count` | Number of stops at red signals |
| `first_red_stop_start_s` | Simulation time when the first red stop began (s) |
| `first_red_restart_s` | Simulation time of restart after the first red stop (s) |
| `restart_edge_id` | Road edge where the vehicle restarted after its first red stop |

### Safety CSV — `safety_conflicts_ttc5_pet3.csv`

One row per road user (vehicle or bicycle) that met the conflict filter:
TTC ≤ 5 s **and** PET ≤ 3 s.

| Column | Description |
|---|---|
| `user_id` | Road-user identifier |
| `type` | `vehicle_right` or `bicycle_straight` |
| `depart_time_s` | Departure time (s) |
| `arrive_time_s` | Arrival time (s) |
| `travel_time_s` | Total travel time (s) |
| `zone_entered_t` | Time of first entry into the conflict zone (s) |
| `zone_exited_t` | Time of first exit from the conflict zone (s) |
| `min_ttc_s` | Minimum Time-To-Collision recorded over the trip (s) |
| `min_ttc_with` | ID of the opposing road user at minimum TTC |
| `min_pet_s` | Minimum Post-Encroachment Time recorded over the trip (s) |
| `min_pet_with` | ID of the opposing road user at minimum PET |

---

## Project Structure

```
glosa_simulation/
├── config.py                        ← Study/variant selector + all shared settings
│
├── sumo_files/                      ← SUMO input files 
│   ├── ingolstadt_tiny.sumocfg
│   ├── net/
│   └── routes/
│
├── outputs/                         ← Auto-created at runtime; one folder per scenario
│
├── evaluators/
│   ├── traffic_efficiency.py        ← RightTurnEfficiencyEvaluator
│   └── safety_metrics.py            ← JunctionSafetyEvaluator (TTC & PET)
│
├── utils/
│   ├── network.py                   ← Lane lengths, distance-to-stop-line, route classification
│   ├── tls.py                       ← TLS cache and future signal state builder
│   └── glosa_core.py                ← GLOSA speed-advisory algorithm
│
├── simulations/
│   ├── baseline.py                  ← No speed control
│   ├── glosa.py                     ← GLOSA speed advisory
│   └── vru_aware_glosa.py           ← GLOSA + VRU bicycle conflict detection
│
├── .gitignore
└── README.md
```
