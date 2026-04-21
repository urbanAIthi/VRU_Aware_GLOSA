# sumo_files/

This folder contains all SUMO input files for the Ingolstadt single-intersection scenario.

```
sumo_files/
├── ingolstadt_tiny.sumocfg                                  ← SUMO configuration (static settings)
├── net/
│   └── ingolstadt_tiny.net.xml                              ← Road network (shared by both studies)
└── routes/
    │
    │── Study A (12-hour demand)
    ├── ingolstadt_tiny_vehicles_12H.rou.xml
    ├── ingolstadt_tiny_bikes_12H.rou.xml
    │
    └── Study B (1-hour demand, 9 bike variants)
        ├── ingolstadt_tiny_vehicles_12H.rou.xml              ← same vehicle file for all B variantsbut traffic starts at 11th hour
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