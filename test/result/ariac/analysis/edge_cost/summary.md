# ARIAC edge-cost comparison

Map: `maps/ariac/ariac_map_3d.pgm`  
Route: `(-4.0, -4.0) -> (7.0, 7.0)` (map frame)  

| Case | lambda_geo | Success | Length (m) | Min clearance (m) | Mean clearance (m) | Planning (ms) |
| --- | ---: | :---: | ---: | ---: | ---: | ---: |
| A_baseline_reference | 0.0 | True | 16.214 | 0.510 | 1.464 | 4265.43 |
| B_unified_distance | 0.0 | True | 16.214 | 0.510 | 1.464 | 4262.92 |
| C_safety_aware | 2.0 | True | 16.676 | 0.832 | 1.585 | 8580.04 |

Safety-aware path length change: `2.85%`; minimum clearance change: `63.20%`.  
Baseline/unified-distance equivalent: `True`.  

## Height evidence

Observed cells: `36350`; observed and 2D-traversable cells: `345`.  
Result: independent height evidence is available on traversable cells.  

Metrics sample every raster cell crossed by each any-angle edge.

`path_height_profile.png` additionally contains three short `height_probe_*`
segments inside the small height-observed/2D-traversable patch near
`(x=2.3..3.3, y=14.4..15.4)`. The requested long route is explicitly marked
“no height returns” because its crossed cells contain no finite PLY height
observation; dotted preferred-height segments denote unknown samples.
