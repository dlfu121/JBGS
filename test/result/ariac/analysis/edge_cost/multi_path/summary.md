# ARIAC multi-route cost-map benchmark

8 fixed routes × 5 methods = **40 planning trials**. All methods use the same saved PGM/PLY and endpoint cells.

| Method | Success | Mean length (m) | Mean min clearance (m) | Mean height clearance (m) | Mean planning (ms) |
|---|---:|---:|---:|---:|---:|
| Distance only | 100% | 23.613 | 0.510 | — | 12651.4 |
| Horizontal safety λg=1 | 100% | 23.993 | 0.802 | — | 22185.2 |
| Horizontal safety λg=2 | 100% | 24.003 | 0.809 | — | 21679.2 |
| Height cost λh=1 | 100% | 23.613 | 0.510 | — | 15208.8 |
| Height cost λh=2 | 100% | 23.613 | 0.510 | — | 15216.4 |

Height map evidence: 36350 observed cells, 345 also 2D-traversable cells; hard height 0.70 m, preferred 1.00 m.

Files: `multi_path_metrics.csv`, `multi_path_metrics.json`, `multi_paths_grid.png`, `length_clearance_scatter.png`, `aggregate_metrics.png`, `height_profiles_grid.png`.
