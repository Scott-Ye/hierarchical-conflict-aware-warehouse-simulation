# Stage Chain (DullPlanner)

| Scenario | Stage | Planner | Packages | PPH | Collisions | Replans | Runtime(s) | Status |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| small | baseline | LayeredAStar | 4 | 240.0 | 8 | 100 | 8.523 | available |
| small | v1 | LayeredAStarCollisionAware | 5 | 300.0 | 0 | 203 | 18.814 | available |
| small | v2 | LayeredAStarReservationAware | 0 | 0.0 | 0 | 1124 | 38.762 | available |
| small | v3 | LayeredAStarQueueAware | 1 | 60.0 | 1 | 965 | 30.711 | available |
| medium | baseline | LayeredAStar | 6 | 360.0 | 235 | 616 | 52.631 | available |
| medium | v1 | LayeredAStarCollisionAware | 6 | 360.0 | 0 | 404 | 84.495 | available |
| medium | v2 | LayeredAStarReservationAware | 7 | 420.0 | 6 | 242 | 81.186 | available |
| medium | v3 | LayeredAStarQueueAware | 5 | 300.0 | 0 | 721 | 67.102 | available |
| high | baseline | LayeredAStar | 5 | 300.0 | 949 | 2082 | 105.051 | available |
| high | v1 | LayeredAStarCollisionAware | 7 | 420.0 | 27 | 470 | 120.35 | available |
| high | v2 | LayeredAStarReservationAware | 3 | 180.0 | 1068 | 3932 | 158.4 | available |
| high | v3 | LayeredAStarQueueAware | 7 | 420.0 | 18 | 523 | 72.804 | available |
