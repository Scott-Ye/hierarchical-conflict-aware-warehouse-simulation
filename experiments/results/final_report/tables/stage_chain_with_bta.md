# Mainline Stage Chain With BaselineTrafficAware

| Scenario | Stage | Planner | Packages | PPH | Collisions | Replans | Runtime(s) | Status |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| small | Baseline | LayeredAStar | 4 | 240.0 | 8 | 100 | 8.523 | available |
| small | CollisionAware | LayeredAStarCollisionAware | 5 | 300.0 | 0 | 203 | 18.814 | available |
| small | ReservationAware | LayeredAStarReservationAware | 0 | 0.0 | 0 | 1124 | 38.762 | available |
| small | QueueAware | LayeredAStarQueueAware | 1 | 60.0 | 1 | 965 | 30.711 | available |
| small | BaselineTrafficAware | LayeredAStarBaselineTrafficAware | 1 | 60.0 | 0 | 63 | 9.685 | available |
| medium | Baseline | LayeredAStar | 6 | 360.0 | 235 | 616 | 52.631 | available |
| medium | CollisionAware | LayeredAStarCollisionAware | 6 | 360.0 | 0 | 404 | 84.495 | available |
| medium | ReservationAware | LayeredAStarReservationAware | 7 | 420.0 | 6 | 242 | 81.186 | available |
| medium | QueueAware | LayeredAStarQueueAware | 5 | 300.0 | 0 | 721 | 67.102 | available |
| medium | BaselineTrafficAware | LayeredAStarBaselineTrafficAware | 6 | 360.0 | 0 | 261 | 33.512 | available |
| high | Baseline | LayeredAStar | 5 | 300.0 | 949 | 2082 | 105.051 | available |
| high | CollisionAware | LayeredAStarCollisionAware | 7 | 420.0 | 27 | 470 | 120.35 | available |
| high | ReservationAware | LayeredAStarReservationAware | 3 | 180.0 | 1068 | 3932 | 158.4 | available |
| high | QueueAware | LayeredAStarQueueAware | 7 | 420.0 | 18 | 523 | 72.804 | available |
| high | BaselineTrafficAware | LayeredAStarBaselineTrafficAware | 6 | 360.0 | 0 | 279 | 57.448 | available |
