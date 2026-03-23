# Results Template

Use this file to publish the first honest benchmark table for the public simulation stack.

## Simulation Scenario Table

| Trial ID | Start pose | Target pose | Success | Time to goal (s) | Final goal error (m) | Notes |
|---|---|---|---|---:|---:|---|
| S-01 | fill | fill | pending | - | - | |
| S-02 | fill | fill | pending | - | - | |
| S-03 | fill | fill | pending | - | - | |

## Metrics To Add

| Area | Metric | Definition |
|---|---|---|
| Navigation | Success rate | Robot reaches defined tolerance around target |
| Navigation | Final goal error | Distance between robot and intended goal at end of run |
| Runtime | Startup time | Launch to usable navigation stack |
| Detection | Update rate | Detector output frequency for each mode |

## Separation Rule

Report these modes separately:
- `sim_person_detector` in Gazebo
- `detector_node` on real camera input

Those are different claims and should stay different in the public repo.
