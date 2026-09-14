# RoboMaster EP Fixed-Point Grasping - Experiment 2

Shared project archive for **Robotics Integration Group Project I**, Experiment 2.
The group adapted the fixed-point pick-and-place task to the DJI RoboMaster EP
arm because the originally specified single-arm equipment was unavailable.

## Team

| Member | Student ID | Main scope |
|---|---:|---|
| Yongyue Qi | 24020036050 | Simulation evidence, result analysis and report integration |
| Bo Ning | 24020036049 | Simulation environment, scene, models and launch setup |
| Junbiao Huang | 24020036027 | Simulation motion program and repeated-cycle debugging |
| Yutong Xue | 24020036072 | Hardware waypoints, approach/lift heights and gripper tuning |
| Lintai Bao | 24020036004 | Hardware SDK communication, task sequence and chassis turn/return |

Xue and Bao jointly performed the physical setup and integrated hardware trials.
All five members share the overall project outcome; the individual reports focus
on different assigned responsibilities and reflections.

## Verified outcome

- Recorded simulation: 5/5 successful placements and 5/5 verified returns home.
- Team-observed hardware test on 12 September 2026: 5/5 successful transfers,
  with no observed abnormality.
- The supplied hardware evidence is observational. No measured hardware position
  error, timing, force or payload-mass record is available.

## Repository structure

- `simulation/`: ROS 2/Gazebo controller, launch/configuration and model resources.
- `hardware/`: RoboMaster Python SDK program used for the final manual-confirmation run.
- `results/`: saved machine-readable simulation logs and validation notes.
- `media/`: simulation recording and media notes. The real-hardware video is pending.
- `reports/`: five individual English LaTeX reports and compiled PDFs.
- `docs/`: course handout copy and traceability notes.

## Version-history note

The experiment was completed before this GitHub repository was assembled.
Therefore, the commit history records the genuine sequence used on 13 September
2026 to review, organize, document and publish the existing project materials; it
must not be interpreted as a contemporaneous history of every experimental edit.
No historical commit dates or authors have been fabricated.

## Reproduction limits

The simulation targets ROS 2 Humble and Gazebo Fortress on the team's Jetson Linux
environment. The archived `start.sh` contains a workspace path that another user
must adapt. The physical program requires a RoboMaster EP connected through the
RoboMaster Python SDK and must be run under supervised low-speed conditions.

## Third-party resource

Bundled robot model resources originate from
[`jeguzzi/robomaster_ros`](https://github.com/jeguzzi/robomaster_ros), archived at
commit `c05a39d7f0fa`. See the source and licensing information beside the model
files. No license is asserted here for third-party content beyond its upstream
terms.
