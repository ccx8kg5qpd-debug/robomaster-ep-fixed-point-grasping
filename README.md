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
All five members share the overall project outcome.

## Verified outcome

- Recorded simulation: 5/5 successful placements and 5/5 verified returns home.
- Team-observed hardware test on 12 September 2026: 5/5 successful transfers,
  with no observed abnormality in those nominal trials.
- Additional hardware abnormal test H-01: the operator deliberately removes the
  object in the final video segment. The empty grasp/lift is excluded from the
  nominal five-trial denominator. No later transfer is visible in the clip.
  See [the abnormal-test record](results/abnormal_test_record.md) for observed
  behavior, the manual-confirmation guard, and evidence limits.
- The supplied hardware evidence is observational. No measured hardware position
  error, timing, force or payload-mass record is available.

## Repository structure

- `simulation/`: ROS 2/Gazebo controller, launch/configuration and model resources.
- `hardware/`: RoboMaster Python SDK program used for the final manual-confirmation run.
- `results/`: saved machine-readable simulation logs and validation notes.
- `media/`: complete simulation recording, real-hardware demonstration and media notes.
- `reports/group/`: the English group report, self-contained Tau-based LaTeX
  source, figures, template license and QA summary.
- `docs/`: copy of the original course handout.

The brief group report required by the experiment handout is under
`reports/group/`. Personal reports are intentionally excluded from this shared
group-project repository.

## Version-history note

The experiment was completed before this GitHub repository was assembled.
Therefore, the commit history records the genuine sequence used on 13--14 September
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
