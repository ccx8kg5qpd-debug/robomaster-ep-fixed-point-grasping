# Real-hardware demonstration

`RoboMaster_EP_Hardware_Demonstration.mp4` is the real-device recording supplied
for the group submission. It shows the physical RoboMaster EP arm, chassis,
gripper, target cubes and operator terminal. A detailed review of the final
segment was used for the 14 September 2026 evidence correction.

## File metadata

- Original supplied filename: `b635ef9bacf591f62e65e10741d55908.mp4`
- Archived filename: `RoboMaster_EP_Hardware_Demonstration.mp4`
- Size: 8,024,795 bytes
- Duration: 74.87 s
- Video: HEVC Main, 1280 x 720, 29.97 fps
- Audio: AAC LC, stereo, 44.1 kHz
- SHA-256: `614a8eb55d8883cce4494496a199c828bb679b693f58a325319570a89fad6a6e`

The group separately confirmed that the five hardware transfers were successful
with no observed abnormality in the five nominal trials. The team clarified
that the final empty grasp was deliberately induced by removing the object for
an additional abnormal-handling test; it is not a failed fifth nominal trial.

## Final-segment interpretation (H-01)

- Approximately 58--61 s: the operator removes the target from the grasp area.
- Approximately 61--69 s: the empty gripper closes and lifts; the cube remains
  on the table away from the gripper.
- Remainder of the 74.87 s clip: no subsequent transfer rotation is visible.

The saved SDK program uses human confirmation, not automatic object detection.
Without a fresh `y`, its negative/blank/timeout branch aborts above the source
and attempts cleanup. The recording does not independently establish the exact
terminal response, a full 30-second wait, exit code or successful SDK closure.
See `results/abnormal_test_record.md` at the repository root for the full record.
The video is observational evidence, not a machine-generated position-error,
force or timing log. The original video bytes and checksum are unchanged.
