# Abnormal-test record / 异常测试记录

Evidence revision: 14 September 2026. Hardware nominal trial date reported by
the team: 12 September 2026. This is a retrospective evidence record, not an
original terminal log.

## Nominal trials and the separate negative test

The team confirms that all five nominal hardware transfers succeeded with no
observed abnormality (5/5). The final empty grasp in the submitted video was
deliberately induced by removing the object to test abnormal handling. It is
recorded separately as H-01 and does not change the nominal denominator.

组员确认：五次正式抓取全部成功。视频末尾故意移走物块造成空抓，属于额外异常测试，
不计为第五次正式抓取失败，也不能据此把正式结果改为 4/5。

## H-01: object removal / empty grasp

| Field | Evidence |
|---|---|
| Platform | Physical DJI RoboMaster EP arm, gripper and chassis |
| Purpose | Exercise the supervised grasp-confirmation barrier with no retained object |
| Intervention | Operator deliberately removes the target before the final visible grasp, as confirmed by the team |
| Video observation | Approx. 58--61 s: removal; approx. 61--69 s: empty closure/lift; no subsequent transfer rotation is visible before the 74.87 s clip ends |
| Expected gate | After the 50 mm lift, stop the chassis and require a fresh `y` confirming retention before transfer |
| Expected abort | Negative/blank input or 30 s timeout returns false; hold above the source, skip rotation and homing, report abort |
| Expected cleanup | `finally` attempts gripper pause, zero wheel speed and SDK close; failures are separately reported |
| Supported conclusion | A deliberate real-device empty-grasp test was performed. The observed no-transfer behavior is consistent with the manual gate |
| Not directly verified | Exact terminal input, timeout branch, exit status, stop latency, every cleanup call succeeding, and the exact executed script version |

The logic above is established by inspection of
`hardware/real_v2_five_cycles_manual.py`, especially `confirm()`, the failed
confirmation branch in `main()`, and `finally`. These paths are code evidence,
not a claim that a terminal transcript was captured. A waiting checkpoint and
an already executed abort cannot be reliably distinguished from the visible
video alone. Do not describe this as automatic vision/force-based detection.

Expected messages (not a recovered runtime log): `STOPPED_ABOVE_CURRENT_SOURCE`,
`COMPLETED_CYCLES=...`, `FIVE_CYCLES_ABORTED`; timeout additionally prints
`CONFIRM_TIMEOUT`. `CONNECTION_CLOSED` is printed after cleanup attempts and by
itself would not prove that every cleanup action succeeded.

## Existing offline evidence and remaining coverage

Archived simulation validation notes report four model tests and four five-cycle
tests passing. An unreachable target is rejected by the numerical solver with
`ValueError`. That solver test is separate from hardware H-01.

No injected communication-loss, runtime joint-limit, stale-feedback or blocked-
refill test is documented. No individual safety-operation checklist or measured
hardware position, force, object-mass or per-cycle timing data was supplied.
H-01 does not establish those properties or complete fault coverage.

## Source traceability

- Original: `b635ef9bacf591f62e65e10741d55908.mp4`.
- Repository video: `media/hardware/RoboMaster_EP_Hardware_Demonstration.mp4`.
- Group ZIP video: `03_videos/hardware/RoboMaster_EP_Hardware_Demonstration.mp4`.
- SHA-256: `614a8eb55d8883cce4494496a199c828bb679b693f58a325319570a89fad6a6e`.
- Test intent and nominal outcome: team clarification relayed by Yongyue Qi.
- The video remains unedited; approximate time ranges are navigation aids.
