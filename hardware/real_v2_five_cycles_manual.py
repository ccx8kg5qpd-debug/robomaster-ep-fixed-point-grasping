"""Five transfers with human confirmation after each 50 mm lift."""
import select
import sys
import time
from robomaster import robot

HOME = (82, 80)
HIGH = (191, 80)
HOVER = (191, -1)
GRASP = (191, -51)
CYCLES = 5
TURN_DEG = 32
CONFIRM_SECONDS = 30


def wait(action, label, timeout=12):
    print(label, flush=True)
    if not action.wait_for_completed(timeout=timeout):
        raise RuntimeError(label + " FAILED")


def stop_chassis(chassis):
    if not chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0):
        raise RuntimeError("STOP_WHEELS_FAILED")


def grip(g, opening):
    ok = g.open(power=30) if opening else g.close(power=100)
    if not ok:
        raise RuntimeError("GRIPPER_COMMAND_FAILED")
    time.sleep(1.5 if opening else 3.0)
    if not g.pause():
        raise RuntimeError("GRIPPER_PAUSE_FAILED")


def confirm(cycle):
    # Discard keys typed during motion; require a new answer at this checkpoint.
    import termios
    termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
    print(f"\n第 {cycle}/5 次：物块是否牢牢夹住并随夹爪抬升？", flush=True)
    print("输入 y 后回车继续；n/直接回车/30秒未确认则停止：", flush=True)
    ready, _, _ = select.select([sys.stdin], [], [], CONFIRM_SECONDS)
    if not ready:
        print("CONFIRM_TIMEOUT：未确认，停止", flush=True)
        return False
    return sys.stdin.readline().strip().lower() == "y"


def main():
    if not sys.stdin.isatty():
        print("请保存成 .py 文件并在交互终端运行，不能用管道或 python - 输入代码。", flush=True)
        return 1
    ep = robot.Robot()
    g = chassis = None
    completed = 0
    heading = 0
    try:
        print("CONNECTING_AP", flush=True)
        ep.initialize(conn_type="ap")
        arm, g, chassis = ep.robotic_arm, ep.gripper, ep.chassis
        stop_chassis(chassis)
        grip(g, True)
        for point, label in [(HOME, "INITIALIZE_HOME"),
                             (HIGH, "EXTEND_AT_SAFE_HEIGHT"),
                             (HOVER, "MOVE_TO_FIRST_HOVER")]:
            wait(arm.moveto(x=point[0], y=point[1]), label)

        for cycle in range(1, CYCLES + 1):
            source, destination = ("A", "B") if cycle % 2 else ("B", "A")
            turn = TURN_DEG if cycle % 2 else -TURN_DEG
            print(f"CYCLE_{cycle}_START_{source}_TO_{destination}", flush=True)
            wait(arm.moveto(x=GRASP[0], y=GRASP[1]), "DESCEND_TO_SOURCE")
            grip(g, False)
            wait(arm.moveto(x=HOVER[0], y=HOVER[1]), "LIFT_50MM_FOR_CONFIRMATION")
            stop_chassis(chassis)
            if not confirm(cycle):
                print("抓取未获确认，停在当前取物点上方，不转动、不回零。", flush=True)
                print("STOPPED_ABOVE_CURRENT_SOURCE", flush=True)
                print(f"COMPLETED_CYCLES={completed}", flush=True)
                print("FIVE_CYCLES_ABORTED", flush=True)
                return 2

            print("MANUAL_GRASP_CONFIRMED", flush=True)
            wait(chassis.move(x=0, y=0, z=turn, z_speed=12), "TURN_TO_DESTINATION", 15)
            heading += turn
            time.sleep(0.5)
            wait(arm.moveto(x=GRASP[0], y=GRASP[1]), "LOWER_AT_DESTINATION")
            grip(g, True)
            time.sleep(0.8)
            wait(arm.moveto(x=HOVER[0], y=HOVER[1]), "LIFT_AFTER_RELEASE")
            completed += 1
            print(f"CYCLE_{cycle}_COMPLETE", flush=True)

        if heading:
            wait(chassis.move(x=0, y=0, z=-heading, z_speed=12), "RETURN_CHASSIS", 15)
        wait(arm.moveto(x=HIGH[0], y=HIGH[1]), "RAISE_TO_SAFE_HEIGHT")
        wait(arm.moveto(x=HOME[0], y=HOME[1]), "RETURN_HOME")
        print(f"COMPLETED_CYCLES={completed}", flush=True)
        print("FIVE_CYCLES_FINISHED", flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print(f"EXPERIMENT_STOPPED={type(exc).__name__}: {exc}", flush=True)
        return 1
    finally:
        actions = []
        if g is not None:
            actions.append(("GRIPPER_PAUSE", g.pause))
        if chassis is not None:
            actions.append(("CHASSIS_STOP", lambda: stop_chassis(chassis)))
        actions.append(("SDK_CLOSE", ep.close))
        for label, action in actions:
            try:
                action()
            except Exception as exc:
                print(f"CLEANUP_ERROR[{label}]={exc}", flush=True)
        print("CONNECTION_CLOSED", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
