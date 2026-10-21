"""UR10e + Robotiq 2F-85 mobile manipulator with a simple VLP-16 lidar.

Lidar implementation uses MuJoCo's built-in mj_ray, so no third-party lidar
package is required. The scan is 360 degrees x 16 vertical channels at 10 Hz;
obstacle detection uses the front +/-30 degree, 2.0 m sector.
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import mujoco
from PIL import Image

import mobile_pick_place as base

# ---------------------------------------------------------------------------
# Lidar configuration
# ---------------------------------------------------------------------------
LIDAR_PERIOD = 0.1                 # 10 Hz
LIDAR_AZ_SAMPLES = 120             # 3 degree horizontal sampling
LIDAR_CHANNELS = 16
LIDAR_ELEV_MIN = np.deg2rad(-15.0)
LIDAR_ELEV_MAX = np.deg2rad(15.0)
LIDAR_MAX_RANGE = 100.0
LIDAR_DETECT_RANGE = 2.0
LIDAR_FRONT_HALF = np.deg2rad(30.0)
LIDAR_SIDE_MAX = np.deg2rad(90.0)
LIDAR_VIS_RANGE = 6.0

# S-shaped waypoint paths designed around the scattered obstacles.
OBJECT_PATH = [
    (0.8, -0.55),
    (1.8, -0.50),
    (2.7, -0.10),
    (3.7, 0.45),
    (4.6, 0.15),
    (5.0, 0.0),
]
BUCKET_PATH = [
    (5.20, 0.15),
    (5.55, 0.45),
    (5.80, 0.85),
    (5.90, 1.30),
    (5.75, 1.75),
    (5.50, 2.20),
    (5.30, 2.65),
    (5.15, 3.10),
    (5.05, 3.55),
    (5.00, 4.00),
]

# Chassis avoidance
AVOID_YAW_RATE = 0.80
AVOID_STEER = 0.55
AVOID_MIN_TIME = 0.8
AVOID_CLEAR_TIME = 0.5
MAX_AVOID_TIME = 6.0

# Geometry groups used by mj_ray. Group 0 contains the floor, task objects and
# obstacles. Group 3 contains robot collision geoms; group 2 contains visuals.
LIDAR_GEOMGROUP = np.array([1, 1, 0, 0, 0, 0], dtype=np.uint8)


class Lidar:
    def __init__(self) -> None:
        self.site_id = mujoco.mj_name2id(
            base.MODEL, mujoco.mjtObj.mjOBJ_SITE, "lidar_site"
        )
        if self.site_id < 0:
            raise RuntimeError("lidar_site not found")
        self.obstacle_geoms = set()
        for geom_id in range(base.MODEL.ngeom):
            name = mujoco.mj_id2name(
                base.MODEL, mujoco.mjtObj.mjOBJ_GEOM, geom_id
            )
            if name and name.startswith("obstacle_"):
                self.obstacle_geoms.add(geom_id)
        if not self.obstacle_geoms:
            raise RuntimeError("no obstacle geoms found")

        self.last_scan_time = -1e9
        self.front_blocked = False
        self.front_min = float("inf")
        self.left_min = float("inf")
        self.right_min = float("inf")
        self.vis_points: list[tuple[np.ndarray, bool]] = []
        self.front_points: list[np.ndarray] = []

    def scan_if_due(self) -> None:
        if base.DATA.time - self.last_scan_time >= LIDAR_PERIOD:
            self.scan()

    def scan(self) -> None:
        mujoco.mj_forward(base.MODEL, base.DATA)
        origin = base.DATA.site_xpos[self.site_id].copy()
        rot = base.DATA.site_xmat[self.site_id].reshape(3, 3).copy()

        vis_points: list[tuple[np.ndarray, bool]] = []
        front_points: list[np.ndarray] = []
        front_min = float("inf")
        left_min = float("inf")
        right_min = float("inf")

        elevations = np.linspace(LIDAR_ELEV_MIN, LIDAR_ELEV_MAX, LIDAR_CHANNELS)
        azimuths = np.linspace(-np.pi, np.pi, LIDAR_AZ_SAMPLES, endpoint=False)
        az_grid, elev_grid = np.meshgrid(azimuths, elevations)
        az_flat = az_grid.reshape(-1)
        elev_flat = elev_grid.reshape(-1)
        local_dirs = np.stack(
            [
                np.cos(elev_flat) * np.cos(az_flat),
                np.cos(elev_flat) * np.sin(az_flat),
                np.sin(elev_flat),
            ],
            axis=1,
        )
        world_dirs = local_dirs @ rot.T
        nray = int(world_dirs.shape[0])
        vec = world_dirs.reshape(-1)
        geomid = np.zeros(nray, dtype=np.int32)
        distances = np.zeros(nray, dtype=np.float64)
        mujoco.mj_multiRay(
            base.MODEL,
            base.DATA,
            origin,
            vec,
            LIDAR_GEOMGROUP,
            True,
            -1,
            geomid,
            distances,
            None,
            nray,
            LIDAR_MAX_RANGE,
        )

        for idx in range(nray):
            distance = float(distances[idx])
            gid = int(geomid[idx])
            if distance < 0.0 or distance > LIDAR_MAX_RANGE:
                continue
            if gid not in self.obstacle_geoms:
                continue

            hit = origin + distance * world_dirs[idx]
            if hit[2] < 0.08:
                continue
            # Pull visualization points slightly toward the sensor so they do
            # not z-fight with the obstacle surface they belong to.
            vis_hit = origin + max(0.0, distance - 0.025) * world_dirs[idx]

            az = float(az_flat[idx])
            is_front = abs(az) <= LIDAR_FRONT_HALF and distance <= LIDAR_DETECT_RANGE
            if is_front:
                front_points.append(hit.copy())
                front_min = min(front_min, distance)
            if distance <= LIDAR_DETECT_RANGE:
                if LIDAR_FRONT_HALF < az <= LIDAR_SIDE_MAX:
                    left_min = min(left_min, distance)
                elif -LIDAR_SIDE_MAX <= az < -LIDAR_FRONT_HALF:
                    right_min = min(right_min, distance)

            if distance <= LIDAR_VIS_RANGE:
                vis_points.append((vis_hit.copy(), is_front))

        self.vis_points = vis_points
        self.front_points = front_points
        self.front_min = front_min
        self.left_min = left_min
        self.right_min = right_min
        self.front_blocked = front_min <= LIDAR_DETECT_RANGE
        self.last_scan_time = base.DATA.time


def update_user_scene(viewer, lidar: Lidar) -> None:
    if viewer is None:
        return
    scn = viewer.user_scn
    scn.ngeom = 0
    for point, is_front in lidar.vis_points:
        if scn.ngeom >= scn.maxgeom:
            break
        color = np.array([1.0, 0.15, 0.05, 1.0] if is_front else [0.1, 0.9, 1.0, 1.0])
        geom = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([0.022, 0.0, 0.0]),
            point,
            np.eye(3).reshape(-1),
            color,
        )
        scn.ngeom += 1


def capture_snapshot(
    points: list[tuple[np.ndarray, bool]],
    path: str,
    lookat: list[float],
    distance: float,
    azimuth: float,
    elevation: float,
) -> None:
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat
    cam.distance = distance
    cam.azimuth = azimuth
    cam.elevation = elevation

    with mujoco.Renderer(base.MODEL, height=480, width=640) as renderer:
        renderer.update_scene(base.DATA, camera=cam)
        scn = renderer._scene
        for point, is_front in points:
            if scn.ngeom >= scn.maxgeom:
                break
            color = np.array([1.0, 0.15, 0.05, 1.0] if is_front else [0.1, 0.9, 1.0, 1.0])
            geom = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(
                geom,
                mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([0.022, 0.0, 0.0]),
                point,
                np.eye(3).reshape(-1),
                color,
            )
            scn.ngeom += 1
        image = renderer.render()
        Image.fromarray(image).save(path)
        print("saved", path)


def pivot_command(side: int) -> tuple[float, float, float]:
    """Return left/right wheel angular velocities and steering angle."""
    yaw_rate = side * AVOID_YAW_RATE
    left = (-yaw_rate * base.WHEEL_TRACK / 2.0) / base.WHEEL_RADIUS
    right = (yaw_rate * base.WHEEL_TRACK / 2.0) / base.WHEEL_RADIUS
    steer = side * AVOID_STEER
    return left, right, steer


def drive_chassis_lidar(
    sim: base.Simulation,
    arm: base.ArmMotion,
    finger: base.FingerMotion,
    lidar: Lidar,
    target_x: float,
    target_y: float,
    save_snapshot: bool = False,
) -> None:
    target = np.array([target_x, target_y], dtype=float)
    start_time = base.DATA.time
    left_cmd = 0.0
    right_cmd = 0.0
    steer_cmd = 0.0
    avoid_active = False
    avoid_waypoint = None
    avoid_stop_active = False
    avoid_stop_start = 0.0
    avoid_start = 0.0
    avoid_snapshot_saved = False
    ignore_front = False

    print(f"\n[LIDAR DRIVE] target=({target_x:.3f}, {target_y:.3f})")

    while base.DATA.time - start_time < base.MAX_DRIVE_TIME:
        lidar.scan_if_due()
        pos, yaw = base.chassis_pose()
        goal = avoid_waypoint if avoid_active else target
        dx = float(goal[0] - pos[0])
        dy = float(goal[1] - pos[1])
        distance = float(np.hypot(dx, dy))

        if avoid_active and distance < 0.15:
            print("  detour waypoint reached; resuming target")
            avoid_active = False
            avoid_waypoint = None
            avoid_stop_active = False
            ignore_front = False
            start_time = base.DATA.time
            continue
        if not avoid_active and distance < base.POS_TOL:
            break

        if avoid_stop_active:
            base.set_wheel_velocity(0.0, 0.0, 0.0)
            if save_snapshot and not avoid_snapshot_saved:
                capture_snapshot(
                    lidar.vis_points,
                    "lidar_avoid.png",
                    [1.00, 0.20, 0.30],
                    3.0,
                    135.0,
                    -22.0,
                )
                avoid_snapshot_saved = True
            update_user_scene(sim.viewer, lidar)
            arm.hold()
            finger.hold()
            sim.step()
            if base.DATA.time - avoid_stop_start >= 0.8:
                avoid_stop_active = False
                print("  obstacle stop complete; driving to detour waypoint")
            continue

        if not avoid_active and not ignore_front and lidar.front_blocked:
            avoid_active = True
            avoid_side = -1 if lidar.right_min >= lidar.left_min else 1
            side_name = "RIGHT" if avoid_side < 0 else "LEFT"
            lateral = 0.4 * avoid_side
            avoid_waypoint = np.array(
                [pos[0] + 1.2, pos[1] + lateral], dtype=float
            )
            avoid_stop_active = True
            avoid_stop_start = base.DATA.time
            avoid_start = base.DATA.time
            print(
                f"  obstacle detected at {lidar.front_min:.2f} m; "
                f"stopping and turning {side_name}; "
                f"detour via {np.round(avoid_waypoint, 2)}"
            )
            continue

        alpha = base.wrap_angle(float(np.arctan2(dy, dx)) - yaw)
        forward = float(np.clip(base.K_POS * distance, 0.0, base.V_MAX))
        forward *= float(np.cos(alpha))
        forward = float(np.clip(forward, -0.25, base.V_MAX))
        if abs(alpha) < np.deg2rad(95.0):
            forward = max(forward, 0.15)
        steer_target = float(
            np.arctan2(
                2.0 * base.STEER_WHEELBASE * np.sin(alpha),
                max(distance, 0.20),
            )
        )
        steer_target = float(np.clip(steer_target, -base.STEER_MAX, base.STEER_MAX))
        if forward > 0.02:
            yaw_rate = forward * float(np.tan(steer_target)) / base.STEER_WHEELBASE
        else:
            yaw_rate = 0.0
        yaw_rate = float(np.clip(yaw_rate, -base.W_MAX, base.W_MAX))
        left_target = (forward - yaw_rate * base.WHEEL_TRACK / 2.0) / base.WHEEL_RADIUS
        right_target = (forward + yaw_rate * base.WHEEL_TRACK / 2.0) / base.WHEEL_RADIUS
        left_cmd += base.COMMAND_RAMP * (left_target - left_cmd)
        right_cmd += base.COMMAND_RAMP * (right_target - right_cmd)
        steer_cmd += base.COMMAND_RAMP * (steer_target - steer_cmd)
        base.set_wheel_velocity(left_cmd, right_cmd, steer_cmd)

        update_user_scene(sim.viewer, lidar)
        arm.hold()
        finger.hold()
        sim.step()

    base.set_wheel_velocity(0.0, 0.0, 0.0)
    settle_steps = max(1, int(round(base.CHASSIS_SETTLE_TIME / sim.dt)))
    for _ in range(settle_steps):
        arm.hold()
        finger.hold()
        update_user_scene(sim.viewer, lidar)
        sim.step()

    pos, yaw = base.chassis_pose()
    print(f"  stopped at ({pos[0]:.3f}, {pos[1]:.3f}), yaw={yaw:.3f} rad")



def drive_path_lidar(
    sim: base.Simulation,
    arm: base.ArmMotion,
    finger: base.FingerMotion,
    lidar: Lidar,
    path: list[tuple[float, float]],
    save_snapshot: bool = False,
    snapshot_name: str = "lidar_avoid.png",
) -> None:
    """Follow a waypoint path and use lidar to avoid blocked corridors.

    When the front sector is blocked the chassis stops, picks the clearer
    side, steers away while creeping forward, and resumes the path once the
    front 2 m sector is clear. A waypoint timeout is fatal rather than being
    silently treated as success.
    """
    snapshot_saved = False
    avoid_active = False
    avoid_side = 0
    avoid_start = 0.0
    avoid_stop_until = 0.0
    left_cmd = 0.0
    right_cmd = 0.0
    steer_cmd = 0.0
    print(f"\n[LIDAR PATH] {len(path)} waypoints")

    for waypoint_index, (wx, wy) in enumerate(path):
        print(f"  waypoint {waypoint_index + 1}: ({wx:.2f}, {wy:.2f})")
        waypoint_start = base.DATA.time
        reached = False
        while base.DATA.time - waypoint_start < 18.0:
            lidar.scan_if_due()
            pos, yaw = base.chassis_pose()
            dx = float(wx - pos[0])
            dy = float(wy - pos[1])
            distance = float(np.hypot(dx, dy))
            waypoint_tol = 0.18 if waypoint_index == len(path) - 1 else 0.32
            if distance < waypoint_tol:
                reached = True
                break

            alpha = base.wrap_angle(float(np.arctan2(dy, dx)) - yaw)
            forward = float(np.clip(base.K_POS * distance, 0.0, base.V_MAX))
            forward *= max(0.0, float(np.cos(alpha)))
            if abs(alpha) < np.deg2rad(95.0):
                forward = max(forward, 0.15)

            if lidar.front_blocked:
                if not avoid_active:
                    avoid_active = True
                    avoid_side = -1 if lidar.right_min >= lidar.left_min else 1
                    avoid_start = base.DATA.time
                    avoid_stop_until = avoid_start + 0.45
                    side_name = "RIGHT" if avoid_side < 0 else "LEFT"
                    print(
                        f"  obstacle at {lidar.front_min:.2f} m: stop, "
                        f"turn {side_name}, then resume when clear"
                    )
                    if save_snapshot and not snapshot_saved:
                        capture_snapshot(
                            lidar.vis_points,
                            snapshot_name,
                            [pos[0], pos[1], 0.25],
                            4.2,
                            135.0,
                            -42.0,
                        )
                        snapshot_saved = True

                if base.DATA.time < avoid_stop_until:
                    forward = 0.0
                else:
                    # Bias the requested heading toward the side with more
                    # free space while approaching at a controlled speed.
                    alpha = base.wrap_angle(alpha + avoid_side * np.deg2rad(28.0))
                    speed_limit = 0.18 if lidar.front_min < 0.85 else 0.35
                    forward = min(forward, speed_limit)
            elif avoid_active and base.DATA.time - avoid_start > 0.6:
                print("  front sector clear; resuming path")
                avoid_active = False
                avoid_side = 0

            # A car cannot converge to a point behind its rear axle with
            # forward-only steering. Use a slow four-wheel differential pivot
            # for this recovery case instead of circling the waypoint forever.
            pivoting = abs(alpha) > np.deg2rad(80.0) and distance < 0.90
            if pivoting:
                yaw_rate = float(np.sign(alpha)) * 0.42
                left_target = (
                    -yaw_rate * base.WHEEL_TRACK / 2.0
                ) / base.WHEEL_RADIUS
                right_target = (
                    yaw_rate * base.WHEEL_TRACK / 2.0
                ) / base.WHEEL_RADIUS
                steer_target = 0.0
            else:
                steer_target = float(
                    np.arctan2(
                        2.0 * base.STEER_WHEELBASE * np.sin(alpha),
                        max(distance, 0.20),
                    )
                )
                steer_target = float(
                    np.clip(steer_target, -base.STEER_MAX, base.STEER_MAX)
                )
                yaw_rate = (
                    forward * float(np.tan(steer_target)) / base.STEER_WHEELBASE
                    if forward > 0.02 else 0.0
                )
                yaw_rate = float(np.clip(yaw_rate, -base.W_MAX, base.W_MAX))
                left_target = (
                    forward - yaw_rate * base.WHEEL_TRACK / 2.0
                ) / base.WHEEL_RADIUS
                right_target = (
                    forward + yaw_rate * base.WHEEL_TRACK / 2.0
                ) / base.WHEEL_RADIUS
            left_cmd += base.COMMAND_RAMP * (left_target - left_cmd)
            right_cmd += base.COMMAND_RAMP * (right_target - right_cmd)
            steer_cmd += base.COMMAND_RAMP * (steer_target - steer_cmd)
            base.set_wheel_velocity(left_cmd, right_cmd, steer_cmd)
            update_user_scene(sim.viewer, lidar)
            arm.hold()
            finger.hold()
            sim.step()

        if not reached:
            pos, yaw = base.chassis_pose()
            raise RuntimeError(
                f"waypoint {waypoint_index + 1} timeout at "
                f"({pos[0]:.3f}, {pos[1]:.3f}), yaw={yaw:.3f}"
            )

    base.set_wheel_velocity(0.0, 0.0, 0.0)
    settle_steps = max(1, int(round(base.CHASSIS_SETTLE_TIME / sim.dt)))
    for _ in range(settle_steps):
        arm.hold()
        finger.hold()
        update_user_scene(sim.viewer, lidar)
        sim.step()
    pos, yaw = base.chassis_pose()
    print(f"  path complete at ({pos[0]:.3f}, {pos[1]:.3f}), yaw={yaw:.3f} rad")

def run_sequence(
    sim: base.Simulation,
    arm: base.ArmMotion,
    finger: base.FingerMotion,
    lidar: Lidar,
    save_snapshots: bool = False,
) -> bool:
    drive_path_lidar(
        sim, arm, finger, lidar, OBJECT_PATH, save_snapshots,
        snapshot_name="lidar_avoid_object.png",
    )
    base.execute_grasp(sim, arm, finger, base.OBJ_POS)
    obj_after_grasp = base.DATA.qpos[
        base.OBJ_QADR : base.OBJ_QADR + 3
    ].copy()
    grasp_success = bool(
        float(obj_after_grasp[2]) > base.OBJ_POS[2] + 0.04
    )

    drive_path_lidar(
        sim, arm, finger, lidar, BUCKET_PATH, save_snapshots,
        snapshot_name="lidar_avoid_bucket.png",
    )
    base.execute_place(sim, arm, finger, base.BUCKET_DROP_WORLD)

    obj = base.DATA.qpos[base.OBJ_QADR : base.OBJ_QADR + 3].copy()
    xy_error = float(np.linalg.norm(obj[:2] - base.BUCKET_POS[:2]))
    inside = bool(
        xy_error < base.BUCKET_XY_TOL
        and 0.0 < float(obj[2]) < base.BUCKET_Z_MAX
    )
    pos, yaw = base.chassis_pose()
    print("\n" + "=" * 68)
    print(f"chassis final: ({pos[0]:.4f}, {pos[1]:.4f}), yaw={yaw:.4f} rad")
    print(f"cylinder final: {np.round(obj, 4)}")
    print(f"bucket xy error: {xy_error:.4f} m")
    print(f"cylinder inside bucket: {inside}")
    print(f"grasp success: {grasp_success}")
    print(f"place success: {inside}")
    print("=" * 68)

    if save_snapshots:
        capture_snapshot(
            [], "lidar_final.png", [5.40, 2.40, 0.30],
            4.40, 135.0, -24.0,
        )
        capture_snapshot(
            [], "lidar_bucket_top.png", [6.00, 4.00, 0.03],
            0.55, 90.0, -80.0,
        )
    return inside


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--snapshot", action="store_true")
    args = parser.parse_args()

    print(f"model: nq={base.MODEL.nq} nv={base.MODEL.nv} nu={base.MODEL.nu}")
    print(
        f"lidar: {LIDAR_CHANNELS} channels, 360 deg, "
        f"{LIDAR_AZ_SAMPLES} az samples, {1.0 / LIDAR_PERIOD:.0f} Hz"
    )

    arm, finger = base.setup_initial_state()
    lidar = Lidar()

    if args.headless or args.snapshot:
        lidar.scan()
        if args.snapshot:
            capture_snapshot(
                lidar.vis_points, "lidar_overview.png",
                [3.00, 2.00, 0.10], 9.00, 135.0, -50.0,
            )
        sim = base.Simulation(realtime=False)
        run_sequence(sim, arm, finger, lidar, save_snapshots=args.snapshot)
        return

    import mujoco.viewer

    with mujoco.viewer.launch_passive(base.MODEL, base.DATA) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.lookat[:] = [3.00, 2.00, 0.30]
        viewer.cam.distance = 8.00
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -24.0

        lidar.scan()
        update_user_scene(viewer, lidar)
        sim = base.Simulation(viewer, realtime=True)
        run_sequence(sim, arm, finger, lidar, save_snapshots=False)

        print("Motion finished. Close the viewer window to exit.")
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.02)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted.")