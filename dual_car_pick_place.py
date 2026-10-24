"""Dual-car nuclear waste recovery simulation.

Phase 1 covers the physical scene, shared chassis controller, independent
lidar avoidance, and the cooperative mission state machine. The four-layer
vision interface is scaffolded but intentionally left as a placeholder for
the next iteration.
"""

from __future__ import annotations

import argparse
import os
import random
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import mujoco
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

HERE = Path(os.path.abspath(__file__)).parent
SNAPSHOT_DIR = HERE / "docs" / "screenshots"
# MuJoCo on Windows does not reliably open Unicode paths. The existing project
# has an ASCII junction at D:/robot_mobile; prefer it when available.
ASCII_ROOT = Path("D:/robot_mobile")
SCENE_ROOT = ASCII_ROOT if (ASCII_ROOT / "dual_car_scene.xml").exists() else HERE
os.environ["MOBILE_SCENE_PATH"] = str(SCENE_ROOT / "dual_car_scene.xml")

import mobile_pick_place as base  # noqa: E402

base.V_MAX = 20.00
base.W_MAX = 20.00
base.K_POS = 3.50
base.COMMAND_RAMP = 0.35


# ---------------------------------------------------------------------------
# Mission geometry
# ---------------------------------------------------------------------------
TARGET_ZONE_X_MIN = -1.00
TARGET_ZONE_X_MAX = 1.00
TARGET_ZONE_Y_MIN = 20.50
TARGET_ZONE_Y_MAX = 21.50
OBJECT_X_RANGE = (-0.70, 0.70)
OBJECT_Y_RANGE = (20.65, 21.35)
OBJECT_Z = 0.01

CAR1_START = np.array([-0.70, -1.00])
CAR2_START = np.array([0.90, -1.20])
CAR_START_YAW = np.pi / 2.0
CAR1_SEARCH_STOP = np.array([0.0, 18.50])
CAR1_SEARCH_PATH = [
    (float(CAR1_SEARCH_STOP[0]), float(CAR1_SEARCH_STOP[1])),
]
RETURN_LINE_Y = -0.25
CAR1_HOME = np.array([-3.50, -3.00])
CAR2_HOME = np.array([4.00, -3.00])
CAR1_RETURN_LANE_PATH = [
    (-3.00, 20.20),
    (-3.50, 22.00),
]
CAR2_RETURN_LANE_PATH = [
    (4.10, 21.20),
    (4.00, 23.00),
]
CAR1_RETURN_Y = -3.00
CAR2_RETURN_Y = -3.00
RADIATION_EXIT_Y = 17.50
CAR2_DOCK_CENTER_Y = TARGET_ZONE_Y_MIN - 0.60
DOCK_OFFSET_MIN = 0.75
DOCK_OFFSET_MAX = 1.00
DOCK_OFFSET_STEP = 0.01
DOCK_COMFORT_WEIGHT = 10.0
CAR2_MIN_SEPARATION = 0.82

BUCKET_BASE_Z = 0.40356
BUCKET_HEIGHT = 0.20
BUCKET_OPENING_Z = BUCKET_BASE_Z + BUCKET_HEIGHT
DROP_WORLD_Z = BUCKET_OPENING_Z + 0.035
PRE_DROP_WORLD_Z = BUCKET_OPENING_Z + 0.10
BUCKET_XY_TOL = 0.05

# Lidar configuration, shared by both cars.
LIDAR_PERIOD = 0.10
LIDAR_AZ_SAMPLES = 120
LIDAR_CHANNELS = 16
LIDAR_ELEV_MIN = np.deg2rad(-15.0)
LIDAR_ELEV_MAX = np.deg2rad(15.0)
LIDAR_MAX_RANGE = 100.0
LIDAR_DETECT_RANGE = 2.8
LIDAR_FRONT_HALF = np.deg2rad(30.0)
LIDAR_SIDE_MAX = np.deg2rad(90.0)
LIDAR_REAR_HALF = np.deg2rad(30.0)
LIDAR_VIS_RANGE = 8.0
LIDAR_GEOMGROUP = np.array([1, 1, 0, 1, 0, 0], dtype=np.uint8)

SIM_TRUTH_TARGET = np.zeros(3, dtype=float)
VISION_LAST: dict[str, np.ndarray] = {}


def _id(obj_type, name: str) -> int:
    value = mujoco.mj_name2id(base.MODEL, obj_type, name)
    if value < 0:
        raise RuntimeError(f"missing {obj_type}: {name}")
    return int(value)


def body_pose(body_name: str) -> tuple[np.ndarray, float]:
    mujoco.mj_forward(base.MODEL, base.DATA)
    body_id = _id(mujoco.mjtObj.mjOBJ_BODY, body_name)
    pos = base.DATA.xpos[body_id].copy()
    rot = base.DATA.xmat[body_id].reshape(3, 3)
    yaw = float(np.arctan2(rot[1, 0], rot[0, 0]))
    return pos, yaw


def yaw_quaternion(yaw: float) -> list[float]:
    return [
        float(np.cos(yaw / 2.0)),
        0.0,
        0.0,
        float(np.sin(yaw / 2.0)),
    ]


def _set_scene_geom(scene, point: np.ndarray, color: np.ndarray) -> None:
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([0.018, 0.0, 0.0]),
        point,
        np.eye(3).reshape(-1),
        color,
    )
    scene.ngeom += 1


class LidarScanner:
    """VLP-16-like scanner implemented with MuJoCo mj_multiRay."""

    def __init__(
        self,
        site_name: str,
        own_body_name: str,
        other_body_name: str,
    ) -> None:
        self.site_id = _id(mujoco.mjtObj.mjOBJ_SITE, site_name)
        self.own_root = int(
            base.MODEL.body_rootid[_id(mujoco.mjtObj.mjOBJ_BODY, own_body_name)]
        )
        self.dynamic_roots = {
            int(base.MODEL.body_rootid[
                _id(mujoco.mjtObj.mjOBJ_BODY, other_body_name)
            ])
        }
        self.obstacle_geoms = {
            geom_id
            for geom_id in range(base.MODEL.ngeom)
            if (
                (name := mujoco.mj_id2name(
                    base.MODEL, mujoco.mjtObj.mjOBJ_GEOM, geom_id
                ))
                is not None
                and name.startswith("obstacle_")
            )
        }
        self.last_scan_time = -1e9
        self.front_blocked = False
        self.front_min = float("inf")
        self.front_lateral = float("inf")
        self.front_lateral_signed = 0.0
        self.left_min = float("inf")
        self.right_min = float("inf")
        self.rear_min = float("inf")
        self.vis_points: list[np.ndarray] = []
        self.front_points: list[np.ndarray] = []

    def scan_if_due(self) -> None:
        if base.DATA.time - self.last_scan_time >= LIDAR_PERIOD:
            self.scan()

    def scan(self) -> None:
        mujoco.mj_forward(base.MODEL, base.DATA)
        origin = base.DATA.site_xpos[self.site_id].copy()
        rot = base.DATA.site_xmat[self.site_id].reshape(3, 3).copy()

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
        geomid = np.zeros(nray, dtype=np.int32)
        distances = np.zeros(nray, dtype=np.float64)
        mujoco.mj_multiRay(
            base.MODEL,
            base.DATA,
            origin,
            world_dirs.reshape(-1),
            LIDAR_GEOMGROUP,
            True,
            -1,
            geomid,
            distances,
            None,
            nray,
            LIDAR_MAX_RANGE,
        )

        vis_points: list[np.ndarray] = []
        front_points: list[np.ndarray] = []
        front_min = float("inf")
        front_lateral = float("inf")
        front_lateral_signed = 0.0
        left_min = float("inf")
        right_min = float("inf")
        rear_min = float("inf")
        for idx in range(nray):
            distance = float(distances[idx])
            geom_id = int(geomid[idx])
            if distance < 0.0 or distance > LIDAR_MAX_RANGE:
                continue
            geom_root = int(base.MODEL.body_rootid[base.MODEL.geom_bodyid[geom_id]])
            if geom_root == self.own_root:
                continue
            if (
                geom_id not in self.obstacle_geoms
                and geom_root not in self.dynamic_roots
            ):
                continue
            hit = origin + distance * world_dirs[idx]
            if hit[2] < 0.08:
                continue
            az = float(az_flat[idx])
            is_front = (
                abs(az) <= LIDAR_FRONT_HALF
                and distance <= LIDAR_DETECT_RANGE
            )
            if is_front:
                front_points.append(hit.copy())
                if distance < front_min:
                    front_min = distance
                    front_lateral_signed = distance * np.sin(az)
                    front_lateral = abs(front_lateral_signed)
            if distance <= LIDAR_DETECT_RANGE:
                if abs(az) >= np.pi - LIDAR_REAR_HALF:
                    rear_min = min(rear_min, distance)
                if LIDAR_FRONT_HALF < az <= LIDAR_SIDE_MAX:
                    left_min = min(left_min, distance)
                elif -LIDAR_SIDE_MAX <= az < -LIDAR_FRONT_HALF:
                    right_min = min(right_min, distance)
            if distance <= LIDAR_VIS_RANGE:
                vis_points.append(
                    (origin + max(0.0, distance - 0.025) * world_dirs[idx]).copy()
                )

        self.vis_points = vis_points
        self.front_points = front_points
        self.front_min = front_min
        self.front_lateral = front_lateral
        self.front_lateral_signed = front_lateral_signed
        self.left_min = left_min
        self.right_min = right_min
        self.rear_min = rear_min
        self.front_blocked = front_min <= LIDAR_DETECT_RANGE
        self.last_scan_time = base.DATA.time


def update_user_scene(viewer, cars: list["CarController"]) -> None:
    if viewer is None:
        return
    scene = viewer.user_scn
    scene.ngeom = 0
    for car in cars:
        for point in car.lidar.vis_points:
            is_front = any(
                np.linalg.norm(point - front) < 0.04
                for front in car.lidar.front_points
            )
            color = (
                np.array([1.0, 0.15, 0.05, 1.0])
                if is_front
                else car.point_color
            )
            _set_scene_geom(scene, point, color)


def capture_snapshot(
    path: str,
    cars: list["CarController"],
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
        scene = renderer._scene
        for car in cars:
            for point in car.lidar.vis_points:
                point_front = any(
                    np.linalg.norm(point - front) < 0.04
                    for front in car.lidar.front_points
                )
                color = (
                    np.array([1.0, 0.15, 0.05, 1.0])
                    if point_front
                    else car.point_color
                )
                _set_scene_geom(scene, point, color)
        output_path = SNAPSHOT_DIR / Path(path).name
        output_path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(renderer.render()).save(output_path)
    print("  saved", output_path)


class CarController:
    """Shared four-wheel chassis and lidar behavior for either car."""

    def __init__(
        self,
        name: str,
        chassis_body: str,
        wheel_joint_names: list[str],
        steer_joint_names: list[str],
        lidar_site: str,
        point_color: np.ndarray,
        other_body: str,
        free_joint_name: str,
    ) -> None:
        self.name = name
        self.chassis_body = chassis_body
        self.body_id = _id(mujoco.mjtObj.mjOBJ_BODY, chassis_body)
        free_joint = _id(mujoco.mjtObj.mjOBJ_JOINT, free_joint_name)
        self.qadr = int(base.MODEL.jnt_qposadr[free_joint])
        self.dadr = int(base.MODEL.jnt_dofadr[free_joint])
        self.wheel_act = np.array(
            [
                _id(mujoco.mjtObj.mjOBJ_ACTUATOR, f"{joint.replace('_joint', '')}_act")
                for joint in wheel_joint_names
            ],
            dtype=int,
        )
        self.steer_act = np.array(
            [
                _id(mujoco.mjtObj.mjOBJ_ACTUATOR, f"{joint.replace('_joint', '')}_act")
                for joint in steer_joint_names
            ],
            dtype=int,
        )
        self.lidar = LidarScanner(
            lidar_site, chassis_body, other_body
        )
        self.point_color = np.asarray(point_color, dtype=float)
        self.other_body = other_body
        self.left_cmd = 0.0
        self.right_cmd = 0.0
        self.steer_cmd = 0.0
        self.min_other_separation = float("inf")

    def pose(self) -> tuple[np.ndarray, float]:
        return body_pose(self.chassis_body)

    def speed(self) -> float:
        return float(np.linalg.norm(base.DATA.qvel[self.dadr : self.dadr + 2]))

    def set_motion(self, left: float, right: float, steer: float) -> None:
        # all four wheels roll; front axle also steers
        base.DATA.ctrl[self.wheel_act[[0, 2]]] = left
        base.DATA.ctrl[self.wheel_act[[1, 3]]] = right
        base.DATA.ctrl[self.steer_act] = steer

    def stop_and_settle(
        self,
        sim: base.Simulation,
        arm: base.ArmMotion,
        finger: base.FingerMotion,
        cars: list["CarController"],
    ) -> None:
        self.left_cmd = 0.0
        self.right_cmd = 0.0
        self.steer_cmd = 0.0
        self.set_motion(0.0, 0.0, 0.0)
        steps = max(1, int(round(base.CHASSIS_SETTLE_TIME / sim.dt)))
        for _ in range(steps):
            arm.hold()
            finger.hold()
            update_user_scene(sim.viewer, cars)
            sim.step()

    def drive_path(
        self,
        sim: base.Simulation,
        path: list[tuple[float, float]],
        arm: base.ArmMotion,
        finger: base.FingerMotion,
        cars: list["CarController"],
        snapshot_name: str | None = None,
        waypoint_timeout: float = 22.0,
        final_tolerance: float = 0.16,
        max_speed: float = base.V_MAX,
    ) -> None:
        follower = PathFollower(
            self,
            path,
            cars,
            snapshot_name=snapshot_name,
            waypoint_timeout=waypoint_timeout,
            final_tolerance=final_tolerance,
            max_speed=max_speed,
        )
        while not follower.done:
            arm.hold()
            finger.hold()
            follower.update(cars)
            update_user_scene(sim.viewer, cars)
            sim.step()
        self.stop_and_settle(sim, arm, finger, cars)
        pos, yaw = self.pose()
        print(f"  [{self.name}] stopped at ({pos[0]:.3f}, {pos[1]:.3f})")


class PathFollower:
    """Non-blocking waypoint + lidar follower shared by both cars."""

    def __init__(
        self,
        car: CarController,
        path: list[tuple[float, float]],
        cars: list[CarController],
        snapshot_name: str | None,
        waypoint_timeout: float,
        final_tolerance: float,
        max_speed: float = base.V_MAX,
    ) -> None:
        if not path:
            raise ValueError("path must not be empty")
        self.car = car
        self.path = [(float(x), float(y)) for x, y in path]
        self.cars = cars
        self.snapshot_name = snapshot_name
        self.waypoint_timeout = waypoint_timeout
        self.final_tolerance = final_tolerance
        self.max_speed = float(max_speed)
        self.index = 0
        self.waypoint_start = base.DATA.time
        self.done = False
        self.snapshot_saved = False
        self.snapshot_pending = False
        self.caution_logged = False
        self.avoid_active = False
        self.avoid_side = 0
        self.avoid_start = 0.0
        self.avoid_stop_until = 0.0
        self.speed_cmd = 0.0
        self.frame_dt = float(
            base.MODEL.opt.timestep * base.STEPS_PER_FRAME
        )
        print(f"\n[{car.name}] lidar path: {len(path)} waypoint(s)")
        self._print_waypoint()

    def _print_waypoint(self) -> None:
        wx, wy = self.path[self.index]
        print(
            f"  waypoint {self.index + 1}/{len(self.path)}: "
            f"({wx:.2f}, {wy:.2f})"
        )

    def append_path(self, extra: list[tuple[float, float]]) -> bool:
        if self.done or not extra:
            return False
        self.path.extend((float(x), float(y)) for x, y in extra)
        print(
            f"  [{self.car.name}] path extended by {len(extra)} waypoint(s)"
        )
        return True

    def replace_final_waypoint(
        self, waypoint: tuple[float, float]
    ) -> bool:
        if self.done:
            return False
        self.path[-1] = (float(waypoint[0]), float(waypoint[1]))
        self.waypoint_start = base.DATA.time
        print(
            f"  [{self.car.name}] final waypoint updated to "
            f"({waypoint[0]:.3f}, {waypoint[1]:.3f}) without stopping"
        )
        return True

    def update(self, cars: list[CarController]) -> bool:
        if self.done:
            return True
        car = self.car
        car.lidar.scan_if_due()
        pos, yaw = car.pose()
        wx, wy = self.path[self.index]
        dx = float(wx - pos[0])
        dy = float(wy - pos[1])
        distance = float(np.hypot(dx, dy))
        tolerance = (
            self.final_tolerance
            if self.index == len(self.path) - 1
            else 0.30
        )
        if distance < tolerance:
            if self.index == len(self.path) - 1:
                self.done = True
                self.speed_cmd = 0.0
                car.set_motion(0.0, 0.0, 0.0)
                print(
                    f"  [{car.name}] path complete at "
                    f"({pos[0]:.3f}, {pos[1]:.3f})"
                )
                return True
            self.index += 1
            self.waypoint_start = base.DATA.time
            self._print_waypoint()
            wx, wy = self.path[self.index]
            dx = float(wx - pos[0])
            dy = float(wy - pos[1])
            distance = float(np.hypot(dx, dy))

        if base.DATA.time - self.waypoint_start > self.waypoint_timeout:
            raise RuntimeError(
                f"{car.name} waypoint {self.index + 1} timeout at "
                f"({pos[0]:.3f}, {pos[1]:.3f}), yaw={yaw:.3f}"
            )

        other_pos, _ = body_pose(car.other_body)
        separation = float(np.linalg.norm(pos[:2] - other_pos[:2]))
        car.min_other_separation = min(car.min_other_separation, separation)
        if separation < CAR2_MIN_SEPARATION:
            alpha = base.wrap_angle(
                float(np.arctan2(pos[1] - other_pos[1], pos[0] - other_pos[0]))
                - yaw
            )
            forward = -0.18
        else:
            alpha = base.wrap_angle(float(np.arctan2(dy, dx)) - yaw)
            forward = float(np.clip(base.K_POS * distance, 0.0, self.max_speed))
            forward *= max(0.0, float(np.cos(alpha)))
            if abs(alpha) < np.deg2rad(95.0):
                forward = max(forward, 0.15)

        if car.lidar.front_blocked:
            # Local avoidance only: target heading remains the macro goal.
            if car.lidar.front_lateral <= 0.85:
                if not self.avoid_active:
                    self.avoid_active = True
                    if abs(car.lidar.front_lateral_signed) < 0.05:
                        self.avoid_side = (
                            -1.0 if car.lidar.right_min >= car.lidar.left_min else 1.0
                        )
                    else:
                        self.avoid_side = (
                            -1.0 if car.lidar.front_lateral_signed > 0.0 else 1.0
                        )
                    self.avoid_start = base.DATA.time
                    side_name = "RIGHT" if self.avoid_side < 0 else "LEFT"
                    print(
                        f"  [{car.name}] 2.8 m radar warning: obstacle "
                        f"{car.lidar.front_min:.2f} m, local detour {side_name}"
                    )
                bias = np.deg2rad(55.0 if car.lidar.front_min < 1.0 else 35.0)
                alpha = base.wrap_angle(alpha + self.avoid_side * bias)
                forward = min(forward, 0.85 if car.lidar.front_min < 1.0 else 1.40)
            else:
                if not self.caution_logged:
                    print(
                        f"  [{car.name}] adjacent obstacle "
                        f"{car.lidar.front_min:.2f} m: radar caution, slow"
                    )
                    self.caution_logged = True
                forward = min(forward, 1.40)
        elif self.avoid_active:
            print(f"  [{car.name}] obstacle cleared: resume direct goal")
            self.avoid_active = False
            self.avoid_side = 0

        # Smooth longitudinal profile: full speed on long straights, speed
        # reduction in turns, and square-root braking near the goal.
        if forward >= 0.0:
            turn_factor = 1.0 - 0.55 * min(
                abs(alpha) / (np.pi / 2.0), 1.0
            )
            goal_speed = forward * turn_factor
            braking_speed = np.sqrt(
                max(0.0, 2.0 * 1.10 * max(distance - 0.05, 0.0))
            )
            goal_speed = min(goal_speed, braking_speed)
        else:
            goal_speed = forward
        accel = 4.5 if goal_speed > self.speed_cmd else 6.0
        max_delta = accel * self.frame_dt
        self.speed_cmd += float(
            np.clip(goal_speed - self.speed_cmd, -max_delta, max_delta)
        )
        forward = self.speed_cmd

        pivoting = abs(alpha) > np.deg2rad(80.0) and distance < 0.90
        terminal_straight = (
            self.index == len(self.path) - 1
            and distance < 0.50
            and abs(alpha) < np.deg2rad(35.0)
        )
        if terminal_straight:
            steer_target = 0.0
            yaw_rate = 0.0
            left_target = forward / base.WHEEL_RADIUS
            right_target = forward / base.WHEEL_RADIUS
        elif pivoting:
            yaw_rate = float(np.sign(alpha)) * 0.42
            left_target = (
                -yaw_rate * base.WHEEL_TRACK / 2.0
            ) / base.WHEEL_RADIUS
            right_target = (
                yaw_rate * base.WHEEL_TRACK / 2.0
            ) / base.WHEEL_RADIUS
            steer_target = 0.0
        else:
            lookahead = float(np.clip(distance, 0.65, 1.80))
            steer_target = float(
                np.arctan2(
                    2.0 * base.STEER_WHEELBASE * np.sin(alpha),
                    lookahead,
                )
            )
            steer_target = float(
                np.clip(steer_target, -base.STEER_MAX, base.STEER_MAX)
            )
            yaw_rate = (
                forward * float(np.tan(steer_target)) / base.STEER_WHEELBASE
                if forward > 0.02
                else 0.0
            )
            yaw_rate = float(np.clip(yaw_rate, -base.W_MAX, base.W_MAX))
            left_target = (
                forward - yaw_rate * base.WHEEL_TRACK / 2.0
            ) / base.WHEEL_RADIUS
            right_target = (
                forward + yaw_rate * base.WHEEL_TRACK / 2.0
            ) / base.WHEEL_RADIUS

        car.left_cmd += base.COMMAND_RAMP * (left_target - car.left_cmd)
        car.right_cmd += base.COMMAND_RAMP * (right_target - car.right_cmd)
        car.steer_cmd += base.COMMAND_RAMP * (steer_target - car.steer_cmd)
        car.set_motion(car.left_cmd, car.right_cmd, car.steer_cmd)
        return False


class SkidTurnFollower:
    """Short steering-assisted yaw correction for the heavy four-wheel car."""

    def __init__(
        self,
        car: CarController,
        target_heading: float,
        timeout: float = 8.0,
    ) -> None:
        self.car = car
        self.target_heading = float(target_heading)
        self.start_time = base.DATA.time
        self.timeout = float(timeout)
        self.done = False
        print(
            f"  [{car.name}] steering-assist turn to heading "
            f"{self.target_heading:.3f} rad"
        )

    def update(self, cars: list[CarController]) -> bool:
        if self.done:
            return True
        car = self.car
        pos, yaw = car.pose()
        error = base.wrap_angle(self.target_heading - yaw)
        if abs(error) <= np.deg2rad(2.0):
            self.done = True
            car.set_motion(0.0, 0.0, 0.0)
            print(
                f"  [{car.name}] heading aligned at yaw={yaw:.3f} rad, "
                f"pose=({pos[0]:.3f}, {pos[1]:.3f})"
            )
            return True
        if base.DATA.time - self.start_time > self.timeout:
            raise RuntimeError(f"{car.name} final heading alignment timeout")

        direction = 1.0 if error < 0.0 else -1.0
        car.set_motion(
            2.5 * direction,
            -2.5 * direction,
            -0.65 * direction,
        )
        return False


def run_turn_with_secondary(
    sim: base.Simulation,
    arm: base.ArmMotion,
    finger: base.FingerMotion,
    cars: list[CarController],
    turner: SkidTurnFollower,
    secondary: PathFollower,
) -> None:
    while not turner.done:
        arm.hold()
        finger.hold()
        turner.update(cars)
        if not secondary.done:
            secondary.update(cars)
        update_user_scene(sim.viewer, cars)
        sim.step()


class ReverseStraightFollower:
    """Straight negative-speed return after heading alignment."""

    def __init__(
        self,
        car: CarController,
        target_y: float,
        max_speed: float = 0.55,
    ) -> None:
        self.car = car
        self.target_y = float(target_y)
        self.max_speed = float(max_speed)
        self.done = False
        print(
            f"  [{car.name}] straight reverse to Y={self.target_y:.2f}, "
            f"heading={car.pose()[1]:.3f} rad"
        )

    def update(self, cars: list[CarController]) -> bool:
        if self.done:
            return True
        car = self.car
        if base.DATA.time - car.lidar.last_scan_time >= 0.50:
            car.lidar.scan()
        pos, yaw = car.pose()
        if pos[1] <= self.target_y + 0.08:
            self.done = True
            car.set_motion(0.0, 0.0, 0.0)
            print(
                f"  [{car.name}] reverse complete at "
                f"({pos[0]:.3f}, {pos[1]:.3f})"
            )
            return True
        speed = -self.max_speed
        if car.lidar.rear_min < 1.20:
            speed = max(speed, -0.28)
        if car.lidar.rear_min < 0.50:
            speed = 0.0
        wheel = speed / base.WHEEL_RADIUS
        car.left_cmd += base.COMMAND_RAMP * (wheel - car.left_cmd)
        car.right_cmd += base.COMMAND_RAMP * (wheel - car.right_cmd)
        car.steer_cmd += base.COMMAND_RAMP * (0.0 - car.steer_cmd)
        car.set_motion(car.left_cmd, car.right_cmd, car.steer_cmd)
        return False


def run_two_turns(
    sim: base.Simulation,
    arm: base.ArmMotion,
    finger: base.FingerMotion,
    cars: list[CarController],
    turners: list[SkidTurnFollower],
) -> None:
    while any(not turner.done for turner in turners):
        arm.hold()
        finger.hold()
        for turner in turners:
            if not turner.done:
                turner.update(cars)
        update_user_scene(sim.viewer, cars)
        sim.step()


def run_two_straight_reverses(
    sim: base.Simulation,
    arm: base.ArmMotion,
    finger: base.FingerMotion,
    cars: list[CarController],
    followers: list[ReverseStraightFollower],
    snapshot_name: str | None = None,
) -> None:
    snapshot_saved = False
    while any(not follower.done for follower in followers):
        arm.hold()
        finger.hold()
        for follower in followers:
            if not follower.done:
                follower.update(cars)
        update_user_scene(sim.viewer, cars)
        sim.step()


class ReverseXAxisFollower:
    """Reverse west along -X while holding an east-facing heading."""

    def __init__(
        self,
        car: CarController,
        target_x: float,
        target_heading: float = 0.0,
        max_speed: float = 0.50,
    ) -> None:
        self.car = car
        self.target_x = float(target_x)
        self.target_heading = float(target_heading)
        self.max_speed = float(max_speed)
        self.start_time = base.DATA.time
        self.done = False
        print(f"  [{car.name}] reverse along -X to X={self.target_x:.2f}")

    def update(self, cars: list[CarController]) -> bool:
        if self.done:
            return True
        car = self.car
        car.lidar.scan_if_due()
        pos, yaw = car.pose()
        if pos[0] <= self.target_x + 0.06:
            self.done = True
            car.set_motion(0.0, 0.0, 0.0)
            print(
                f"  [{car.name}] reverse alignment complete at "
                f"({pos[0]:.3f}, {pos[1]:.3f}), yaw={yaw:.3f}"
            )
            return True
        if base.DATA.time - self.start_time > 20.0:
            raise RuntimeError(f"{car.name} reverse X alignment timeout")

        heading_ref = self.target_heading
        if self.target_x_lane is not None:
            heading_ref += float(
                np.clip(0.50 * (self.target_x_lane - pos[0]), -0.25, 0.25)
            )
        yaw_error = base.wrap_angle(heading_ref - yaw)
        speed = -self.max_speed
        if car.lidar.rear_min < 1.20:
            speed = max(speed, -0.26)
        if car.lidar.rear_min < 0.50:
            speed = 0.0
        yaw_rate = float(np.clip(1.10 * yaw_error, -0.42, 0.42))
        steer_target = (
            float(np.arctan2(yaw_rate * base.STEER_WHEELBASE, speed))
            if abs(speed) > 1e-4 else 0.0
        )
        steer_target = float(
            np.clip(steer_target, -base.STEER_MAX, base.STEER_MAX)
        )
        left_target = (
            speed - yaw_rate * base.WHEEL_TRACK / 2.0
        ) / base.WHEEL_RADIUS
        right_target = (
            speed + yaw_rate * base.WHEEL_TRACK / 2.0
        ) / base.WHEEL_RADIUS
        car.left_cmd += base.COMMAND_RAMP * (left_target - car.left_cmd)
        car.right_cmd += base.COMMAND_RAMP * (right_target - car.right_cmd)
        car.steer_cmd += base.COMMAND_RAMP * (steer_target - car.steer_cmd)
        car.set_motion(car.left_cmd, car.right_cmd, car.steer_cmd)
        return False


def run_reverse_x_with_secondary(
    sim: base.Simulation,
    arm: base.ArmMotion,
    finger: base.FingerMotion,
    cars: list[CarController],
    reverse: ReverseXAxisFollower,
    secondary: PathFollower,
) -> None:
    while not reverse.done:
        arm.hold()
        finger.hold()
        reverse.update(cars)
        if not secondary.done:
            secondary.update(cars)
        update_user_scene(sim.viewer, cars)
        sim.step()


class ReverseFollower:
    """Negative-speed return controller with rear-lidar speed limiting."""

    def __init__(
        self,
        car: CarController,
        target_y: float,
        target_heading: float,
        max_speed: float = 0.55,
        target_x_lane: float | None = None,
    ) -> None:
        self.car = car
        self.target_y = float(target_y)
        self.target_heading = float(target_heading)
        self.max_speed = float(max_speed)
        self.target_x_lane = target_x_lane
        self.done = False
        print(
            f"  [{car.name}] reverse return to Y={self.target_y:.2f}, "
            f"heading={self.target_heading:.3f} rad"
        )

    def update(self, cars: list[CarController]) -> bool:
        if self.done:
            return True
        car = self.car
        car.lidar.scan_if_due()
        pos, yaw = car.pose()
        if pos[1] <= self.target_y + 0.08:
            self.done = True
            car.set_motion(0.0, 0.0, 0.0)
            print(
                f"  [{car.name}] reverse return complete at "
                f"({pos[0]:.3f}, {pos[1]:.3f})"
            )
            return True

        yaw_error = base.wrap_angle(self.target_heading - yaw)
        speed = -self.max_speed
        if car.lidar.rear_min < 1.20:
            speed = max(speed, -0.28)
        if car.lidar.rear_min < 0.55:
            speed = 0.0

        yaw_rate = float(np.clip(1.10 * yaw_error, -0.48, 0.48))
        if abs(speed) > 1e-4:
            steer_target = float(
                np.arctan2(
                    yaw_rate * base.STEER_WHEELBASE,
                    speed,
                )
            )
        else:
            steer_target = 0.0
        steer_target = float(
            np.clip(steer_target, -base.STEER_MAX, base.STEER_MAX)
        )
        left_target = (
            speed - yaw_rate * base.WHEEL_TRACK / 2.0
        ) / base.WHEEL_RADIUS
        right_target = (
            speed + yaw_rate * base.WHEEL_TRACK / 2.0
        ) / base.WHEEL_RADIUS
        car.left_cmd += base.COMMAND_RAMP * (left_target - car.left_cmd)
        car.right_cmd += base.COMMAND_RAMP * (right_target - car.right_cmd)
        car.steer_cmd += base.COMMAND_RAMP * (steer_target - car.steer_cmd)
        car.set_motion(car.left_cmd, car.right_cmd, car.steer_cmd)
        return False


def run_two_followers(
    sim: base.Simulation,
    arm: base.ArmMotion,
    finger: base.FingerMotion,
    cars: list[CarController],
    followers: list[PathFollower],
) -> None:
    while any(not follower.done for follower in followers):
        arm.hold()
        finger.hold()
        for follower in followers:
            if not follower.done:
                follower.update(cars)
        update_user_scene(sim.viewer, cars)
        sim.step()


def run_followers_until_primary_done(
    sim: base.Simulation,
    arm: base.ArmMotion,
    finger: base.FingerMotion,
    cars: list[CarController],
    primary: PathFollower,
    secondary: PathFollower,
) -> None:
    while not primary.done:
        arm.hold()
        finger.hold()
        primary.update(cars)
        if not secondary.done:
            secondary.update(cars)
        update_user_scene(sim.viewer, cars)
        sim.step()


def run_two_reverse_returns(
    sim: base.Simulation,
    arm: base.ArmMotion,
    finger: base.FingerMotion,
    cars: list[CarController],
    followers: list[ReverseFollower],
    snapshot_name: str | None = None,
) -> None:
    snapshot_saved = False
    while any(not follower.done for follower in followers):
        arm.hold()
        finger.hold()
        for follower in followers:
            if not follower.done:
                follower.update(cars)
        update_user_scene(sim.viewer, cars)
        sim.step()


# ---------------------------------------------------------------------------
# Four-layer camera interface scaffold. Phase 2 will fill these in.
# ---------------------------------------------------------------------------
def get_camera_data() -> tuple[np.ndarray, np.ndarray]:
    """Layer 1: data acquisition from the wrist-mounted D435i.

    Simulation: the camera pose comes from the XML camera named ``d435i``
    attached below wrist_3_link. ROS2 replacement: subscribe to
    /camera/color/image_raw and /camera/depth/image_rect_raw, convert both
    messages to NumPy arrays, and preserve the same RGB/depth convention.
    """
    cam_id = _id(mujoco.mjtObj.mjOBJ_CAMERA, "d435i")
    with mujoco.Renderer(base.MODEL, height=480, width=640) as renderer:
        renderer.update_scene(base.DATA, camera="d435i")
        rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        depth = renderer.render().copy()
        renderer.disable_depth_rendering()
    VISION_LAST["rgb"] = rgb
    VISION_LAST["depth"] = depth
    VISION_LAST["cam_pos"] = base.DATA.cam_xpos[cam_id].copy()
    VISION_LAST["cam_mat"] = base.DATA.cam_xmat[cam_id].reshape(3, 3).copy()
    print(
        f"[相机安装] local_pos={np.round(base.MODEL.cam_pos[cam_id], 4)}, "
        f"local_quat={np.round(base.MODEL.cam_quat[cam_id], 4)}"
    )
    return rgb, depth


def detect_yellow_target(
    rgb: np.ndarray,
) -> tuple[int, int, np.ndarray, np.ndarray] | None:
    """Layer 2: HSV segmentation, morphology, largest-contour centroid."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(
        hsv,
        np.array([15, 100, 100], dtype=np.uint8),
        np.array([40, 255, 255], dtype=np.uint8),
    )
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    moments = cv2.moments(contour)
    if moments["m00"] <= 0.0:
        return None
    u = int(round(moments["m10"] / moments["m00"]))
    v = int(round(moments["m01"] / moments["m00"]))
    VISION_LAST["mask"] = mask
    return u, v, contour, mask


def pixel_to_world(u: int, v: int, depth: float) -> np.ndarray:
    """Layer 3: pixel + depth -> camera optical frame -> MuJoCo world.

    Simulation uses cam_xpos/cam_xmat as the extrinsic transform. In ROS2,
    replace this part with a tf2_ros lookup from the camera optical frame to
    the world frame; the intrinsic projection remains identical.
    """
    rgb = VISION_LAST["rgb"]
    image_h, image_w = rgb.shape[:2]
    cam_id = _id(mujoco.mjtObj.mjOBJ_CAMERA, "d435i")
    fovy = np.deg2rad(float(base.MODEL.cam_fovy[cam_id]))
    fy = (image_h / 2.0) / np.tan(fovy / 2.0)
    fx = fy
    cx = image_w / 2.0
    cy = image_h / 2.0

    x_cam = (u - cx) * depth / fx
    y_cam = -(v - cy) * depth / fy
    # MuJoCo camera +Z points opposite the viewing direction. Keep the
    # physical depth positive and negate it when forming the local vector.
    point_cam = np.array([x_cam, y_cam, -depth], dtype=float)
    cam_pos = VISION_LAST.get(
        "cam_pos", base.DATA.cam_xpos[cam_id]
    ).copy()
    cam_mat = VISION_LAST.get(
        "cam_mat", base.DATA.cam_xmat[cam_id].reshape(3, 3)
    ).copy()
    return cam_pos + cam_mat @ point_cam


def _save_vision_debug(
    rgb: np.ndarray,
    u: int,
    v: int,
    contour: np.ndarray,
) -> None:
    vis = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    cv2.drawContours(vis, [contour], -1, (0, 255, 0), 2)
    cv2.circle(vis, (u, v), 5, (255, 0, 0), -1)
    cv2.putText(
        vis,
        f"target pixel=({u},{v})",
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    SNAPSHOT_DIR.mkdir(parents=True, exist_ok=True)
    Image.fromarray(cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)).save(
        SNAPSHOT_DIR / "dual_vision_rgb.png"
    )
    if "mask" in VISION_LAST:
        Image.fromarray(VISION_LAST["mask"]).save(
            SNAPSHOT_DIR / "dual_vision_mask.png"
        )
    depth = VISION_LAST["depth"]
    depth_vis = np.clip(depth / 2.0, 0.0, 1.0)
    Image.fromarray(
        np.round(depth_vis * 255.0).astype(np.uint8)
    ).save(SNAPSHOT_DIR / "dual_vision_depth.png")


def get_object_truth_for_validation() -> np.ndarray:
    """Read object truth only after vision output, for error validation."""
    return base.DATA.qpos[
        base.OBJ_QADR : base.OBJ_QADR + 3
    ].copy()


def process_camera_frame(
    rgb: np.ndarray,
    depth: np.ndarray,
) -> np.ndarray:
    """Process a frozen camera frame without reading object truth."""
    print("[VISION] processing frozen RGB/depth frame in worker thread")
    detection = detect_yellow_target(rgb)
    if detection is None:
        raise RuntimeError("yellow radioactive target not found in D435i image")
    u, v, contour, _ = detection

    v0, v1 = max(0, v - 2), min(depth.shape[0], v + 3)
    u0, u1 = max(0, u - 2), min(depth.shape[1], u + 3)
    patch = depth[v0:v1, u0:u1]
    patch_mask = VISION_LAST["mask"][v0:v1, u0:u1] > 0
    valid_mask = np.isfinite(patch) & (patch > 0.0) & (patch < 10.0)
    masked_depth = patch[valid_mask & patch_mask]
    valid = masked_depth if masked_depth.size >= 3 else patch[valid_mask]
    if valid.size == 0:
        raise RuntimeError("no valid depth samples around target centroid")
    target_depth = float(np.median(valid))
    vision_world = pixel_to_world(u, v, target_depth)
    _save_vision_debug(rgb, u, v, contour)

    truth = get_object_truth_for_validation()
    error = float(np.linalg.norm(vision_world - truth))
    print(
        f"[视觉识别] 像素({u},{v}) -> 深度Z={target_depth:.5f} m "
        f"-> 视觉世界坐标(X_vis={vision_world[0]:.4f}, "
        f"Y_vis={vision_world[1]:.4f}, Z_vis={vision_world[2]:.4f})"
    )
    print(
        f"           仿真真实坐标(X_true={truth[0]:.4f}, "
        f"Y_true={truth[1]:.4f}, Z_true={truth[2]:.4f}), "
        f"误差={error * 100.0:.2f} cm"
    )
    if error > 0.03:
        raise RuntimeError(f"vision error too large: {error * 1000.0:.2f} mm")
    return vision_world


def get_vision_target_position() -> np.ndarray:
    """Synchronous wrapper used by tests and diagnostics."""
    rgb, depth = get_camera_data()
    return process_camera_frame(rgb, depth)


# ---------------------------------------------------------------------------
# Arm-side cooperative placement
# ---------------------------------------------------------------------------
def execute_cooperative_place(
    sim: base.Simulation,
    arm: base.ArmMotion,
    finger: base.FingerMotion,
    bucket_xy: np.ndarray,
) -> None:
    target_xy = np.asarray(bucket_xy, dtype=float)
    pre_drop = np.array([target_xy[0], target_xy[1], PRE_DROP_WORLD_Z])
    drop = np.array([target_xy[0], target_xy[1], DROP_WORLD_Z])

    spin_deg = choose_grasp_spin(
        pre_drop, candidates=PLACE_SPIN_CANDIDATES_DEG
    )
    start_arm_world_flexible(arm, pre_drop, 2.4, spin_deg)
    base.run_arm_motion(sim, arm, finger)
    start_arm_world_flexible(arm, drop, 1.8, spin_deg)
    base.run_arm_motion(sim, arm, finger)
    base.wait_with_pose(sim, arm, finger, 0.7)

    site_id = _id(mujoco.mjtObj.mjOBJ_SITE, "pinch")
    print(f"  bucket release site: {np.round(base.DATA.site_xpos[site_id], 4)}")
    finger.start(base.FINGER_OPEN, 1.0)
    base.run_finger_motion(sim, arm, finger)
    base.wait_with_pose(sim, arm, finger, 0.9)
    arm.start_fixed(base.CARRY_FIXED, 2.0)
    base.run_arm_motion(sim, arm, finger)
    base.wait_with_pose(sim, arm, finger, 1.2)


def _update_follower(
    follower: PathFollower | None,
    cars: list[CarController],
) -> None:
    if follower is not None and not follower.done:
        follower.update(cars)


def run_arm_motion_concurrent(
    sim: base.Simulation,
    arm: base.ArmMotion,
    finger: base.FingerMotion,
    cars: list[CarController],
    follower: PathFollower | None = None,
) -> None:
    while arm.active:
        arm.update(sim.dt)
        finger.hold()
        _update_follower(follower, cars)
        update_user_scene(sim.viewer, cars)
        sim.step()
    arm.hold()


def run_finger_motion_concurrent(
    sim: base.Simulation,
    arm: base.ArmMotion,
    finger: base.FingerMotion,
    cars: list[CarController],
    follower: PathFollower | None = None,
) -> None:
    while finger.active:
        arm.hold()
        finger.update(sim.dt)
        _update_follower(follower, cars)
        update_user_scene(sim.viewer, cars)
        sim.step()
    finger.hold()


def run_wait_concurrent(
    sim: base.Simulation,
    arm: base.ArmMotion,
    finger: base.FingerMotion,
    cars: list[CarController],
    duration: float,
    follower: PathFollower | None = None,
) -> None:
    steps = max(1, int(round(duration / sim.dt)))
    for _ in range(steps):
        arm.hold()
        finger.hold()
        _update_follower(follower, cars)
        update_user_scene(sim.viewer, cars)
        sim.step()


def finish_follower(
    sim: base.Simulation,
    arm: base.ArmMotion,
    finger: base.FingerMotion,
    cars: list[CarController],
    follower: PathFollower,
) -> None:
    while not follower.done:
        arm.hold()
        finger.hold()
        follower.update(cars)
        update_user_scene(sim.viewer, cars)
        sim.step()



GRASP_SPIN_AXIS = "z"
GRASP_SPIN_CANDIDATES_DEG = (90.0, -90.0)
PLACE_SPIN_CANDIDATES_DEG = (
    0.0, 30.0, 45.0, 60.0, 90.0, 120.0, 135.0, 150.0, 180.0,
    -30.0, -45.0, -60.0, -90.0, -120.0, -135.0, -150.0,
)


def choose_grasp_spin(
    target_world: np.ndarray,
    candidates: tuple[float, ...] = GRASP_SPIN_CANDIDATES_DEG,
) -> float:
    target_fixed, _ = base.world_target_to_fixed(target_world)
    q_base = Rotation.from_quat(base.TARGET_QUAT, scalar_first=True)
    best_angle = 0.0
    best_error = float("inf")
    best_comfort = float("inf")
    best_score = float("inf")
    for angle_deg in candidates:
        q_candidate = (
            q_base * Rotation.from_euler(GRASP_SPIN_AXIS, np.deg2rad(angle_deg))
        ).as_quat(scalar_first=True)
        q_solution, error = base.solve_ik_fixed(target_fixed, q_candidate)
        comfort = float(np.sqrt(np.mean((q_solution / np.pi) ** 2)))
        score = error * 1000.0 + 10.0 * comfort
        if score < best_score:
            best_score = score
            best_error = error
            best_comfort = comfort
            best_angle = angle_deg
    print(
        f"  tool-spin selected: {best_angle:+.1f} deg, "
        f"IK residual={best_error * 1000:.2f} mm, "
        f"comfort={best_comfort:.3f}"
    )
    if best_error > 0.005:
        raise RuntimeError(
            f"no reachable equivalent grasp orientation, best error={best_error * 1000:.2f} mm"
        )
    return float(best_angle)


def find_optimal_car2_dock(
    car1_pos: np.ndarray,
    car1_yaw: float,
) -> tuple[np.ndarray, float, float]:
    """Minimize arm-base-to-bucket distance under frame and IK constraints."""
    car2_y = CAR2_DOCK_CENTER_Y
    dy = car2_y - car1_pos[1]
    min_offset = max(
        DOCK_OFFSET_MIN,
        float(np.sqrt(max(CAR2_MIN_SEPARATION**2 - dy**2, 0.0))) + 0.02,
    )
    q_base = Rotation.from_quat(base.TARGET_QUAT, scalar_first=True)
    best = None
    for offset in np.arange(
        min_offset, DOCK_OFFSET_MAX + 1e-6, DOCK_OFFSET_STEP
    ):
        candidate = np.array([car1_pos[0] + float(offset), car2_y])
        bucket_distance = float(np.linalg.norm(candidate - car1_pos[:2]))
        best_tool = None
        for angle_deg in PLACE_SPIN_CANDIDATES_DEG:
            quat_fixed = (
                q_base * Rotation.from_euler("z", np.deg2rad(angle_deg))
            ).as_quat(scalar_first=True)
            errors, comforts = [], []
            for z in (PRE_DROP_WORLD_Z, DROP_WORLD_Z):
                target_world = np.array([candidate[0], candidate[1], z])
                target_fixed, _ = base.world_target_to_fixed(target_world)
                q, error = base.solve_ik_fixed(target_fixed, quat_fixed)
                errors.append(float(error))
                comforts.append(float(np.sqrt(np.mean((q / np.pi) ** 2))))
            max_error = max(errors)
            comfort = float(np.mean(comforts))
            tool_score = max_error * 1000.0 + 10.0 * comfort
            record = (tool_score, max_error, comfort, angle_deg)
            if best_tool is None or record[0] < best_tool[0]:
                best_tool = record
        _, max_error, comfort, angle_deg = best_tool
        score = bucket_distance * 10.0 + max_error * 1000.0 + 0.10 * comfort
        record = (score, candidate, max_error, comfort, float(offset), bucket_distance, angle_deg)
        if best is None or record[0] < best[0]:
            best = record
        if max_error < 0.0010 and comfort < 0.65:
            best = record
            break
    if best is None:
        raise RuntimeError("no legal CAR2 docking point found")
    _, candidate, error, comfort, offset, distance, angle_deg = best
    print(
        f"  optimal CAR2 dock: ({candidate[0]:.3f}, {candidate[1]:.3f}), "
        f"bucket-to-arm={distance:.3f} m, offset={offset:.2f} m, "
        f"IK={error * 1000:.2f} mm, comfort={comfort:.3f}, "
        f"place-spin={angle_deg:+.1f} deg"
    )
    return candidate, error, comfort

def find_optimal_dock_for_predicted_pose(
    car: CarController,
    predicted_pos: np.ndarray,
    predicted_yaw: float,
) -> tuple[np.ndarray, float, float]:
    """Evaluate dock IK using CAR1's future predicted chassis transform."""
    saved_qpos = base.DATA.qpos[car.qadr : car.qadr + 7].copy()
    saved_qvel = base.DATA.qvel[car.dadr : car.dadr + 6].copy()
    base.DATA.qpos[car.qadr : car.qadr + 3] = [
        predicted_pos[0], predicted_pos[1], 0.0
    ]
    base.DATA.qpos[car.qadr + 3 : car.qadr + 7] = yaw_quaternion(
        predicted_yaw
    )
    base.DATA.qvel[car.dadr : car.dadr + 6] = 0.0
    mujoco.mj_forward(base.MODEL, base.DATA)
    try:
        return find_optimal_car2_dock(predicted_pos, predicted_yaw)
    finally:
        base.DATA.qpos[car.qadr : car.qadr + 7] = saved_qpos
        base.DATA.qvel[car.dadr : car.dadr + 6] = saved_qvel
        mujoco.mj_forward(base.MODEL, base.DATA)


def start_arm_world_flexible(
    arm: base.ArmMotion,
    target_world: np.ndarray,
    duration: float,
    spin_deg: float,
) -> None:
    target_fixed, _ = base.world_target_to_fixed(target_world)
    q_base = Rotation.from_quat(base.TARGET_QUAT, scalar_first=True)
    quat = (
        q_base * Rotation.from_euler(GRASP_SPIN_AXIS, np.deg2rad(spin_deg))
    ).as_quat(scalar_first=True)
    print(f"  arm target world={np.round(target_world, 3)}")
    arm.start_fixed(target_fixed, duration, quat)


def execute_grasp_concurrent(
    sim: base.Simulation,
    arm: base.ArmMotion,
    finger: base.FingerMotion,
    cars: list[CarController],
    target_pos: np.ndarray,
    follower: PathFollower | None,
) -> bool:
    print("\n[GRASP] CAR1 arm starts while CAR2 continues driving")
    approach = np.array([target_pos[0], target_pos[1], base.APPROACH_WORLD_Z])
    grasp = np.array([target_pos[0], target_pos[1], base.GRASP_WORLD_Z])
    lift = np.array([target_pos[0], target_pos[1], base.LIFT_WORLD_Z])

    spin_deg = choose_grasp_spin(approach)
    start_arm_world_flexible(arm, approach, base.APPROACH_TIME, spin_deg)
    run_arm_motion_concurrent(sim, arm, finger, cars, follower)
    start_arm_world_flexible(arm, grasp, base.DESCEND_TIME, spin_deg)
    run_arm_motion_concurrent(sim, arm, finger, cars, follower)

    finger.start(base.FINGER_CLOSE, base.CLOSE_TIME)
    run_finger_motion_concurrent(sim, arm, finger, cars, follower)
    force = float(abs(base.DATA.actuator_force[base.FINGER_ACT]))
    print(f"  finger force after close: {force:.2f} N")

    start_arm_world_flexible(arm, lift, base.LIFT_TIME, spin_deg)
    run_arm_motion_concurrent(sim, arm, finger, cars, follower)
    arm.start_fixed(base.CARRY_FIXED, base.CARRY_TIME)
    run_arm_motion_concurrent(sim, arm, finger, cars, follower)

    obj = base.DATA.qpos[base.OBJ_QADR : base.OBJ_QADR + 3].copy()
    print(f"  cylinder after grasp: {np.round(obj, 4)}")
    return bool(obj[2] > target_pos[2] + 0.04)


class DualCarMission:
    def __init__(
        self,
        sim: base.Simulation,
        arm: base.ArmMotion,
        finger: base.FingerMotion,
        car1: CarController,
        car2: CarController,
        target_truth: np.ndarray,
        save_snapshots: bool,
    ) -> None:
        self.sim = sim
        self.arm = arm
        self.finger = finger
        self.car1 = car1
        self.car2 = car2
        self.target_truth = target_truth
        self.save_snapshots = save_snapshots
        self.cars = [car1, car2]

    def inter_car_contacts(self) -> int:
        car1_root = int(base.MODEL.body_rootid[self.car1.body_id])
        car2_root = int(base.MODEL.body_rootid[self.car2.body_id])
        count = 0
        for contact_index in range(base.DATA.ncon):
            contact = base.DATA.contact[contact_index]
            geom1_root = int(
                base.MODEL.body_rootid[base.MODEL.geom_bodyid[contact.geom1]]
            )
            geom2_root = int(
                base.MODEL.body_rootid[base.MODEL.geom_bodyid[contact.geom2]]
            )
            if {geom1_root, geom2_root} == {car1_root, car2_root}:
                count += 1
        return count

    def run(self) -> bool:
        print("\n[阶段 1] 两车位于起点红线外侧，任务开始。")
        print(
            f"           CAR1={np.round(self.car1.pose()[0][:2], 2)}, "
            f"CAR2={np.round(self.car2.pose()[0][:2], 2)}, "
            f"red_line_Y={RETURN_LINE_Y:.2f}"
        )

        print("\n[阶段 2] 一号车前往目标区，正在雷达避障...")
        print("[模块解耦] 移动控制阶段：仅使用雷达和底盘控制，不调用相机/Renderer。")
        self.car1.drive_path(
            self.sim,
            CAR1_SEARCH_PATH,
            self.arm,
            self.finger,
            self.cars,
            snapshot_name="dual_car1_avoid.png" if self.save_snapshots else None,
            final_tolerance=0.12,
            max_speed=6.50,
        )
        observation_pos, observation_yaw = self.car1.pose()

        print(
            f"\n[阶段 3] 一号车抵达停靠点，坐标 "
            f"({observation_pos[0]:.3f}, {observation_pos[1]:.3f})，通知二号车出发。"
        )

        provisional_goal = np.array([0.84, CAR2_DOCK_CENTER_Y])
        car2_follower = PathFollower(
            self.car2,
            [(float(provisional_goal[0]), float(provisional_goal[1]))],
            self.cars,
            snapshot_name="dual_car2_avoid.png" if self.save_snapshots else None,
            waypoint_timeout=32.0,
            final_tolerance=0.07,
            max_speed=6.50,
        )
        print("[并发调度] 二号车已收到出发指令，开始连续前往目标区。")
        run_wait_concurrent(
            self.sim, self.arm, self.finger, self.cars, 0.15, car2_follower
        )

        print("[模块解耦] 一号车已经停车，采集一帧 D435i RGB/深度。")
        print("\n[视觉识别] 采集相机帧并将图像处理放入后台线程...")
        rgb, depth = get_camera_data()
        with ThreadPoolExecutor(max_workers=1) as executor:
            future = executor.submit(process_camera_frame, rgb, depth)
            while not future.done():
                self.arm.hold()
                self.finger.hold()
                if not car2_follower.done:
                    car2_follower.update(self.cars)
                update_user_scene(self.sim.viewer, self.cars)
                self.sim.step()
            target = future.result()
        vision_error = float(np.linalg.norm(target - self.target_truth))
        print("[模块解耦] 后台视觉完成；二号车运动期间仿真主循环未阻塞。")

        predicted_car1_pos = np.array(
            [target[0], target[1] - 0.85], dtype=float
        )
        dock_dy = CAR2_DOCK_CENTER_Y - predicted_car1_pos[1]
        dock_offset = max(
            DOCK_OFFSET_MIN,
            float(np.sqrt(max(CAR2_MIN_SEPARATION**2 - dock_dy**2, 0.0)))
            + 0.02,
        )
        car2_goal = np.array(
            [predicted_car1_pos[0] + dock_offset, CAR2_DOCK_CENTER_Y]
        )
        car2_follower.replace_final_waypoint(
            (float(car2_goal[0]), float(car2_goal[1]))
        )
        dock_ik_error = float("nan")
        dock_comfort = float("nan")

        approach_xy = np.array([target[0], target[1] - 0.85], dtype=float)
        car1_align = PathFollower(
            self.car1,
            [(float(approach_xy[0]), float(approach_xy[1]))],
            self.cars,
            snapshot_name=None,
            waypoint_timeout=18.0,
            final_tolerance=0.12,
            max_speed=1.20,
        )
        run_followers_until_primary_done(
            self.sim,
            self.arm,
            self.finger,
            self.cars,
            car1_align,
            car2_follower,
        )
        turner = SkidTurnFollower(self.car1, CAR_START_YAW, timeout=8.0)
        run_turn_with_secondary(
            self.sim,
            self.arm,
            self.finger,
            self.cars,
            turner,
            car2_follower,
        )
        run_wait_concurrent(
            self.sim,
            self.arm,
            self.finger,
            self.cars,
            0.5,
            car2_follower,
        )

        if self.save_snapshots:
            capture_snapshot(
                "dual_parallel_transit.png",
                self.cars,
                [0.5, 17.0, 0.35],
                13.0,
                135.0,
                -42.0,
            )

        grasp_start = base.DATA.time
        grasp_success = execute_grasp_concurrent(
            self.sim,
            self.arm,
            self.finger,
            self.cars,
            target,
            car2_follower,
        )
        if not grasp_success:
            raise RuntimeError("car1 failed to grasp the target cylinder")
        print(
            f"\n[阶段 4] 一号车抓取成功，夹爪闭合。"
            f" 仿真时间 {base.DATA.time:.2f} s"
        )

        finish_follower(
            self.sim, self.arm, self.finger, self.cars, car2_follower
        )
        self.car2.stop_and_settle(
            self.sim, self.arm, self.finger, self.cars
        )
        car2_pos, car2_yaw = self.car2.pose()
        car1_pose, _ = self.car1.pose()
        car2_y_extent = (
            abs(np.sin(car2_yaw)) * 0.445
            + abs(np.cos(car2_yaw)) * 0.35
        )
        car2_top_edge = float(car2_pos[1] + car2_y_extent)
        car2_outside_frame = car2_top_edge < TARGET_ZONE_Y_MIN
        separation = float(np.linalg.norm(car2_pos[:2] - car1_pose[:2]))
        print(
            f"\n[阶段 5] 二号车已抵达动态停靠点，坐标 "
            f"({car2_pos[0]:.3f}, {car2_pos[1]:.3f})。"
        )
        print(
            f"           两车中心距={separation:.3f} m, "
            f"最小过程距离={self.car2.min_other_separation:.3f} m"
        )
        print(
            f"           CAR2 最上沿 Y={car2_top_edge:.3f} m, "
            f"绿框下边界 Y={TARGET_ZONE_Y_MIN:.3f} m, "
            f"完全在框外={car2_outside_frame}"
        )
        print("           停靠点按最小安全侧向偏移计算，投放姿态由实际 IK 自动选择。")
        if not car2_outside_frame:
            raise RuntimeError("car2 body intersects the radiation frame")
        if separation < CAR2_MIN_SEPARATION:
            raise RuntimeError("unsafe inter-car separation")

        if self.save_snapshots:
            capture_snapshot(
                "dual_parallel_grasp.png",
                self.cars,
                [float((car1_pose[0] + car2_pos[0]) / 2.0),
                 float((car1_pose[1] + car2_pos[1]) / 2.0), 0.40],
                5.6,
                135.0,
                -38.0,
            )

        execute_cooperative_place(
            self.sim, self.arm, self.finger, car2_pos[:2]
        )
        obj_after_drop = base.DATA.qpos[
            base.OBJ_QADR : base.OBJ_QADR + 3
        ].copy()
        drop_xy_error = float(
            np.linalg.norm(obj_after_drop[:2] - car2_pos[:2])
        )
        dropped_inside = bool(
            drop_xy_error < BUCKET_XY_TOL
            and BUCKET_BASE_Z < float(obj_after_drop[2]) < BUCKET_OPENING_Z
        )
        if not dropped_inside:
            raise RuntimeError(
                f"cooperative drop failed, xy error={drop_xy_error:.4f} m"
            )

        print("\n[阶段 6] 投放完成，两车开始倒车/返航。")
        reverse_exit = [
            ReverseStraightFollower(self.car1, RADIATION_EXIT_Y, max_speed=0.55),
            ReverseStraightFollower(self.car2, RADIATION_EXIT_Y, max_speed=0.55),
        ]
        run_two_straight_reverses(
            self.sim,
            self.arm,
            self.finger,
            self.cars,
            reverse_exit,
            snapshot_name="dual_reverse_exit.png" if self.save_snapshots else None,
        )

        car1_return_path = [
            (-2.80, 19.50),
            (-4.80, 18.00),
            (-4.80, 13.00),
            (-4.80, 8.00),
            (-3.50, 3.00),
            (-1.50, 0.00),
            (float(CAR1_START[0]), float(CAR1_START[1])),
        ]
        car2_return_path = [
            (5.00, 20.00),
            (6.20, 17.00),
            (6.20, 13.00),
            (5.00, 8.00),
            (4.50, 3.00),
            (3.00, 0.00),
            (float(CAR2_START[0]), float(CAR2_START[1])),
        ]
        car1_return = PathFollower(
            self.car1,
            car1_return_path,
            self.cars,
            snapshot_name=None,
            waypoint_timeout=28.0,
            final_tolerance=0.10,
            max_speed=6.50,
        )
        car2_return = PathFollower(
            self.car2,
            car2_return_path,
            self.cars,
            snapshot_name=None,
            waypoint_timeout=28.0,
            final_tolerance=0.10,
            max_speed=6.50,
        )
        run_two_followers(
            self.sim,
            self.arm,
            self.finger,
            self.cars,
            [car1_return, car2_return],
        )

        car1_final, _ = self.car1.pose()
        car2_final, _ = self.car2.pose()
        obj_final = base.DATA.qpos[
            base.OBJ_QADR : base.OBJ_QADR + 3
        ].copy()
        final_xy_error = float(
            np.linalg.norm(obj_final[:2] - car2_final[:2])
        )
        final_inside = bool(
            final_xy_error < BUCKET_XY_TOL
            and BUCKET_BASE_Z < float(obj_final[2]) < BUCKET_OPENING_Z
        )
        car1_home_error = float(np.linalg.norm(car1_final[:2] - CAR1_START))
        car2_home_error = float(np.linalg.norm(car2_final[:2] - CAR2_START))
        both_home = bool(car1_home_error < 0.20 and car2_home_error < 0.20)
        both_returned = bool(
            car1_final[1] < RETURN_LINE_Y - 0.20
            and car2_final[1] < RETURN_LINE_Y - 0.20
            and both_home
        )
        contacts = self.inter_car_contacts()

        if not both_returned:
            raise RuntimeError("one or both cars did not return behind the red line")
        if contacts:
            raise RuntimeError("physical contact detected between CAR1 and CAR2")
        print(
            f"\n[阶段 7] 两车均已安全退回红线外侧，任务结束。\n"
            f"           CAR1=({car1_final[0]:.3f}, {car1_final[1]:.3f}), "
            f"CAR2=({car2_final[0]:.3f}, {car2_final[1]:.3f})\n"
            f"           视觉误差={vision_error * 1000:.2f} mm, "
            f"桶内误差={final_xy_error:.4f} m, 车车接触={contacts}\n"
            f"           回起点误差: CAR1={car1_home_error:.3f} m, "
            f"CAR2={car2_home_error:.3f} m"
        )

        if self.save_snapshots:
            capture_snapshot(
                "dual_final_returned.png",
                self.cars,
                [0.25, -3.5, 0.35],
                9.0,
                135.0,
                -38.0,
            )
            capture_snapshot(
                "dual_bucket_top.png",
                self.cars,
                [float(car2_final[0]), float(car2_final[1]), 0.42],
                0.62,
                90.0,
                -80.0,
            )
        return bool(final_inside and both_returned and contacts == 0)

def initialize_mission(seed: int | None) -> tuple[
    base.ArmMotion,
    base.FingerMotion,
    CarController,
    CarController,
    np.ndarray,
]:
    global SIM_TRUTH_TARGET
    arm, finger = base.setup_initial_state()

    car1 = CarController(
        "CAR1",
        "mobile_chassis",
        [
            "wheel_fl_joint",
            "wheel_fr_joint",
            "wheel_rl_joint",
            "wheel_rr_joint",
        ],
        ["wheel_fl_steer_joint", "wheel_fr_steer_joint"],
        "lidar_site",
        np.array([0.1, 0.9, 1.0, 1.0]),
        "car2_chassis",
        "chassis_free",
    )
    car2 = CarController(
        "CAR2",
        "car2_chassis",
        [
            "car2_wheel_fl_joint",
            "car2_wheel_fr_joint",
            "car2_wheel_rl_joint",
            "car2_wheel_rr_joint",
        ],
        ["car2_wheel_fl_steer_joint", "car2_wheel_fr_steer_joint"],
        "car2_lidar_site",
        np.array([0.2, 1.0, 0.45, 1.0]),
        "mobile_chassis",
        "car2_chassis_free",
    )

    base.DATA.qpos[car1.qadr : car1.qadr + 3] = [
        CAR1_START[0],
        CAR1_START[1],
        0.0,
    ]
    base.DATA.qpos[car1.qadr + 3 : car1.qadr + 7] = yaw_quaternion(
        CAR_START_YAW
    )

    car2_qpos = car2.qadr
    base.DATA.qpos[car2_qpos : car2_qpos + 3] = [
        CAR2_START[0],
        CAR2_START[1],
        0.0,
    ]
    base.DATA.qpos[car2_qpos + 3 : car2_qpos + 7] = yaw_quaternion(
        CAR_START_YAW
    )

    rng = random.Random(seed)
    obstacle_xy = []
    for obstacle_id in range(6):
        body_id = _id(
            mujoco.mjtObj.mjOBJ_BODY, f"obstacle_{obstacle_id}"
        )
        obstacle_xy.append(base.MODEL.body_pos[body_id, :2].copy())

    target = np.zeros(3, dtype=float)
    for _ in range(200):
        candidate = np.array(
            [
                rng.uniform(*OBJECT_X_RANGE),
                rng.uniform(*OBJECT_Y_RANGE),
                OBJECT_Z,
            ],
            dtype=float,
        )
        if all(
            np.linalg.norm(candidate[:2] - xy) >= 0.62
            for xy in obstacle_xy
        ):
            target = candidate
            break
    else:
        raise RuntimeError("could not sample a collision-free target")
    base.DATA.qpos[base.OBJ_QADR : base.OBJ_QADR + 3] = target
    base.DATA.qpos[base.OBJ_QADR + 3 : base.OBJ_QADR + 7] = [1.0, 0.0, 0.0, 0.0]
    base.DATA.qvel[:] = 0.0
    base.DATA.ctrl[:] = 0.0
    mujoco.mj_forward(base.MODEL, base.DATA)

    SIM_TRUTH_TARGET = target.copy()
    arm.q_hold = base.CARRY_Q.copy()
    arm.hold()
    finger.target = base.FINGER_OPEN
    finger.hold()
    return arm, finger, car1, car2, target


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--snapshot", action="store_true")
    parser.add_argument("--seed", type=int, default=None)
    args = parser.parse_args()

    print(f"scene: {base.MOBILE_SCENE}")
    print(f"model: nq={base.MODEL.nq} nv={base.MODEL.nv} nu={base.MODEL.nu}")
    arm, finger, car1, car2, target = initialize_mission(args.seed)
    print(f"random radioactive target: {np.round(target, 4)}")

    if args.headless or args.snapshot:
        sim = base.Simulation(realtime=False)
        if args.snapshot:
            capture_snapshot(
                "dual_overview.png",
                [car1, car2],
                [2.0, 11.0, 0.20],
                38.0,
                135.0,
                -52.0,
            )
        mission = DualCarMission(
            sim, arm, finger, car1, car2, target,
            save_snapshots=args.snapshot,
        )
        mission.run()
        return

    import mujoco.viewer

    with mujoco.viewer.launch_passive(base.MODEL, base.DATA) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.lookat[:] = [2.0, 11.0, 0.30]
        viewer.cam.distance = 38.0
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -52.0

        sim = base.Simulation(viewer, realtime=True)
        mission = DualCarMission(
            sim, arm, finger, car1, car2, target, save_snapshots=False
        )
        mission.run()
        print("Mission complete. Close viewer to exit.")
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.02)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted.")
