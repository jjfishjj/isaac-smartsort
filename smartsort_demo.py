"""Isaac SmartSort: cloud-based vision-guided robotic sorting demo.

Run inside the Isaac Sim 5.1.0 container:
    ./python.sh /workspace/isaac-smartsort/smartsort_demo.py
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {"headless": True, "width": 1280, "height": 720},
    experience="/isaac-sim/apps/isaacsim.exp.full.streaming.kit",
)

import csv
import math
import os
import random
import shutil
from datetime import datetime
from pathlib import Path

import numpy as np
from PIL import Image
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid, VisualCuboid
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers import PickPlaceController
from isaacsim.sensors.camera import Camera


TRIALS = int(os.getenv("SMARTSORT_TRIALS", "20"))
CAPTURE_VIDEO = os.getenv("SMARTSORT_CAPTURE_VIDEO", "0") == "1"
FRAME_INTERVAL = int(os.getenv("SMARTSORT_FRAME_INTERVAL", "5"))
SEED = 42
RESULTS_DIR = Path(__file__).resolve().parent / "results"
RESULTS_DIR.mkdir(parents=True, exist_ok=True)
VIDEO_FRAMES_DIR = RESULTS_DIR / "video_frames"

# The Franka pick-and-place controller is validated on a z=0 work plane.
# The visible table body is placed beneath that plane.
TABLE_TOP_Z = 0.0
CUBE_SIZE = 0.045
LEFT_BIN = np.array([0.48, 0.34, TABLE_TOP_Z + CUBE_SIZE / 2])
RIGHT_BIN = np.array([0.48, -0.34, TABLE_TOP_Z + CUBE_SIZE / 2])
COLORS = {
    "red": np.array([0.85, 0.08, 0.05]),
    "blue": np.array([0.05, 0.18, 0.90]),
}


def build_scene():
    print("[SmartSort] build scene", flush=True)
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    world.scene.add(
        FixedCuboid(
            prim_path="/World/SmartSort/Table",
            name="table",
            position=np.array([0.48, 0.0, -0.38]),
            scale=np.array([1.15, 0.82, 0.76]),
            color=np.array([0.32, 0.34, 0.38]),
        )
    )
    world.scene.add(
        VisualCuboid(
            prim_path="/World/SmartSort/LeftBin",
            name="left_bin",
            position=LEFT_BIN - np.array([0.0, 0.0, 0.018]),
            scale=np.array([0.20, 0.18, 0.025]),
            color=COLORS["red"],
        )
    )
    world.scene.add(
        VisualCuboid(
            prim_path="/World/SmartSort/RightBin",
            name="right_bin",
            position=RIGHT_BIN - np.array([0.0, 0.0, 0.018]),
            scale=np.array([0.20, 0.18, 0.025]),
            color=COLORS["blue"],
        )
    )

    robot = world.scene.add(
        Franka(
            prim_path="/World/SmartSort/Franka",
            name="franka",
            position=np.array([0.0, 0.0, TABLE_TOP_Z]),
        )
    )
    cubes = []
    for index in range(3):
        cubes.append(
            world.scene.add(
                DynamicCuboid(
                    prim_path=f"/World/SmartSort/Cube_{index}",
                    name=f"cube_{index}",
                    position=np.array([0.42 + index * 0.06, 0.0, CUBE_SIZE / 2]),
                    scale=np.array([CUBE_SIZE] * 3),
                    color=COLORS["red"],
                    mass=0.05,
                )
            )
        )

    camera = Camera(
        prim_path="/World/SmartSort/Camera",
        position=np.array([1.45, 1.15, 1.75]),
        frequency=20,
        resolution=(1280, 720),
    )
    camera.set_world_pose(
        position=np.array([1.45, 1.15, 1.75]),
        orientation=np.array([0.3338, 0.5430, 0.2811, 0.7172]),
    )

    robot.gripper.set_default_state(robot.gripper.joint_opened_positions)
    world.reset()
    camera.initialize()
    for _ in range(30):
        world.step(render=True)
    print("[SmartSort] world ready", flush=True)
    return world, robot, cubes, camera


def save_rgb(camera, output_path):
    rgba = camera.get_rgba()
    if rgba is None:
        return False
    Image.fromarray(np.asarray(rgba, dtype=np.uint8)[..., :3]).save(output_path)
    return True


def run_trial(world, robot, cubes, camera, trial, rng):
    cube_index = (trial - 1) % len(cubes)
    cube = cubes[cube_index]
    color_name = rng.choice(("red", "blue"))
    target = LEFT_BIN if color_name == "red" else RIGHT_BIN
    target_name = "left" if color_name == "red" else "right"
    pick = np.array([
        rng.uniform(0.38, 0.53),
        rng.uniform(-0.12, 0.10),
        CUBE_SIZE / 2,
    ])

    # Park the unused cubes outside the work area and randomize this trial.
    for index, item in enumerate(cubes):
        if index != cube_index:
            item.set_world_pose(position=np.array([-0.4, 0.4 + 0.08 * index, CUBE_SIZE / 2]))
    cube.set_world_pose(position=pick)
    cube.set_color(COLORS[color_name])
    world.reset()

    controller = PickPlaceController(
        name=f"pick_place_{trial}",
        gripper=robot.gripper,
        robot_articulation=robot,
        end_effector_offset=np.array([0.0, 0.005, 0.0]),
    )
    controller.reset()
    start = datetime.now()
    initial_position, _ = cube.get_world_pose()
    initial_z = float(initial_position[2])
    max_cube_z = initial_z
    initial_gripper_width = float(np.sum(np.abs(robot.gripper.get_joint_positions())))
    min_gripper_width = initial_gripper_width
    lifted = False
    gripper_closed_on_object = False
    if CAPTURE_VIDEO and trial == 1:
        if VIDEO_FRAMES_DIR.exists():
            shutil.rmtree(VIDEO_FRAMES_DIR)
        VIDEO_FRAMES_DIR.mkdir(parents=True)
        save_rgb(camera, RESULTS_DIR / "rgb-scene-start.png")

    for step in range(3000):
        cube_position, _ = cube.get_local_pose()
        current_gripper_width = float(np.sum(np.abs(robot.gripper.get_joint_positions())))
        max_cube_z = max(max_cube_z, float(cube_position[2]))
        min_gripper_width = min(min_gripper_width, current_gripper_width)
        lifted = lifted or float(cube_position[2]) >= initial_z + 0.08
        gripper_closed_on_object = gripper_closed_on_object or current_gripper_width <= max(
            0.055, initial_gripper_width * 0.8
        )
        joints = robot.get_joint_positions()
        actions = controller.forward(
            picking_position=pick,
            placing_position=target,
            current_joint_positions=joints,
            end_effector_orientation=None,
        )
        robot.get_articulation_controller().apply_action(actions)
        world.step(render=True)
        if CAPTURE_VIDEO and trial == 1 and step % FRAME_INTERVAL == 0:
            save_rgb(camera, VIDEO_FRAMES_DIR / f"frame-{step // FRAME_INTERVAL:04d}.png")
        if controller.is_done():
            break

    final_position, _ = cube.get_local_pose()
    elapsed = (datetime.now() - start).total_seconds()
    error = math.dist(final_position[:2], target[:2])
    lift_height = max_cube_z - initial_z
    controller_done = controller.is_done()
    success = controller_done and lifted and gripper_closed_on_object and error <= 0.05
    print(
        f"[GraspVerify] controller_done={controller_done} lifted={lifted} "
        f"lift_height={lift_height:.3f}m gripper_closed={gripper_closed_on_object} "
        f"gripper_width={initial_gripper_width:.4f}->{min_gripper_width:.4f} "
        f"placed_error={error:.3f}m genuine_success={success}",
        flush=True,
    )
    if CAPTURE_VIDEO and trial == 1:
        save_rgb(camera, RESULTS_DIR / "rgb-scene-final.png")
    print(
        f"[SmartSort] Trial {trial}/{TRIALS} | color={color_name} | "
        f"target={target_name.upper()} | error={error:.3f} m | success={success}",
        flush=True,
    )
    return [
        trial, cube_index, color_name, target_name,
        round(float(pick[0]), 4), round(float(pick[1]), 4),
        round(float(final_position[0]), 4), round(float(final_position[1]), 4),
        round(error, 4), round(elapsed, 2), controller_done,
        round(lift_height, 4), round(initial_gripper_width, 4),
        round(min_gripper_width, 4), gripper_closed_on_object, success, success,
    ]


def save_results(rows):
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    csv_path = RESULTS_DIR / f"smartsort-{stamp}.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "trial", "cube", "color", "target_bin", "pick_x", "pick_y",
            "final_x", "final_y", "horizontal_error_m", "elapsed_seconds",
            "controller_done", "lift_height_m", "gripper_initial_width",
            "gripper_min_width", "gripper_closed_on_object",
            "genuine_grasp_verified", "success",
        ])
        writer.writerows(rows)

    successes = sum(bool(row[-1]) for row in rows)
    summary = (
        "Isaac SmartSort\n"
        f"Trials: {len(rows)}\n"
        f"Successes: {successes}\n"
        f"Success rate: {100.0 * successes / len(rows):.1f}%\n"
    )
    (RESULTS_DIR / "latest-summary.txt").write_text(summary, encoding="utf-8")
    print(summary, flush=True)
    print(f"[SmartSort] CSV: {csv_path}", flush=True)


def main():
    print("[SmartSort] main started", flush=True)
    rng = random.Random(SEED)
    world, robot, cubes, camera = build_scene()
    rows = [
        run_trial(world, robot, cubes, camera, trial, rng)
        for trial in range(1, TRIALS + 1)
    ]
    save_results(rows)

    # Keep the final scene visible for WebRTC recording.
    while simulation_app.is_running():
        world.step(render=True)


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()

