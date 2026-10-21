"""Render initial and final mobile-manipulator views."""
import mujoco
from PIL import Image

import mobile_pick_place as task


def shot(name, lookat, distance, azimuth, elevation):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat
    cam.distance = distance
    cam.azimuth = azimuth
    cam.elevation = elevation
    with mujoco.Renderer(task.MODEL, height=480, width=640) as renderer:
        renderer.update_scene(task.DATA, camera=cam)
        image = renderer.render()
    Image.fromarray(image).save(name)
    print('saved', name)


arm, finger = task.setup_initial_state()
shot('mobile_overview.png', [1.25, 0.90, 0.25], 3.60, 135.0, -24.0)

sim = task.Simulation(realtime=False)
task.run_sequence(sim, arm, finger)
shot('mobile_final.png', [1.25, 0.90, 0.25], 3.60, 135.0, -24.0)
shot('mobile_bucket_top.png', [2.50, 2.00, 0.03], 0.55, 90.0, -80.0)