from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import csv
import os
import numpy as np
import omni.usd
from pxr import Gf
from PIL import Image

from isaacsim.core.api import World
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.rotations import lookat_to_quatf
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import PickPlaceController
from isaacsim.sensors.camera import Camera
from isaacsim.sensors.physics import IMUSensor, ContactSensor

STAGE_PATH = "/home/pw/Documents/isaacsim/practicum/m04_manipulation/franka_scene.usd"
OUT_DIR = "/home/pw/Documents/isaacsim/practicum/m05_sensors/multisensor_log"
IMG_DIR = f"{OUT_DIR}/images"
os.makedirs(IMG_DIR, exist_ok=True)

omni.usd.get_context().open_stage(STAGE_PATH)
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

franka = world.scene.add(Franka(prim_path="/World/franka", name="franka"))
cube = world.scene.add(SingleRigidPrim(prim_path="/World/Cube", name="cube"))
world.reset()

imu = IMUSensor(prim_path="/World/franka/panda_hand/imu")
contact_left = ContactSensor(prim_path="/World/franka/panda_leftfinger/contact")
contact_right = ContactSensor(prim_path="/World/franka/panda_rightfinger/contact")

camera = Camera(prim_path="/World/Camera", position=np.array([1.5, 1.5, 1.5]), resolution=(320, 240))
camera.initialize()

target = Gf.Vec3d(0.4, 0.0, 0.2)
eye = Gf.Vec3d(1.5, 1.5, 1.5)
up = Gf.Vec3d(0.0, 0.0, 1.0)
quat = lookat_to_quatf(target, eye, up)
orientation = np.array([quat.GetReal(), *quat.GetImaginary()])
camera.set_world_pose(position=np.array([eye[0], eye[1], eye[2]]), orientation=orientation, camera_axes="usd")

world.reset()
for _ in range(30):
    world.step(render=True)

controller = PickPlaceController(name="pick_place", gripper=franka.gripper, robot_articulation=franka)
franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)

cube_pos_start, _ = cube.get_world_pose()
place_position = cube_pos_start + np.array([0.0, 0.3, 0.0])

CSV_PATH = f"{OUT_DIR}/log.csv"
IMAGE_EVERY_N_STEPS = 20

with open(CSV_PATH, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "step", "event",
        "imu_ang_vel_x", "imu_ang_vel_y", "imu_ang_vel_z",
        "imu_lin_acc_x", "imu_lin_acc_y", "imu_lin_acc_z",
        "contact_left_in_contact", "contact_left_force",
        "contact_right_in_contact", "contact_right_force",
        "image_file",
    ])

    step = 0
    while not controller.is_done() and step < 2000:
        actions = controller.forward(
            picking_position=cube_pos_start,
            placing_position=place_position,
            current_joint_positions=franka.get_joint_positions(),
        )
        franka.apply_action(actions)
        world.step(render=True)

        imu_frame = imu.get_current_frame()
        cl = contact_left.get_current_frame()
        cr = contact_right.get_current_frame()

        image_file = ""
        if step % IMAGE_EVERY_N_STEPS == 0:
            rgb = camera.get_rgba()
            if rgb is not None and rgb.size > 0:
                image_file = f"images/step_{step:04d}.png"
                Image.fromarray(rgb[:, :, :3]).save(f"{OUT_DIR}/{image_file}")

        writer.writerow([
            step, controller.get_current_event(),
            *imu_frame["ang_vel"].tolist(),
            *imu_frame["lin_acc"].tolist(),
            cl["in_contact"], cl["force"],
            cr["in_contact"], cr["force"],
            image_file,
        ])
        step += 1

print("총 스텝:", step)
print("CSV 저장 완료:", CSV_PATH)
print("이미지 저장 개수:", len(os.listdir(IMG_DIR)))

simulation_app.close()
